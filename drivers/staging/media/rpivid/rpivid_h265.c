// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cedrus VPU driver
 *
 * Copyright (C) 2013 Jens Kuske <jenskuske@gmail.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2018 Bootlin
 */

#include <linux/types.h>
#include <linux/delay.h>

#include <media/videobuf2-dma-contig.h>

#include "rpivid.h"
#include "rpivid_hw.h"


enum HEVCSliceType {
    HEVC_SLICE_B = 0,
    HEVC_SLICE_P = 1,
    HEVC_SLICE_I = 2,
};

enum HEVCLayer {
    L0 = 0,
    L1 = 1
};


static inline unsigned int rnd64(const unsigned int x)
{
    return (x + 63) & ~63U;
}

static int gptr_alloc(struct rpivid_dev * const dev,
                      struct rpivid_gptr * gptr, size_t size, unsigned long attrs)
{
    gptr->size = size;
    gptr->attrs = attrs;
    gptr->addr = 0;
    gptr->ptr = dma_alloc_attrs(dev->dev, gptr->size, &gptr->addr, GFP_KERNEL, gptr->attrs);
    return gptr->ptr == NULL ? -ENOMEM : 0;
}

static void gptr_free(struct rpivid_dev * const dev, struct rpivid_gptr * const gptr)
{
    if (gptr->ptr != NULL)
        dma_free_attrs(dev->dev, gptr->size, gptr->ptr, gptr->addr, gptr->attrs);
    gptr->size = 0;
    gptr->ptr = NULL;
    gptr->addr = 0;
    gptr->attrs = 0;
}

#define NUM_SCALING_FACTORS 4064

#define AXI_BASE64 0

#define PROB_BACKUP ((20<<12) + (20<<6) + (0<<0))
#define PROB_RELOAD ((20<<12) + (20<<0) + (0<<6))

#define HEVC_MAX_REFS   V4L2_HEVC_DPB_ENTRIES_NUM_MAX
#define RPIVID_COL_PICS 17                 // 16 ref & current

#define RPIVID_BITBUFS          2          // Bit + Cmd bufs (phase 0 & 1)
#define RPIVID_BITBUF_SIZE      (4 << 20)  // Bit + Cmd buf size

#define RPIVID_COEFFBUFS        3          // PU + Coeff bufs (phase 1 & 2)
#define RPIVID_COEFFBUF_SIZE    (16 << 20) // PU + Coeff buf size

//////////////////////////////////////////////////////////////////////////////
//
// Register offsets

#define RPI_SPS0         0
#define RPI_SPS1         4
#define RPI_PPS          8
#define RPI_SLICE        12
#define RPI_TILESTART    16
#define RPI_TILEEND      20
#define RPI_SLICESTART   24
#define RPI_MODE         28
#define RPI_LEFT0        32
#define RPI_LEFT1        36
#define RPI_LEFT2        40
#define RPI_LEFT3        44
#define RPI_QP           48
#define RPI_CONTROL      52
#define RPI_STATUS       56
#define RPI_VERSION      60
#define RPI_BFBASE       64
#define RPI_BFNUM        68
#define RPI_BFCONTROL    72
#define RPI_BFSTATUS     76
#define RPI_PUWBASE      80
#define RPI_PUWSTRIDE    84
#define RPI_COEFFWBASE   88
#define RPI_COEFFWSTRIDE 92
#define RPI_SLICECMDS    96
#define RPI_BEGINTILEEND 100
#define RPI_TRANSFER     104
#define RPI_CFBASE       108
#define RPI_CFNUM        112
#define RPI_CFSTATUS     116

#define RPI_PURBASE       0x8000
#define RPI_PURSTRIDE     0x8004
#define RPI_COEFFRBASE    0x8008
#define RPI_COEFFRSTRIDE  0x800C
#define RPI_NUMROWS       0x8010
#define RPI_CONFIG2       0x8014
#define RPI_OUTYBASE      0x8018
#define RPI_OUTYSTRIDE    0x801C
#define RPI_OUTCBASE      0x8020
#define RPI_OUTCSTRIDE    0x8024
#define RPI_STATUS2       0x8028
#define RPI_FRAMESIZE     0x802C
#define RPI_MVBASE        0x8030
#define RPI_MVSTRIDE      0x8034
#define RPI_COLBASE       0x8038
#define RPI_COLSTRIDE     0x803C
#define RPI_CURRPOC       0x8040

//////////////////////////////////////////////////////////////////////////////

struct RPI_CMD {
    u32 addr;
    u32 data;
} __attribute__((packed));


typedef struct rpivid_q_aux_s {
    unsigned int refcount;
    unsigned int q_index;
    struct rpivid_q_aux_s * next;
    struct rpivid_gptr col;
} rpivid_q_aux_t;

//////////////////////////////////////////////////////////////////////////////

typedef enum rpivid_decode_state_e {
    RPIVID_DECODE_SLICE_START,
    RPIVID_DECODE_SLICE_CONTINUE,
    RPIVID_DECODE_ERROR_CONTINUE,
    RPIVID_DECODE_ERROR_DONE,
    RPIVID_DECODE_PHASE1,
    RPIVID_DECODE_END,
} rpivid_decode_state_t;

typedef struct rpivid_dec_env_s {
    struct rpivid_ctx * ctx;
    struct rpivid_dec_env_s * next;

    rpivid_decode_state_t state;
    unsigned int    decode_order;

    int             phase_no;           // Current phase (i.e. the last one we waited for)
    struct dec_env_s * phase_wait_q_next;

    struct RPI_CMD *cmd_fifo;
    unsigned int    cmd_len, cmd_max;
    unsigned int    num_slice_msgs;
    unsigned int    PicWidthInCtbsY;
    unsigned int    PicHeightInCtbsY;
    unsigned int    dpbno_col;
    u32        reg_slicestart;
    int             collocated_from_l0_flag;
    unsigned int    wpp_entry_x;
    unsigned int    wpp_entry_y;

    u32             rpi_config2;
    u32             rpi_framesize;
    u32             rpi_currpoc;

    unsigned int    frame_c_offset;
    unsigned int    frame_stride;
    dma_addr_t      frame_addr;
    dma_addr_t      ref_addrs[16];
    rpivid_q_aux_t * frame_aux;
    rpivid_q_aux_t * col_aux;

    dma_addr_t pu_base_vc;
    dma_addr_t coeff_base_vc;
    u32 pu_stride;
    u32 coeff_stride;

    const struct rpivid_gptr * bit_copy_gptr;
    size_t    bit_copy_len;
    const struct rpivid_gptr * cmd_copy_gptr;

    u16        slice_msgs[2*HEVC_MAX_REFS*8+3];
    u8         scaling_factors[NUM_SCALING_FACTORS];
} dec_env_t;

typedef struct rpivid_dec_state_s {
    struct v4l2_ctrl_hevc_sps sps;
    struct v4l2_ctrl_hevc_pps pps;

    // Helper vars & tables derived from sps/pps
    unsigned int log2_ctb_size; /* log2 width of a CTB */
    unsigned int ctb_width;  /* Width in CTBs */
    unsigned int ctb_height; /* Height in CTBs */
    unsigned int ctb_size;   /* Pic area in CTBs */
    unsigned int num_tile_columns;
    unsigned int num_tile_rows;
    __u8 column_width[sizeof(((struct v4l2_ctrl_hevc_pps *)0)->column_width_minus1)];
    __u8 row_height[sizeof(((struct v4l2_ctrl_hevc_pps *)0)->row_height_minus1)];

    int * col_bd;
    int * row_bd;
    int * ctb_addr_rs_to_ts;
    int * ctb_addr_ts_to_rs;
    int * tile_id;

    // Aux starage for DPB
    // Hold refs
    rpivid_q_aux_t * ref_aux[HEVC_MAX_REFS];
    rpivid_q_aux_t * frame_aux;

    // Slice vars
    unsigned int slice_idx;
    int frame_end;

    // Temp vars per run - don't actually need to persist
    __u8 * src_buf;
    const struct v4l2_ctrl_hevc_slice_params * sh;
    unsigned int nb_refs[2];
    unsigned int slice_qp;
    unsigned int max_num_merge_cand;  // 0 if I-slice
	int dependent_slice_segment_flag; // **** Currently broken - always 0
} dec_state_t;


static inline int clip_int(const int x, const int lo, const int hi)
{
    return x < lo ? lo : x > hi ? hi : x;
}

//////////////////////////////////////////////////////////////////////////////
// Phase 1 command and bit FIFOs

static int p1_z = 0;

// ???? u16 addr - put in u32
static int p1_apb_write(dec_env_t * const de, const u16 addr, const u32 data) {

    if (de->cmd_len==de->cmd_max)
        de->cmd_fifo = krealloc(de->cmd_fifo, (de->cmd_max*=2)*sizeof(struct RPI_CMD), GFP_KERNEL);
    de->cmd_fifo[de->cmd_len].addr = addr;
    de->cmd_fifo[de->cmd_len].data = data;

    if (++p1_z < 256) {
        v4l2_info(&de->ctx->dev->v4l2_dev, "[%02x] %x %x\n", de->cmd_len, addr, data);
    }

    return de->cmd_len++;
}

static int ctb_to_tile (unsigned int ctb, unsigned int *bd, int num) {
    int i;
    for (i=1; ctb >= bd[i]; i++); // bd[] has num+1 elements; bd[0]=0; see hevc_ps.c
    return i-1;
}

static int ctb_to_slice_w_h (unsigned int ctb, int ctb_size, int width, unsigned int *bd, int num) {
    if (ctb < bd[num-1]) return ctb_size;
    else if (width % ctb_size) return width % ctb_size;
    else return ctb_size;
}


static void aux_q_free(struct rpivid_ctx * const ctx, rpivid_q_aux_t *const aq)
{
	struct rpivid_dev * const dev = ctx->dev;
    gptr_free(dev, &aq->col);
    kfree(aq);
}

static rpivid_q_aux_t * aux_q_alloc(struct rpivid_ctx * const ctx)
{
	struct rpivid_dev * const dev = ctx->dev;
    rpivid_q_aux_t *const aq = kzalloc(sizeof(*aq), GFP_KERNEL);

    if (aq == NULL)
        return NULL;

    aq->refcount = 1;
    if (gptr_alloc(dev, &aq->col, ctx->colmv_picsize,
                   DMA_ATTR_FORCE_CONTIGUOUS | DMA_ATTR_NO_KERNEL_MAPPING) != 0) {
        goto fail;
    }

    return aq;

fail:
    kfree(aq);
    return NULL;
}

static rpivid_q_aux_t * aux_q_new(struct rpivid_ctx * const ctx, const unsigned int q_index)
{
    rpivid_q_aux_t * aq;
    unsigned long lockflags;
    spin_lock_irqsave(&ctx->aux_lock, lockflags);
    if ((aq = ctx->aux_free) != NULL)
    {
        ctx->aux_free = aq->next;
        aq->next = NULL;
        aq->refcount = 1;
    }
    spin_unlock_irqrestore(&ctx->aux_lock, lockflags);

    if (aq == NULL) {
        if ((aq = aux_q_alloc(ctx)) == NULL)
            return NULL;
    }

    aq->q_index = q_index;
    ctx->aux_ents[q_index] = aq;
    return aq;
}

static rpivid_q_aux_t * aux_q_ref(struct rpivid_ctx * const ctx, rpivid_q_aux_t * const aq)
{
    if (aq != NULL) {
        unsigned long lockflags;
        spin_lock_irqsave(&ctx->aux_lock, lockflags);

        ++aq->refcount;

        spin_unlock_irqrestore(&ctx->aux_lock, lockflags);
    }
    return aq;
}

static rpivid_q_aux_t * aux_q_release(struct rpivid_ctx * const ctx, rpivid_q_aux_t ** const paq)
{
    rpivid_q_aux_t * const aq = *paq;
    *paq = NULL;

    if (aq != NULL) {
        unsigned long lockflags;
        spin_lock_irqsave(&ctx->aux_lock, lockflags);

        if (--aq->refcount == 0) {
            aq->next = ctx->aux_free;
            ctx->aux_free = aq;
            ctx->aux_ents[aq->q_index] = NULL;
        }

        spin_unlock_irqrestore(&ctx->aux_lock, lockflags);
    }
}

static void aux_q_init(struct rpivid_ctx * const ctx)
{
    spin_lock_init(&ctx->aux_lock);
    ctx->aux_free = NULL;
}

static void aux_q_uninit(struct rpivid_ctx * const ctx)
{
    rpivid_q_aux_t * aq;
    ctx->colmv_picsize = 0;
    ctx->colmv_stride = 0;
    while ((aq = ctx->aux_free) != NULL) {
        ctx->aux_free = aq->next;
        aux_q_free(ctx, aq);
    }
}

//////////////////////////////////////////////////////////////////////////////

#define RPI_PROB_VALS 154U
#define RPI_PROB_ARRAY_SIZE ((154 + 3) & ~3)

static const uint8_t prob_init[3][156] = {
	{
		 153, 200, 139, 141, 157, 154, 154, 154,
		 154, 154, 184, 154, 154, 154, 184,  63,
		 154, 154, 154, 154, 154, 154, 154, 154,
		 154, 154, 154, 154, 154, 153, 138, 138,
		 111, 141,  94, 138, 182, 154, 154, 154,
		 140,  92, 137, 138, 140, 152, 138, 139,
		 153,  74, 149,  92, 139, 107, 122, 152,
		 140, 179, 166, 182, 140, 227, 122, 197,
		 110, 110, 124, 125, 140, 153, 125, 127,
		 140, 109, 111, 143, 127, 111,  79, 108,
		 123,  63, 110, 110, 124, 125, 140, 153,
		 125, 127, 140, 109, 111, 143, 127, 111,
		  79, 108, 123,  63,  91, 171, 134, 141,
		 138, 153, 136, 167, 152, 152, 139, 139,
		 111, 111, 125, 110, 110,  94, 124, 108,
		 124, 107, 125, 141, 179, 153, 125, 107,
		 125, 141, 179, 153, 125, 107, 125, 141,
		 179, 153, 125, 140, 139, 182, 182, 152,
		 136, 152, 136, 153, 136, 139, 111, 136,
		 139, 111,   0,   0,	},
	{
		 153, 185, 107, 139, 126, 197, 185, 201,
		 154, 149, 154, 139, 154, 154, 154, 152,
		 110, 122,  95,  79,  63,  31,  31, 153,
		 153, 168, 140, 198,  79, 124, 138,  94,
		 153, 111, 149, 107, 167, 154, 154, 154,
		 154, 196, 196, 167, 154, 152, 167, 182,
		 182, 134, 149, 136, 153, 121, 136, 137,
		 169, 194, 166, 167, 154, 167, 137, 182,
		 125, 110,  94, 110,  95,  79, 125, 111,
		 110,  78, 110, 111, 111,  95,  94, 108,
		 123, 108, 125, 110,  94, 110,  95,  79,
		 125, 111, 110,  78, 110, 111, 111,  95,
		  94, 108, 123, 108, 121, 140,  61, 154,
		 107, 167,  91, 122, 107, 167, 139, 139,
		 155, 154, 139, 153, 139, 123, 123,  63,
		 153, 166, 183, 140, 136, 153, 154, 166,
		 183, 140, 136, 153, 154, 166, 183, 140,
		 136, 153, 154, 170, 153, 123, 123, 107,
		 121, 107, 121, 167, 151, 183, 140, 151,
		 183, 140,   0,   0,	},
	{
		 153, 160, 107, 139, 126, 197, 185, 201,
		 154, 134, 154, 139, 154, 154, 183, 152,
		 154, 137,  95,  79,  63,  31,  31, 153,
		 153, 168, 169, 198,  79, 224, 167, 122,
		 153, 111, 149,  92, 167, 154, 154, 154,
		 154, 196, 167, 167, 154, 152, 167, 182,
		 182, 134, 149, 136, 153, 121, 136, 122,
		 169, 208, 166, 167, 154, 152, 167, 182,
		 125, 110, 124, 110,  95,  94, 125, 111,
		 111,  79, 125, 126, 111, 111,  79, 108,
		 123,  93, 125, 110, 124, 110,  95,  94,
		 125, 111, 111,  79, 125, 126, 111, 111,
		  79, 108, 123,  93, 121, 140,  61, 154,
		 107, 167,  91, 107, 107, 167, 139, 139,
		 170, 154, 139, 153, 139, 123, 123,  63,
		 124, 166, 183, 140, 136, 153, 154, 166,
		 183, 140, 136, 153, 154, 166, 183, 140,
		 136, 153, 154, 170, 153, 138, 138, 122,
		 121, 122, 121, 167, 151, 183, 140, 151,
		 183, 140,   0,   0,	},
};

static void WriteProb(dec_env_t *const de, const dec_state_t *const s) {
    uint8_t dst[RPI_PROB_ARRAY_SIZE];

    const unsigned int init_type = ((s->sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_CABAC_INIT) != 0 &&
                                    s->sh->slice_type != HEVC_SLICE_I) ?
        s->sh->slice_type + 1 : 2 - s->sh->slice_type;
    const uint8_t * p = prob_init[init_type];
    const int q = clip_int(s->slice_qp, 0, 51);
    unsigned int i;

    for (i = 0; i < RPI_PROB_VALS; i++) {
        int init_value = p[i];
        int m = (init_value >> 4) * 5 - 45;
        int n = ((init_value & 15) << 3) - 16;
        int pre = 2 * (((m * q) >> 4) + n) - 127;

        pre ^= pre >> 31;
        if (pre > 124)
            pre = 124 + (pre & 1);
        dst[i] = pre;
    }
    for (i = RPI_PROB_VALS; i != RPI_PROB_ARRAY_SIZE; ++i) {
        dst[i] = 0;
    }

    for (i=0; i < RPI_PROB_ARRAY_SIZE; i+=4)
        p1_apb_write(de, 0x1000+i, dst[i] + (dst[i+1]<<8) + (dst[i+2]<<16) + (dst[i+3]<<24));

}

static void WriteScalingFactors(dec_env_t * const de) {
    int i;
    const u8 *p = (u8 *) de->scaling_factors;
    for (i=0; i<NUM_SCALING_FACTORS; i+=4, p+=4)
        p1_apb_write(de, 0x2000+i, p[0] + (p[1]<<8) + (p[2]<<16) + (p[3]<<24));
}

static inline __u32 dma_to_axi_addr(dma_addr_t a)
{
    return (__u32)(a >> 6);
}

static void WriteBitstream(dec_env_t * const de, const dec_state_t * const s) {
//    struct rpivid_ctx *const ctx = de->ctx;
#warning "EMU set - probably shouldn't be wanted"
    const int rpi_use_emu = 1; // FFmpeg removes emulation prevention bytes
    const int offset = 0; // Always 64-byte aligned in sim, need not be on real hardware
    const int len = (s->sh->bit_size + 7) / 8 - (s->sh->data_bit_offset / 8 + 1);
    const __u8 *ptr = s->src_buf + s->sh->data_bit_offset / 8 + 1;

//    v4l2_info(&ctx->dev->v4l2_dev, "%s: len=%d: b=%02x %02x %02x %02x\n", __func__,
//              len, ptr[0], ptr[1], ptr[2], ptr[3]);

    memcpy(de->bit_copy_gptr->ptr + de->bit_copy_len, ptr, len);

    p1_apb_write(de, RPI_BFBASE, dma_to_axi_addr(de->bit_copy_gptr->addr + de->bit_copy_len));
    p1_apb_write(de, RPI_BFNUM, len);
    p1_apb_write(de, RPI_BFCONTROL, offset + (1<<7)); // Stop
    p1_apb_write(de, RPI_BFCONTROL, offset + (rpi_use_emu<<6));

    de->bit_copy_len += (len + 63) & ~63;
}

//////////////////////////////////////////////////////////////////////////////

static void write_slice(dec_env_t * const de, const dec_state_t * const s,
                        const unsigned int slice_w, const unsigned int slice_h) {
    u32 u32 =
          (s->sh->slice_type                           << 12)
        + (((s->sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_LUMA) != 0)   << 14)
        + (((s->sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_SAO_CHROMA) != 0) << 15)
        + (slice_w                                    << 17)
        + (slice_h                                    << 24);

    u32 |=
          (s->max_num_merge_cand << 0)
        + (s->nb_refs[L0]        << 4)
        + (s->nb_refs[L1]        << 8);

    if (s->sh->slice_type==HEVC_SLICE_B)
        u32 |= ((s->sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_MVD_L1_ZERO) != 0) << 16;
    p1_apb_write(de, RPI_SLICE, u32);
}

//////////////////////////////////////////////////////////////////////////////
// Tiles mode

static void new_entry_point(dec_env_t * const de, const dec_state_t * const s,
                            const int do_bte, const int resetQPY, const int ctb_addr_ts) {

    int ctb_col = s->ctb_addr_ts_to_rs[ctb_addr_ts] % de->PicWidthInCtbsY;
    int ctb_row = s->ctb_addr_ts_to_rs[ctb_addr_ts] / de->PicWidthInCtbsY;

    int tile_x = ctb_to_tile(ctb_col, s->col_bd, s->num_tile_columns);
    int tile_y = ctb_to_tile(ctb_row, s->row_bd, s->num_tile_rows);

    int endx = s->col_bd[tile_x+1] - 1;
    int endy = s->row_bd[tile_y+1] - 1;

    u8 slice_w = ctb_to_slice_w_h(ctb_col, 1 << s->log2_ctb_size, s->sps.pic_width_in_luma_samples, s->col_bd, s->num_tile_columns);
    u8 slice_h = ctb_to_slice_w_h(ctb_row, 1 << s->log2_ctb_size, s->sps.pic_height_in_luma_samples, s->row_bd, s->num_tile_rows);

    p1_apb_write(de, RPI_TILESTART, s->col_bd[tile_x] + (s->row_bd[tile_y]<<16));
    p1_apb_write(de, RPI_TILEEND, endx + (endy<<16));

    if (do_bte)
        p1_apb_write(de, RPI_BEGINTILEEND, endx + (endy<<16));

    write_slice(de, s, slice_w, slice_h);

    if (resetQPY)
    {
        unsigned int sps_qp_bd_offset = 6 * s->sps.bit_depth_luma_minus8;
        p1_apb_write(de, RPI_QP, sps_qp_bd_offset + s->slice_qp);
    }

    p1_apb_write(de, RPI_MODE, (0xFFFF                            <<  0)
                              + (0x0                               << 16)
                 + ((tile_x == s->num_tile_columns - 1) << 17)
                 + ((tile_y == s->num_tile_rows - 1) << 18));

    p1_apb_write(de, RPI_CONTROL, (ctb_col<<0) + (ctb_row<<16));
}

//////////////////////////////////////////////////////////////////////////////

static void new_slice_segment(dec_env_t * const de, const dec_state_t * const s)
{
    const struct v4l2_ctrl_hevc_sps * const sps = &s->sps;
    const struct v4l2_ctrl_hevc_pps * const pps = &s->pps;

    p1_apb_write(de, RPI_SPS0,
        ((sps->log2_min_luma_coding_block_size_minus3 + 3)    <<  0) |
        (s->log2_ctb_size                                     <<  4) |
        ((sps->log2_min_luma_transform_block_size_minus2 + 2) <<  8) |
        ((sps->log2_min_luma_transform_block_size_minus2 + 2 +
          sps->log2_diff_max_min_luma_transform_block_size)   << 12) |
        ((sps->bit_depth_luma_minus8 + 8)                     << 16) |
        ((sps->bit_depth_chroma_minus8 + 8)                   << 20) |
        (sps->max_transform_hierarchy_depth_intra             << 24) |
        (sps->max_transform_hierarchy_depth_inter             << 28));

    p1_apb_write(de, RPI_SPS1,
        ((sps->pcm_sample_bit_depth_luma_minus1 + 1)               <<  0) |
        ((sps->pcm_sample_bit_depth_chroma_minus1 + 1)             <<  4) |
        ((sps->log2_min_pcm_luma_coding_block_size_minus3 + 3)     <<  8) |
        ((sps->log2_min_pcm_luma_coding_block_size_minus3 + 3 +
          sps->log2_diff_max_min_pcm_luma_coding_block_size)       << 12) |
        (((sps->flags & V4L2_HEVC_SPS_FLAG_SEPARATE_COLOUR_PLANE) != 0 ? 0 :
           sps->chroma_format_idc)                                 << 16) |
        (((sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED) != 0)      << 18) |
        (((sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) != 0)      << 19) |
        (((sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) != 0) << 20) |
        (((sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) != 0) << 21));

    p1_apb_write(de, RPI_PPS,
        ((s->log2_ctb_size - pps->diff_cu_qp_delta_depth)                   <<  0) |
        (((pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED) != 0)       <<  4) |
        (((pps->flags & V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED) != 0) <<  5) |
        (((pps->flags & V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED) != 0)    <<  6) |
        (((pps->flags & V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED) != 0)  <<  7) |
        (((pps->pps_cb_qp_offset + s->sh->slice_cb_qp_offset)&255)          <<  8) |
        (((pps->pps_cr_qp_offset + s->sh->slice_cr_qp_offset)&255)          << 16) |
        (((pps->flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) != 0)    << 24));

    if ((sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) != 0)
        WriteScalingFactors(de);

    if (!s->dependent_slice_segment_flag) {
        int ctb_col = s->sh->slice_segment_addr % de->PicWidthInCtbsY;
        int ctb_row = s->sh->slice_segment_addr / de->PicWidthInCtbsY;
        de->reg_slicestart = (ctb_col<<0) + (ctb_row<<16);
    }

    p1_apb_write(de, RPI_SLICESTART, de->reg_slicestart);
}

//////////////////////////////////////////////////////////////////////////////
// Slice messages

static void msg_slice(dec_env_t * const de, const u16 msg) {
    de->slice_msgs[de->num_slice_msgs++] = msg;
}

static void program_slicecmds(dec_env_t * const de, const int sliceid) {
    int i;
    p1_apb_write(de, RPI_SLICECMDS, de->num_slice_msgs+(sliceid<<8));
    for(i=0; i < de->num_slice_msgs; i++) {
        p1_apb_write(de, 0x4000+4*i, de->slice_msgs[i] & 0xffff);
    }
}

// POCs here are u16! maybe use timestamp?
static int has_backward(const struct v4l2_hevc_dpb_entry * const dpb, const __u8 * const idx, const unsigned int n, const int cur_poc)
{
#warning Maybe rps BEF?
    unsigned int i;
    for (i = 0; i < n; ++i) {
        if (cur_poc < dpb[idx[i]].pic_order_cnt[0])
            return 0;
    }
    return 1;
}

static void pre_slice_decode(dec_env_t *const de, const dec_state_t *const s)
{
    const struct v4l2_ctrl_hevc_slice_params * const sh = s->sh;
    int weightedPredFlag, rIdx;
    u16 cmd_slice;
    unsigned int collocated_from_l0_flag;

    de->num_slice_msgs=0;

    cmd_slice = 0;
    if (sh->slice_type==HEVC_SLICE_I) cmd_slice = 1;
    if (sh->slice_type==HEVC_SLICE_P) cmd_slice = 2;
    if (sh->slice_type==HEVC_SLICE_B) cmd_slice = 3;

    cmd_slice |= (s->nb_refs[L0] << 2) | (s->nb_refs[L1] << 6) |
        (s->max_num_merge_cand << 11);

    collocated_from_l0_flag =
        (sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) == 0 ||
        sh->slice_type != HEVC_SLICE_B ||
        (sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_COLLOCATED_FROM_L0) != 0;
    cmd_slice |= collocated_from_l0_flag<<14;

    if (sh->slice_type==HEVC_SLICE_P || sh->slice_type==HEVC_SLICE_B) {
        // Flag to say all reference pictures are from the past
        const int NoBackwardPredFlag =
            has_backward(sh->dpb, sh->ref_idx_l0, s->nb_refs[L0], sh->slice_pic_order_cnt) &&
            has_backward(sh->dpb, sh->ref_idx_l1, s->nb_refs[L1], sh->slice_pic_order_cnt);
        cmd_slice |= NoBackwardPredFlag<<10;
        msg_slice(de, cmd_slice);

        if ((s->sps.flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) != 0)
        {
            const __u8 * const rpl = collocated_from_l0_flag ?
                sh->ref_idx_l0 :
                sh->ref_idx_l1;
            de->dpbno_col = rpl[sh->collocated_ref_idx];
//            v4l2_info(&de->ctx->dev->v4l2_dev, "L0=%d col_ref_idx=%d, dpb_no=%d\n", collocated_from_l0_flag, sh->collocated_ref_idx, de->dpbno_col);
        }

        // Write reference picture descriptions
        weightedPredFlag = sh->slice_type == HEVC_SLICE_P ?
            (s->pps.flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED) != 0 :
            (s->pps.flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED) != 0;

        for (rIdx = 0; rIdx < s->nb_refs[L0]; ++rIdx) {
            unsigned int dpb_no = sh->ref_idx_l0[rIdx];
//            v4l2_info(&de->ctx->dev->v4l2_dev, "L0[%d]=dpb[%d]\n", rIdx, dpb_no);

            msg_slice(de,
                      dpb_no |
                      (sh->dpb[dpb_no].rps == V4L2_HEVC_DPB_ENTRY_RPS_LT_CURR ? (1 << 4) : 0) |
                      (weightedPredFlag ? (3 << 5) : 0));
            msg_slice(de, sh->dpb[dpb_no].pic_order_cnt[0]);
            if (weightedPredFlag) {
                const struct v4l2_hevc_pred_weight_table * const w = &sh->pred_weight_table;
                const int luma_weight_denom = (1 << w->luma_log2_weight_denom);
                const unsigned int chroma_log2_weight_denom = (w->luma_log2_weight_denom + w->delta_chroma_log2_weight_denom);
                const int chroma_weight_denom = (1 << chroma_log2_weight_denom);
                msg_slice(de, w->luma_log2_weight_denom | (((w->delta_luma_weight_l0[rIdx] + luma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->luma_offset_l0[rIdx] & 0xff);
                msg_slice(de, chroma_log2_weight_denom | (((w->delta_chroma_weight_l0[rIdx][0] + chroma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->chroma_offset_l0[rIdx][0] & 0xff);
                msg_slice(de, chroma_log2_weight_denom | (((w->delta_chroma_weight_l0[rIdx][1] + chroma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->chroma_offset_l0[rIdx][1] & 0xff);
            }
        }

        for (rIdx = 0; rIdx < s->nb_refs[L1]; ++rIdx) {
            unsigned int dpb_no = sh->ref_idx_l1[rIdx];
//            v4l2_info(&de->ctx->dev->v4l2_dev, "L1[%d]=dpb[%d]\n", rIdx, dpb_no);
            msg_slice(de,
                      dpb_no |
                      (sh->dpb[dpb_no].rps == V4L2_HEVC_DPB_ENTRY_RPS_LT_CURR ? (1 << 4) : 0) |
                      (weightedPredFlag ? (3 << 5) : 0));
            msg_slice(de, sh->dpb[dpb_no].pic_order_cnt[0]);
            if (weightedPredFlag) {
                const struct v4l2_hevc_pred_weight_table * const w = &sh->pred_weight_table;
                const int luma_weight_denom = (1 << w->luma_log2_weight_denom);
                const unsigned int chroma_log2_weight_denom = (w->luma_log2_weight_denom + w->delta_chroma_log2_weight_denom);
                const int chroma_weight_denom = (1 << chroma_log2_weight_denom);
                msg_slice(de, w->luma_log2_weight_denom | (((w->delta_luma_weight_l1[rIdx] + luma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->luma_offset_l1[rIdx] & 0xff);
                msg_slice(de, chroma_log2_weight_denom | (((w->delta_chroma_weight_l1[rIdx][0] + chroma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->chroma_offset_l1[rIdx][0] & 0xff);
                msg_slice(de, chroma_log2_weight_denom | (((w->delta_chroma_weight_l1[rIdx][1] + chroma_weight_denom) & 0x1ff) << 3));
                msg_slice(de, w->chroma_offset_l1[rIdx][1] & 0xff);
            }
        }
    }
    else
        msg_slice(de, cmd_slice);

    msg_slice(de,
              (sh->slice_beta_offset_div2 & 15) |
              ((sh->slice_tc_offset_div2 & 15) << 4) |
              ((sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_DEBLOCKING_FILTER_DISABLED) != 0 ? 1 << 8 : 0) |
              ((sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_LOOP_FILTER_ACROSS_SLICES_ENABLED) != 0 ? 1 << 9 : 0) |
              ((s->pps.flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED) != 0 ? 1 << 10 : 0));

    msg_slice(de, ((sh->slice_cr_qp_offset&31)<<5) + (sh->slice_cb_qp_offset&31)); // CMD_QPOFF
}


//////////////////////////////////////////////////////////////////////////////
// Write STATUS register with expected end CTU address of previous slice

static void end_previous_slice(dec_env_t * const de, const dec_state_t * const s, const int ctb_addr_ts) {
    int last_x = s->ctb_addr_ts_to_rs[ctb_addr_ts-1] % de->PicWidthInCtbsY;
    int last_y = s->ctb_addr_ts_to_rs[ctb_addr_ts-1] / de->PicWidthInCtbsY;
    p1_apb_write(de, RPI_STATUS, 1 + (last_x<<5) + (last_y<<18));
}

static void wpp_pause(dec_env_t * const de, int ctb_row) {
    p1_apb_write(de, RPI_STATUS, (ctb_row<<18) + 0x25);
    p1_apb_write(de, RPI_TRANSFER, PROB_BACKUP);
    p1_apb_write(de, RPI_MODE, ctb_row==de->PicHeightInCtbsY-1 ? 0x70000 : 0x30000);
    p1_apb_write(de, RPI_CONTROL, (ctb_row<<16) + 2);
}

static void wpp_end_previous_slice(dec_env_t * const de, const dec_state_t * const s, int ctb_addr_ts) {
    int new_x = s->sh->slice_segment_addr % de->PicWidthInCtbsY;
    int new_y = s->sh->slice_segment_addr / de->PicWidthInCtbsY;
    int last_x = s->ctb_addr_ts_to_rs[ctb_addr_ts-1] % de->PicWidthInCtbsY;
    int last_y = s->ctb_addr_ts_to_rs[ctb_addr_ts-1] / de->PicWidthInCtbsY;
    if (de->wpp_entry_x<2 && (de->wpp_entry_y<new_y || new_x>2) && de->PicWidthInCtbsY>2)
        wpp_pause(de, last_y);
    p1_apb_write(de, RPI_STATUS, 1 + (last_x<<5) + (last_y<<18));
    if (new_x==2 || (de->PicWidthInCtbsY==2 && de->wpp_entry_y<new_y))
        p1_apb_write(de, RPI_TRANSFER, PROB_BACKUP);
}

//////////////////////////////////////////////////////////////////////////////
// Wavefront mode

static void wpp_entry_point(dec_env_t * const de, const dec_state_t * const s,
                            const int do_bte, const int resetQPY, const int ctb_addr_ts) {

    int ctb_size = 1 << s->log2_ctb_size;
    int ctb_addr_rs = s->ctb_addr_ts_to_rs[ctb_addr_ts];

    int ctb_col = de->wpp_entry_x = ctb_addr_rs % de->PicWidthInCtbsY;
    int ctb_row = de->wpp_entry_y = ctb_addr_rs / de->PicWidthInCtbsY;

    int endx = de->PicWidthInCtbsY-1;
    int endy = ctb_row;

    u8 slice_w = ctb_to_slice_w_h(ctb_col, ctb_size, s->sps.pic_width_in_luma_samples,
                                  s->col_bd, s->num_tile_columns);
    u8 slice_h = ctb_to_slice_w_h(ctb_row, ctb_size, s->sps.pic_height_in_luma_samples,
                                  s->row_bd, s->num_tile_rows);

    p1_apb_write(de, RPI_TILESTART, 0);
    p1_apb_write(de, RPI_TILEEND, endx + (endy<<16));

    if (do_bte)
        p1_apb_write(de, RPI_BEGINTILEEND, endx + (endy<<16));

    write_slice(de, s, slice_w, ctb_row==de->PicHeightInCtbsY-1? slice_h : ctb_size);

    if (resetQPY)
    {
        unsigned int sps_qp_bd_offset = 6 * s->sps.bit_depth_luma_minus8;
        p1_apb_write(de, RPI_QP, sps_qp_bd_offset + s->slice_qp);
    }

    p1_apb_write(de, RPI_MODE, ctb_row==de->PicHeightInCtbsY-1? 0x60001 : 0x20001);
    p1_apb_write(de, RPI_CONTROL, (ctb_col<<0) + (ctb_row<<16));
}

//////////////////////////////////////////////////////////////////////////////
// Wavefront mode

static void wpp_decode_slice(dec_env_t * const de, const dec_state_t * const s, const struct v4l2_ctrl_hevc_slice_params * sh, int ctb_addr_ts)
{
    int i, resetQPY=1;
    int indep = !s->dependent_slice_segment_flag;
    int ctb_col = s->sh->slice_segment_addr % de->PicWidthInCtbsY;

    if (ctb_addr_ts)
        wpp_end_previous_slice(de, s, ctb_addr_ts);
    pre_slice_decode(de, s);
    WriteBitstream(de, s);
    if (ctb_addr_ts==0 || indep || de->PicWidthInCtbsY==1)
        WriteProb(de, s);
    else if (ctb_col==0)
        p1_apb_write(de, RPI_TRANSFER, PROB_RELOAD);
    else
        resetQPY=0;
    program_slicecmds(de, s->slice_idx);
    new_slice_segment(de, s);
    wpp_entry_point(de, s, indep, resetQPY, ctb_addr_ts);

    for (i=0; i < s->sh->num_entry_point_offsets; i++) {
        int ctb_addr_rs = s->ctb_addr_ts_to_rs[ctb_addr_ts];
        int ctb_row = ctb_addr_rs / de->PicWidthInCtbsY;
        int last_x = de->PicWidthInCtbsY-1;
        if (de->PicWidthInCtbsY>2)
            wpp_pause(de, ctb_row);
        p1_apb_write(de, RPI_STATUS, (ctb_row<<18) + (last_x<<5) + 2);
        if (de->PicWidthInCtbsY==2)
            p1_apb_write(de, RPI_TRANSFER, PROB_BACKUP);
        if (de->PicWidthInCtbsY==1)
            WriteProb(de, s);
        else
            p1_apb_write(de, RPI_TRANSFER, PROB_RELOAD);
        ctb_addr_ts += s->column_width[0];
        wpp_entry_point(de, s, 0, 1, ctb_addr_ts);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Tiles mode

static void decode_slice(dec_env_t * const de, const dec_state_t * const s, const struct v4l2_ctrl_hevc_slice_params * const sh, int ctb_addr_ts) {
    int i, resetQPY;

    if (ctb_addr_ts)
        end_previous_slice(de, s, ctb_addr_ts);

    pre_slice_decode(de, s);
    WriteBitstream(de, s);

    if (p1_z < 256) {
        v4l2_info(&de->ctx->dev->v4l2_dev, "TS=%d, tile=%d/%d, dss=%d, flags=%#llx\n", ctb_addr_ts,
            s->tile_id[ctb_addr_ts], s->tile_id[ctb_addr_ts-1],
            s->dependent_slice_segment_flag, sh->flags);
    }

    resetQPY = ctb_addr_ts == 0
            || s->tile_id[ctb_addr_ts] != s->tile_id[ctb_addr_ts-1]
            || !s->dependent_slice_segment_flag;
    if (resetQPY)
        WriteProb(de, s);

    program_slicecmds(de, s->slice_idx);
    new_slice_segment(de, s);
    new_entry_point(de, s, !s->dependent_slice_segment_flag, resetQPY, ctb_addr_ts);

    for (i=0; i < s->sh->num_entry_point_offsets; i++) {
        int ctb_addr_rs = s->ctb_addr_ts_to_rs[ctb_addr_ts];
        int ctb_col = ctb_addr_rs % de->PicWidthInCtbsY;
        int ctb_row = ctb_addr_rs / de->PicWidthInCtbsY;
        int tile_x = ctb_to_tile(ctb_col, s->col_bd, s->num_tile_columns - 1);
        int tile_y = ctb_to_tile(ctb_row, s->row_bd, s->num_tile_rows - 1);
        int last_x = s->col_bd[tile_x+1]-1;
        int last_y = s->row_bd[tile_y+1]-1;
        p1_apb_write(de, RPI_STATUS, 2 + (last_x<<5) + (last_y<<18));
        WriteProb(de, s);
        ctb_addr_ts += s->column_width[tile_x] * s->row_height[tile_y];
        new_entry_point(de, s, 0, 1, ctb_addr_ts);
    }
}

//////////////////////////////////////////////////////////////////////////////
// Scaling factors

static void expand_scaling_list(
    const unsigned int sizeID,
    const unsigned int matrixID,
    uint8_t * const dst0,
    const uint8_t * const src0,
    uint8_t dc)
{
    switch (sizeID) {
        case 0:
            memcpy(dst0, src0, 16);
            break;
        case 1:
            memcpy(dst0, src0, 64);
            break;
        case 2:
        {
            uint8_t * d = dst0;
            unsigned int x, y;
            for (y=0; y != 16; y++) {
                const uint8_t * s = src0 + (y >> 1) * 8;
                for (x = 0; x != 8; ++x) {
                    *d++ = *s;
                    *d++ = *s++;
                }
            }
            dst0[0] = dc;
            break;
        }
        default:
        {
            uint8_t * d = dst0;
            unsigned int x, y;
            for (y=0; y != 32; y++) {
                const uint8_t * s = src0 + (y >> 2) * 8;
                for (x = 0; x != 8; ++x) {
                    *d++ = *s;
                    *d++ = *s;
                    *d++ = *s;
                    *d++ = *s++;
                }
            }
            dst0[0] = dc;
            break;
        }
    }
}

static void populate_scaling_factors(const struct rpivid_run * const run,
                                     dec_env_t * const de, const dec_state_t * const s)
{
    const struct v4l2_ctrl_hevc_scaling_matrix *const sl = run->h265.scaling_matrix;
    // Array of constants for scaling factors
    static const uint32_t scaling_factor_offsets[4][6] = {
        // MID0    MID1    MID2    MID3    MID4    MID5
        {0x0000, 0x0010, 0x0020, 0x0030, 0x0040, 0x0050},   // SID0 (4x4)
        {0x0060, 0x00A0, 0x00E0, 0x0120, 0x0160, 0x01A0},   // SID1 (8x8)
        {0x01E0, 0x02E0, 0x03E0, 0x04E0, 0x05E0, 0x06E0},   // SID2 (16x16)
        {0x07E0, 0x0BE0,      0,      0,      0,      0}};  // SID3 (32x32)

    unsigned int mid;

    for (mid=0; mid<6; mid++)
        expand_scaling_list(0, mid,
            de->scaling_factors + scaling_factor_offsets[0][mid],
            sl->scaling_list_4x4[mid], 0);
    for (mid=0; mid<6; mid++)
        expand_scaling_list(1, mid,
            de->scaling_factors + scaling_factor_offsets[1][mid],
            sl->scaling_list_8x8[mid], 0);
    for (mid=0; mid<6; mid++)
        expand_scaling_list(2, mid,
            de->scaling_factors + scaling_factor_offsets[2][mid],
            sl->scaling_list_16x16[mid],
            sl->scaling_list_dc_coef_16x16[mid]);
    for (mid=0; mid<2; mid += 1)
        expand_scaling_list(3, mid,
            de->scaling_factors + scaling_factor_offsets[3][mid],
            sl->scaling_list_32x32[mid],
            sl->scaling_list_dc_coef_32x32[mid]);
}

static void free_ps_info(dec_state_t * const s)
{
    kfree(s->ctb_addr_rs_to_ts);
    s->ctb_addr_rs_to_ts = NULL;
    kfree(s->ctb_addr_ts_to_rs);
    s->ctb_addr_ts_to_rs = NULL;
    kfree(s->tile_id);
    s->tile_id = NULL;

    kfree(s->col_bd);
    s->col_bd = NULL;
    kfree(s->row_bd);
    s->row_bd = NULL;
}

static int updated_ps(dec_state_t * const s)
{
    unsigned int ctb_addr_rs;
    unsigned int i;

    free_ps_info(s);

    // Inferred parameters
    s->log2_ctb_size = s->sps.log2_min_luma_coding_block_size_minus3 + 3 +
        s->sps.log2_diff_max_min_luma_coding_block_size;

    s->ctb_width  = (s->sps.pic_width_in_luma_samples + (1 << s->log2_ctb_size) - 1) >> s->log2_ctb_size;
    s->ctb_height = (s->sps.pic_height_in_luma_samples + (1 << s->log2_ctb_size) - 1) >> s->log2_ctb_size;
    s->ctb_size   = s->ctb_width * s->ctb_height;

    // Inferred parameters

    if ((s->pps.flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) == 0) {
        s->num_tile_columns = 1;
        s->num_tile_rows = 1;
        s->column_width[0] = s->ctb_width;
        s->row_height[0] = s->ctb_height;
    }
    else
    {
        s->num_tile_columns = s->pps.num_tile_columns_minus1 + 1;
        s->num_tile_rows = s->pps.num_tile_rows_minus1 + 1;
        for (i = 0; i < s->num_tile_columns; ++i)
            s->column_width[i] = s->pps.column_width_minus1[i] + 1;
        for (i = 0; i < s->num_tile_rows; ++i)
            s->row_height[i] = s->pps.row_height_minus1[i] + 1;
    }

    s->col_bd   = kmalloc((s->num_tile_columns + 1) * sizeof(*s->col_bd), GFP_KERNEL);
    s->row_bd   = kmalloc((s->num_tile_rows + 1) * sizeof(*s->row_bd), GFP_KERNEL);

    s->col_bd[0] = 0;
    for (i = 0; i < s->num_tile_columns; i++)
        s->col_bd[i + 1] = s->col_bd[i] + s->column_width[i];

    s->row_bd[0] = 0;
    for (i = 0; i < s->num_tile_rows; i++)
        s->row_bd[i + 1] = s->row_bd[i] + s->row_height[i];

    s->ctb_addr_rs_to_ts = kmalloc(s->ctb_size * sizeof(*s->ctb_addr_rs_to_ts), GFP_KERNEL);
    s->ctb_addr_ts_to_rs = kmalloc(s->ctb_size * sizeof(*s->ctb_addr_ts_to_rs), GFP_KERNEL);
    s->tile_id           = kmalloc(s->ctb_size * sizeof(*s->tile_id), GFP_KERNEL);

    for (ctb_addr_rs = 0; ctb_addr_rs < s->ctb_size; ctb_addr_rs++) {
        int tb_x   = ctb_addr_rs % s->ctb_width;
        int tb_y   = ctb_addr_rs / s->ctb_width;
        int tile_x = 0;
        int tile_y = 0;
        int val    = 0;

        for (i = 0; i < s->num_tile_columns; i++) {
            if (tb_x < s->col_bd[i + 1]) {
                tile_x = i;
                break;
            }
        }

        for (i = 0; i < s->num_tile_rows; i++) {
            if (tb_y < s->row_bd[i + 1]) {
                tile_y = i;
                break;
            }
        }

        for (i = 0; i < tile_x; i++)
            val += s->row_height[tile_y] * s->column_width[i];
        for (i = 0; i < tile_y; i++)
            val += s->ctb_width * s->row_height[i];

        val += (tb_y - s->row_bd[tile_y]) * s->column_width[tile_x] +
               tb_x - s->col_bd[tile_x];

        s->ctb_addr_rs_to_ts[ctb_addr_rs] = val;
        s->ctb_addr_ts_to_rs[val]         = ctb_addr_rs;
    }

    {
        int j, x, y, tile_id;
        for (j = 0, tile_id = 0; j < s->num_tile_rows; j++)
            for (i = 0; i < s->num_tile_columns; i++, tile_id++)
                for (y = s->row_bd[j]; y < s->row_bd[j + 1]; y++)
                    for (x = s->col_bd[i]; x < s->col_bd[i + 1]; x++)
                        s->tile_id[s->ctb_addr_rs_to_ts[y * s->ctb_width + x]] = tile_id;
    }

    return 0;
}

static void frame_end(dec_env_t * const de, const dec_state_t * const s)
{
    const unsigned int last_x = s->col_bd[s->num_tile_columns] - 1;
    const unsigned int last_y = s->row_bd[s->num_tile_rows]-1;
    if ((s->pps.flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) != 0) {
        if (de->wpp_entry_x<2 && de->PicWidthInCtbsY>2)
            wpp_pause(de, last_y);
    }
    p1_apb_write(de, RPI_STATUS, 1 + (last_x<<5) + (last_y<<18));

    // Copy commands out to dma buf
    memcpy(de->cmd_copy_gptr->ptr, de->cmd_fifo, de->cmd_len * sizeof(de->cmd_fifo[0]));

    // End of Phase 0

}

static int setup_colmv(struct rpivid_ctx * const ctx, struct rpivid_run *run, dec_state_t * const s)
{
    ctx->colmv_stride = rnd64(s->sps.pic_width_in_luma_samples);
    ctx->colmv_picsize = ctx->colmv_stride *
        (rnd64(s->sps.pic_height_in_luma_samples) >> 4);
}

// Can be called from irq context
static dec_env_t * dec_env_new(struct rpivid_ctx * const ctx)
{
    dec_env_t * de;
    unsigned long lock_flags;
    spin_lock_irqsave(&ctx->dec_lock, lock_flags);

    de = ctx->dec_free;
    if (de != NULL)
    {
        ctx->dec_free = de->next;
        de->next = NULL;
        de->state = RPIVID_DECODE_SLICE_START;
    }

    spin_unlock_irqrestore(&ctx->dec_lock, lock_flags);
    return de;
}

// Can be called from irq context
static void dec_env_delete(struct rpivid_ctx * const ctx, dec_env_t * const de)
{
    aux_q_release(ctx, &de->frame_aux);
    aux_q_release(ctx, &de->col_aux);

    if (de != NULL) {
        unsigned long lock_flags;
        spin_lock_irqsave(&ctx->dec_lock, lock_flags);

        de->state = RPIVID_DECODE_END;
        de->next = ctx->dec_free;
        ctx->dec_free = de;

        spin_unlock_irqrestore(&ctx->dec_lock, lock_flags);
    }
}

static void dec_env_uninit(struct rpivid_ctx * const ctx)
{
    unsigned int i;

    if (ctx->dec_pool != NULL) {
        for (i = 0; i != RPIVID_DEC_ENV_COUNT; ++i) {
            dec_env_t * const de = ctx->dec_pool + i;
            kfree(de->cmd_fifo);
        }

        kfree(ctx->dec_pool);
    }

    ctx->dec_pool = NULL;
    ctx->dec_free = NULL;
}

static int dec_env_init(struct rpivid_ctx * const ctx)
{
    unsigned int i;

    ctx->dec_pool = kzalloc(sizeof(*ctx->dec_pool) * RPIVID_DEC_ENV_COUNT, GFP_KERNEL);
    if (ctx->dec_pool == NULL)
        return -1;

    spin_lock_init(&ctx->dec_lock);

    // Build free chain
    ctx->dec_free = ctx->dec_pool;
    for (i = 0; i != RPIVID_DEC_ENV_COUNT - 1; ++i)
        ctx->dec_pool[i].next = ctx->dec_pool + i + 1;

    // Fill in other bits
    for (i = 0; i != RPIVID_DEC_ENV_COUNT; ++i)
    {
        dec_env_t * const de = ctx->dec_pool + i;
        de->ctx = ctx;
        de->cmd_max = 1024;
        if ((de->cmd_fifo = kmalloc(de->cmd_max*sizeof(struct RPI_CMD), GFP_KERNEL)) == NULL)
            goto fail;
    }

    return 0;

fail:
    dec_env_uninit(ctx);
    return -1;
}

static void rpivid_h265_setup(struct rpivid_ctx *ctx,
			      struct rpivid_run *run)
{
	struct rpivid_dev * const dev = ctx->dev;
    const struct v4l2_ctrl_hevc_slice_params *const sh = run->h265.slice_params;
	const struct v4l2_hevc_pred_weight_table *pred_weight_table;
	dec_env_t * de;
    dec_state_t * const s = ctx->state;
    int ctb_addr_ts;

	pred_weight_table = &sh->pred_weight_table;

//    v4l2_info(&dev->v4l2_dev, "rpivid_h265_setup: ts=%d:%d:%d.%d\n",
//              run->src->timecode.hours,
//              run->src->timecode.minutes,
//              run->src->timecode.seconds,
//              run->src->timecode.frames);
//    v4l2_info(&dev->v4l2_dev, "vptr=%p\n", vb2_plane_vaddr(&run->src->vb2_buf, 0));

    s->frame_end = ((run->src->flags & V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF) == 0);

    if ((de = ctx->dec0) != NULL && de->state != RPIVID_DECODE_END) {
        ++s->slice_idx;

        switch (de->state){
            case RPIVID_DECODE_SLICE_CONTINUE:
                // Expected state
                break;
            default:
                v4l2_err(&dev->v4l2_dev, "%s: Unexpected state: %d\n", __func__, de->state);
                /* FALLTHRU */
            case RPIVID_DECODE_ERROR_CONTINUE:
                // Uncleared error - fail now
                goto fail;
        }
    }
    else {
		/* Frame start */
        bool sps_changed = false;

        if (memcmp(&s->sps, run->h265.sps, sizeof(s->sps)) != 0) {
            /* SPS changed */
            v4l2_info(&dev->v4l2_dev, "SPS changed\n");
            memcpy(&s->sps, run->h265.sps, sizeof(s->sps));
            sps_changed = true;
        }
        if (sps_changed ||
            memcmp(&s->pps, run->h265.pps, sizeof(s->pps)) != 0) {
            /* SPS changed */
            v4l2_info(&dev->v4l2_dev, "PPS changed\n");
            memcpy(&s->pps, run->h265.pps, sizeof(s->pps));

            /* Recalc stuff as required */
            updated_ps(s);
        }

        {
            const unsigned int CtbSizeY = 1U << (s->sps.log2_min_luma_coding_block_size_minus3 + 3 +
                                                 s->sps.log2_diff_max_min_luma_coding_block_size);

            if ((de = dec_env_new(ctx)) == NULL) {
                v4l2_err(&dev->v4l2_dev, "Failed to find free decode env\n");
                goto fail;
            }
            ctx->dec0 = de;

            de->PicWidthInCtbsY  = (s->sps.pic_width_in_luma_samples + CtbSizeY - 1) / CtbSizeY;  //7-15
            de->PicHeightInCtbsY = (s->sps.pic_height_in_luma_samples + CtbSizeY - 1) / CtbSizeY;  //7-17
            de->cmd_len = 0;
            de->dpbno_col = ~0U;

            de->bit_copy_gptr = ctx->bitbufs + 0;  //***
            de->bit_copy_len = 0;
            de->cmd_copy_gptr = ctx->cmdbufs + 0;  //***

            de->frame_c_offset = ctx->dst_fmt.height * 128;
            de->frame_stride   = ctx->dst_fmt.bytesperline * 128;
            de->frame_addr     = vb2_dma_contig_plane_dma_addr(&run->dst->vb2_buf, 0);
            de->frame_aux      = NULL;

            if (s->sps.bit_depth_luma_minus8 != s->sps.bit_depth_chroma_minus8) {
                v4l2_warn(&dev->v4l2_dev, "Chroma depth (%d) != Luma depth (%d)\n", s->sps.bit_depth_chroma_minus8 + 8, s->sps.bit_depth_luma_minus8 + 8);
                goto fail;
            }
            if (s->sps.bit_depth_luma_minus8 == 0) {
                if (ctx->dst_fmt.pixelformat != V4L2_PIX_FMT_SAND8) {
                    v4l2_err(&dev->v4l2_dev, "Pixel format %#x != SAND8 for 8-bit output", ctx->dst_fmt.pixelformat);
                    goto fail;
                }
            }
            else if (s->sps.bit_depth_luma_minus8 == 2) {
                if (ctx->dst_fmt.pixelformat != V4L2_PIX_FMT_SAND30) {
                    v4l2_err(&dev->v4l2_dev, "Pixel format %#x != SAND30 for 10-bit output", ctx->dst_fmt.pixelformat);
                    goto fail;
                }
            }
            else {
                v4l2_warn(&dev->v4l2_dev, "Luma depth (%d) unsupported\n", s->sps.bit_depth_luma_minus8 + 8);
                goto fail;
            }
            if (run->dst->vb2_buf.num_planes != 1) {
                v4l2_warn(&dev->v4l2_dev, "Capture planes (%d) != 1\n", run->dst->vb2_buf.num_planes);
                goto fail;
            }
            if (run->dst->planes[0].length < ctx->dst_fmt.sizeimage) {
                v4l2_warn(&dev->v4l2_dev, "Capture plane[0] length (%d) < sizeimage (%d(\n", run->dst->planes[0].length, ctx->dst_fmt.sizeimage);
                goto fail;
            }

            if (s->sps.pic_width_in_luma_samples > 4096 || s->sps.pic_height_in_luma_samples > 4096) {
                v4l2_warn(&dev->v4l2_dev, "Pic dimension (%dx%d) exeeds 4096\n", s->sps.pic_width_in_luma_samples, s->sps.pic_height_in_luma_samples);
                goto fail;
            }

            // Fill in ref planes with our address s.t. if we cock
            // up refs somehow then we still have a valid address entry
            {
                unsigned int i;
                for (i = 0; i != 16; ++i)
                    de->ref_addrs[i] = de->frame_addr;
            }

            // Phase 2 reg pre-calc
            de->rpi_config2 =
                ((s->sps.bit_depth_luma_minus8 + 8)             << 0) | // BitDepthY
                ((s->sps.bit_depth_chroma_minus8 + 8)           << 4) | // BitDepthC
                ((s->sps.bit_depth_luma_minus8 != 0)            << 8) | // BitDepthY
                ((s->sps.bit_depth_chroma_minus8 != 0)          << 9) | // BitDepthC
                (s->log2_ctb_size                               <<10) |
                (((s->pps.flags & V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED) != 0) <<13) |
                (((s->sps.flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED) != 0) <<14) |
                (((s->sps.flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) != 0) << 15) |
                ((s->pps.log2_parallel_merge_level_minus2 + 2)  <<16) |
                // ?? Slice info here ??
                (((sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_SLICE_TEMPORAL_MVP_ENABLED) != 0) <<19) |
                (((s->sps.flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED) != 0) <<20) |
                ((s->pps.pps_cb_qp_offset&31)                   <<21) |
                ((s->pps.pps_cr_qp_offset&31)                   <<26);
            de->rpi_framesize = (s->sps.pic_height_in_luma_samples << 16) |
                s->sps.pic_width_in_luma_samples;
            de->rpi_currpoc = sh->slice_pic_order_cnt;  // ?? 16 bit

            if (s->sps.flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
            {
                setup_colmv(ctx, run, s);
            }

            s->slice_idx = 0;
        }

        if (sh->slice_segment_addr != 0) {
            v4l2_warn(&dev->v4l2_dev, "New frame but segment_addr=%d\n", sh->slice_segment_addr);
            goto fail;
        }
	}

    // Pre calc a few things
    s->src_buf = vb2_plane_vaddr(&run->src->vb2_buf, 0);
    s->sh = sh;
    s->slice_qp = 26 + s->pps.init_qp_minus26 + s->sh->slice_qp_delta;
    s->max_num_merge_cand = sh->slice_type == HEVC_SLICE_I ? 0 :
        (5 - sh->five_minus_max_num_merge_cand);
// * SH DSS flag invented by me - but clearly needed
    s->dependent_slice_segment_flag = ((sh->flags & V4L2_HEVC_SLICE_PARAMS_FLAG_DEPENDENT_SLICE_SEGMENT) != 0);

    s->nb_refs[0] = (sh->slice_type == HEVC_SLICE_I) ?
        0 : sh->num_ref_idx_l0_active_minus1 + 1;
    s->nb_refs[1] = (sh->slice_type != HEVC_SLICE_B) ?
        0 : sh->num_ref_idx_l1_active_minus1 + 1;

    if ((s->sps.flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED) != 0)
        populate_scaling_factors(run, de, s);

    ctb_addr_ts = s->ctb_addr_rs_to_ts[sh->slice_segment_addr];

    if ((s->pps.flags & V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED) != 0)
        wpp_decode_slice(de, s, sh, ctb_addr_ts);
    else
        decode_slice(de, s, sh, ctb_addr_ts);

    if (s->frame_end) {
        // Frame end
        rpivid_q_aux_t * dpb_q_aux[V4L2_HEVC_DPB_ENTRIES_NUM_MAX] = {NULL};
        const int use_aux = ((s->sps.flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED) != 0);

        // Locate ref frames
        // At least in the current implementation this is constant across all slices
        // If this changes we will need idx mapping code
        // Uses sh so here rather than trigger

        struct vb2_queue * const vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx,
                               V4L2_BUF_TYPE_VIDEO_CAPTURE);
        unsigned int i;

        if (vq == NULL) {
            v4l2_err(&dev->v4l2_dev, "VQ gone!\n");
            goto fail;
        }

//        v4l2_info(&dev->v4l2_dev, "rpivid_h265_end of frame\n");
        frame_end(de, s);

        // Assume that we get exactly the same DPB for every slice
        // it makes no real sense otherwise
#if V4L2_HEVC_DPB_ENTRIES_NUM_MAX > 16
#error HEVC_DPB_ENTRIES > h/w slots
#endif

        for (i = 0; i < sh->num_active_dpb_entries; ++i) {
            int buffer_index = vb2_find_timestamp(vq, sh->dpb[i].timestamp, 0);
            struct vb2_buffer *buf = buffer_index < 0 ? NULL : vb2_get_buffer(vq, buffer_index);
            if (buf == NULL)
                v4l2_warn(&dev->v4l2_dev, "Missing DPB ent %d, timestamp=%lld, index=%d\n", i, (long long)sh->dpb[i].timestamp, buffer_index);
            else
            {
                if (use_aux) {
                    dpb_q_aux[i] = aux_q_ref(ctx, ctx->aux_ents[buffer_index]);
                    if (dpb_q_aux[i] == NULL)
                        v4l2_warn(&dev->v4l2_dev, "Missing DPB AUX ent %d index=%d\n", i, buffer_index);
                }

                de->ref_addrs[i] = vb2_dma_contig_plane_dma_addr(buf, 0);
            }
//            v4l2_info(&dev->v4l2_dev, "Ref: %d, idx=%d, addr=%x\n", i, buffer_index,
//                      (unsigned int)de->ref_addrs[i]);
        }

        // Move DPB from temp
        for (i = 0; i != V4L2_HEVC_DPB_ENTRIES_NUM_MAX; ++i) {
            aux_q_release(ctx, &s->ref_aux[i]);
            s->ref_aux[i] = dpb_q_aux[i];
        }
        // Unref the old frame aux too - it is eith in the DPB or not now
        aux_q_release(ctx, &s->frame_aux);

        if (use_aux)
        {
            // New frame so new aux ent
            // ??? Do we need this if non-ref ??? can we tell
            s->frame_aux = aux_q_new(ctx, run->dst->vb2_buf.index);

            if (s->frame_aux == NULL) {
                v4l2_err(&dev->v4l2_dev, "Failed to obtain aux storage for frame\n");
                goto fail;
            }

            de->frame_aux = aux_q_ref(ctx, s->frame_aux);
        }

        if (de->dpbno_col != ~0U)
        {
            if (de->dpbno_col >= sh->num_active_dpb_entries) {
                v4l2_err(&dev->v4l2_dev, "Col ref index %d >= %d\n",
                         de->dpbno_col, sh->num_active_dpb_entries);
            }
            else
            {
                // Standard requires that the col pic is constant for the duration of the pic
                // (text of collocated_ref_idx in H265-2 2018 7.4.7.1)

                // Spot the collocated ref in passing
                de->col_aux = aux_q_ref(ctx, dpb_q_aux[de->dpbno_col]);

                if (de->col_aux == NULL) {
                    v4l2_warn(&dev->v4l2_dev, "Missing DPB ent for col\n");
                    // Probably need to abort if this fails as P2 may explode on bad data
                    goto fail;
                }
            }
        }

        de->state = RPIVID_DECODE_PHASE1;
    }
    return;

fail:
    if (de != NULL) {
        // Actual error reporting happens in Trigger
        de->state = s->frame_end ?
            RPIVID_DECODE_ERROR_DONE :
            RPIVID_DECODE_ERROR_CONTINUE;
    }
    return;
}

//////////////////////////////////////////////////////////////////////////////
// Handle PU and COEFF stream overflow


// Returns:
// -2 Other error
// -1 Out of coeff space
//  0  OK
//  1  Out of PU space

static int check_status(const struct rpivid_dev * const dev) {
    const uint32_t cfstatus = apb_read(dev, RPI_CFSTATUS);
    const uint32_t cfnum = apb_read(dev, RPI_CFNUM);
    const uint32_t status = apb_read(dev, RPI_STATUS);

//    v4l2_info(&dev->v4l2_dev, "%s: cfs=%#x, cfn=%#x, s=%#x\n", __func__, cfstatus, cfnum, status);

    // this is the definition of successful completion of phase 1
    // it assures that status register is zero and all blocks in each tile have completed
    if (cfstatus == cfnum)
        return 0;

    if ((status & 8) != 0)
        return -1;

    if ((status & 0x10) != 0)
        return 1;

    return -2;
}


#if OPT_DEBUG_POLL_IRQ
static int wait_phase(const struct rpivid_dev * const dev, const unsigned int phase)
{
    u32 mask = phase == 1 ? ARG_IC_ICTRL_ACTIVE1_INT_SET : ARG_IC_ICTRL_ACTIVE2_INT_SET;
    unsigned int i;
    u32 t;
    for (i = 0; i != 500; ++i) {
        if (((t = irq_read(dev , ARG_IC_ICTRL)) & mask) != 0) {
            irq_write(dev, ARG_IC_ICTRL, t & (~ARG_IC_ICTRL_ALL_IRQ_MASK | mask));
            v4l2_info(&dev->v4l2_dev, "%s[%d]: irq=%#x: OK\n", __func__, phase, t);
            return 0;
        }
        msleep(2);
    }
    v4l2_info(&dev->v4l2_dev, "%s[%d]: irq=%#x: TIMEOUT\n", __func__, phase, t);
    return -1;
}
#endif

static int poll_phase2(struct rpivid_ctx * const ctx);

static void cb_phase2(struct rpivid_dev * const dev, void * v)
{
    struct rpivid_ctx * const ctx = v;
//    v4l2_info(&dev->v4l2_dev, "Phase 2 done!\n");

    // If we get here then we are done successfully
    rpivid_hw_irq_active2_release(dev);

    if (ctx->dec2 == NULL) {
        v4l2_err(&dev->v4l2_dev, "%s: Dec env NULL\n", __func__);
        return;
    }

    dec_env_delete(ctx, ctx->dec2);
    ctx->dec2 = NULL;
    v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx,
                     VB2_BUF_STATE_DONE);

    // *** Check for errors
    poll_phase2(ctx);  // In the current world this will do nothing
    return;
}

static int do_phase2(struct rpivid_dev * const dev, dec_env_t * const de)
{
    unsigned int i;

    if (rpivid_hw_irq_active2_claim(dev, cb_phase2, de->ctx) != 0) {
        v4l2_err(&dev->v4l2_dev, "Failed to claim phase 2 h/w\n");
        return -1;
    }

    apb_write_vc_addr(dev, RPI_PURBASE, de->pu_base_vc);
    apb_write_vc_len(dev, RPI_PURSTRIDE, de->pu_stride);
    apb_write_vc_addr(dev, RPI_COEFFRBASE, de->coeff_base_vc);
    apb_write_vc_len(dev, RPI_COEFFRSTRIDE, de->coeff_stride);

    apb_write_vc_addr(dev, RPI_OUTYBASE, de->frame_addr);
    apb_write_vc_addr(dev, RPI_OUTCBASE, de->frame_addr + de->frame_c_offset);
    apb_write_vc_len(dev, RPI_OUTYSTRIDE, de->frame_stride);
    apb_write_vc_len(dev, RPI_OUTCSTRIDE, de->frame_stride);

//    v4l2_info(&dev->v4l2_dev, "Frame: Y=%llx, C=%llx, Stride=%x\n",
//              de->frame_addr, de->frame_addr + de->frame_c_offset, de->frame_stride);

    for(i=0; i<16; i++) {
        // Strides are in fact unused but fill in anyway
        apb_write_vc_addr(dev, 0x9000 + 16 * i, de->ref_addrs[i]);
        apb_write_vc_len(dev, 0x9004 + 16 * i, de->frame_stride);
        apb_write_vc_addr(dev, 0x9008 + 16 * i, de->ref_addrs[i] + de->frame_c_offset);
        apb_write_vc_len(dev, 0x900C + 16 * i, de->frame_stride);
    }

    apb_write(dev, RPI_CONFIG2,   de->rpi_config2);
    apb_write(dev, RPI_FRAMESIZE, de->rpi_framesize);
    apb_write(dev, RPI_CURRPOC,   de->rpi_currpoc);
//    v4l2_info(&dev->v4l2_dev, "Config2=%#x, FrameSize=%#x, POC=%#x\n", de->rpi_config2, de->rpi_framesize, de->rpi_currpoc);

    // collocated reads/writes
    apb_write_vc_len(dev, RPI_COLSTRIDE, de->ctx->colmv_stride);  // Read vals
    apb_write_vc_len(dev, RPI_MVSTRIDE,  de->ctx->colmv_stride);  // Write vals
    apb_write_vc_addr(dev, RPI_MVBASE,  de->frame_aux == NULL ? 0 : de->frame_aux->col.addr);
    apb_write_vc_addr(dev, RPI_COLBASE, de->col_aux == NULL ? 0 : de->col_aux->col.addr);

//        v4l2_info(&dev->v4l2_dev, "Mv=%llx, Col=%llx, Stride=%x, Buf=%llx->%llx\n",
//                  de->rpi_mvbase, de->rpi_colbase, de->ctx->colmv_stride,
//                  de->ctx->colmvbuf.addr, de->ctx->colmvbuf.addr + de->ctx->colmvbuf.size);

#if 0
    apb_dump_regs(rpi, 0x0, 32);
    apb_dump_regs(rpi, 0x8000, 24);
#endif

//    v4l2_info(&dev->v4l2_dev, "Phase 2 trigger!\n");

#if 1
    apb_write(dev, RPI_NUMROWS, de->PicHeightInCtbsY);
    apb_read(dev, RPI_NUMROWS); // Read back to confirm write has reached block
#else
    cb_phase2(dev, de->ctx);
#endif

#if OPT_DEBUG_POLL_IRQ
    if (wait_phase(dev, 2) != 0) {
        rpivid_hw_irq_active2_release(dev);
        return -1;
    }
    cb_phase2(dev, de->ctx);
#endif

    return 0;
}

// Returns:
//  0  OK - Phase 2 started
//  1  No ready phase 1
//  2  Phase 2 already active
// -1  Error (finish needed but NOT called)
//
// ?? May need device level Q (single entry) of pending phase 2 requests
//    as we may need to swap contexts.  Only need single entry as we should
//    do all the actual Qing pre phase1 ??
static int poll_phase2(struct rpivid_ctx * const ctx)
{
    dec_env_t * const de = ctx->dec1;
    int rv;

    if (de == NULL)
        return 1;
    if (ctx->dec2 != NULL)
        return 2;

    ctx->dec2 = de;
    ctx->dec1 = NULL;

    rv = do_phase2(ctx->dev, de);

    if (rv < 0) {
        ctx->dec2 = NULL;
    }
    return rv;
}

static void cb_phase1(struct rpivid_dev * const dev, void * v)
{
    struct rpivid_ctx * const ctx = v;
    int status;

    if (ctx->dec1 == NULL) {
        v4l2_err(&dev->v4l2_dev, "%s: Dec env NULL\n", __func__);
        return;
    }

    status = check_status(dev);
//	v4l2_info(&dev->v4l2_dev, "%s: Post wait: %d\n", __func__, status);

    if (status != 0)
    {
        v4l2_err(&dev->v4l2_dev, "%s: Post wait: %d\n", __func__, status);
        goto fail1;
    }
    rpivid_hw_irq_active1_release(dev);

    status = poll_phase2(ctx);
//    v4l2_info(&dev->v4l2_dev, "%s: Post poll: %d\n", __func__, status);
    // As it stands even "soft" fails are hard here (all unexpected)
    if (status != 0)
    {
        v4l2_err(&dev->v4l2_dev, "%s: Post poll: %d\n", __func__, status);
        goto fail0;
    }
    return;

fail1:
    rpivid_hw_irq_active1_release(dev);
fail0:
    dec_env_delete(ctx, ctx->dec1);
    ctx->dec1 = NULL;
    v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx,
                     VB2_BUF_STATE_ERROR);
    return;
}

static void do_phase1(struct rpivid_ctx * const ctx)
{
    // (Re-)allocate PU/COEFF stream space
    struct rpivid_dev * const dev = ctx->dev;
    const struct rpivid_gptr * const p2gptr = ctx->p2bufs + 0;
    unsigned int pu_size;
	dec_env_t * const de = ctx->dec0;

    ctx->dec0 = NULL;

    if (ctx->dec1 != NULL) {
        v4l2_err(&dev->v4l2_dev, "Phase 1 start before finished!\n");
        goto fail0;
    }

    if (rpivid_hw_irq_active1_claim(dev, cb_phase1, ctx) != 0) {
        v4l2_err(&dev->v4l2_dev, "Failed to claim phase 1 h/w\n");
        goto fail0;
    }

    // Move dec0->dec1
    // ?? Spinlock ??
    // ?? Q here for multi open scenario ??
    // ?? Make into cb_phase0 & move copies into here so we only need
    //    1 set of intermediates per device rather than per open
    //    ... maybe keep bit buffers per open - avoids need to fix up
    //    buffer addresses ??
    ctx->dec1 = de;

    de->pu_base_vc = p2gptr->addr;
    de->pu_stride = rnd64(ctx->max_pu_msgs * 2 * de->PicWidthInCtbsY);
    pu_size = de->pu_stride * de->PicHeightInCtbsY;

    // Allocate all remaining space to coeff
    de->coeff_base_vc = de->pu_base_vc + pu_size;
    de->coeff_stride = ((p2gptr->size - pu_size) / de->PicHeightInCtbsY) & ~63;  // Round down to multiple of 64

    apb_write_vc_addr(dev, RPI_PUWBASE, de->pu_base_vc);
    apb_write_vc_len(dev, RPI_PUWSTRIDE, de->pu_stride);
    apb_write_vc_addr(dev, RPI_COEFFWBASE, de->coeff_base_vc);
    apb_write_vc_len(dev, RPI_COEFFWSTRIDE, de->coeff_stride);

//	v4l2_info(&dev->v4l2_dev, "%s: Pre trigger\n", __func__);

    // Trigger command FIFO
    apb_write(dev, RPI_CFNUM, de->cmd_len);
#if 0
    apb_dump_regs(rpi, 0x0, 32);
    apb_dump_regs(rpi, 0x8000, 24);
    axi_dump(de, ((uint64_t)a64)<<6, de->cmd_len * sizeof(struct RPI_CMD));
#endif
    apb_write_vc_addr(dev, RPI_CFBASE, de->cmd_copy_gptr->addr);

#if OPT_DEBUG_POLL_IRQ
    if (wait_phase(dev, 1) != 0) {
        goto fail1;
    }
    cb_phase1(dev, de->ctx);
#endif
    return;

#if OPT_DEBUG_POLL_IRQ
fail1:
    rpivid_hw_irq_active1_release(dev);
    ctx->dec1 = NULL;
#endif
fail0:
    dec_env_delete(ctx, de);
    v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx,
                     VB2_BUF_STATE_ERROR);
}

static void dec_state_delete(struct rpivid_ctx * const ctx)
{
    unsigned int i;
    dec_state_t *const s = ctx->state;

    if (s == NULL)
        return;
    ctx->state = NULL;

    free_ps_info(s);

    for (i = 0; i != HEVC_MAX_REFS; ++i)
        aux_q_release(ctx, &s->ref_aux[i]);
    aux_q_release(ctx, &s->frame_aux);

    kfree(s);
}

static dec_state_t * dec_state_new(void)
{
    dec_state_t * s = kzalloc(sizeof(*s), GFP_KERNEL);
    if (s == NULL)
        return NULL;
    return s;
}

static void rpivid_h265_stop(struct rpivid_ctx *ctx)
{
	struct rpivid_dev * const dev = ctx->dev;
    unsigned int i;

	v4l2_info(&dev->v4l2_dev, "rpivid_h265_stop\n");

    dec_env_uninit(ctx);
	dec_state_delete(ctx);

    // dec_env & state must be killed before this to release the buffer to the free pool
    aux_q_uninit(ctx);

    for (i = 0; i != sizeof(ctx->bitbufs) / sizeof(ctx->bitbufs[0]); ++i)
    {
        struct rpivid_gptr * const gptr = ctx->bitbufs + i;

        if (gptr->ptr != NULL)
            dma_free_coherent(dev->dev, gptr->size, gptr->ptr, gptr->addr);
        gptr->ptr = NULL;
        gptr->addr = 0;
        gptr->size = 0;
    }
    for (i = 0; i != sizeof(ctx->cmdbufs) / sizeof(ctx->cmdbufs[0]); ++i)
    {
        struct rpivid_gptr * const gptr = ctx->cmdbufs + i;

        if (gptr->ptr != NULL)
            dma_free_coherent(dev->dev, gptr->size, gptr->ptr, gptr->addr);
        gptr->ptr = NULL;
        gptr->addr = 0;
        gptr->size = 0;
    }
    for (i = 0; i != sizeof(ctx->p2bufs) / sizeof(ctx->p2bufs[0]); ++i)
        gptr_free(dev, ctx->p2bufs + i);
}

#define RPIVID_BIT_BUF_SIZE (4 * 1024 * 1024)
#define RPIVID_CMD_BUF_SIZE (4 * 1024 * 1024)
#define RPIVID_P2_BUF_SIZE  (16 * 1024 * 1024)

static int rpivid_h265_start(struct rpivid_ctx *ctx)
{
	struct rpivid_dev * const dev = ctx->dev;
    unsigned int i;

	v4l2_info(&dev->v4l2_dev, "rpivid_h265_start\n");

	ctx->dec0 = NULL;
	ctx->dec1 = NULL;
	ctx->dec2 = NULL;
	if ((ctx->state = dec_state_new()) == NULL)
    {
        v4l2_err(&dev->v4l2_dev, "Failed to allocate decode state\n");
        goto fail;
    }

    if (dec_env_init(ctx) != 0)
    {
        v4l2_err(&dev->v4l2_dev, "Failed to allocate decode envs\n");
        goto fail;
    }

    ctx->max_pu_msgs = 768; // 7.2 says at most 1611 messages per CTU

    for (i = 0; i != sizeof(ctx->bitbufs) / sizeof(ctx->bitbufs[0]); ++i)
    {
        struct rpivid_gptr * const gptr = ctx->bitbufs + i;
        gptr->size = RPIVID_BIT_BUF_SIZE;
        gptr->ptr =
            dma_alloc_coherent(dev->dev, gptr->size,
                       &gptr->addr, GFP_KERNEL);
        if (gptr->ptr == NULL)
            goto fail;
    }
    for (i = 0; i != sizeof(ctx->cmdbufs) / sizeof(ctx->cmdbufs[0]); ++i)
    {
        struct rpivid_gptr * const gptr = ctx->cmdbufs + i;
        gptr->size = RPIVID_CMD_BUF_SIZE;
        gptr->ptr =
            dma_alloc_coherent(dev->dev, gptr->size,
                       &gptr->addr, GFP_KERNEL);
        if (gptr->ptr == NULL)
            goto fail;
    }
    for (i = 0; i != sizeof(ctx->p2bufs) / sizeof(ctx->p2bufs[0]); ++i)
    {
        // Don't actually need a kernel mapping here
        if (gptr_alloc(dev, ctx->p2bufs + i,
                       RPIVID_P2_BUF_SIZE,
                       DMA_ATTR_FORCE_CONTIGUOUS | DMA_ATTR_NO_KERNEL_MAPPING) != 0) {
            goto fail;
        }
    }
    aux_q_init(ctx);

	return 0;

fail:
    rpivid_h265_stop(ctx);
    return -ENOMEM;
}

static void rpivid_h265_trigger(struct rpivid_ctx *ctx)
{
	struct rpivid_dev * const dev = ctx->dev;
    dec_env_t *const de = ctx->dec0;

//	v4l2_info(&dev->v4l2_dev, "rpivid_h265_trigger start\n");

    switch (de == NULL ? RPIVID_DECODE_ERROR_CONTINUE : de->state) {
        case RPIVID_DECODE_SLICE_START:
            de->state = RPIVID_DECODE_SLICE_CONTINUE;
            /* FALLTHRU */
        case RPIVID_DECODE_SLICE_CONTINUE:
            v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx,
                             VB2_BUF_STATE_DONE);
            break;
        default:
            v4l2_err(&dev->v4l2_dev, "%s: Unexpected state: %d\n", __func__, de->state);
            /* FALLTHRU */
        case RPIVID_DECODE_ERROR_DONE:
            ctx->dec0 = NULL;
            dec_env_delete(ctx, de);
            /* FALLTHRU */
        case RPIVID_DECODE_ERROR_CONTINUE:
            v4l2_m2m_buf_done_and_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx,
                             VB2_BUF_STATE_ERROR);
            break;
        case RPIVID_DECODE_PHASE1:
#warning This should really be a poll
            do_phase1(ctx);
            break;
    }
    return;
}

struct rpivid_dec_ops rpivid_dec_ops_h265 = {
	.setup		= rpivid_h265_setup,
	.start		= rpivid_h265_start,
	.stop		= rpivid_h265_stop,
	.trigger	= rpivid_h265_trigger,
};
