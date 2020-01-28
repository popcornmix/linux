/* SPDX-License-Identifier: GPL-2.0 */

#ifndef _RPIVID_HW_H_
#define _RPIVID_HW_H_


static inline void apb_write(const struct rpivid_dev * const dev, const unsigned int offset, const __u32 val)
{
    *(volatile __u32 *)((char *)dev->base_h265 + offset) = val;
}

static inline __u32 apb_read(const struct rpivid_dev * const dev, const unsigned int offset)
{
    return *(volatile __u32 *)((char *)dev->base_h265 + offset);
}

static inline void irq_write(const struct rpivid_dev * const dev, const unsigned int offset, const __u32 val)
{
    *(volatile __u32 *)((char *)dev->base_irq + offset) = val;
}

static inline __u32 irq_read(const struct rpivid_dev * const dev, const unsigned int offset)
{
    return *(volatile __u32 *)((char *)dev->base_irq + offset);
}

static inline void apb_write_vc_addr(const struct rpivid_dev * const dev, const unsigned int offset, const dma_addr_t a)
{
    apb_write(dev, offset, (__u32)(a >> 6));
}

static inline void apb_write_vc_len(const struct rpivid_dev * const dev, const unsigned int offset, const unsigned int x)
{
    apb_write(dev, offset, (x + 63) >> 6);
}


/* *ARG_IC_ICTRL - Interupt control for ARGON Core*
 * Offset (byte space) = 40'h2b10000
 * Physical Address (byte space) = 40'h7eb10000
 * Verilog Macro Address = `ARG_IC_REG_START + `ARGON_INTCTRL_ICTRL
 * Reset Value = 32'b100x100x_100xxxxx_xxxxxxx0_x100x100
 * Access = RW (32-bit only)
 * Interrupt control logic for ARGON Core.
*/
#define ARG_IC_ICTRL 0

/* acc=LWC ACTIVE1_INT FIELD ACCESS: LWC
 *
 * Interrupt 1
 * This is set and held when an hevc_active1 interrupt edge is detected
 * The polarity of the edge is set by the ACTIVE1_EDGE field
 * Write a 1 to this bit to clear down the latched interrupt
 * The latched interrupt is only enabled out onto the interrupt line if
 * ACTIVE1_EN is set
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_ACTIVE1_INT_SET                   0x00000001

/* ACTIVE1_EDGE Sets the polarity of the interrupt edge detection logic
 * This logic detects edges of the hevc_active1 line from the argon core
 * 0 = negedge, 1 = posedge
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_ACTIVE1_EDGE_SET                  0x00000002

/* ACTIVE1_EN Enables ACTIVE1_INT out onto the argon interrupt line.
 * If this isnt set, the interrupt logic will work but no interrupt will be
 * set to the interrupt controler
 * Reset value is *1* decimal.
 *
 * [JC] The above appears to be a lie - if unset then b0 is never set
*/
#define ARG_IC_ICTRL_ACTIVE1_EN_SET                    0x00000004

/* acc=RO ACTIVE1_STATUS FIELD ACCESS: RO
 *
 * The current status of the hevc_active1 signal
*/
#define ARG_IC_ICTRL_ACTIVE1_STATUS_SET                0x00000008

/* acc=LWC ACTIVE2_INT FIELD ACCESS: LWC
 *
 * Interrupt 2
 * This is set and held when an hevc_active2 interrupt edge is detected
 * The polarity of the edge is set by the ACTIVE2_EDGE field
 * Write a 1 to this bit to clear down the latched interrupt
 * The latched interrupt is only enabled out onto the interrupt line if
 * ACTIVE2_EN is set
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_ACTIVE2_INT_SET                   0x00000010

/* ACTIVE2_EDGE Sets the polarity of the interrupt edge detection logic
 * This logic detects edges of the hevc_active2 line from the argon core
 * 0 = negedge, 1 = posedge
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_ACTIVE2_EDGE_SET                  0x00000020

/* ACTIVE2_EN Enables ACTIVE2_INT out onto the argon interrupt line.
 * If this isnt set, the interrupt logic will work but no interrupt will be
 * set to the interrupt controler
 * Reset value is *1* decimal.
*/
#define ARG_IC_ICTRL_ACTIVE2_EN_SET                    0x00000040

/* acc=RO ACTIVE2_STATUS FIELD ACCESS: RO
 *
 * The current status of the hevc_active2 signal
*/
#define ARG_IC_ICTRL_ACTIVE2_STATUS_SET                0x00000080

/* TEST_INT Forces the argon int high for test purposes.
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_TEST_INT                          (1U << 8)
#define ARG_IC_ICTRL_SPARE                             (1U << 9)

/* acc=RO VP9_INTERRUPT_STATUS FIELD ACCESS: RO
 *
 * The current status of the vp9_interrupt signal
*/
#define ARG_IC_ICTRL_VP9_INTERRUPT_STATUS              (1U << 10)

/* AIO_INT_ENABLE 1 = Or the AIO int in with the Argon int so the VPU can see
 * it
 * 0 = the AIO int is masked. (It should still be connected to the GIC though).
*/
#define ARG_IC_ICTRL_AIO_INT_ENABLE                    (1U << 20)
#define ARG_IC_ICTRL_H264_ACTIVE_INT                   (1U << 21)
#define ARG_IC_ICTRL_H264_ACTIVE_EDGE                  (1U << 22)
#define ARG_IC_ICTRL_H264_ACTIVE_EN                    (1U << 23)
#define ARG_IC_ICTRL_H264_ACTIVE_STATUS                (1U << 24)
#define ARG_IC_ICTRL_H264_INTERRUPT_INT                (1U << 25)
#define ARG_IC_ICTRL_H264_INTERRUPT_EDGE               (1U << 26)
#define ARG_IC_ICTRL_H264_INTERRUPT_EN                 (1U << 27)

/* acc=RO H264_INTERRUPT_STATUS FIELD ACCESS: RO
 *
 * The current status of the h264_interrupt signal
*/
#define ARG_IC_ICTRL_H264_INTERRUPT_STATUS             (1U << 28)

/* acc=LWC VP9_INTERRUPT_INT FIELD ACCESS: LWC
 *
 * Interrupt 1
 * This is set and held when an vp9_interrupt interrupt edge is detected
 * The polarity of the edge is set by the VP9_INTERRUPT_EDGE field
 * Write a 1 to this bit to clear down the latched interrupt
 * The latched interrupt is only enabled out onto the interrupt line if
 * VP9_INTERRUPT_EN is set
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_VP9_INTERRUPT_INT                 (1U << 29)

/* VP9_INTERRUPT_EDGE Sets the polarity of the interrupt edge detection logic
 * This logic detects edges of the vp9_interrupt line from the argon h264 core
 * 0 = negedge, 1 = posedge
 * Reset value is *0* decimal.
*/
#define ARG_IC_ICTRL_VP9_INTERRUPT_EDGE                (1U << 30)

/* VP9_INTERRUPT_EN Enables VP9_INTERRUPT_INT out onto the argon interrupt line.
 * If this isnt set, the interrupt logic will work but no interrupt will be
 * set to the interrupt controler
 * Reset value is *1* decimal.
*/
#define ARG_IC_ICTRL_VP9_INTERRUPT_EN                  (1U << 31)

// Bits 19:12, 11 reserved - read ?, write 0
#define ARG_IC_ICTRL_SET_ZERO_MASK  ((0xff << 12) | (1 << 11))

/* All IRQ bits */
#define ARG_IC_ICTRL_ALL_IRQ_MASK   (\
    ARG_IC_ICTRL_VP9_INTERRUPT_INT  |\
    ARG_IC_ICTRL_H264_INTERRUPT_INT |\
    ARG_IC_ICTRL_ACTIVE1_INT_SET    |\
    ARG_IC_ICTRL_ACTIVE2_INT_SET)

int rpivid_hw_irq_active1_claim(struct rpivid_dev *dev, rpivid_irq_callback cb, void * ctx);
void rpivid_hw_irq_active1_release(struct rpivid_dev *dev);
int rpivid_hw_irq_active2_claim(struct rpivid_dev *dev, rpivid_irq_callback cb, void * ctx);
void rpivid_hw_irq_active2_release(struct rpivid_dev *dev);
int rpivid_hw_probe(struct rpivid_dev * dev);
void rpivid_hw_remove(struct rpivid_dev * dev);

#endif
