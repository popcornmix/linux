/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_NUMA_H
#define __ASM_NUMA_H

#include <asm/topology.h>
#include <asm-generic/numa.h>

#ifdef CONFIG_OF_NUMA
extern int arm64_fake_numa_init(void);
#else
static inline int arm64_fake_numa_init(void)
{
	return -ENOSYS;
}
#endif

#endif	/* __ASM_NUMA_H */
