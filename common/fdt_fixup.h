/*
 * Copyright (c) 2019, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef FDT_FIXUP_H
#define FDT_FIXUP_H

int dt_add_psci_node(void *fdt);
int dt_add_psci_cpu_enable_methods(void *fdt);
int fdt_add_reserved_memory_ex(void *dtb, const char *node_name, const char* info,
			    uintptr_t base, size_t size);
int fdt_add_reserved_memory(void *dtb, const char *node_name,
			    uintptr_t base, size_t size);
int fdt_set_usable_memory(void *dtb, const char* mem_path, uintptr_t start[], uintptr_t size[], uint8_t nr_areas);
#endif /* FDT_FIXUP_H */
