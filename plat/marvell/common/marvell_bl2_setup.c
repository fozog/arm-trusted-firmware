/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <assert.h>
#include <string.h>

#include <platform_def.h>

#include <arch_helpers.h>
#include <common/bl_common.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#include <drivers/console.h>
#include <lib/utils.h>

#include <common/fdt_fixup.h>
#include <libfdt.h>

#include <marvell_def.h>
#include <plat_marvell.h>

/* Data structure which holds the extents of the trusted SRAM for BL2 */
static meminfo_t bl2_tzram_layout __aligned(CACHE_WRITEBACK_GRANULE);

/* Weak definitions may be overridden in specific MARVELL standard platform */
#pragma weak bl2_early_platform_setup2
#pragma weak bl2_platform_setup
#pragma weak bl2_plat_arch_setup
#pragma weak bl2_plat_sec_mem_layout

meminfo_t *bl2_plat_sec_mem_layout(void)
{
	return &bl2_tzram_layout;
}

/*****************************************************************************
 * BL1 has passed the extents of the trusted SRAM that should be visible to BL2
 * in x0. This memory layout is sitting at the base of the free trusted SRAM.
 * Copy it to a safe location before its reclaimed by later BL2 functionality.
 *****************************************************************************
 */
void marvell_bl2_early_platform_setup(meminfo_t *mem_layout)
{
	/* Initialize the console to provide early debug support */
	marvell_console_boot_init();

	/* Setup the BL2 memory layout */
	bl2_tzram_layout = *mem_layout;

	/* Initialise the IO layer and register platform IO devices */
	plat_marvell_io_setup();
}


void bl2_early_platform_setup2(u_register_t arg0, u_register_t arg1,
			       u_register_t arg2, u_register_t arg3)
{
	struct meminfo *mem_layout = (struct meminfo *)arg1;

	marvell_bl2_early_platform_setup(mem_layout);
}

void bl2_platform_setup(void)
{
	/* Nothing to do */
}

/*****************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only initializes the mmu in a quick and dirty way.
 *****************************************************************************
 */
void marvell_bl2_plat_arch_setup(void)
{
	marvell_setup_page_tables(bl2_tzram_layout.total_base,
				  bl2_tzram_layout.total_size,
				  BL_CODE_BASE,
				  BL_CODE_END,
				  BL_RO_DATA_BASE,
				  BL_RO_DATA_END
#if USE_COHERENT_MEM
				, BL_COHERENT_RAM_BASE,
				  BL_COHERENT_RAM_END
#endif
			      );
	enable_mmu_el1(0);
}

void bl2_plat_arch_setup(void)
{
	marvell_bl2_plat_arch_setup();
}

static uint32_t dtb_size(const void *dtb)
{
	const uint32_t *dtb_header = dtb;

	return fdt32_to_cpu(dtb_header[1]);
}

static void marvell_prepare_dtb(void)
{
	int ret;
	void *dtb = (void *)MARVELL_BL33_DTB;
	if (fdt_check_header(dtb) != 0)
		return;
	ret = fdt_open_into(dtb, dtb, MARVELL_BL33_DTB_SIZE);
	if (ret < 0) {
		ERROR("Invalid Device Tree at %p: error %d\n", dtb, ret);
		return;
	}
	if (dt_add_psci_node(dtb)) {
		ERROR("Failed to add PSCI Device Tree node\n");
		return;
	}

	if (dt_add_psci_cpu_enable_methods(dtb)) {
		ERROR("Failed to add PSCI cpu enable methods in Device Tree\n");
		return;
	}

	uintptr_t address_space[2];
	uintptr_t address_space_size[2];
	address_space[0]=0;
	address_space_size[0]=0xc0000000; /* 3GB */
	address_space[1]=0x100000000; /* 4GB */
	address_space_size[1]=0x40000000; /* 1GB */
	fdt_set_usable_memory(dtb, "/memory@0", address_space, address_space_size, 2);

	/* Reserve memory used by Trusted Firmware. */
	if (fdt_add_reserved_memory_ex(dtb, "bl33_dtb",NULL,
		MARVELL_BL33_DTB, MARVELL_BL33_DTB_SIZE))
		WARN("Failed to add reserved memory nodes to DT.\n");
/*	if (fdt_add_reserved_memory(dtb, "securemem", 0x4000000, 0x200000))
		WARN("Failed to add reserved memory nodes to DT.\n"); */

	ret = fdt_pack(dtb);
	if (ret < 0)
		ERROR("Failed to pack Device Tree at %p: error %d\n", dtb, ret);

	clean_dcache_range((uintptr_t)dtb, dtb_size(dtb));
	INFO("Changed device tree to advertise PSCI.\n");

}

int marvell_bl2_handle_post_image_load(unsigned int image_id)
{
	int err = 0;
	bl_mem_params_node_t *bl_mem_params = get_bl_mem_params_node(image_id);

	assert(bl_mem_params);

	switch (image_id) {

	case BL33_IMAGE_ID:
#if ARM_LINUX_KERNEL_AS_BL33
        /*
         * According to the file ``Documentation/arm64/booting.txt`` of
         * the Linux kernel tree, Linux expects the physical address of
         * the device tree blob (DTB) in x0, while x1-x3 are reserved
         * for future use and must be 0.
         */
        bl_mem_params->ep_info.args.arg0 = (u_register_t)MARVELL_BL33_DTB;
        bl_mem_params->ep_info.args.arg1 = 0U;
        bl_mem_params->ep_info.args.arg2 = 0U;
        bl_mem_params->ep_info.args.arg3 = 0U;
		bl_mem_params->ep_info.spsr = marvell_get_spsr_for_bl33_entry();
		/* shall we check Image format ? */
#else

		/* BL33 expects to receive the primary CPU MPID (through r0) */
		bl_mem_params->ep_info.args.arg0 = 0xffff & read_mpidr();
		bl_mem_params->ep_info.spsr = marvell_get_spsr_for_bl33_entry();
#endif
		break;
#ifdef SCP_BL2_BASE
	case SCP_BL2_IMAGE_ID:
		/* The subsequent handling of SCP_BL2 is platform specific */
		err = bl2_plat_handle_scp_bl2(&bl_mem_params->image_info);
		if (err) {
			WARN("Failure in platform-specific handling of SCP_BL2 image.\n");
		}
		break;
#endif
#ifdef ARM_LINUX_KERNEL_AS_BL33
	case NT_FW_CONFIG_ID:
			marvell_prepare_dtb();
		break;
#endif
	default:
		/* Do nothing in default case */
		break;
	}

	return err;

}

/*******************************************************************************
 * This function can be used by the platforms to update/use image
 * information for given `image_id`.
 ******************************************************************************/
int bl2_plat_handle_post_image_load(unsigned int image_id)
{
	return marvell_bl2_handle_post_image_load(image_id);
}

