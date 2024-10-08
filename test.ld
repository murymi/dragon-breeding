ENTRY(_start)

/* Changes to KERNEL_ADDR_OFFSET must also be made to KERNEL_PAGE_NUMBER in boot.zig */
KERNEL_ADDR_OFFSET = 0xC0000000;

SECTIONS {
	. = KERNEL_ADDR_OFFSET;

	.text ALIGN(4K) : {
		*(.text)
	}

	.rodata ALIGN(4K) : {
		*(.rodata)
	}

	.data ALIGN(4K) : {
		*(.data)
	}

	.bss ALIGN(4K) : {
		*(COMMON)
		*(.bss)
	}
}
