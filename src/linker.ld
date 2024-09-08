ENTRY(_start)

KERNEL_ADDR_OFFSET = 0xC0000000;
 
SECTIONS {
    . = 1M;
    _kernel_start = .;

    .rodata.multiboot : {
		*(.rodata.multiboot.magic)
		*(.rodata.multiboot.flags)
		*(.rodata.multiboot.checksum)
	}

    .data.multiboot : {
        *(.data.multiboot)
    }

	.text.boot : {
		*(.text.boot)
	}

    . += 0xc0000000;
    .text ALIGN (4K) : AT (ADDR (.text) - 0xc0000000) {
        *(.text)
    }
 
    .rodata ALIGN(4K) : AT (ADDR (.rodata) - 0xc0000000){
        *(.rodata)
    }
 
    .data ALIGN(4K) : AT (ADDR (.data) - 0xc0000000) {
        *(.data)
    }
 
    .bss ALIGN(4K) : AT (ADDR (.bss) - 0xc0000000) {
        *(COMMON)
        *(.bss)
    }


    .bss.stack ALIGN(4K) : AT (ADDR(.bss.stack) - 0xc0000000) {
		KERNEL_STACK_START = .;
		KEEP(*(.bss.stack))
		KERNEL_STACK_END = .;
	}

    _kernel_end = .;
}

