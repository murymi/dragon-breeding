
const std = @import("std");
const vga = @import("vga.zig");
const interrupts = @import("interrupts.zig");
const gdt = @import("gdt.zig");
const multiboot = @import("multiboot.zig");

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    vga.print("Panic: {s}\n", .{message});
    while (true) {
    }

}


export fn kmain(ptr: usize)  noreturn {
    //gdt.init();
    //interrupts.initIdt();

    //vga.clearScreen(.nightBlue);

    const mptr =@as(*multiboot.multiboot_info_t, @ptrFromInt(ptr));


    vga.print("Halt cpu: {}\n", .{mptr.*});

    //asm volatile("cli");

    while (true) {
        asm volatile("cli");
        asm volatile("hlt");
    }
}

const alignment: i32 = 1 << 0;
const mem_info: i32 = 1 << 1;
const flags = alignment | mem_info;
const magic: i32 = 0x1badb002;
const checksum: i32 = -(magic + flags);

export var magic_stuff: i32 align(4) linksection(".rodata.multiboot.magic") = magic;
export var flags_stuff: i32 align(4) linksection(".rodata.multiboot.flags") = flags;
export var checksum_stuff: i32 align(4) linksection(".rodata.multiboot.checksum") = checksum;

export var kernel_stack: [16384]u8 align(16) linksection(".bss.stack") = undefined; 
const stack_top = kernel_stack[kernel_stack.len..].ptr;

pub export fn _start2() noreturn {
    asm volatile ("invlpg (0)");


    asm volatile (
        \\mov %[a], %%esp
        \\mov %%esp, %%ebp
        : :[a] "{eax}" (stack_top) 
    );

    vga.clearScreen(.nightBlue);
    gdt.init();

    const bptr = asm (
        \\mov %%ebx, %[res]
        : [res] "=r" (-> usize),
    ) + 0xC0000000;
    //@intFromPtr(&KERNEL_ADDR_OFFSET);

    kmain(bptr);

    //asm volatile("jmp kmain");
    //vga.print("TOP: 0x{x}\n", .{@intFromPtr(stack_top)});

    while (true) {
        asm volatile("cli");
        asm volatile("hlt");
    }
}



const KERNEL_PAGE_NUMBER = 0xC0000000 >> 22;
// The number of pages occupied by the kernel, will need to be increased as we add a heap etc.
const KERNEL_NUM_PAGES = 1;


export var boot_page_directory:[1024]u32 align(4096) linksection(".data.multiboot") = init: {
    // Increase max number of branches done by comptime evaluator
    @setEvalBranchQuota(1024);
    // Temp value
    var dir: [1024]u32 = undefined;

    // Page for 0 -> 4 MiB. Gets unmapped later
    dir[0] = 0x00000083;

    var i = 0;
    var idx = 1;

    // Fill preceding pages with zeroes. May be unnecessary but incurs no runtime cost
    while (i < KERNEL_PAGE_NUMBER-1) : ({
        i += 1;
        idx += 1;
    }) {
        dir[idx] = 0;
    }

    // Map the kernel's higher half pages increasing by 4 MiB every time
    i = 0;
    while (i < KERNEL_NUM_PAGES) : ({
        i += 1;
        idx += 1;
    }) {
        dir[idx] = 0x00000083 | (i << 22);
    }
    // Fill succeeding pages with zeroes. May be unnecessary but incurs no runtime cost
    i = 0;
    while (i < 1024 - KERNEL_PAGE_NUMBER - KERNEL_NUM_PAGES) : ({
        i += 1;
        idx += 1;
    }) {
        dir[idx] = 0;
    }
    break :init dir;
};
//export var boot_page_table1:[1024]u32 align(4096) linksection(".data.multiboot") = [1]u32{0} ** 1024;



pub export fn _start() linksection(".text.boot") callconv(.Naked) noreturn {
    //asm volatile ();
        // Set the page directory to the boot directory
    asm volatile (
        \\.extern boot_page_directory
        \\mov $boot_page_directory, %%ecx
        \\mov %%ecx, %%cr3
    );

    // Enable 4 MiB pages
    asm volatile (
        \\mov %%cr4, %%ecx
        \\or $0x00000010, %%ecx
        \\mov %%ecx, %%cr4
    );

    //pse

    // Enable paging
    asm volatile (
        \\mov %%cr0, %%ecx
        \\or $0x80000000, %%ecx
        \\mov %%ecx, %%cr0
    );

    asm volatile("jmp _start2");

    while (true) {
        asm volatile("cli");
        asm volatile("hlt");
    }

}



