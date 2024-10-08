const std = @import("std");
const vga = @import("vga.zig");
const interrupts = @import("interrupts.zig");
const gdt = @import("gdt.zig");
const multiboot = @import("multiboot.zig");
const shed = @import("sched.zig");
const debug = @import("debug.zig");

pub const panic = debug.panic;


const sys = @import("sys/registers.zig");

extern const _kernel_start: u8;
extern const _kernel_end: u8;

const mem = @import("mem.zig");
const paging = @import("paging.zig");

pub var physical_layout: mem.PhysicalLayout linksection(".bss") = undefined;

extern const doggy: u32;

extern const __kernel_stack_stop: u32;
extern const __kernel_data_stop: u32;
extern const __kernel_bss_stop: u32;
extern const __kernel_rodata_stop: u32;
extern const __kernel_text_stop: u32;

extern const __kernel_stack_start: u32;
extern const __kernel_data_start: u32;
extern const __kernel_bss_start: u32;
extern const __kernel_rodata_start: u32;
extern const __kernel_text_start: u32;


fn printOffsets() void {
        vga.print(
        \\text: 0x{x} -> 0x{x}  size: {}
        \\rodata: 0x{x} -> 0x{x}    size : {}   
        \\data: 0x{x} -> 0x{x}      size : {}   
        \\stack: 0x{x} -> 0x{x}     size : {}   
        \\bss: 0x{x} -> 0x{x}       size : {}   
        \\end: {} -> {}       size : {}   
        \\
    , .{
    &__kernel_text_start,    &__kernel_text_stop, 
    @intFromPtr(&__kernel_text_stop) - @intFromPtr(&__kernel_text_start),
    &__kernel_rodata_start,    &__kernel_rodata_stop,
    @intFromPtr(&__kernel_rodata_stop) - @intFromPtr(&__kernel_rodata_start),
    &__kernel_data_start,    &__kernel_data_stop, 
    @intFromPtr(&__kernel_data_stop) - @intFromPtr(&__kernel_data_start),

    &__kernel_stack_start, &__kernel_stack_stop,
    @intFromPtr(&__kernel_stack_stop) - @intFromPtr(&__kernel_stack_start),
    &__kernel_bss_start, &__kernel_bss_stop,
    @intFromPtr(&__kernel_bss_stop) - @intFromPtr(&__kernel_bss_start),
    @intFromPtr(&__kernel_higher_half_start),    @intFromPtr(&__kernel_higher_half_stop),
    @intFromPtr(&__kernel_higher_half_stop) - @intFromPtr(&__kernel_higher_half_start)

    //@intFromPtr(&kernel_page_dir.bitset),
    //@intFromPtr(&kernel_page_dir.pd),
    //@intFromPtr(&kernel_page_dir.pts),
    //@intFromPtr(&kernel_page_dir.phys),



    });


    vga.print("\nbitset: {}\npagedir: {}\npage_table: {}\nphys: {}\nself: {}\n", .{
        @intFromPtr(kernel_page_dir.bitset),
        @intFromPtr(&kernel_page_dir.pd),
        @intFromPtr(&kernel_page_dir.pts),
        @intFromPtr(&kernel_page_dir.phys),
        @intFromPtr(&kernel_page_dir),
        //@intFromPtr(void_buffer[void_buffer.len-1..].ptr),
        //void_buffer[void_buffer.len-1]
    });
}


var kernel_page_dir: paging.PageDirectory linksection(".bss") = undefined;

//var void_buffer: [1024 * 1024]u8 = undefined;

fn kmain(ptr: *multiboot.multiboot_info_t) !void {
    gdt.init();


    physical_layout.init(ptr);
    
    vga.print("size: {}\n", .{@sizeOf(@TypeOf(kernel_page_dir.bitset))});
    //const pa = try physical_layout.allocPage();

    //vga.print("{}\n", .{@intFromPtr(pa.ptr) % 4096});

    //try physical_layout.freePage(pa);
    kernel_page_dir.init(
        &physical_layout
    );
//0x1268c0
    const vga_buffer = try physical_layout.allocPageSpecific(0xb8000);
    vga.init(try kernel_page_dir.mapRandom(@intFromPtr(vga_buffer)));
    interrupts.initIdt();

    //var k = try fixed_heap.allocator().alloc(u8, 1);
    //k[0] = 6;

    //kernel_page_dir.copyPageSystem(&proco.page_directory, &proco.page_tables);
    //try proco.init(0, &kernel_page_dir, @intFromPtr(&__kernel_higher_half_stop));
    //mapKernel(@intFromPtr(&__kernel_higher_half_stop));

    //var cr3 = sys.Cr3.read();
    //cr3.setPageDirectoryBase(@ptrFromInt(@intFromPtr(&proco.page_directory) - 0xC0000000));
    //cr3.write();

    //vga.print("0x{x}\n",.{&__higher_half_start});
    //vga.clearScreen(.nightGreen);

    //for(0..100) |u| {
    //    vga.print("{}\n", .{u});
    //}
    var a: usize = 0;
    for(kernel_page_dir.bitset.bytes) |_| {
        //a += b;
    }
    vga.print("phys bitset: {}\n", .{physical_layout.bitset.addressable});

    a = 0;
    for(physical_layout.bitset.bytes) |b| {
        a += b;
    }

    //printOffsets();


//const add:usize = 0xC0100000;
//const idx = add/4096;
    //const is = try kernel_page_dir.bitset.isSet(idx);
    //_ = is;

    //mem.phys_bitset.addressable = 8;

    vga.print("set: {}\n", .{mem.phys_bitset.addressable});

//sys.spinloop();

    //const p = try physical_layout.allocPageBug();
    //kernel_page_dir.mapVirtualAddress(add);
    //_ = p;
    //p.*[0] = 'A';

   //try kernel_page_dir.unmap(@intFromPtr(p), false);
   //try kernel_page_dir.unmap(@intFromPtr(p), false);

    //try kernel_page_dir.m


    //const any = try kernel_page_dir.mapRandom(0xb8000);
    //any.*[0] = 'J';

    //const any2 = try kernel_page_dir.mapAnyFree();
    //any2.slice[0] = 'X';
    //try kernel_page_dir.phys.bitset.set(0);

    while (true) {
        sys.spinloop();
    }
}

fn t() u32 {
    return 0;
}

fn switchToUser() void {
    asm volatile (
        \\ cli
        \\ mov $0x23, %ax
        \\mov %ax, %ds
        \\ mov %ax, %es
        \\ mov %ax, %fs
        \\ mov %ax, %gs
        \\
        \\ mov %esp, %eax
        \\ pushl $0x23
        \\ pushl %eax
        \\ pushf
        \\ pop %eax
        \\ orl $0x200, %eax
        \\ push %eax
        \\ pushl $0x1B
        \\ push $1f
        \\ iret
        \\ 1:
    );
}


pub export var kernel_stack: [16384]u8 align(16) linksection(".bss.stack") = undefined;
const stack_top = kernel_stack[kernel_stack.len..].ptr;


pub export fn _start2(bptr: *multiboot.multiboot_info_t) noreturn {
    //while (true) {}

    asm volatile (
        \\mov %[a], %%esp
        \\mov %%esp, %%ebp
        :
        : [a] "{eax}" (stack_top),
    );

    // kmain can return through error propagation 
    kmain(bptr) catch |e|{
        //@panic(@errorName(e));
        panic(@errorName(e), null, null);
        //sys.spinloop();
    };

    while (true) {
        asm volatile ("cli");
        asm volatile ("hlt");
    }
}



//===========================================================================================
const alignment: i32 = 1 << 0;
const mem_info: i32 = 1 << 1;
const flags = alignment | mem_info;
const magic: i32 = 0x1badb002;
const checksum: i32 = -(magic + flags);

export var magic_stuff: i32 align(4) linksection(".rodata.multiboot.magic") = magic;
export var flags_stuff: i32 align(4) linksection(".rodata.multiboot.flags") = flags;
export var checksum_stuff: i32 align(4) linksection(".rodata.multiboot.checksum") = checksum;


pub export var boot_stack: [16384]u8 align(16) linksection(".multiboot.bss.boot") = undefined;
const boot_stack_top = boot_stack[boot_stack.len..].ptr;

extern const __kernel_stop: usize;
extern const __kernel_higher_half_position: usize;
extern const __higher_half_start: usize;
extern const __kernel_start: usize;
extern const __kernel_higher_half_start: usize;
extern const __kernel_higher_half_stop: usize;


extern const kain: usize;

var boot_page_directory: sys.PageDirectory align(4096) linksection(".multiboot.rodata.boot") = blk: {
    @setEvalBranchQuota(10000);
    var pd: sys.PageDirectory = undefined;

    for (&pd) |*entry| {
        entry.* = @bitCast(@as(usize, 0));
        //std.mem.zeroes(sys.PageDirectoryEntry4K);
    }
    break :blk pd;
};

pub export fn initBootPage() linksection(".multiboot.text.boot") void {
    const bptr = asm (
        \\mov %%ebx, %[res]
        : [res] "=r" (-> *multiboot.multiboot_info_t),
    );

    //vga.clearScreen(.brown);

    const page_size = 1024 * 1024 * 4;
    const kernel_pages:usize = alignTo(@intFromPtr(&__kernel_stop), page_size)/page_size;

    for (0..kernel_pages) |i| {
        var identity_entry = sys.PageDirectoryEntry4M{};
        const int: usize = @bitCast(identity_entry);
        identity_entry = @bitCast(int | (i * page_size));
        boot_page_directory[i] = sys.PageDirectoryEntry{ .@"4M" = identity_entry };
    }

    const higher_half: sys.LinearAddress4M = @bitCast(@intFromPtr(&__higher_half_start));

    for (0..kernel_pages) |i| {
        var identity_entry = sys.PageDirectoryEntry4M{
            .page_base_address = 0
        };
        const int: usize = @bitCast(identity_entry);
        identity_entry = @bitCast(int | ((i * page_size)));
        boot_page_directory[i + higher_half.page_directory_entry] = sys.PageDirectoryEntry{ .@"4M" = identity_entry };
    }

    var cr4 = readCr4();
    cr4.page_size_extensions = true;
    writeCr4(cr4);

    var cr3 = readCr3();
    const int: usize = @bitCast(cr3);
    cr3 = @bitCast(int | @intFromPtr(&boot_page_directory));
    writeCr3(cr3);

    var cr0 = readCr0();
    cr0.paging = true;
    writeCr0(cr0);

    //while (true) {}

    //asm volatile ("jmp _start2");

    _start2(bptr);

}

inline fn alignTo(n:usize, alig: usize) linksection(".multiboot.text.boot") usize {
    return (n + alig - 1) & ~(alig - 1);
}

inline fn readCr0() linksection(".multiboot.text.boot") sys.Cr0 {
    return @bitCast(asm volatile (
        \\ mov %%cr0, %%eax
        : [res] "={eax}" (-> u32),
    ));
}

inline fn writeCr0(cro: sys.Cr0) linksection(".multiboot.text.boot") void {
    const raw: u32 = @bitCast(cro);
    asm volatile (
        \\ mov %[a], %%cr0
        :
        : [a] "{eax}" (raw),
    );
}

inline fn readCr3() linksection(".multiboot.text.boot") sys.Cr3 {
    return @bitCast(asm volatile (
        \\ mov %%cr3, %%eax
        : [res] "={eax}" (-> u32),
    ));
}

inline fn writeCr3(cro: sys.Cr3) linksection(".multiboot.text.boot") void {
    const raw: u32 = @bitCast(cro);
    asm volatile (
        \\ mov %[a], %%cr3
        :
        : [a] "{eax}" (raw),
    );
}

inline fn readCr4() linksection(".multiboot.text.boot") sys.Cr4 {
    return @bitCast(asm volatile (
        \\ mov %%cr4, %%eax
        : [res] "={eax}" (-> u32),
    ));
}

inline fn writeCr4(cro: sys.Cr4) linksection(".multiboot.text.boot") void {
    const raw: u32 = @bitCast(cro);
    asm volatile (
        \\ mov %[a], %%cr4
        :
        : [a] "{eax}" (raw),
    );
}

pub export fn _start() linksection(".multiboot.text.boot") callconv(.Naked) noreturn {
    asm volatile (
        \\mov %[a], %%esp
        \\mov %%esp, %%ebp
        :
        : [a] "{eax}" (boot_stack_top),
    );

    asm volatile ("jmp initBootPage");

    //asm volatile("jmp _start2");

    while (true) {
        asm volatile ("cli");
        asm volatile ("hlt");
    }
}
