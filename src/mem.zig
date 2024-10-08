const multiboot = @import("multiboot.zig");
const vga = @import("vga.zig");
const bitset = @import("bitset.zig").bitSet;
const std = @import("std");
const debug = @import("debug.zig");
const sys = @import("sys/registers.zig");

extern const __kernel_higher_half_start: usize;
extern const __kernel_higher_half_stop: usize;

pub const kernel_offset:usize = 0xC0000000;

//extern const __kernel_stop: usize;

pub var phys_bitset: bitset(4096, 1024 * 1024 * 1024) = .{};


pub const PhysicalLayout = struct {
    bitset: *bitset(4096, 1024 * 1024 * 1024),

    pub fn init(
        ptr: *PhysicalLayout,
        boot_config: *multiboot.multiboot_info_t,
        //kstart: usize,
        //kend: usize,
    ) void {
        //ptr.* = PhysicalLayout{};

        phys_bitset.addressable = ((boot_config.mem_lower + boot_config.mem_upper + 1024) * 1024) / 4096;
        @memset(&phys_bitset.bytes, 0);


        ptr.bitset = &phys_bitset;

        //vga.print("start: {}\n", args: anytype)
        //ptr.bitset.setRange(.{ 
        //    .start = @intFromPtr(&__kernel_higher_half_start) - kernel_offset,
        //    .stop = @intFromPtr(&__kernel_higher_half_stop) - kernel_offset 
        //}) catch unreachable;

        // zig refusing to use address 0
        ptr.bitset.set(0) catch unreachable;
        //ptr.bitset.setRange(.{ .start = @truncate(boot_config.framebuffer_addr), .stop = @truncate(boot_config.framebuffer_addr + 4096) }) catch unreachable;
    }

    pub const Error = error{NoPhysicalMem};

    pub fn allocPage(self: *PhysicalLayout) !*[4096]u8 {
        if(self.bitset.getFirstClear())|slot| {
            try self.bitset.set(slot.index);
            return @ptrFromInt(slot.address);
        } else return error.NoPhysicalMem;
    }

    pub fn allocPageBug(self: *PhysicalLayout) !*[4096]u8 {
        //sys.spinloop();
        if(try self.bitset.getFirstClearBug())|slot| {
            //sys.spinloop();
            try self.bitset.set(slot.index);
            return @ptrFromInt(slot.address);
        } else return error.NoPhysicalMem;
    }

    pub fn allocPageSpecific(self: *PhysicalLayout, paddr: usize) !*[4096]u8 {
        debug.assert(paddr % 4096 == 0, "page ptr must be aligned to 4096");
        const idx = paddr/4096;
        try self.bitset.set(idx);
        return @ptrFromInt(paddr);
    }

    pub fn freePage(self: *PhysicalLayout, page: *[4096]u8) !void {
        //debug.assert(page.len == 4096, "page length must be 4096");
        const ptr = @intFromPtr(page.ptr);
        debug.assert(ptr % 4096 == 0, "page ptr must be aligned to 4096");
        //_ = self;
        try self.bitset.clearAddress(ptr);
    } 
};
