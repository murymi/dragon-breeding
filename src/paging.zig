const sys = @import("sys/registers.zig");
const std = @import("std");
const vga = @import("vga.zig");
const mem = @import("mem.zig");
const bitset = @import("bitset.zig").bitSet;
const debug = @import("debug.zig");


extern const __kernel_higher_half_start: usize;
extern const __kernel_higher_half_stop: usize;

var vitual_bitset: bitset(4096, 1024 * 1024 * 1024 * 4) = .{
        .addressable = (1024 * 1024 * 1024 * 4)/4096
    };

pub const PageDirectory = extern struct {
    phys: *mem.PhysicalLayout = @ptrFromInt(0x1000),
    bitset: *bitset(4096, 1024 * 1024 * 1024 * 4),

    pd: sys.PageDirectory align(4096) = [1]sys.PageDirectoryEntry{sys.PageDirectoryEntry.zero()} ** 1024,
    pts: [1024]sys.PageTable align(4096) = [1]sys.PageTable{[1]sys.PageTableEntry4K{sys.PageTableEntry4K.zero()} ** 1024} ** 1024,
    //[] [1]sys.PageTableEntry4K{sys.PageTableEntry4K.zero()} ** 1024,
    

    //allocator: std.mem.Allocator = undefined,
    //allocations: std.AutoHashMap(usize, usize) = undefined,

    pub fn init(
        self: *PageDirectory,
        //low_half_start: usize,
        //high_half_start: usize,
        //low_half_end: usize,
        //kernel_load_addr: usize,
        phys_layout: *mem.PhysicalLayout,
        //allocator: std.mem.Allocator,
    ) void {
        //_ = kernel_load_addr;
        self.bitset = &vitual_bitset;
        self.phys = phys_layout;
        const self_physical = @intFromPtr(&self.pd) - mem.kernel_offset;

        debug.assert( @intFromPtr(&__kernel_higher_half_stop) % 4096 == 0, "kernel stop must be aligned");
        debug.assert( @intFromPtr(&__kernel_higher_half_start) % 4096 == 0, "kernel start must be aligned");


        const kernel_size = 
        std.mem.alignForward(usize, 
        @intFromPtr(&__kernel_higher_half_stop) - @intFromPtr(&__kernel_higher_half_start)
        , 4096);
        //_= kernel_size;
        //_ = low_half_start;
        
        //self.allocator = allocator;
        //self.allocations = std.AutoHashMap(usize, usize).init(allocator);
        for (0..1024) |ent| {
            var directory_entry = sys.PageDirectoryEntry4K{};
            directory_entry.setPageTableBaseAddress(@intFromPtr(&self.pts[ent]) - mem.kernel_offset);
            self.pd[ent] = sys.PageDirectoryEntry{ .@"4K" = directory_entry };
        }

        var kern_high_idx = @intFromPtr(&__kernel_higher_half_start);
        //high_half_start + kernel_load_addr;
        var kern_low_idx = kern_high_idx - mem.kernel_offset;
        //low_half_start + kernel_load_addr;

        const kernel_end_aligned = kern_high_idx + kernel_size;

        while (kernel_end_aligned > kern_high_idx): (
            {
                kern_high_idx += 4096;
                kern_low_idx += 4096;
            }
        ) {
            self.bitset.set(kern_high_idx/4096) catch unreachable;
            //self.allocations.put(kern_low_idx, kern_high_idx) catch unreachable;
            const ld: sys.LinearAddress4K = @bitCast(kern_high_idx);
            var page_table_entry = sys.PageTableEntry4K{.sepervisor = true};
            page_table_entry.setPageBaseAddress(kern_low_idx);
            self.pts[ld.page_directory_entry][ld.page_table_entry] = page_table_entry;
        }

        var cr3 = sys.Cr3.read();
        cr3.setPageDirectoryBase(@ptrFromInt(self_physical));
        cr3.write();

        self.bitset.set(0) catch unreachable;
        //_ = self.mapSpecificAddress(0xb8000 + 0xC0000000, 0xb8000, true) catch unreachable;
        //const addr:*u8 = @ptrFromInt(self.mapSpecific(9, 0xb8000) catch unreachable);
        //while (true) {}
    }


    const Error = error{
        Unmapped,
        Unaligned,
        Busy,
        OutOfMem
    };

    pub fn mapRandom(self: *PageDirectory, paddress: usize) !*[4096]u8 {
        if(paddress % 4096 != 0) return error.Unaligned;
        const free = try self.bitset.getFirstClear() orelse {
            return error.OutOfMem;
        };
        return self.mapSpecific(free.index, paddress);
    }

    fn mapSpecific(self: *PageDirectory, idx: usize, paddress: usize) !*[4096]u8 {
        if(paddress % 4096 != 0) return error.Unaligned;

        if(try self.bitset.isSet(idx)) return error.Busy;
        try self.bitset.set(idx);
        const v = idx * 4096;
        const ld: sys.LinearAddress4K = @bitCast(v);
        var page_table_entry = sys.PageTableEntry4K{.sepervisor = true};
        page_table_entry.setPageBaseAddress(paddress);
        self.pts[ld.page_directory_entry][ld.page_table_entry] = page_table_entry;
        //try self.allocations.put(paddress, v);
        //if(ap) try self.phys.bitset.setRange(.{.start = paddress, .stop = paddress + 4096});
        return @ptrFromInt(v);
    }

    // fn mapSpecificBug(self: *PageDirectory, idx: usize, paddress: usize) !*[4096]u8 {
    //     if(paddress % 4096 != 0) return error.Unaligned;
    //     if(try self.bitset.isSet(idx)) return error.Busy;
    //     //sys.spinloop();
    //     try self.bitset.set(idx);
    //     const v = idx * 4096;
    //     const ld: sys.LinearAddress4K = @bitCast(v);
    //     var page_table_entry = sys.PageTableEntry4K{.sepervisor = true};
    //     page_table_entry.setPageBaseAddress(paddress);
    //     self.pts[ld.page_directory_entry][ld.page_table_entry] = page_table_entry;
    //     //try self.allocations.put(paddress, v);
    //     //if(ap) try self.phys.bitset.setRange(.{.start = paddress, .stop = paddress + 4096});
    //     return @ptrFromInt(v);
    // }

    pub const Random = struct {
        page: *[4096]u8,
        physical_address: usize
    };

// address assumed to be allocated
    fn mapSpecificAddress(self: *PageDirectory, virtual_address: usize, address: usize) !*[4096]u8 {
        if(address % 4096 != 0) return error.Unaligned;
        if(virtual_address % 4096 != 0) return error.Unaligned;
        const idx = virtual_address/4096;
        return self.mapSpecific(idx, address);
    }

    // fn mapSpecificAddressBug(self: *PageDirectory, virtual_address: usize, address: usize) !*[4096]u8 {
    //     if(address % 4096 != 0) return error.Unaligned;
    //     if(virtual_address % 4096 != 0) return error.Unaligned;
    //     const idx = virtual_address/4096;
    //     return self.mapSpecificBug(idx, address);
    // }

    pub fn mapAnyFree(self: *PageDirectory) !Random {
        const address = try self.phys.allocPage();
        //self.phys.bitset.getFirstClear() orelse return error.OutOfMem;
        const idx = self.bitset.getFirstClear() orelse return error.OutOfMem;
        //try self.phys.bitset.set(address.index);
        //vga.print("address {}: index: {}\n", .{address.address, idx.index});
        //sys.spinloop();
        const page = try self.mapSpecific(idx.index, @intFromPtr(address));
        return Random{
            .page = page,
            .physical_address = address.address
        };
    }

    pub fn mapVirtualAddress(self: *PageDirectory, vaddr: usize) !*[4096]u8 {
        const address = try self.phys.allocPage();
        return try self.mapSpecificAddress(vaddr, @intFromPtr(address));
    }

    // virtual
    pub fn unmap(self: *PageDirectory, address: usize, free_pys: bool) !void {

        if(address % 4096 != 0) return error.Unaligned;
        //@panic("attempt to map unaligned address");
        const idx = address/4096;
        if(self.bitset.isSet(idx)) |b|{
            if(!b) return error.Unmapped;
        } else |e| return e;
        const ld: sys.LinearAddress4K = @bitCast(address);

        // free phsical
        if(free_pys) {
            const pyhsical_addr= self.pts[ld.page_directory_entry][ld.page_table_entry].getPageBaseAddress();
            try self.phys.freePage(@ptrFromInt(pyhsical_addr));
        }

        // zero
        self.pts[ld.page_directory_entry][ld.page_table_entry] = sys.PageTableEntry4K.zero();
        try self.bitset.clear(idx);

        sys.TlbFlush();
    }

    // pub fn unmapPhysical(self: *PageDirectory, address: usize, fp: bool) !void {
    //     if(self.allocations.fetchRemove(address)) |add| {
    //         const idx = add.value/4096;
    //         self.bitset.clear(idx) catch unreachable;
    //         const ld: sys.LinearAddress4K = @bitCast(add.value);
    //         if(fp) try self.phys.bitset.clearAddress(self.pts[ld.page_directory_entry][ld.page_table_entry].getPageBaseAddress());
    //         self.pts[ld.page_directory_entry][ld.page_table_entry] = sys.PageTableEntry4K.zero();
    //     } else return error.Unmapped;
    // }

    // pub fn unmapIndex(self: *PageDirectory, idx: usize, fp: bool) !void {
    //     if(self.bitset.isSet(idx)) |b|{
    //         if(!b) return error.Unmapped;
    //     } else |e| return e;
    //     const ld: sys.LinearAddress4K = @bitCast(idx*4096);
    //     if(fp) try self.phys.bitset.clearAddress(self.pts[ld.page_directory_entry][ld.page_table_entry].getPageBaseAddress());
    //     self.pts[ld.page_directory_entry][ld.page_table_entry] = sys.PageTableEntry4K.zero();
    //     self.bitset.clear(idx) catch unreachable;
    // }
};
