//const is_test = @import("builtin").is_test;
const std = @import("std");
const log = std.log.scoped(.pmm);
//const build_options = @import("build_options");
//const arch = @import("arch.zig").internals;
const MemProfile = @import("mem.zig").MemProfile;
//const testing = std.testing;
//const panic = @import("panic.zig").panic;
const bitmap = @import("bitmap.zig");
const Bitmap = bitmap.Bitmap;
const Allocator = std.mem.Allocator;

const PmmBitmap = Bitmap(null, u32);
const vga = @import("vga.zig");

/// The possible errors thrown by bitmap functions
const PmmError = error{
    /// The address given hasn't been allocated
    NotAllocated,
};

/// The size of memory associated with each bitmap entry
pub const BLOCK_SIZE: usize = 4096;

var the_bitmap: PmmBitmap = undefined;

///
/// Set the bitmap entry for an address as occupied
///
/// Arguments:
///     IN addr: usize - The address.
///
/// Error: PmmBitmap.BitmapError.
///     *: See PmmBitmap.setEntry. Could occur if the address is out of bounds.
///
pub fn setAddr(addr: usize) bitmap.BitmapError!void {
    try the_bitmap.setEntry(@intCast(addr / BLOCK_SIZE));
}

///
/// Check if an address is set as occupied.
///
/// Arguments:
///     IN addr: usize - The address to check.
///
/// Return: True if occupied, else false.
///
/// Error: PmmBitmap.BitmapError.
///     *: See PmmBitmap.setEntry. Could occur if the address is out of bounds.
///
pub fn isSet(addr: usize) bitmap.BitmapError!bool {
    return the_bitmap.isSet(@intCast(addr / BLOCK_SIZE));
}

///
/// Find the next free memory block, set it as occupied and return it. The region allocated will be of size BLOCK_SIZE.
///
/// Return: The address that was allocated.
///
pub fn alloc() ?usize {
    if (the_bitmap.setFirstFree()) |entry| {
        return entry * BLOCK_SIZE;
    }
    return null;
}

///
/// Set the address as free so it can be allocated in the future. This will free a block of size BLOCK_SIZE.
///
/// Arguments:
///     IN addr: usize - The previously allocated address to free. Will be aligned down to the nearest multiple of BLOCK_SIZE.
///
/// Error: PmmError || PmmBitmap.BitmapError.
///     PmmError.NotAllocated: The address wasn't allocated.
///     PmmBitmap.BitmapError.OutOfBounds: The address given was out of bounds.
///
pub fn free(addr: usize) (bitmap.BitmapError || PmmError)!void {
    const idx:u32 = @intCast(addr / BLOCK_SIZE);
    if (try the_bitmap.isSet(idx)) {
        try the_bitmap.clearEntry(idx);
    } else {
        return PmmError.NotAllocated;
    }
}

///
/// Get the number of unallocated blocks of memory.
///
/// Return: usize.
///     The number of unallocated blocks of memory
///
pub fn blocksFree() usize {
    return the_bitmap.num_free_entries;
}

/// Intiialise the physical memory manager and set all unavailable regions as occupied (those from the memory map and those from the linker symbols).
///
/// Arguments:
///     IN mem: *const MemProfile - The system's memory profile.
///     IN allocator: Allocator - The allocator to use to allocate the bitmaps.
///
pub fn init(mem_profile: *const MemProfile, allocator: Allocator) void {
    vga.print("Init pmm\n", .{});
    defer vga.print("Done\n", .{});

    the_bitmap = PmmBitmap.init(mem_profile.mem_kb * 1024 / BLOCK_SIZE, allocator) catch |e| {
        vga.print("Bitmap allocation failed: {}\n", .{e});
        @panic("pmm init");
    };

    // Occupy the regions of memory that the memory map describes as reserved
    for (mem_profile.physical_reserved) |entry| {
        var addr = std.mem.alignBackward(usize, entry.start, BLOCK_SIZE);
        var end = entry.end - 1;
        // If the end address can be aligned without overflowing then align it
        if (end <= std.math.maxInt(usize) - BLOCK_SIZE) {
            end = std.mem.alignForward(usize, end, BLOCK_SIZE);
        }
        while (addr < end) : (addr += BLOCK_SIZE) {
            setAddr(addr) catch |e| switch (e) {
                // We can ignore out of bounds errors as the memory won't be available anyway
                bitmap.BitmapError.OutOfBounds => break,
                //else => { 
                //    vga.print(@errorReturnTrace(), "Failed setting address 0x{x} from memory map as occupied: {}", .{ addr, e });
                //    @panic("========\n");
                //},
            };
        }
    }

    //switch (build_options.test_mode) {
    //    .Initialisation => runtimeTests(mem_profile, allocator),
    //    else => {},
    //}
}

///
/// Free the internal state of the PMM. Is unusable aftwards unless re-initialised
///
pub fn deinit() void {
    the_bitmap.deinit();
}