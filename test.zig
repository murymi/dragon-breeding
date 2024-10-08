const std = @import("std");
const elf = std.elf;
const fs = std.fs;

const blo:usize = 4096;
const tot = 4096 * 20;

pub fn bitSet(block_size: usize, total_memory: usize) type {
    return struct {
        bytes: [std.mem.alignForward(usize, total_memory, block_size)/block_size]u8 = undefined,
        const Self = @This();

        pub fn set(self: *Self, index: usize) void {
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            self.bytes[byte_position] |= std.math.pow(u8, 2, bit_position);
        }

        pub fn clear(self: *Self, index: usize) void {
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            self.bytes[byte_position] &= ~std.math.pow(u8, 2, bit_position);
        }

        pub fn isSet(self: *Self, index: usize) bool {
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            return self.bytes[byte_position] & std.math.pow(u8, 2, bit_position) > 0;
        }

        pub const Slot = struct {
            index: usize,
            address: usize
        };

        pub fn getFirstClear(self: *Self) ?Slot {
            for(0..self.bytes.len) |i| {
                if(!self.isSet(i)) {
                    return .{ .index = i, .address = i * block_size };
                }
            }
            return null;
        }

    };
}

pub fn main() !void {
    var bitset = init(4096, 1024 * 1024 * 1024 * 4) {};
    //std.debug.print("{}\n", .{bitset.bits.len});
    for(0..8) |i| bitset.set(i);
    bitset.clear(2);

    //var t:[8]u1 = undefined;
    //t[0] = 1;

    std.debug.print("{}\n", .{@sizeOf(@TypeOf(bitset))});

    
}

//std.heap.PageAllocator.vtable.alloc