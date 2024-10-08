const std = @import("std");
const elf = std.elf;
const fs = std.fs;
const sys = @import("sys/registers.zig");
const vga = @import("vga.zig");

pub fn bitSet(block_size: usize, total_memory: u64) type {
    return struct {
        addressable: usize = 0,
        bytes: [std.mem.alignForward(u64, total_memory, block_size)/block_size]u8 = 
        [1]u8{0} ** (std.mem.alignForward(u64, total_memory, block_size)/block_size),
        const Self = @This();

        const Error = error{IndexOutOfBounds, UnAligned, Busy, Unmapped};

        pub fn set(self: *Self, index: usize) !void {
            if(index > self.addressable) return error.IndexOutOfBounds;
            if(try self.isSet(index)) return error.Busy;
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            self.bytes[byte_position] |= std.math.pow(u8, 2, bit_position);
        }

        pub fn clear(self: *Self, index: usize) !void {
            //sys.spinloop();
            if(index > self.addressable) return error.IndexOutOfBounds;
            if(!(try self.isSet(index))) {
                return error.Unmapped;
                //vga.print("Is set: idx-> {}\n", .{index});
                //sys.spinloop();
            }
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            //_ = byte_position;
            //_ = bit_position;
           self.bytes[byte_position] &= ~std.math.pow(u8, 2, bit_position);
        }

        pub fn isSet(self: *Self, index: usize) !bool {
            if(index > self.addressable) return error.IndexOutOfBounds;
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            return self.bytes[byte_position] & std.math.pow(u8, 2, bit_position) > 0;
            //sys.spinloop();
        }

        pub fn isSetBug(self: *Self, index: usize) !bool {
            _ = self.addressable;
            vga.print("here loop: {}\n",.{self.bytes[2]});
            sys.spinloop();

            //if(index > self.addressable) {
            //    //return error.IndexOutOfBounds;
            //} 
            const byte_position = index/8;
            const bit_position: u8 = @truncate(index%8);
            return self.bytes[byte_position] & std.math.pow(u8, 2, bit_position) > 0;
            //sys.spinloop();
        }

//'4294967296'
        pub const Slot = struct {
            index: usize,
            address: usize
        };

        pub fn getFirstClear(self: *Self) !?Slot {
            for(0..@min(self.addressable, self.bytes.len)) |i| {
                if(!(try self.isSet(i))) {
                    return .{ .index = i, .address = i * block_size };
                }
            }
            return null;
        }

        pub fn getFirstClearBug(self: *Self) !?Slot {
            for(0..@min(self.addressable, self.bytes.len)) |_| {
            sys.spinloop();
                //if(!(try self.isSet(i))) {
                //    return .{ .index = i, .address = i * block_size };
                //}
            }
            return null;
        }
        
        /// get total free blocks
        pub fn getTatalFree(self: *Self) !usize {
            var tot: usize = 0;
            for(0..@min(self.addressable, self.bytes.len)) |i| {
                if(!(try self.isSet(i))) {
                    tot += 1;
                }
            }
            return tot;
        }

        pub const Range = struct {
            start: usize,
            stop: usize
        };

        pub fn setRange(self: *Self, range: Range) !void {
            const range_stop = std.mem.alignForward(usize, range.stop, block_size);
            const range_start = std.mem.alignBackward(usize, range.start,block_size);
            var i = range_start;
            while (i < range_stop): (i += block_size ) {
                try self.set(i/block_size);
            }
        }

        pub fn isAddressSet(self: *Self, address: usize) !bool {
            if(address % block_size != 0) return error.UnAligned;
            const index = address/block_size;
            return if(self.isSet(index))|b| b else |e| e;
        }

        pub fn clearAddress(self: *Self, address: usize) !void {
            if(address % block_size != 0) return error.UnAligned;
            const index = address/block_size;
            if(self.isSet(index)) |b| {
                if(!b) return error.Unmapped;
            } else |e| return e;
            //sys.spinloop();
            try self.clear(index);
        }
    };
}

// pub fn main() !void {
//     var b: bitSet(4096, 1024 * 1024 * 1024 * 4) = .{};
// 
//     //var b = bitSet(block_size: usize, total_memory: usize)
// 
//     try b.setRange(.{.start = 0, .stop = 4097});
// 
//     std.debug.print("{b}\n", .{b.bytes[0]});
//     
// }