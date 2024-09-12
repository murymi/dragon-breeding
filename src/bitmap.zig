const std = @import("std");
const builtin = std.builtin;
const testing = std.testing;
const expectEqual = testing.expectEqual;
const Allocator = std.mem.Allocator;

/// The possible errors thrown by bitmap functions
pub const BitmapError = error{
    /// The address given was outside the region covered by a bitmap
    OutOfBounds,
};

///
/// A bitmap that uses a specific type to store the entries.
/// The bitmap can either be statically or dynamically allocated.
/// Statically allocated bitmaps are allocated at compile time and therefore must know the number of entries at compile time.
/// Dynamically allocated bitmaps do not need to know the number of entries until being initialised, but do need an allocator.
///
/// Arguments:
///     IN static: comptime bool - Whether this bitmap is statically or dynamically allocated
///     IN BitmapType: type - The integer type to use to store entries.
///     IN num_entries: usize or ?usize - The number of entries if static, else ignored (can be null)
///
/// Return: type.
///     The bitmap type created.
///
pub fn Bitmap(comptime num_entries: ?usize, comptime BitmapType: type) type {
    return struct {
        const Self = @This();
        const static = num_entries != null;

        /// The number of entries that one bitmap type can hold. Evaluates to the number of bits the type has
        pub const ENTRIES_PER_BITMAP: usize = @sizeOf(BitmapType) * 8;
        //std.meta.bitCount(BitmapType);

        /// The value that a full bitmap will have
        pub const BITMAP_FULL = std.math.maxInt(BitmapType);

        /// The type of an index into a bitmap entry. The smallest integer needed to represent all bit positions in the bitmap entry type
        pub const IndexType = std.meta.Int(.unsigned, 
        std.math.log2(std.math.ceilPowerOfTwo(u16, ENTRIES_PER_BITMAP) catch unreachable));

        num_bitmaps: usize,
        num_entries: usize,
        bitmaps: if (static) [std.mem.alignForward(num_entries.?, ENTRIES_PER_BITMAP) / ENTRIES_PER_BITMAP]BitmapType else []BitmapType,
        num_free_entries: usize,
        allocator: if (static) ?Allocator else Allocator,

        ///
        /// Create an instance of this bitmap type.
        ///
        /// Arguments:
        ///     IN num: ?usize or usize - The number of entries that the bitmap created will have if dynamically allocated, else ignored (can be null)
        ///         The number of BitmapType required to store this many entries will be allocated (dynamically or statically) and each will be zeroed.
        ///     IN allocator: ?Allocator or Allocator - The allocator to use when allocating the BitmapTypes required. Ignored if statically allocated.
        ///
        /// Return: Self.
        ///     The bitmap instance.
        ///
        /// Error: Allocator.Error
        ///     OutOfMemory: There isn't enough memory available to allocate the required number of BitmapType. A statically allocated bitmap will not throw this error.
        ///
        pub fn init(num: if (static) ?usize else usize, allocator: if (static) ?Allocator else Allocator) !Self {
            if (static) {
                const n = std.mem.alignForward(num_entries.?, ENTRIES_PER_BITMAP) / ENTRIES_PER_BITMAP;
                return Self{
                    .num_bitmaps = n,
                    .bitmaps = [_]BitmapType{0} ** (std.mem.alignForward(num_entries.?, ENTRIES_PER_BITMAP) / ENTRIES_PER_BITMAP),
                    .num_entries = num_entries.?,
                    .num_free_entries = num_entries.?,
                    .allocator = null,
                };
            } else {
                
                const n = std.mem.alignForward(usize, num, ENTRIES_PER_BITMAP) / ENTRIES_PER_BITMAP;
                const self = Self{
                    .num_bitmaps = n,
                    .num_entries = num,
                    .bitmaps = try allocator.alloc(BitmapType, n),
                    .num_free_entries = num,
                    .allocator = allocator,
                };
                for (self.bitmaps) |*bmp| {
                    bmp.* = 0;
                }
                return self;
            }
        }

        ///
        /// Clone this bitmap.
        ///
        /// Arguments:
        ///     IN self: *Self - The bitmap to clone.
        ///
        /// Return: Self
        ///     The cloned bitmap
        ///
        /// Error: Allocator.Error
        ///     OutOfMemory: There isn't enough memory available to allocate the required number of BitmapType.
        ///
        pub fn clone(self: *const Self) Allocator.Error!Self {
            var copy = try init(self.num_entries, self.allocator);
            var i: usize = 0;
            while (i < copy.num_entries) : (i += 1) {
                if (self.isSet(i) catch unreachable) {
                    copy.setEntry(i) catch unreachable;
                }
            }
            return copy;
        }

        ///
        /// Free the memory occupied by this bitmap's internal state. It will become unusable afterwards.
        /// Does nothing if the bitmap was statically allocated.
        ///
        /// Arguments:
        ///     IN self: *Self - The bitmap that should be deinitialised
        ///
        pub fn deinit(self: *Self) void {
            if (!static) self.allocator.free(self.bitmaps);
        }

        ///
        /// Set an entry within a bitmap as occupied.
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The bitmap to modify.
        ///     IN idx: usize - The index within the bitmap to set.
        ///
        /// Error: BitmapError.
        ///     OutOfBounds: The index given is out of bounds.
        ///
        pub fn setEntry(self: *Self, idx: usize) BitmapError!void {
            if (idx >= self.num_entries) {
                return BitmapError.OutOfBounds;
            }
            if (!try self.isSet(idx)) {
                const bit = indexToBit(idx);
                self.bitmaps[idx / ENTRIES_PER_BITMAP] |= bit;
                self.num_free_entries -= 1;
            }
        }

        ///
        /// Set an entry within a bitmap as unoccupied.
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The bitmap to modify.
        ///     IN idx: usize - The index within the bitmap to clear.
        ///
        /// Error: BitmapError.
        ///     OutOfBounds: The index given is out of bounds.
        ///
        pub fn clearEntry(self: *Self, idx: usize) BitmapError!void {
            if (idx >= self.num_entries) {
                return BitmapError.OutOfBounds;
            }
            if (try self.isSet(idx)) {
                const bit = indexToBit(idx);
                self.bitmaps[idx / ENTRIES_PER_BITMAP] &= ~bit;
                self.num_free_entries += 1;
            }
        }

        ///
        /// Convert a global bitmap index into the bit corresponding to an entry within a single BitmapType.
        ///
        /// Arguments:
        ///     IN self: *const Self - The bitmap to use.
        ///     IN idx: usize - The index into all of the bitmaps entries.
        ///
        /// Return: BitmapType.
        ///     The bit corresponding to that index but within a single BitmapType.
        ///
        fn indexToBit(idx: usize) BitmapType {
            return @as(BitmapType, 1) << @as(IndexType, @intCast(idx % ENTRIES_PER_BITMAP));
        }

        ///
        /// Find a number of contiguous free entries and set them.
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The bitmap to modify.
        ///     IN num: usize - The number of entries to set.
        ///     IN from: ?usize - The entry number to allocate from or null if it can start anywhere
        ///
        /// Return: ?usize
        ///     The first entry set or null if there weren't enough contiguous entries.
        ///     If `from` was not null and any entry between `from` and `from` + num is set then null is returned.
        ///
        pub fn setContiguous(self: *Self, num: usize, from: ?usize) ?usize {
            if (num > self.num_free_entries) {
                return null;
            }

            var count: usize = 0;
            var start: ?usize = from;
            var i: usize = if (from) |f| f / ENTRIES_PER_BITMAP else 0;
            var bit: IndexType = if (from) |f| @truncate(f % ENTRIES_PER_BITMAP) else 0;
            while (i < self.bitmaps.len) : ({
                i += 1;
                bit = 0;
            }) {
                const bmp = self.bitmaps[i];
                while (true) {
                    const entry = bit + i * ENTRIES_PER_BITMAP;
                    if (entry >= self.num_entries) {
                        return null;
                    }
                    if ((bmp & @as(BitmapType, 1) << bit) != 0) {
                        // This is a one so clear the progress
                        count = 0;
                        start = null;
                        // If the caller requested the allocation to start from
                        // a specific entry and it failed then return null
                        if (from) |_| {
                            return null;
                        }
                    } else {
                        // It's a zero so increment the count
                        count += 1;
                        if (start == null) {
                            // Start of the contiguous zeroes
                            start = entry;
                        }
                        if (count == num) {
                            // Reached the desired number
                            break;
                        }
                    }
                    // Avoiding overflow by checking if bit is less than the max - 1
                    if (bit < ENTRIES_PER_BITMAP - 1) {
                        bit += 1;
                    } else {
                        // Reached the end of the bitmap
                        break;
                    }
                }
                if (count == num) {
                    break;
                }
            }

            if (count == num) {
                if (start) |start_entry| {
                    var j: usize = 0;
                    while (j < num) : (j += 1) {
                        // Can't fail as the entry was found to be free
                        self.setEntry(start_entry + j) catch unreachable;
                    }
                    return start_entry;
                }
            }
            return null;
        }

        ///
        /// Set the first free entry within the bitmaps as occupied.
        ///
        /// Return: ?usize.
        ///     The index within all bitmaps that was set or null if there wasn't one free.
        ///     0 .. ENTRIES_PER_BITMAP - 1 if in the first bitmap, ENTRIES_PER_BITMAP .. ENTRIES_PER_BITMAP * 2 - 1 if in the second etc.
        ///
        pub fn setFirstFree(self: *Self) ?usize {
            if (self.num_free_entries == 0) {
                return null;
            }
            for (self.bitmaps, 0..) |*bmp, i| {
                if (bmp.* == BITMAP_FULL) {
                    continue;
                }
                const bit:IndexType = @truncate(@ctz(~bmp.*));
                const idx = bit + i * ENTRIES_PER_BITMAP;
                // Failing here means that the index is outside of the bitmap, so there are no free entries
                self.setEntry(idx) catch return null;
                return idx;
            }
            return null;
        }

        ///
        /// Check if an entry is set.
        ///
        /// Arguments:
        ///     IN self: *const Self - The bitmap to check.
        ///     IN idx: usize - The entry to check.
        ///
        /// Return: bool.
        ///     True if the entry is set, else false.
        ///
        /// Error: BitmapError.
        ///     OutOfBounds: The index given is out of bounds.
        ///
        pub fn isSet(self: *const Self, idx: usize) BitmapError!bool {
            if (idx >= self.num_entries) {
                return BitmapError.OutOfBounds;
            }
            return (self.bitmaps[idx / ENTRIES_PER_BITMAP] & indexToBit(idx)) != 0;
        }
    };
}