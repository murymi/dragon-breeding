//const build_options = @import("build_options");
//const mock_path = build_options.mock_path;
const builtin = std.builtin;
//const is_test = builtin.is_test;
const std = @import("std");
//const log = std.log.scoped(.vmm);
const bitmap = @import("bitmap.zig");
//const pmm = @import("pmm.zig");
const mem = @import("mem.zig");
//const tty = @import("tty.zig");
//const panic = @import("panic.zig").panic;
//const arch = @import("arch.zig").internals;
const Allocator = std.mem.Allocator;
const assert = std.debug.assert;
const pmm = @import("pmm.zig");
const vga = @import("vga.zig");

/// Attributes for a virtual memory allocation
pub const Attributes = struct {
    /// Whether this memory belongs to the kernel and can therefore not be accessed in user mode
    kernel: bool,

    /// If this memory can be written to
    writable: bool,

    /// If this memory can be cached. Memory mapped to a device shouldn't, for example
    cachable: bool,
};

/// All data that must be remembered for a virtual memory allocation
const Allocation = struct {
    /// The physical blocks of memory associated with this allocation
    physical: std.ArrayList(usize),
};

/// The size of each allocatable block, the same as the physical memory manager's block size
pub const BLOCK_SIZE: usize = pmm.BLOCK_SIZE;

pub const MapperError = error{
    InvalidVirtualAddress,
    InvalidPhysicalAddress,
    AddressMismatch,
    MisalignedVirtualAddress,
    MisalignedPhysicalAddress,
    NotMapped,
};

///
/// Returns a container that can map and unmap virtual memory to physical memory.
/// The mapper can pass some payload data when mapping an unmapping, which is of type
///  `Payload`. This can be anything that the underlying mapper needs to carry out the mapping process.
/// For x86, it would be the page directory that is being mapped within. An architecture or 
/// other mapper can specify the data it needs when mapping by specifying this type.
///
/// Arguments:
///     IN comptime Payload: type - The type of the VMM-specific payload to pass when mapping and unmapping
///
/// Return: type
///     The Mapper type constructed.
///
const Mapper = struct {
        ///
        /// Map a region (can span more than one block) of virtual memory to physical memory. After a call to this function, the memory should be present the next time it is accessed.
        /// The attributes given must be obeyed when possible.
        ///
        /// Arguments:
        ///     IN virtual_start: usize - The start of the virtual memory to map
        ///     IN virtual_end: usize - The end of the virtual memory to map
        ///     IN physical_start: usize - The start of the physical memory to map to
        ///     IN physical_end: usize - The end of the physical memory to map to
        ///     IN attrs: Attributes - The attributes to apply to this region of memory
        ///     IN/OUT allocator: Allocator - The allocator to use when mapping, if required
        ///     IN spec: Payload - The payload to pass to the mapper
        ///
        /// Error: AllocatorError || MapperError
        ///     The causes depend on the mapper used
        ///
        mapFn: *const fn (virtual_start: usize,
         virtual_end: usize, physical_start: usize, physical_end: usize, attrs: Attributes, allocator: Allocator,
          spec: VmmPayload) (Allocator.Error || MapperError)!void,

        ///
        /// Unmap a region (can span more than one block) of virtual memory from its physical memory. After a call to this function, the memory should not be accessible without error.
        ///
        /// Arguments:
        ///     IN virtual_start: usize - The start of the virtual region to unmap
        ///     IN virtual_end: usize - The end of the virtual region to unmap
        ///     IN/OUT allocator: Allocator - The allocator to use to free the mapping
        ///     IN spec: Payload - The payload to pass to the mapper
        ///
        /// Error: MapperError
        ///     The causes depend on the mapper used
        ///
        unmapFn: *const fn (virtual_start: usize, virtual_end: usize, allocator: Allocator, spec: VmmPayload) MapperError!void,
    };


/// Errors that can be returned by VMM functions
pub const VmmError = error{
    /// A memory region expected to be allocated wasn't
    NotAllocated,

    /// A memory region expected to not be allocated was
    AlreadyAllocated,

    /// A physical memory region expected to not be allocated was
    PhysicalAlreadyAllocated,

    /// A physical region of memory isn't of the same size as a virtual region
    PhysicalVirtualMismatch,

    /// Virtual addresses are invalid
    InvalidVirtAddresses,

    /// Physical addresses are invalid
    InvalidPhysAddresses,

    /// Not enough virtual space in the VMM
    OutOfMemory,
};

/// The boot-time offset that the virtual addresses are from the physical addresses
/// This is the start of the memory owned by the kernel and so is where the kernel VMM starts
extern var KERNEL_ADDR_OFFSET: *u32;

const paging = @import("paging.zig");

pub const VmmPayload = *paging.Directory;

/// The virtual memory manager associated with the kernel address space
pub var kernel_vmm: VirtualMemoryManager = undefined;

///
/// Construct a virtual memory manager to keep track of allocated and free virtual memory regions within a certain space
///
/// Arguments:
///     IN comptime Payload: type - The type of the payload to pass to the mapper
///
/// Return: type
///     The constructed type
///
const VirtualMemoryManager =  struct {
        /// The bitmap that keeps track of allocated and free regions
        bmp: bitmap.Bitmap(null, usize),

        /// The start of the memory to be tracked
        start: usize,

        /// The end of the memory to be tracked
        end: usize,

        /// The allocator to use when allocating and freeing regions
        allocator: Allocator,

        /// All allocations that have been made with this manager
        allocations: std.hash_map.AutoHashMap(usize, Allocation),

        /// The mapper to use when allocating and freeing regions
        mapper: Mapper,

        /// The payload to pass to the mapper functions
        payload: VmmPayload,

        const Self = @This();

        ///
        /// Initialise a virtual memory manager
        ///
        /// Arguments:
        ///     IN start: usize - The start of the memory region to manage
        ///     IN end: usize - The end of the memory region to manage. Must be greater than the start
        ///     IN/OUT allocator: Allocator - The allocator to use when allocating and freeing regions
        ///     IN mapper: Mapper - The mapper to use when allocating and freeing regions
        ///     IN payload: Payload - The payload data to be passed to the mapper
        ///
        /// Return: Self
        ///     The manager constructed
        ///
        /// Error: Allocator.Error
        ///     error.OutOfMemory - The allocator cannot allocate the memory required
        ///
        pub fn init(start: usize, end: usize, allocator: Allocator, mapper: Mapper, payload: VmmPayload) Allocator.Error!Self {
            const size = end - start;
            const bmp = 
            try bitmap.Bitmap(null, usize).init(std.mem.alignForward(usize, size, pmm.BLOCK_SIZE) / pmm.BLOCK_SIZE, allocator);
            return Self{
                .bmp = bmp,
                .start = start,
                .end = end,
                .allocator = allocator,
                .allocations = std.hash_map.AutoHashMap(usize, Allocation).init(allocator),
                .mapper = mapper,
                .payload = payload,
            };
        }

        ///
        /// Copy this VMM. Changes to one copy will not affect the other
        ///
        /// Arguments:
        ///     IN self: *Self - The VMM to copy
        ///
        /// Error: Allocator.Error
        ///     OutOfMemory - There wasn't enough memory for copying
        ///
        /// Return: Self
        ///     The copy
        ///
        pub fn copy(self: *const Self) Allocator.Error!Self {
            var clone = Self{
                .bmp = try self.bmp.clone(),
                .start = self.start,
                .end = self.end,
                .allocator = self.allocator,
                .allocations = std.hash_map.AutoHashMap(usize, Allocation).init(self.allocator),
                .mapper = self.mapper,
                .payload = self.payload,
            };
            var it = self.allocations.iterator();
            while (it.next()) |entry| {
                var list = std.ArrayList(usize).init(self.allocator);
                for (entry.value_ptr.physical.items) |block| {
                    _ = try list.append(block);
                }
                _ = try clone.allocations.put(entry.key_ptr.*, Allocation{ .physical = list });
            }
            return clone;
        }

        ///
        /// Free the internal state of the VMM. It is unusable afterwards
        ///
        /// Arguments:
        ///     IN self: *Self - The VMM to deinitialise
        ///
        pub fn deinit(self: *Self) void {
            self.bmp.deinit();
            var it = self.allocations.iterator();
            while (it.next()) |entry| {
                entry.value_ptr.physical.deinit();
            }
            self.allocations.deinit();
        }

        ///
        /// Find the physical address that a given virtual address is mapped to.
        ///
        /// Arguments:
        ///     IN self: *const Self - The VMM to check for mappings in
        ///     IN virt: usize - The virtual address to find the physical address for
        ///
        /// Return: usize
        ///     The physical address that the virtual address is mapped to
        ///
        /// Error: VmmError
        ///     VmmError.NotAllocated - The virtual address hasn't been mapped within the VMM
        ///
        pub fn virtToPhys(self: *const Self, virt: usize) VmmError!usize {
            var it = self.allocations.iterator();
            while (it.next()) |entry| {
                const vaddr = entry.key_ptr.*;

                const allocation = entry.value_ptr.*;
                // If this allocation range covers the virtual address then figure out the corresponding physical block
                if (vaddr <= virt and vaddr + (allocation.physical.items.len * BLOCK_SIZE) > virt) {
                    const block_number = (virt - vaddr) / BLOCK_SIZE;
                    const block_offset = (virt - vaddr) % BLOCK_SIZE;
                    return allocation.physical.items[block_number] + block_offset;
                }
            }
            return VmmError.NotAllocated;
        }

        ///
        /// Find the virtual address that a given physical address is mapped to.
        ///
        /// Arguments:
        ///     IN self: *const Self - The VMM to check for mappings in
        ///     IN phys: usize - The physical address to find the virtual address for
        ///
        /// Return: usize
        ///     The virtual address that the physical address is mapped to
        ///
        /// Error: VmmError
        ///     VmmError.NotAllocated - The physical address hasn't been mapped within the VMM
        ///
        pub fn physToVirt(self: *const Self, phys: usize) VmmError!usize {
            var it = self.allocations.iterator();
            while (it.next()) |entry| {
                const vaddr = entry.key_ptr.*;
                const allocation = entry.value_ptr.*;

                for (allocation.physical.items, 0..) |block, i| {
                    if (block <= phys and block + BLOCK_SIZE > phys) {
                        const block_addr = vaddr + i * BLOCK_SIZE;
                        const block_offset = phys % BLOCK_SIZE;
                        return block_addr + block_offset;
                    }
                }
            }
            return VmmError.NotAllocated;
        }

        ///
        /// Check if a virtual memory address has been set
        ///
        /// Arguments:
        ///     IN self: *Self - The manager to check
        ///     IN virt: usize - The virtual memory address to check
        ///
        /// Return: bool
        ///     Whether the address is set
        ///
        /// Error: pmm.PmmError
        ///     Bitmap(u32).Error.OutOfBounds - The address given is outside of the memory managed
        ///
        pub fn isSet(self: *const Self, virt: usize) bitmap.BitmapError!bool {
            if (virt < self.start) {
                return bitmap.BitmapError.OutOfBounds;
            }
            return self.bmp.isSet((virt - self.start) / BLOCK_SIZE);
        }

        ///
        /// Map a region (can span more than one block) of virtual memory to a specific region of memory
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The manager to modify
        ///     IN virtual: mem.Range - The virtual region to set
        ///     IN physical: ?mem.Range - The physical region to map to or null if only the virtual region is to be set
        ///     IN attrs: Attributes - The attributes to apply to the memory regions
        ///
        /// Error: VmmError || Bitmap(u32).BitmapError || Allocator.Error || MapperError
        ///     VmmError.AlreadyAllocated - The virtual address has already been allocated
        ///     VmmError.PhysicalAlreadyAllocated - The physical address has already been allocated
        ///     VmmError.PhysicalVirtualMismatch - The physical region and virtual region are of different sizes
        ///     VmmError.InvalidVirtAddresses - The start virtual address is greater than the end address
        ///     VmmError.InvalidPhysicalAddresses - The start physical address is greater than the end address
        ///     Bitmap.BitmapError.OutOfBounds - The physical or virtual addresses are out of bounds
        ///     Allocator.Error.OutOfMemory - Allocating the required memory failed
        ///     MapperError.* - The causes depend on the mapper used
        ///
        pub fn set(self: *Self, virtual: mem.Range, physical: ?mem.Range, attrs: Attributes) (VmmError || bitmap.BitmapError || Allocator.Error || MapperError)!void {
            var virt = virtual.start;
            while (virt < virtual.end) : (virt += BLOCK_SIZE) {
                if (try self.isSet(virt)) {
                    return VmmError.AlreadyAllocated;
                }
            }
            if (virtual.start > virtual.end) {
                return VmmError.InvalidVirtAddresses;
            }

            if (physical) |p| {
                if (virtual.end - virtual.start != p.end - p.start) {
                    return VmmError.PhysicalVirtualMismatch;
                }
                if (p.start > p.end) {
                    return VmmError.InvalidPhysAddresses;
                }
                var phys = p.start;
                while (phys < p.end) : (phys += BLOCK_SIZE) {
                    if (try pmm.isSet(phys)) {
                        return VmmError.PhysicalAlreadyAllocated;
                    }
                }
            }

            var phys_list = std.ArrayList(usize).init(self.allocator);

            virt = virtual.start;
            while (virt < virtual.end) : (virt += BLOCK_SIZE) {
                try self.bmp.setEntry((virt - self.start) / BLOCK_SIZE);
            }

            if (physical) |p| {
                var phys = p.start;
                while (phys < p.end) : (phys += BLOCK_SIZE) {
                    try pmm.setAddr(phys);
                    try phys_list.append(phys);
                }
            }

            // Do this before mapping as the mapper may depend on the allocation being tracked
            _ = try self.allocations.put(virtual.start, Allocation{ .physical = phys_list });

            if (physical) |p| {
                try self.mapper.mapFn(virtual.start, virtual.end, p.start, p.end, attrs, self.allocator, self.payload);
            }
        }

        ///
        /// Allocate a number of contiguous blocks of virtual memory
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The manager to allocate for
        ///     IN num: usize - The number of blocks to allocate
        ///     IN virtual_addr: ?usize - The virtual address to allocate to or null if any address is acceptable
        ///     IN attrs: Attributes - The attributes to apply to the mapped memory
        ///
        /// Return: ?usize
        ///     The address at the start of the allocated region, or null if no region could be allocated due to a lack of contiguous blocks.
        ///
        /// Error: Allocator.Error
        ///     error.OutOfMemory: The required amount of memory couldn't be allocated
        ///
        pub fn alloc(self: *Self, num: usize, virtual_addr: ?usize, attrs: Attributes) Allocator.Error!?usize {
            if (num == 0) {
                return null;
            }
            // Ensure that there is both enough physical and virtual address space free
            if (pmm.blocksFree() >= num and self.bmp.num_free_entries >= num) {
                // The virtual address space must be contiguous
                // Allocate from a specific entry if the caller requested it
                if (self.bmp.setContiguous(num, if (virtual_addr) |a| (a - self.start) / BLOCK_SIZE else null)) |entry| {
                    var block_list = std.ArrayList(usize).init(self.allocator);
                    try block_list.ensureUnusedCapacity(num);

                    var i: usize = 0;
                    const vaddr_start = self.start + entry * BLOCK_SIZE;
                    var vaddr = vaddr_start;
                    // Map the blocks to physical memory
                    while (i < num) : (i += 1) {
                        const addr = pmm.alloc() orelse unreachable;
                        try block_list.append(addr);
                        // The map function failing isn't the caller's responsibility so panic as it shouldn't happen
                        self.mapper.mapFn(vaddr, vaddr + BLOCK_SIZE, addr, addr + BLOCK_SIZE, attrs, self.allocator, self.payload) catch |e| {
                            vga.print("Failed to map virtual memory: {X}\n", .{e});
                        @panic("=========\n");
                        };
                        vaddr += BLOCK_SIZE;
                    }
                    _ = try self.allocations.put(vaddr_start, Allocation{ .physical = block_list });
                    return vaddr_start;
                }
            }
            return null;
        }

        ///
        /// Copy data from an address in a virtual memory manager to an address in another virtual memory manager
        ///
        /// Arguments:
        ///     IN self: *Self - One of the VMMs to copy between. This should be the currently active VMM
        ///     IN other: *Self - The second of the VMMs to copy between
        ///     IN from: bool - Whether the data should be copied from `self` to `other`, or the other way around
        ///     IN data: if (from) []const u8 else []u8 - The being copied from or written to (depending on `from`). Must be mapped within the VMM being copied from/to
        ///     IN address: usize - The address within `other` that is to be copied from or to
        ///
        /// Error: VmmError || pmm.PmmError || Allocator.Error
        ///     VmmError.NotAllocated - Some or all of the destination isn't mapped
        ///     VmmError.OutOfMemory - There wasn't enough space in the VMM to use for temporary mapping
        ///     Bitmap(u32).Error.OutOfBounds - The address given is outside of the memory managed
        ///     Allocator.Error.OutOfMemory - There wasn't enough memory available to fulfill the request
        ///
        pub fn copyData(self: *Self, other: *const Self, comptime from: bool, data: if (from) []const u8 else []u8, address: usize) (bitmap.BitmapError || VmmError || Allocator.Error)!void {
            if (data.len == 0) {
                return;
            }
            const start_addr = std.mem.alignBackward(address, BLOCK_SIZE);
            const end_addr = std.mem.alignForward(address + data.len, BLOCK_SIZE);

            if (end_addr >= other.end or start_addr < other.start)
                return bitmap.BitmapError.OutOfBounds;
            // Find physical blocks for the address
            var blocks = std.ArrayList(usize).init(self.allocator);
            defer blocks.deinit();
            var it = other.allocations.iterator();
            while (it.next()) |allocation| {
                const virtual = allocation.key_ptr.*;
                const physical = allocation.value_ptr.*.physical.items;
                if (start_addr >= virtual and virtual + physical.len * BLOCK_SIZE >= end_addr) {
                    const first_block_idx = (start_addr - virtual) / BLOCK_SIZE;
                    const last_block_idx = (end_addr - virtual) / BLOCK_SIZE;

                    try blocks.appendSlice(physical[first_block_idx..last_block_idx]);
                }
            }
            // Make sure the address is actually mapped in the destination VMM
            if (blocks.items.len != std.mem.alignForward(data.len, BLOCK_SIZE) / BLOCK_SIZE) {
                return VmmError.NotAllocated;
            }

            // Map them into self for some vaddr so they can be accessed from this VMM
            if (self.bmp.setContiguous(blocks.items.len, null)) |entry| {
                const v_start = entry * BLOCK_SIZE + self.start;
                for (blocks.items, 0..) |block, i| {
                    const v = v_start + i * BLOCK_SIZE;
                    const v_end = v + BLOCK_SIZE;
                    const p = block;
                    const p_end = p + BLOCK_SIZE;
                    self.mapper.mapFn(v, v_end, p, p_end, .{ .kernel = true, .writable = true, .cachable = false }, self.allocator, self.payload) catch |e| {
                        // If we fail to map one of the blocks then attempt to free all previously mapped
                        if (i > 0) {
                            self.mapper.unmapFn(v_start, v_end, self.allocator, self.payload) catch |e2| {
                                // If we can't unmap then just panic
                                vga.print("Failed to unmap virtual region 0x{X} -> 0x{X}: {}\n", .{ v_start, v_end, e2 });
                            };
                        }
                        vga.print("Failed to map virtual region 0x{X} -> 0x{X} to 0x{X} -> 0x{X}: {}\n", .{ v, v_end, p, p_end, e });
                        @panic("============\n");
                    };
                }
                // Copy to vaddr from above
                const align_offset = address - start_addr;
                var data_copy :[*]u8 = @ptrFromInt(v_start + align_offset);
                //[0..data.len];
                if (from) {
                    std.mem.copy(u8, data_copy[0..data.len], data);
                } else {
                    std.mem.copy(u8, data, data_copy[0..data.len]);
                }
                // TODO Unmap and freee virtual blocks from self so they can be used in the future
            } else {
                return VmmError.OutOfMemory;
            }
        }

        ///
        /// Free a previous allocation
        ///
        /// Arguments:
        ///     IN/OUT self: *Self - The manager to free within
        ///     IN vaddr: usize - The start of the allocation to free. This should be the address returned from a prior `alloc` call
        ///
        /// Error: Bitmap.BitmapError || VmmError
        ///     VmmError.NotAllocated - This address hasn't been allocated yet
        ///     Bitmap.BitmapError.OutOfBounds - The address is out of the manager's bounds
        ///
        pub fn free(self: *Self, vaddr: usize) (bitmap.BitmapError || VmmError)!void {
            const entry = (vaddr - self.start) / BLOCK_SIZE;
            if (try self.bmp.isSet(entry)) {
                // There will be an allocation associated with this virtual address
                const allocation = self.allocations.get(vaddr).?;
                const physical = allocation.physical;
                defer physical.deinit();
                const num_physical_allocations = physical.items.len;
                for (physical.items, 0..) |block, i| {
                    // Clear the address space entry and free the physical memory
                    try self.bmp.clearEntry(entry + i);
                    pmm.free(block) catch |e| {
                        vga.print("Failed to free PMM reserved memory at 0x{X}: {}\n", .{ block * BLOCK_SIZE, e });
                        @panic("===========\n");
                    };
                }
                // Unmap the entire range
                const region_start = vaddr;
                const region_end = vaddr + (num_physical_allocations * BLOCK_SIZE);
                self.mapper.unmapFn(region_start, region_end, self.allocator, self.payload) catch |e| {
                    vga.print(@errorReturnTrace(), "Failed to unmap VMM reserved memory from 0x{X} to 0x{X}: {}\n", .{ region_start, region_end, e });
                    @panic("===========\n");
                };
                // The allocation is freed so remove from the map
                assert(self.allocations.remove(vaddr));
            } else {
                return VmmError.NotAllocated;
            }
        }
};

pub const VMM_MAPPER = Mapper{ .mapFn = paging.map, .unmapFn = paging.unmap };
pub const KERNEL_VMM_PAYLOAD = &paging.kernel_directory;


///
/// Initialise the main system virtual memory manager covering 4GB. Maps in the kernel code and reserved virtual memory
///
/// Arguments:
///     IN mem_profile: *const mem.MemProfile - The system's memory profile. This is used to find the kernel code region and boot modules
///     IN/OUT allocator: Allocator - The allocator to use when needing to allocate memory
///
/// Return: VirtualMemoryManager
///     The virtual memory manager created with all reserved virtual regions allocated
///
/// Error: Allocator.Error
///     error.OutOfMemory - The allocator cannot allocate the memory required
/// 
/// 
///
pub fn init(mem_profile: *const mem.MemProfile, allocator: Allocator) Allocator.Error!*VirtualMemoryManager {
    vga.print("Init vmm\n", .{});
    defer vga.print("Done vmm\n", .{});

    kernel_vmm = try VirtualMemoryManager.init(@intFromPtr(&KERNEL_ADDR_OFFSET),
     0xFFFFFFFF, allocator,
      VMM_MAPPER, KERNEL_VMM_PAYLOAD);

    // Map all the reserved virtual addresses.
    for (mem_profile.virtual_reserved) |entry| {
        const virtual = mem.Range{
            .start = std.mem.alignBackward(usize, entry.virtual.start, BLOCK_SIZE),
            .end = std.mem.alignForward(usize, entry.virtual.end, BLOCK_SIZE),
        };
        const physical: ?mem.Range = if (entry.physical) |phys|
            mem.Range{
                .start = std.mem.alignBackward(usize, phys.start, BLOCK_SIZE),
                .end = std.mem.alignForward(usize, phys.end, BLOCK_SIZE),
            }
        else
            null;
        kernel_vmm.set(
            virtual,
            physical,
             .{ .kernel = true, .writable = true, .cachable = true }) catch |e| switch (e) {
            VmmError.AlreadyAllocated => {},
            else => {
                vga.print("Failed mapping region in VMM {any}: {}\n", .{ entry, e });
                @panic("============\n");
            },
        };
    }

    return &kernel_vmm;
}
