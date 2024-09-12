const mboot = @import("multiboot.zig");
const std = @import("std");
const Allocator = std.mem.Allocator;
const mem = @import("mem.zig");
const MemProfile = mem.MemProfile;
const vga = @import("vga.zig");


/// The virtual end of the kernel code.
extern var KERNEL_VADDR_END: *u32;

/// The virtual start of the kernel code.
extern var KERNEL_VADDR_START: *u32;

/// The physical end of the kernel code.
extern var KERNEL_PHYSADDR_END: *u32;

/// The physical start of the kernel code.
extern var KERNEL_PHYSADDR_START: *u32;

/// The boot-time offset that the virtual addresses are from the physical addresses.
extern var KERNEL_ADDR_OFFSET: *u32;

/// The virtual address of the top limit of the stack.
extern var KERNEL_STACK_START: *u32;

/// The virtual address of the base of the stack.
extern var KERNEL_STACK_END: *u32;

pub fn getVideoBufferAddress() usize {
    return @intFromPtr(&KERNEL_ADDR_OFFSET) + 0xB8000;
}

///
/// Initialise the system's memory. Populates a memory profile with boot modules from grub, the amount of available memory, the reserved regions of virtual and physical memory as well as the start and end of the kernel code
///
/// Arguments:
///     IN mb_info: *multiboot.multiboot_info_t - The multiboot info passed by grub
///
/// Return: mem.MemProfile
///     The constructed memory profile
///
/// Error: Allocator.Error
///     Allocator.Error.OutOfMemory - There wasn't enough memory in the allocated created to populate the memory profile, consider increasing mem.FIXED_ALLOC_SIZE
///
pub fn initMem(mb_info: *mboot.multiboot_info_t) Allocator.Error!MemProfile {
    vga.print("Init mem\n", .{});
    defer vga.print("Done\n", .{});

    vga.print("{any}\n", .{mb_info.*});

    vga.print("KERNEL_ADDR_OFFSET:    0x{X}\n", .{@intFromPtr(&KERNEL_ADDR_OFFSET)});
    vga.print("KERNEL_STACK_START:    0x{X}\n", .{@intFromPtr(&KERNEL_STACK_START)});
    vga.print("KERNEL_STACK_END:      0x{X}\n", .{@intFromPtr(&KERNEL_STACK_END)});
    vga.print("KERNEL_VADDR_START:    0x{X}\n", .{@intFromPtr(&KERNEL_VADDR_START)});
    vga.print("KERNEL_VADDR_END:      0x{X}\n", .{@intFromPtr(&KERNEL_VADDR_END)});
    vga.print("KERNEL_PHYSADDR_START: 0x{X}\n", .{@intFromPtr(&KERNEL_PHYSADDR_START)});
    vga.print("KERNEL_PHYSADDR_END:   0x{X}\n", .{@intFromPtr(&KERNEL_PHYSADDR_END)});

    const mods_count = mb_info.mods_count;
    mem.ADDR_OFFSET = @intFromPtr(&KERNEL_ADDR_OFFSET);
    const mmap_addr = mb_info.mmap_addr;
    const num_mmap_entries = mb_info.mmap_length / @sizeOf(multiboot.multiboot_memory_map_t);


    const allocator = mem.fixed_buffer_allocator.allocator();
    var reserved_physical_mem = std.ArrayList(mem.Range).init(allocator);
    var reserved_virtual_mem = std.ArrayList(mem.Map).init(allocator);

    //const a = 
    //@as(*multiboot.multiboot_memory_map_t, @ptrFromInt(mmap_addr));
    //_= a;

    //vga.print("MMAP address: {} align:{}\n", .{mmap_addr, @alignOf(multiboot.multiboot_memory_map_t)});

    //const mem_map_ptr:[*]multiboot.multiboot_memory_map_t = @ptrCast(
    //@alignCast(
    //    @as([*]multiboot.multiboot_memory_map_t ,@ptrCast(mmap_addr))
    //    )
    //);

    //const mapone:multiboot.multiboot_memory_map_t  = @bitCast(mmap_addr.*);
    //vga.print("-----------{}------{any}-------\n", .{num_mmap_entries, mapone});


    var mmap_address:[*]multiboot.multiboot_memory_map_t = @ptrFromInt(mmap_addr);
    const mem_map = mmap_address[0..num_mmap_entries];

    // Reserve the unavailable sections from the multiboot memory map
    for (mem_map) |entry| {

        //const curr_ptr: *[24]u8 = @ptrFromInt(mmap_address);

            //const entry:multiboot.multiboot_memory_map_t  = @bitCast(curr_ptr.*);
    vga.print("-----------{}------{any}-------\n", .{num_mmap_entries, entry});


        if (entry.@"type" != multiboot.MULTIBOOT_MEMORY_AVAILABLE) {
            //vga.print("mem not avilable\n", .{});
            //If addr + len is greater than maxInt(usize) just ignore whatever comes after maxInt(usize) since it can't be addressed anyway
            const end: usize = if (entry.addr > std.math.maxInt(usize) - entry.len) std.math.maxInt(usize) 
            else @intCast(entry.addr + entry.len);
            try reserved_physical_mem.append(.{
                .start = @intCast(entry.addr),
                .end = end,
            });
        }

        //mmap_address += 24;
    }
//vga.print("=================0000=============\n", .{});
    // Map the kernel code
    const kernel_virt = mem.Range{
        .start = @intFromPtr(&KERNEL_VADDR_START),
        .end = @intFromPtr(&KERNEL_STACK_START),
    };
    const kernel_phy = mem.Range{
        .start = mem.virtToPhys(kernel_virt.start),
        .end = mem.virtToPhys(kernel_virt.end),
    };
    try reserved_virtual_mem.append(.{
        .virtual = kernel_virt,
        .physical = kernel_phy,
    });

    // Map the multiboot info struct itself
    const mb_region = mem.Range{
        .start = @intFromPtr(mb_info),
        .end = @intFromPtr(mb_info) + @sizeOf(multiboot.multiboot_info_t),
    };
    const mb_physical = mem.Range{
        .start = mem.virtToPhys(mb_region.start),
        .end = mem.virtToPhys(mb_region.end),
    };
    try reserved_virtual_mem.append(.{
        .virtual = mb_region,
        .physical = mb_physical,
    });

    // Map the tty buffer
    const tty_addr = mem.virtToPhys(getVideoBufferAddress());
    const tty_region = mem.Range{
        .start = tty_addr,
        .end = tty_addr + 32 * 1024,
    };
    try reserved_virtual_mem.append(.{
        .physical = tty_region,
        .virtual = .{
            .start = mem.physToVirt(tty_region.start),
            .end = mem.physToVirt(tty_region.end),
        },
    });

    // Map the boot modules
    const boot_modules_ptr:[*]multiboot.multiboot_mod_list = @ptrFromInt(mem.physToVirt(mb_info.mods_addr));
    //[0..mods_count];
    const boot_modules = boot_modules_ptr[0..mods_count];
    var modules = std.ArrayList(mem.Module).init(allocator);
    for (boot_modules) |module| {
        const virtual = mem.Range{
            .start = mem.physToVirt(module.mod_start),
            .end = mem.physToVirt(module.mod_end),
        };
        const physical = mem.Range{
            .start = module.mod_start,
            .end = module.mod_end,
        };
        try modules.append(.{
            .region = virtual,
            .name = std.mem.span(mem.physToVirt(@as([*:0]u8,@ptrFromInt(module.cmdline)))),
        });
        try reserved_virtual_mem.append(.{
            .physical = physical,
            .virtual = virtual,
        });
    }

    // Map the kernel stack
    const kernel_stack_virt = mem.Range{
        .start = @intFromPtr(&KERNEL_STACK_START),
        .end = @intFromPtr(&KERNEL_STACK_END),
    };
    const kernel_stack_phy = mem.Range{
        .start = mem.virtToPhys(kernel_stack_virt.start),
        .end = mem.virtToPhys(kernel_stack_virt.end),
    };
    try reserved_virtual_mem.append(.{
        .virtual = kernel_stack_virt,
        .physical = kernel_stack_phy,
    });

    return MemProfile{
        .vaddr_end = @ptrCast(&KERNEL_VADDR_END),
        .vaddr_start = @ptrCast(&KERNEL_VADDR_START),
        .physaddr_end = @ptrCast(&KERNEL_PHYSADDR_END),
        .physaddr_start = @ptrCast(&KERNEL_PHYSADDR_START),
        // Total memory available including the initial 1MiB that grub doesn't include
        .mem_kb = mb_info.mem_upper + mb_info.mem_lower + 1024,
        .modules = modules.items,
        .physical_reserved = reserved_physical_mem.items,
        .virtual_reserved = reserved_virtual_mem.items,
        .fixed_allocator = mem.fixed_buffer_allocator,
    };
}


//const std = @import("std");
//const testing = std.testing;
//const expectEqual = testing.expectEqual;
//const expect = testing.expect;
//const log = std.log.scoped(.x86_paging);
//const builtin = @import("builtin");
//const is_test = builtin.is_test;
//const panic = @import("../../panic.zig").panic;
//const build_options = @import("build_options");
//const arch = if (builtin.is_test) @import("../../../../test/mock/kernel/arch_mock.zig") else @import("arch.zig");
//const isr = @import("isr.zig");
//const MemProfile = @import("../../mem.zig").MemProfile;
//const tty = @import("../../tty.zig");
//const mem = @import("../../mem.zig");
//const vmm = @import("../../vmm.zig");
//const pmm = @import("../../pmm.zig");
const multiboot = @import("multiboot.zig");
//const Allocator = std.mem.Allocator;
const vmm = @import("vmm.zig");

/// An array of directory entries and page tables. Forms the first level of paging and covers the entire 4GB memory space.
pub const Directory = struct {
    /// The directory entries.
    entries: [ENTRIES_PER_DIRECTORY]DirectoryEntry,

    /// The tables allocated for the directory. This is ignored by the CPU.
    tables: [ENTRIES_PER_DIRECTORY]?*Table,

    ///
    /// Copy the page directory. Changes to one copy will not affect the other
    ///
    /// Arguments:
    ///     IN self: *const Directory - The directory to copy
    ///
    /// Return: Directory
    ///     The copy
    ///
    pub fn copy(self: *const Directory) Directory {
        return self.*;
    }
};

/// An array of table entries. Forms the second level of paging and covers a 4MB memory space.
const Table = struct {
    /// The table entries.
    entries: [ENTRIES_PER_TABLE]TableEntry,
};

/// An entry within a directory. References a single page table.
/// Bit 0: Present. Set if present in physical memory.
///        When not set, all remaining 31 bits are ignored and available for use.
/// Bit 1: Writable. Set if writable.
/// Bit 2: User. Set if accessible by user mode.
/// Bit 3: Write through. Set if write-through caching is enabled.
/// Bit 4: Cache disabled. Set if caching is disabled for this table.
/// Bit 5: Accessed. Set by the CPU when the table is accessed. Not cleared by CPU.
/// Bit 6: Zero.
/// Bit 7: Page size. Set if this entry covers a single 4MB page rather than 1024 4KB pages.
/// Bit 8: Ignored.
/// Bits 9-11: Ignored and available for use by kernel.
/// Bits 12-31: The 4KB aligned physical address of the corresponding page table.
///             Must be 4MB aligned if the page size bit is set.
const DirectoryEntry = u32;

/// An entry within a page table. References a single page.
/// Bit 0: Present. Set if present in physical memory.
///        When not set, all remaining 31 bits are ignored and available for use.
/// Bit 1: Writable. Set if writable.
/// Bit 2: User. Set if accessible by user mode.
/// Bit 3: Write through. Set if write-through caching is enabled.
/// Bit 4: Cache disabled. Set if caching is disabled for this page.
/// Bit 5: Accessed. Set by the CPU when the page is accessed. Not cleared by CPU.
/// Bit 6: Dirty. Set by the CPU when the page has been written to. Not cleared by the CPU.
/// Bit 7: Zero.
/// Bit 8: Global. Set if the cached address for this page shouldn't be updated when cr3 is changed.
/// Bits 9-11: Ignored and available for use by the kernel.
/// Bits 12-31: The 4KB aligned physical address mapped to this page.
const TableEntry = u32;

/// Each directory has 1024 entries
const ENTRIES_PER_DIRECTORY: u32 = 1024;

/// Each table has 1024 entries
const ENTRIES_PER_TABLE: u32 = 1024;

/// There are 1024 entries per directory with each one covering 4KB
const PAGES_PER_DIR_ENTRY: u32 = 1024;

/// There are 1 million pages per directory
const PAGES_PER_DIR: u32 = ENTRIES_PER_DIRECTORY * PAGES_PER_DIR_ENTRY;

/// The bitmasks for the bits in a DirectoryEntry
const DENTRY_PRESENT: u32 = 0x1;
const DENTRY_WRITABLE: u32 = 0x2;
const DENTRY_USER: u32 = 0x4;
const DENTRY_WRITE_THROUGH: u32 = 0x8;
const DENTRY_CACHE_DISABLED: u32 = 0x10;
const DENTRY_ACCESSED: u32 = 0x20;
const DENTRY_ZERO: u32 = 0x40;
const DENTRY_4MB_PAGES: u32 = 0x80;
const DENTRY_IGNORED: u32 = 0x100;
const DENTRY_AVAILABLE: u32 = 0xE00;
const DENTRY_PAGE_ADDR: u32 = 0xFFFFF000;

/// The bitmasks for the bits in a TableEntry
const TENTRY_PRESENT: u32 = 0x1;
const TENTRY_WRITABLE: u32 = 0x2;
const TENTRY_USER: u32 = 0x4;
const TENTRY_WRITE_THROUGH: u32 = 0x8;
const TENTRY_CACHE_DISABLED: u32 = 0x10;
const TENTRY_ACCESSED: u32 = 0x20;
const TENTRY_DIRTY: u32 = 0x40;
const TENTRY_ZERO: u32 = 0x80;
const TENTRY_GLOBAL: u32 = 0x100;
const TENTRY_AVAILABLE: u32 = 0xE00;
const TENTRY_PAGE_ADDR: u32 = 0xFFFFF000;

/// The number of bytes in 4MB
pub const PAGE_SIZE_4MB: usize = 0x400000;

/// The number of bytes in 4KB
pub const PAGE_SIZE_4KB: usize = PAGE_SIZE_4MB / 1024;

/// The kernel's page directory. Should only be used to map kernel-owned code and data
pub var kernel_directory: Directory align(@truncate(PAGE_SIZE_4KB)) = Directory{ 
    .entries = [_]DirectoryEntry{0} ** ENTRIES_PER_DIRECTORY,
    .tables = [_]?*Table{null} ** ENTRIES_PER_DIRECTORY 
};

///
/// Convert a virtual address to an index within an array of directory entries.
///
/// Arguments:
///     IN virt: usize - The virtual address to convert.
///
/// Return: usize
///     The index into an array of directory entries.
///
inline fn virtToDirEntryIdx(virt: usize) usize {
    return virt / PAGE_SIZE_4MB;
}

///
/// Convert a virtual address to an index within an array of table entries.
///
/// Arguments:
///     IN virt: usize - The virtual address to convert.
///
/// Return: usize
///     The index into an array of table entries.
///
inline fn virtToTableEntryIdx(virt: usize) usize {
    return (virt / PAGE_SIZE_4KB) % ENTRIES_PER_TABLE;
}

///
/// Set the bit(s) associated with an attribute of a table or directory entry.
///
/// Arguments:
///     val: *align(1) u32 - The entry to modify
///     attr: u32 - The bits corresponding to the attribute to set
///
inline fn setAttribute(val: *align(1) u32, attr: u32) void {
    val.* |= attr;
}

///
/// Clear the bit(s) associated with an attribute of a table or directory entry.
///
/// Arguments:
///     val: *align(1) u32 - The entry to modify
///     attr: u32 - The bits corresponding to the attribute to clear
///
inline fn clearAttribute(val: *align(1) u32, attr: u32) void {
    val.* &= ~attr;
}

///
/// Map a page directory entry, setting the present, size, writable, write-through and physical address bits.
/// Clears the user and cache disabled bits. Entry should be zeroed.
///
/// Arguments:
///     IN virt_addr: usize - The start of the virtual space to map
///     IN virt_end: usize - The end of the virtual space to map
///     IN phys_addr: usize - The start of the physical space to map
///     IN phys_end: usize - The end of the physical space to map
///     IN attrs: vmm.Attributes - The attributes to apply to this mapping
///     IN allocator: Allocator - The allocator to use to map any tables needed
///     OUT dir: *Directory - The directory that this entry is in
///
/// Error: vmm.MapperError || Allocator.Error
///     vmm.MapperError.InvalidPhysicalAddress - The physical start address is greater than the end
///     vmm.MapperError.InvalidVirtualAddress - The virtual start address is greater than the end or is larger than 4GB
///     vmm.MapperError.AddressMismatch - The differences between the virtual addresses and the physical addresses aren't the same
///     vmm.MapperError.MisalignedPhysicalAddress - One or both of the physical addresses aren't page size aligned
///     vmm.MapperError.MisalignedVirtualAddress - One or both of the virtual addresses aren't page size aligned
///     Allocator.Error.* - See Allocator.alignedAlloc
///
fn mapDirEntry(dir: *Directory, virt_start: usize, virt_end: usize, phys_start: usize, phys_end: usize, attrs: vmm.Attributes, allocator: Allocator) (vmm.MapperError || Allocator.Error)!void {
    if (phys_start > phys_end) {
        return vmm.MapperError.InvalidPhysicalAddress;
    }
    if (virt_start > virt_end) {
        return vmm.MapperError.InvalidVirtualAddress;
    }
    if (phys_end - phys_start != virt_end - virt_start) {
        return vmm.MapperError.AddressMismatch;
    }
    if (!std.mem.isAligned(phys_start, PAGE_SIZE_4KB) or !std.mem.isAligned(phys_end, PAGE_SIZE_4KB)) {
        return vmm.MapperError.MisalignedPhysicalAddress;
    }
    if (!std.mem.isAligned(virt_start, PAGE_SIZE_4KB) or !std.mem.isAligned(virt_end, PAGE_SIZE_4KB)) {
        return vmm.MapperError.MisalignedVirtualAddress;
    }

    const entry = virtToDirEntryIdx(virt_start);
    const dir_entry = &dir.entries[entry];

    // Only create a new table if one hasn't already been created for this dir entry.
    // Prevents us from overriding previous mappings.
    var table: *Table = undefined;
    if (dir.tables[entry]) |tbl| {
        table = tbl;
    } else {
        // Create a table and put the physical address in the dir entry
        table = &(try allocator.alignedAlloc(Table, @truncate(PAGE_SIZE_4KB), 1))[0];
        //@memset(table, 0);
        table.* = std.mem.zeroes(Table);
        const table_phys_addr = 
        vmm.kernel_vmm.virtToPhys(@intFromPtr(table)) catch |e| {
            vga.print("Failed getting the physical address for a page table: {}\n", .{e});
            @panic("==map dir entry==");
        };
        dir_entry.* |= DENTRY_PAGE_ADDR & table_phys_addr;
        dir.tables[entry] = table;
    }

    setAttribute(dir_entry, DENTRY_PRESENT);
    setAttribute(dir_entry, DENTRY_WRITE_THROUGH);
    clearAttribute(dir_entry, DENTRY_4MB_PAGES);

    if (attrs.writable) {
        setAttribute(dir_entry, DENTRY_WRITABLE);
    } else {
        clearAttribute(dir_entry, DENTRY_WRITABLE);
    }

    if (attrs.kernel) {
        clearAttribute(dir_entry, DENTRY_USER);
    } else {
        setAttribute(dir_entry, DENTRY_USER);
    }

    if (attrs.cachable) {
        clearAttribute(dir_entry, DENTRY_CACHE_DISABLED);
    } else {
        setAttribute(dir_entry, DENTRY_CACHE_DISABLED);
    }

    // Map the table entries within the requested space
    var virt = virt_start;
    var phys = phys_start;
    var tentry = virtToTableEntryIdx(virt);
    while (virt < virt_end) : ({
        virt += PAGE_SIZE_4KB;
        phys += PAGE_SIZE_4KB;
        tentry += 1;
    }) {
        try mapTableEntry(dir, &table.entries[tentry], virt, phys, attrs);
    }
}

///
/// Unmap a page directory entry, clearing the present bits.
///
/// Arguments:
///     IN virt_addr: usize - The start of the virtual space to map
///     IN virt_end: usize - The end of the virtual space to map
///     OUT dir: *Directory - The directory that this entry is in
///     IN allocator: Allocator - The allocator used to map the region to be freed.
///
/// Error: vmm.MapperError
///     vmm.MapperError.NotMapped - If the region being unmapped wasn't mapped in the first place
///
fn unmapDirEntry(dir: *Directory, virt_start: usize, virt_end: usize, allocator: Allocator) vmm.MapperError!void {
    // Suppress unused var warning
    _ = allocator;
    const entry = virtToDirEntryIdx(virt_start);
    const table = dir.tables[entry] orelse return vmm.MapperError.NotMapped;
    var addr = virt_start;
    while (addr < virt_end) : (addr += PAGE_SIZE_4KB) {
        const table_entry = &table.entries[virtToTableEntryIdx(addr)];
        if (table_entry.* & TENTRY_PRESENT != 0) {
            clearAttribute(table_entry, TENTRY_PRESENT);
            if (dir == &kernel_directory) {
                asm volatile ("invlpg (%[addr])"
                    :
                    : [addr] "r" (addr),
                    : "memory"
                );
            }
        } else {
            return vmm.MapperError.NotMapped;
        }
    }
}

///
/// Map a table entry by setting its bits to the appropriate values.
/// Sets the entry to be present, writable, kernel access, write through, cache enabled, non-global and the page address bits.
///
/// Arguments:
///     IN dir: *const Directory - The directory that is being mapped within.
///                           The function checks if this is the kernel directory and if so invalidates the page being mapped so the TLB reloads it.
///     OUT entry: *align(1) TableEntry - The entry to map. 1 byte aligned.
///     IN virt_addr: usize - The virtual address that this table entry is responsible for.
///                           Used to invalidate the page if mapping within the kernel page directory.
///     IN phys_addr: usize - The physical address to map the table entry to.
///
/// Error: PagingError
///     PagingError.UnalignedPhysAddresses - If the physical address isn't page size aligned.
///
fn mapTableEntry(dir: *const Directory, entry: *align(1) TableEntry, virt_addr: usize, phys_addr: usize, attrs: vmm.Attributes) vmm.MapperError!void {
    if (!std.mem.isAligned(phys_addr, PAGE_SIZE_4KB)) {
        return vmm.MapperError.MisalignedPhysicalAddress;
    }
    setAttribute(entry, TENTRY_PRESENT);
    if (attrs.writable) {
        setAttribute(entry, TENTRY_WRITABLE);
    } else {
        clearAttribute(entry, TENTRY_WRITABLE);
    }
    if (attrs.kernel) {
        clearAttribute(entry, TENTRY_USER);
    } else {
        setAttribute(entry, TENTRY_USER);
    }

    if (attrs.cachable) {
        clearAttribute(entry, TENTRY_WRITE_THROUGH);
        clearAttribute(entry, TENTRY_CACHE_DISABLED);
    } else {
        setAttribute(entry, TENTRY_WRITE_THROUGH);
        setAttribute(entry, TENTRY_CACHE_DISABLED);
    }

    clearAttribute(entry, TENTRY_GLOBAL);
    setAttribute(entry, TENTRY_PAGE_ADDR & phys_addr);
    if (dir == &kernel_directory) {
        asm volatile ("invlpg (%[addr])"
            :
            : [addr] "r" (virt_addr),
            : "memory"
        );
    }
}

///
/// Map a virtual region of memory to a physical region with a set of attributes within a directory.
/// If this call is made to a directory that has been loaded by the CPU, the virtual memory will immediately be accessible (given the proper attributes)
/// and will be mirrored to the physical region given. Otherwise it will be accessible once the given directory is loaded by the CPU.
///
/// This call will panic if mapDir returns an error when called with any of the arguments given.
///
/// Arguments:
///     IN virtual_start: usize - The start of the virtual region to map
///     IN virtual_end: usize - The end (exclusive) of the virtual region to map
///     IN physical_start: usize - The start of the physical region to map to
///     IN physical_end: usize - The end (exclusive) of the physical region to map to
///     IN attrs: vmm.Attributes - The attributes to apply to this mapping
///     IN/OUT allocator: Allocator - The allocator to use to allocate any intermediate data structures required to map this region
///     IN/OUT dir: *Directory - The page directory to map within
///
/// Error: vmm.MapperError || Allocator.Error
///     * - See mapDirEntry
///
pub fn map(virtual_start: usize, virtual_end: usize, phys_start: usize, phys_end: usize, attrs: vmm.Attributes, allocator: Allocator, dir: *Directory) (Allocator.Error || vmm.MapperError)!void {
    var virt_addr = virtual_start;
    var phys_addr = phys_start;
    var virt_next = @min(virtual_end, std.mem.alignBackward(usize, virt_addr, PAGE_SIZE_4MB) + PAGE_SIZE_4MB);
    var phys_next = @min(phys_end, std.mem.alignBackward(usize, phys_addr, PAGE_SIZE_4MB) + PAGE_SIZE_4MB);
    var entry_idx = virtToDirEntryIdx(virt_addr);
    while (entry_idx < ENTRIES_PER_DIRECTORY and virt_addr < virtual_end) : ({
        virt_addr = virt_next;
        phys_addr = phys_next;
        virt_next = @min(virtual_end, virt_next + PAGE_SIZE_4MB);
        phys_next = @min(phys_end, phys_next + PAGE_SIZE_4MB);
        entry_idx += 1;
    }) {
        try mapDirEntry(dir, virt_addr, virt_next, phys_addr, phys_next, attrs, allocator);
    }
}

///
/// Unmap a virtual region of memory within a directory so that it is no longer accessible.
///
/// Arguments:
///     IN virtual_start: usize - The start of the virtual region to unmap
///     IN virtual_end: usize - The end (exclusive) of the virtual region to unmap
///     IN/OUT dir: *Directory - The page directory to unmap within
///
/// Error: vmm.MapperError
///     vmm.MapperError.NotMapped - If the region being unmapped wasn't mapped in the first place
///
pub fn unmap(virtual_start: usize, virtual_end: usize, allocator: Allocator, dir: *Directory) vmm.MapperError!void {
    var virt_addr = virtual_start;
    var virt_next = @min(virtual_end, std.mem.alignBackward(usize, virt_addr, PAGE_SIZE_4MB) + PAGE_SIZE_4MB);
    var entry_idx = virtToDirEntryIdx(virt_addr);
    while (entry_idx < ENTRIES_PER_DIRECTORY and virt_addr < virtual_end) : ({
        virt_addr = virt_next;
        virt_next = @min(virtual_end, virt_next + PAGE_SIZE_4MB);
        entry_idx += 1;
    }) {
        try unmapDirEntry(dir, virt_addr, virt_next, allocator);
        if (std.mem.isAligned(virt_addr, PAGE_SIZE_4MB) and virt_next - virt_addr >= PAGE_SIZE_4MB) {
            clearAttribute(&dir.entries[entry_idx], DENTRY_PRESENT);

            const table = dir.tables[entry_idx] orelse return vmm.MapperError.NotMapped;
            const table_free:[*]Table = @ptrCast(table) ;
            //[0..1];
            allocator.free(table_free[0..1]);
        }
    }
}

const interrupts = @import("interrupts.zig");

///
/// Called when a page fault occurs.
/// This will log the CPU state and control registers as well as some human-readable information.
///
/// Arguments:
///     IN state: *arch.CpuState - The CPU's state when the fault occurred.
///
pub fn pageFault(state: *interrupts.CpuState) u32 {
    const err = state.error_code;
    const diag_present = if (err & 0b1 != 0) "present" else "non-present";
    const diag_rw = if (err & 0b10 != 0) "writing to" else "reading from";
    const diag_ring = if (err & 0b100 != 0) "user" else "kernel";
    const diag_reserved = if (err & 0b1000 != 0) " with reserved bit set" else "";
    const diag_fetch = if (err & 0b10000 != 0) "instruction" else "data";
    vga.print("Page fault: {s} process {s} a {s} page during {s} fetch{s}\n", .{ diag_ring, diag_rw, diag_present, diag_fetch, diag_reserved });
    const cr0 = asm volatile ("mov %%cr0, %[cr0]"
        : [cr0] "=r" (-> u32),
    );
    const cr2 = asm volatile ("mov %%cr2, %[cr2]"
        : [cr2] "=r" (-> u32),
    );
    const cr3 = asm volatile ("mov %%cr3, %[cr3]"
        : [cr3] "=r" (-> u32),
    );
    const cr4 = asm volatile ("mov %%cr4, %[cr4]"
        : [cr4] "=r" (-> u32),
    );
    vga.print("CR0: 0x{X}, CR2/address: 0x{X}, CR3: 0x{X}, CR4: 0x{X}, EIP: 0x{X}\n", .{ cr0, cr2, cr3, cr4, state.eip });
    vga.print("State: {any}\n", .{state});
    @panic("Page fault");
}

///
/// Initialise x86 paging, overwriting any previous paging set up.
///
/// Arguments:
///     IN mem_profile: *const MemProfile - The memory profile of the system and kernel
///
pub fn init(mem_profile: *const MemProfile) void {
    _ = mem_profile;
    vga.print("Init paging\n", .{});
    defer vga.print("Done\n", .{});

    //isr.registerIsr(isr.PAGE_FAULT, if (build_options.test_mode == .Initialisation) rt_pageFault else pageFault) catch |e| {
    //    panic(@errorReturnTrace(), "Failed to register page fault ISR: {}\n", .{e});
    //};
    const dir_physaddr = @intFromPtr(mem.virtToPhys(&kernel_directory.entries));
    vga.print("0x{x}\n", .{dir_physaddr});
    asm volatile ("mov %[addr], %%cr3"
        :
        : [addr] "{eax}" (dir_physaddr),
    );

    //while (true) {}
    //const v_end = std.mem.alignForward(@intFromPtr(mem_profile.vaddr_end), PAGE_SIZE_4KB);
    //switch (build_options.test_mode) {
    //    .Initialisation => runtimeTests(v_end),
    //    else => {},
    //}
}

//fn checkDirEntry(entry: DirectoryEntry, virt_start: usize, virt_end: usize, phys_start: usize, attrs: vmm.Attributes, table: *Table, present: bool) !void {
//    try expectEqual(entry & DENTRY_PRESENT, if (present) DENTRY_PRESENT else 0);
//    try expectEqual(entry & DENTRY_WRITABLE, if (attrs.writable) DENTRY_WRITABLE else 0);
//    try expectEqual(entry & DENTRY_USER, if (attrs.kernel) 0 else DENTRY_USER);
//    try expectEqual(entry & DENTRY_WRITE_THROUGH, DENTRY_WRITE_THROUGH);
//    try expectEqual(entry & DENTRY_CACHE_DISABLED, if (attrs.cachable) 0 else DENTRY_CACHE_DISABLED);
//    try expectEqual(entry & DENTRY_4MB_PAGES, 0);
//    try expectEqual(entry & DENTRY_ZERO, 0);
//
//    var tentry_idx = virtToTableEntryIdx(virt_start);
//    var tentry_idx_end = virtToTableEntryIdx(virt_end);
//    var phys = phys_start;
//    while (tentry_idx < tentry_idx_end) : ({
//        tentry_idx += 1;
//        phys += PAGE_SIZE_4KB;
//    }) {
//        const tentry = table.entries[tentry_idx];
//        try checkTableEntry(tentry, phys, attrs, present);
//    }
//}

//fn checkTableEntry(entry: TableEntry, page_phys: usize, attrs: vmm.Attributes, present: bool) !void {
//    try expectEqual(entry & TENTRY_PRESENT, if (present) TENTRY_PRESENT else 0);
//    try expectEqual(entry & TENTRY_WRITABLE, if (attrs.writable) TENTRY_WRITABLE else 0);
//    try expectEqual(entry & TENTRY_USER, if (attrs.kernel) 0 else TENTRY_USER);
//    try expectEqual(entry & TENTRY_WRITE_THROUGH, TENTRY_WRITE_THROUGH);
//    try expectEqual(entry & TENTRY_CACHE_DISABLED, if (attrs.cachable) 0 else TENTRY_CACHE_DISABLED);
//    try expectEqual(entry & TENTRY_ZERO, 0);
//    try expectEqual(entry & TENTRY_GLOBAL, 0);
//    try expectEqual(entry & TENTRY_PAGE_ADDR, page_phys);
//}