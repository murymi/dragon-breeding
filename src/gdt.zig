const std = @import("std");
const vga = @import("vga.zig");
//const expect = std.testing.expect;
//const expectEqual = std.testing.expectEqual;
//const log = std.log.scoped(.x86_gdt);
//const builtin = @import("builtin");
//const is_test = builtin.is_test;
//const panic = @import("../../panic.zig").panic;
//const build_options = @import("build_options");

/// The access bits for a GDT entry.
const AccessBits = packed struct {
    /// Whether the segment has been access. This shouldn't be set as it is set by the CPU when the
    /// segment is accessed.
    accessed: u1 = 0,

    /// For code segments, when set allows the code segment to be readable. Code segments are
    /// always executable. For data segments, when set allows the data segment to be writeable.
    /// Data segments are always readable.
    read_write: u1 = 0,

    /// For code segments, when set allows this code segments to be executed from a equal or lower
    /// privilege level. The privilege bits represent the highest privilege level that is allowed
    /// to execute this segment. If not set, then the code segment can only be executed from the
    /// same ring level specified in the privilege level bits. For data segments, when set the data
    /// segment grows downwards. When not set, the data segment grows upwards. So for both code and
    /// data segments, this shouldn't be set.
    direction_conforming: u1 = 0,

    /// When set, the segment can be executed, a code segments. When not set, the segment can't be
    /// executed, data segment.
    executable: u1 = 0,

    /// Should be set for code and data segments, but not set for TSS.
    descriptor: u1 = 0,

    /// Privilege/ring level. The kernel level is level 3, the highest privilege. The user level is
    /// level 0, the lowest privilege.
    privilege: u2 = 0,

    /// Whether the segment is present. This must be set for all valid selectors, not the null
    /// segment.
    present: u1 = 0,
};

/// The flag bits for a GDT entry.
const FlagBits = packed struct {
    /// The lowest bits must be 0 as this is reserved for future use.
    reserved_zero: u1 = 0,

    /// When set indicates the segment is a x86-64 segment. If set, then the IS_32_BIT flag must
    /// not be set. If both are set, then will throw an exception.
    is_64_bit: u1 = 0,

    /// When set indicates the segment is a 32 bit protected mode segment. When not set, indicates
    /// the segment is a 16 bit protected mode segment.
    is_32_bit: u1 = 0,

    /// The granularity bit. When set the limit is in 4KB blocks (page granularity). When not set,
    /// then limit is in 1B blocks (byte granularity). This should be set as we are doing paging.
    granularity: u1 = 0,
};

/// The structure that contains all the information that each GDT entry needs.
const GdtEntry = packed struct {
    /// The lower 16 bits of the limit address. Describes the size of memory that can be addressed.
    limit_low: u16,

    /// The lower 24 bits of the base address. Describes the start of memory for the entry.
    base_low: u24,

    /// The access bits, see AccessBits for all the options. 8 bits.
    access: AccessBits,

    /// The upper 4 bits of the limit address. Describes the size of memory that can be addressed.
    limit_high: u4,

    /// The flag bits, see above for all the options. 4 bits.
    flags: FlagBits,

    /// The upper 8 bits of the base address. Describes the start of memory for the entry.
    base_high: u8,
};

/// The TSS entry structure
pub const Tss = packed struct {
    /// Pointer to the previous TSS entry
    prev_tss: u16,
    reserved1: u16,

    /// Ring 0 32 bit stack pointer.
    esp0: u32,

    /// Ring 0 32 bit stack pointer.
    ss0: u16,
    reserved2: u16,

    /// Ring 1 32 bit stack pointer.
    esp1: u32,

    /// Ring 1 32 bit stack pointer.
    ss1: u16,
    reserved3: u16,

    /// Ring 2 32 bit stack pointer.
    esp2: u32,

    /// Ring 2 32 bit stack pointer.
    ss2: u16,
    reserved4: u16,

    /// The CR3 control register 3.
    cr3: u32,

    /// 32 bit instruction pointer.
    eip: u32,

    /// 32 bit flags register.
    eflags: u32,

    /// 32 bit accumulator register.
    eax: u32,

    /// 32 bit counter register.
    ecx: u32,

    /// 32 bit data register.
    edx: u32,

    /// 32 bit base register.
    ebx: u32,

    /// 32 bit stack pointer register.
    esp: u32,

    /// 32 bit base pointer register.
    ebp: u32,

    /// 32 bit source register.
    esi: u32,

    /// 32 bit destination register.
    edi: u32,

    /// The extra segment.
    es: u16,
    reserved5: u16,

    /// The code segment.
    cs: u16,
    reserved6: u16,

    /// The stack segment.
    ss: u16,
    reserved7: u16,

    /// The data segment.
    ds: u16,
    reserved8: u16,

    /// A extra segment FS.
    fs: u16,
    reserved9: u16,

    /// A extra segment GS.
    gs: u16,
    reserved10: u16,

    /// The local descriptor table register.
    ldtr: u16,
    reserved11: u16,

    /// ?
    trap: u16,

    /// A pointer to a I/O port bitmap for the current task which specifies individual ports the program should have access to.
    io_permissions_base_offset: u16,
};

/// The GDT pointer structure that contains the pointer to the beginning of the GDT and the number
/// of the table (minus 1). Used to load the GDT with LGDT instruction.
pub const GdtPtr = packed struct {
    /// 16bit entry for the number of entries (minus 1).
    limit: u16,

    /// 32bit entry for the base address for the GDT.
    base: u32,
};

/// The total number of entries in the GDT including: null, kernel code, kernel data, user code,
/// user data and the TSS.
const NUMBER_OF_ENTRIES: u16 = 0x06;

/// The size of the GTD in bytes (minus 1).
const TABLE_SIZE: u16 = @sizeOf(GdtEntry) * NUMBER_OF_ENTRIES - 1;

// ----------
// The indexes into the GDT where each segment resides.
// ----------

/// The index of the NULL GDT entry.
const NULL_INDEX: u16 = 0x00;

/// The index of the kernel code GDT entry.
const KERNEL_CODE_INDEX: u16 = 0x01;

/// The index of the kernel data GDT entry.
const KERNEL_DATA_INDEX: u16 = 0x02;

/// The index of the user code GDT entry.
const USER_CODE_INDEX: u16 = 0x03;

/// The index of the user data GDT entry.
const USER_DATA_INDEX: u16 = 0x04;

/// The index of the task state segment GDT entry.
const TSS_INDEX: u16 = 0x05;

/// The null segment, everything is set to zero.
const NULL_SEGMENT: AccessBits = AccessBits{
    .accessed = 0,
    .read_write = 0,
    .direction_conforming = 0,
    .executable = 0,
    .descriptor = 0,
    .privilege = 0,
    .present = 0,
};

/// This bit pattern represents a kernel code segment with bits: readable, executable, descriptor,
/// privilege 0, and present set.
const KERNEL_SEGMENT_CODE: AccessBits = AccessBits{
    .accessed = 0,
    .read_write = 1,
    .direction_conforming = 0,
    .executable = 1,
    .descriptor = 1,
    .privilege = 0,
    .present = 1,
};

/// This bit pattern represents a kernel data segment with bits: writeable, descriptor, privilege 0,
/// and present set.
const KERNEL_SEGMENT_DATA: AccessBits = AccessBits{
    .accessed = 0,
    .read_write = 1,
    .direction_conforming = 0,
    .executable = 0,
    .descriptor = 1,
    .privilege = 0,
    .present = 1,
};

/// This bit pattern represents a user code segment with bits: readable, executable, descriptor,
/// privilege 3, and present set.
const USER_SEGMENT_CODE: AccessBits = AccessBits{
    .accessed = 0,
    .read_write = 1,
    .direction_conforming = 0,
    .executable = 1,
    .descriptor = 1,
    .privilege = 3,
    .present = 1,
};

/// This bit pattern represents a user data segment with bits: writeable, descriptor, privilege 3,
/// and present set.
const USER_SEGMENT_DATA: AccessBits = AccessBits{
    .accessed = 0,
    .read_write = 1,
    .direction_conforming = 0,
    .executable = 0,
    .descriptor = 1,
    .privilege = 3,
    .present = 1,
};

/// This bit pattern represents a TSS segment with bits: accessed, executable and present set.
const TSS_SEGMENT: AccessBits = AccessBits{
    .accessed = 1,
    .read_write = 0,
    .direction_conforming = 0,
    .executable = 1,
    .descriptor = 0,
    .privilege = 0,
    .present = 1,
};

/// The bit pattern for all bits set to zero.
const NULL_FLAGS: FlagBits = FlagBits{
    .reserved_zero = 0,
    .is_64_bit = 0,
    .is_32_bit = 0,
    .granularity = 0,
};

/// The bit pattern for all segments where we are in 32 bit protected mode and paging enabled.
const PAGING_32_BIT: FlagBits = FlagBits{
    .reserved_zero = 0,
    .is_64_bit = 0,
    .is_32_bit = 1,
    .granularity = 1,
};

// ----------
// The offsets into the GDT where each segment resides.
// ----------

/// The offset of the NULL GDT entry.
pub const NULL_OFFSET: u16 = 0x00;

/// The offset of the kernel code GDT entry.
pub const KERNEL_CODE_OFFSET: u16 = 0x08;

/// The offset of the kernel data GDT entry.
pub const KERNEL_DATA_OFFSET: u16 = 0x10;

/// The offset of the user code GDT entry.
pub const USER_CODE_OFFSET: u16 = 0x18;

/// The offset of the user data GDT entry.
pub const USER_DATA_OFFSET: u16 = 0x20;

/// The offset of the TTS GDT entry.
pub const TSS_OFFSET: u16 = 0x28;

/// The GDT entry table of NUMBER_OF_ENTRIES entries.
var gdt_entries: [NUMBER_OF_ENTRIES]sys.SegmentDescriptor = undefined;

const sys = @import("sys/registers.zig");


fn initDdtEntries() void {
    //var gdt_entries_temp: [NUMBER_OF_ENTRIES]GdtEntry = undefined;

    // Null descriptor
    gdt_entries[0] = sys.SegmentDescriptor.zero();
    //makeGdtEntry(0, 0, NULL_SEGMENT, NULL_FLAGS);

    //const max =
    //0xfffff;

    gdt_entries[0] = sys.SegmentDescriptor.zero();
    const limit = std.math.maxInt(u20);
    gdt_entries[1] = sys.SegmentDescriptor.init(0, limit, .{.code = .{}}, .{.privilege_level = 0});
    //sys.SegmentDescriptor{ .descriptor_type = .{ .code = .{} }, .flags = .{privilege_level = 0,} };
    gdt_entries[2] = sys.SegmentDescriptor.init(0, limit, .{.data = .{}}, .{.privilege_level = 0});

    gdt_entries[3] = sys.SegmentDescriptor.init(0, limit, .{.code = .{}}, .{.privilege_level = 3});
    gdt_entries[4] = sys.SegmentDescriptor.init(0, limit, .{.data = .{}}, .{.privilege_level = 3});

    //std.math.maxInt(u20);
   
   gdt_entries[TSS_INDEX] = sys.TSSDescriptor(&main_tss_entry);

}

/// The GDT pointer that the CPU is loaded with that contains the base address of the GDT and the
/// size.
var gdt_ptr = sys.PseudoDescriptor{
    .limit = TABLE_SIZE,
    .base = undefined,
};

/// The main task state segment entry.
pub var main_tss_entry: sys.TaskStateSegment = std.mem.zeroes(sys.TaskStateSegment);
// init: {
//     var tss_temp = std.mem.zeroes(sys.TaskStateSegment);
//     tss_temp.ss0 = @bitCast(KERNEL_DATA_OFFSET);
//     //tss_temp.io_permissions_base_offset = @sizeOf(Tss);
//     break :init tss_temp;
// };

///
/// Make a GDT entry.
///
/// Arguments:
///     IN base: u32          - The linear address where the segment begins.
///     IN limit: u20         - The maximum addressable unit whether it is 1B units or page units.
///     IN access: AccessBits - The access bits for the descriptor.
///     IN flags: FlagBits    - The flag bits for the descriptor.
///
/// Return: GdtEntry
///     A new GDT entry with the give access and flag bits set with the base at 0x00000000 and
///     limit at 0xFFFFF.
///
pub fn makeGdtEntry(base: u32, limit: u20, access: AccessBits, flags: FlagBits) GdtEntry {
    return .{
        .limit_low = @truncate(limit),
        .base_low = @truncate(base),
        .access = .{
            .accessed = access.accessed,
            .read_write = access.read_write,
            .direction_conforming = access.direction_conforming,
            .executable = access.executable,
            .descriptor = access.descriptor,
            .privilege = access.privilege,
            .present = access.present,
        },
        .limit_high = @truncate(limit >> 16),
        .flags = .{
            .reserved_zero = flags.reserved_zero,
            .is_64_bit = flags.is_64_bit,
            .is_32_bit = flags.is_32_bit,
            .granularity = flags.granularity,
        },
        .base_high = @truncate(base >> 24),
    };
}

pub var kernel_stack: [16384]u8 align(16) = undefined;
const stack_top = kernel_stack[kernel_stack.len..].ptr;

///
/// Initialise the Global Descriptor table.
///
pub fn init() void {
    initDdtEntries();
    vga.print("Init\n", .{});
    defer vga.print("Done\n", .{});
    // Initiate TSS
   // gdt_entries[TSS_INDEX] = makeGdtEntry(@intFromPtr(&main_tss_entry), @sizeOf(Tss) - 1, TSS_SEGMENT, NULL_FLAGS);

    // Set the base address where all the GDT entries are.
    gdt_ptr.base = @intFromPtr(&gdt_entries[0]);

    // Load the GDT
    lgdt(gdt_ptr);

    //sys.Gdtr.write(gdt_ptr);

    rt_loadedGDTSuccess();


    main_tss_entry.ss0 = .{ .index = 2, .requested_privilege_level = 0, .table_indicator = .gdt };
    main_tss_entry.esp0 =  //0xC0000000;
    @intFromPtr(&stack_top);
    main_tss_entry.cs = .{ .index = 1, .requested_privilege_level = 3, .table_indicator = .gdt };
    const ds = sys.SegmentSelector{ .index = 2, .requested_privilege_level = 3, .table_indicator = .gdt };
    main_tss_entry.ss = ds;
    main_tss_entry.ds = ds;
    main_tss_entry.es = ds;
    main_tss_entry.fs = ds;
    main_tss_entry.gs = ds;

    //main_tss_entry.

//const d: u16 = @bitCast(main_tss_entry.cs);
//vga.print("test: 0x{x}\n", .{d});
//sys.spinloop();
    // Load the TSS
    ltr(TSS_OFFSET);

    //switch (build_options.test_mode) {
    //    .Initialisation => runtimeTests(),
    //    else => {},
    //}
}



///
/// Load the GDT and refreshing the code segment with the code segment offset of the kernel as we
/// are still in kernel land. Also loads the kernel data segment into all the other segment
/// registers.
///
/// Arguments:
///     IN gdt_ptr: *gdt.GdtPtr - The address to the GDT.
///
pub fn lgdt(ptr: sys.PseudoDescriptor) void {
    // Load the GDT into the CPU
    asm volatile (
        \\lgdt (%%eax)
        :
        : [ptr] "{eax}" (&ptr),
    );

    // Load the kernel data segment, index into the GDT
    const selector = sys.SegmentSelector{
        .index = KERNEL_DATA_INDEX,
        .requested_privilege_level = 0,
        .table_indicator = .gdt
    };

    asm volatile ("mov %%bx, %%ds"
        :
        : [KERNEL_DATA_OFFSET] "{bx}" (selector),
    );

    asm volatile ("mov %%bx, %%es");
    asm volatile ("mov %%bx, %%fs");
    asm volatile ("mov %%bx, %%gs");
    asm volatile ("mov %%bx, %%ss");

    //const code_selector = sys.SegmentSelector{
    //    .index = KERNEL_CODE_INDEX,
    //    .requested_privilege_level = 0,
    //    .table_indicator = .gdt
    //};


    // Load the kernel code segment into the CS register
    //sys.setCodeSegment(code_selector);
    asm volatile (
        \\ pushw $8
        \\ pushl $1f
        \\ljmp *(%%esp)
        \\1:
        \\add $6, %%esp
    );
}

///
/// Tell the CPU where the TSS is located in the GDT.
///
/// Arguments:
///     IN offset: u16 - The offset in the GDT where the TSS segment is located.
///
pub fn ltr(offset: u16) void {
    asm volatile ("ltr %%ax"
        :
        : [offset] "{ax}" (offset),
    );
}


///
/// Check that the GDT table was loaded properly by getting the previously loaded table and
/// compare the limit and base address.
///
fn rt_loadedGDTSuccess() void {
    const loaded_gdt = sgdt();
    if (gdt_ptr.limit != loaded_gdt.limit) {
        vga.print("FAILURE: GDT not loaded properly: 0x{X} != 0x{X}\n", .{ gdt_ptr.limit, loaded_gdt.limit });
    }
    if (gdt_ptr.base != loaded_gdt.base) {
        vga.print("FAILURE: GDT not loaded properly: 0x{X} != {X}\n", .{ gdt_ptr.base, loaded_gdt.base });
    }
    vga.print("Tested loading GDT\n", .{});
}


///
/// Get the previously loaded GDT from the CPU.
///
/// Return: gdt.GdtPtr
///     The previously loaded GDT from the CPU.
///
pub fn sgdt() GdtPtr {
    var ptr = GdtPtr{ .limit = 0, .base = 0 };
    asm volatile ("sgdt %[tab]"
        : [tab] "=m" (ptr),
    );
    return ptr;
}