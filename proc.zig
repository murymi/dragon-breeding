const main = @import("bug.zig");

pub fn mapKernel(proc: *Process) void {
    _ = proc;
}

    gdt_entries[0] = sys.SegmentDescriptor.zero();
    const limit = std.math.maxInt(u20);
    gdt_entries[1] = sys.SegmentDescriptor.init(0, limit, .{.code = .{}}, .{.privilege_level = 0});
    //sys.SegmentDescriptor{ .descriptor_type = .{ .code = .{} }, .flags = .{privilege_level = 0,} };
    gdt_entries[2] = sys.SegmentDescriptor.init(0, limit, .{.data = .{}}, .{.privilege_level = 0});
    //sys.SegmentDescriptor{ .descriptor_type = .{ .data = .{} }, .flags = .{.privilege_level = 0,} };

    // gdt_entries[3] = sys.SegmentDescriptor{ .descriptor_type = .{ .code = .{} }, .flags = .{
    //     .privilege_level = 3,
    // } };
    // gdt_entries[4] = sys.SegmentDescriptor{ .descriptor_type = .{ .data = .{} }, .flags = .{ .privilege_level = 3 } };
    gdt_entries[TSS_INDEX] = sys.TSSDescriptor(&main_tss_entry);









fn initDdtEntries() void {
    //var gdt_entries_temp: [NUMBER_OF_ENTRIES]GdtEntry = undefined;

    // Null descriptor
    gdt_entries[0] = sys.SegmentDescriptor.zero();
    //makeGdtEntry(0, 0, NULL_SEGMENT, NULL_FLAGS);

    const max =
    0xfffff;
    //std.math.maxInt(u20);

    // Kernel code descriptor
    gdt_entries[1] = sys.SegmentDescriptor.init(0, max, .{.code = sys.SegmentDescriptor.CodeType{
        .accessed = false,
        .conforming = false,
        .is_code = true,
        .read_enabled = true
    }}, .{
        .descriptor_type = true,
        .granularity = true,
        .present = true,
        .privilege_level = 0,
    });
    //makeGdtEntry(0, 0xFFFFF, KERNEL_SEGMENT_CODE, PAGING_32_BIT);

    // Kernel data descriptor
    gdt_entries[2] = 
    sys.SegmentDescriptor.init(0, max, .{.data = sys.SegmentDescriptor.DataType{
        .accessed = false,
        .expand_down = false,
        .is_code = false,
        .write_enabled = true
    }}, .{
        .descriptor_type = true,
        .granularity = true,
        .present = true,
        .privilege_level = 0,
    });
    //makeGdtEntry(0, 0xFFFFF, KERNEL_SEGMENT_DATA, PAGING_32_BIT);

    // User code descriptor
    //gdt_entries[3] = 
    //makeGdtEntry(0, 0xFFFFF, USER_SEGMENT_CODE, PAGING_32_BIT);

    // User data descriptor
    //gdt_entries[4] = 
    //makeGdtEntry(0, 0xFFFFF, USER_SEGMENT_DATA, PAGING_32_BIT);

    // TSS descriptor, one each for each processor
    // Will initialise the TSS at runtime
    //gdt_entries[5] = makeGdtEntry(0, 0, NULL_SEGMENT, NULL_FLAGS);

    //_= main_tss_entry;
   
   gdt_entries[TSS_INDEX] = sys.SegmentDescriptor.init(
    @intFromPtr(&main_tss_entry),  @sizeOf(Tss) - 1,
     .{.system =  sys.SegmentDescriptor.SystemType.tss32Available()}
    , .{
        .db = false,
        .descriptor_type = false,
        .privilege_level = 0,
        .present = true,
                
    });
   //makeGdtEntry(@intFromPtr(&main_tss_entry), @sizeOf(Tss) - 1, TSS_SEGMENT, NULL_FLAGS);

}





    main_tss_entry.ss0 = .{ .index = 2, .requested_privilege_level = 0, .table_indicator = .gdt };
    main_tss_entry.esp0 = @intFromPtr(&stack_top);
    main_tss_entry.cs = .{ .index = 1, .requested_privilege_level = 3, .table_indicator = .gdt };
    const ds = sys.SegmentSelector{ .index = 2, .requested_privilege_level = 3, .table_indicator = .gdt };
    main_tss_entry.ds = ds;
    main_tss_entry.fs = ds;
    main_tss_entry.ss = ds;
    main_tss_entry.es = ds;
    main_tss_entry.gs = ds;