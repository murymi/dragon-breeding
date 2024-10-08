const std = @import("std");
const sys = @import("sys/registers.zig");
const mn = @import("main.zig");
const paging = @import("paging.zig");

pub const Process = struct {
    //stack: []u8,
    context: sys.cpuContext(.{}) = undefined,
    page_directory: sys.PageDirectory,
    page_tables: [1024]sys.PageTable align(4096) = undefined,

    pub fn init(
        proc: *Process,
        //allocator: std.mem.Allocator,
        //stack: []u8,
        entry: u32,
        page_allocator: *paging.PageDirectory,
        kernel_end: u32
        //ctx: sys.cpuContext(.{})
    ) !void {
        //vga.print("=============================\n", .{});

        //_ = page_allocator;

        //var proc: Process = undefined;
        //proc.context = sys.cpuContext(.{}){};
        const stack = try page_allocator.mapAnyFree();
        _ = stack;
        proc.context.cr3 = .{};
        //proc.context.esp = @intFromPtr(stack.slice[stack.slice.len..].ptr);
        proc.context.ebp = proc.context.esp;
        proc.context.eip = entry;
        proc.context.eflags = sys.Eflags{ .interrupts_enable = true, .reserved1 = true };
        //const page_dir = try page_allocator.mapAnyFree();
        //proc.page_directory = @ptrCast(@alignCast(page_dir.slice.ptr));
        //try page_allocator.mapAnyFree();
        // if(allocator.alignedAlloc(sys.PageDirectory, 4096,1)) |slice|
        //     &slice[0]
        // else |_|{
        //     @panic("");
        // };
        //mn.mapKernel(&proc);
        //proc.context.cr3.setPageDirectoryBase(page_dir.phsical_address);
        proc.doUserstuff();
        proc.mapKernel(kernel_end);
        //return proc;
    }

    pub fn mapKernel(self: *Process, kernel_end: usize) void {
        var kl: usize = 0x100000;
        var kh: usize = 0xC0100000;

        const kern_end_aligned = std.mem.alignForward(usize, kernel_end, 4096);

        var i: usize = 0;
        while (kh < kern_end_aligned) : ({
            kh += 4096;
            kl += 4096;
            i += 1;
        }) {
            var page_table_entry = sys.PageTableEntry4K{
                .sepervisor = true,
            };
            page_table_entry.setPageBaseAddress(kl);
            const lad = @as(sys.LinearAddress4K, @bitCast(kh));
            self.page_tables[lad.page_directory_entry][lad.page_table_entry] = page_table_entry;
        }

        for (0..1024) |ent| {
            var directory_entry = sys.PageDirectoryEntry4K{};
            directory_entry.setPageTableBaseAddress(@intFromPtr(&self.page_tables[ent]) - 0xC0000000);
            self.page_directory[ent] = sys.PageDirectoryEntry{ .@"4K" = directory_entry };
        }
    }

    pub fn mapAddress(self: *Process, address: usize, free_page: usize) void {
        const lad: sys.LinearAddress4K = @bitCast(address);
        self.page_tables[lad.page_directory_entry][lad.page_table_entry] = sys.PageTableEntry4K{
            .sepervisor = true,
        };

        self.page_tables[lad.page_directory_entry][lad.page_table_entry]
            .setPageBaseAddress(free_page);

        //vga.print("{}\n", .{lad});

        //sys.spinloop();

        var directory_entry = sys.PageDirectoryEntry4K{
            .sepervisor = true,
        };
        directory_entry.setPageTableBaseAddress(@intFromPtr(&self.page_tables[lad.page_directory_entry]));
        self.page_directory[lad.page_directory_entry] = sys.PageDirectoryEntry{ .@"4K" = directory_entry };
        //self.page_directory[lad.page_directory_entry]
    }

    pub fn doUserstuff(self: *Process) void {
        //_ = self;
        const usercode = sys.SegmentSelector{ .index = 3, .requested_privilege_level = 3, .table_indicator = .gdt };
        const userdata = sys.SegmentSelector{ .index = 4, .requested_privilege_level = 3, .table_indicator = .gdt };
        self.context.cs.selector = usercode;

        self.context.ds.selector = userdata;
        self.context.fs.selector = userdata;
        self.context.gs.selector = userdata;
        //self.context.ss.selector = userdata;
        self.context.es.selector = userdata;
    }
};

const List = std.SinglyLinkedList(Process);

procs: List,
current: *List.Node,
allocator: std.mem.Allocator,

const Self = @This();

pub fn init(allocator: std.mem.Allocator, first_proc: Process) !Self {
    var sched: Self = undefined;

    sched.procs = .{ .first = try allocator.create(List.Node) };

    sched.allocator = allocator;
    sched.procs.first.?.data = first_proc;
    sched.procs.first.?.next = null;
    sched.current = sched.procs.first.?;

    return sched;
}

pub fn pickNext(self: *Self, ctx: sys.cpuContext(.{})) sys.cpuContext(.{}) {
    self.current.data.context = ctx;

    if (self.current.next) |node| {
        self.current = node;
    } else {
        self.current = self.procs.first.?;
    }

    return self.current.data.context;
}

const vga = @import("vga.zig");

pub fn add(self: *Self, proc: Process) !void {
    //_ = self; _ = proc;
    const node = try self.allocator.create(List.Node);
    node.*.data = proc;
    self.procs.prepend(node);
}
