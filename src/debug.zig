const std = @import("std");
const vga = @import("vga.zig");
const sys = @import("sys/registers.zig");

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    vga.print("Panic: {s}\n", .{message});
    sys.disableinterrupts();
    while (true) {
        sys.haltProcessor();
    }
}

pub fn assert(x: bool, message: []const u8) void {
    if(x) return;
    panic(message, null, null);
}