const io = @import("io.zig");

pub const screen_control_register = 0x3d4;
pub const screen_data_register = 0x3d5;   
pub const high_offset = 0x0e;
pub const low_offset = 0x0f;

pub fn getCursor() Pos {
    io.portByteOut(screen_control_register, high_offset);
    var offset = @as(u16, @intCast(io.portByteIn(screen_data_register))) << 8;
    io.portByteOut(screen_control_register, low_offset);
    offset += io.portByteIn(screen_data_register);
    //return offset * 2;

    return Pos{
        .x = @intCast(offset % 80),
        .y = @intCast(offset/80)
    };
}

pub fn getOffset(pos: Pos) u16 {
    //_ = pos; return 0;
    return 2 * ((@as(u16, @intCast(pos.y)) * 80) + pos.x);
}

pub fn setCursor(pos: Pos) void {
    const offset = getOffset(pos)/2;
    io.portByteOut(screen_control_register, high_offset);
    io.portByteOut(screen_data_register, @truncate(offset >> 8));
    io.portByteOut(screen_control_register, low_offset);
    io.portByteOut(screen_data_register, @truncate(offset & 0xff));
}


pub const ConsoleColor = enum(u8) {
    black = 0,
    blue = 1,
    green = 2,
    cyan = 3,
    red = 4,
    magenta = 5,
    brown = 6,
    nightGray = 7,
    darkGray = 8,
    nightBlue = 9,
    nightGreen = 10,
    nightCyan = 11,
    nightRed = 12,
    nightMagenta = 13,
    nightBrown = 14,
    white = 15,
};

const VgaCell = packed struct(u16) {
    char: u8,
    attribute: u8,

    pub fn init(char: u8, fg: ConsoleColor, bg: ConsoleColor) VgaCell {
        return VgaCell{ .char = char, .attribute = @intFromEnum(fg) | (@intFromEnum(bg) << 4) };
    }
};

var vga_buffer: [25][]volatile VgaCell = init: {
    var video_mem = @as([*]volatile VgaCell, @ptrFromInt(0xb8000 + 0xC0000000));
    var buf: [25][]volatile VgaCell = undefined;
    for (0..25) |i| {
        buf[i].len = 80;
        buf[i].ptr = video_mem;
        video_mem += 80;
    }
    break :init buf;
};

const Pos = struct { x: u8, y: u8 };

pub fn printChar(char: u8, pos: Pos) void {
    vga_buffer[pos.y][pos.x] = VgaCell.init(char, .white, .black);
}

pub fn clearScreen(color: ConsoleColor) void {
    for (vga_buffer) |row| {
        for (row) |*col| {
            col.* = VgaCell.init(' ', .white, color);
        }
    }

    setCursor(.{.x =0, .y = 0});
}

pub fn scrollUp(bg: ConsoleColor) void {
    for(vga_buffer[0..vga_buffer.len-1], 1..) |row, i| {
        @memcpy(row, vga_buffer[i]);
    }
    for (vga_buffer[vga_buffer.len-1]) |*col| {
        col.* = VgaCell.init(' ', .white, bg);
    }
}

pub fn newLine(row: *u8, col: *u8) void {
    if(row.* >= 25) {
        scrollUp(.cyan);
        row.* -= 1;
    }


    if (col.* == 80) {
        col.* = 0;
        row.* += 1;
        if(row.* == 25) {
            scrollUp(.cyan);
            row.* -= 1;
        }
    }
}

pub fn puts(string: []const u8) void {
    //var row: u8 = 0;
    //var col: u8 = 0;

    var pos = getCursor();

    for (string) |value| {
        switch (value) {
            '\n' => {
                pos.y += 1;
                pos.x = 0;
                newLine(&pos.y, &pos.x);
                setCursor(pos);
                continue;
            },
            '\t' => {
                printChar(' ', .{ .x = pos.x, .y = pos.y });
                pos.x += 1;
                newLine(&pos.y, &pos.x);
                printChar(' ', .{ .x = pos.x+1, .y = pos.y });
            },
            else => printChar(value, .{ .x = pos.x, .y = pos.y }),
        }

        pos.x += 1;
        newLine(&pos.y, &pos.x);
        setCursor(pos);
    }
}

pub const StackTrace = struct {
    index: usize,
    instruction_addresses: []usize,
};

const std = @import("std");

pub fn write(_: void, buf:[]const u8) !usize{
    puts(buf);
    return buf.len;
}

pub const Writer = std.io.GenericWriter(void,
 error{},
write){.context = {}};

pub fn print(comptime fmt: []const u8, args: anytype) void {
    std.fmt.format(Writer, fmt, args) catch unreachable;
}