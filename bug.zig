pub const Process = struct {
    a: u32,

    pub fn init() Process {
        const proc: Process = .{.a = 0};
        //undefined;
        var a: u8 = 0;
        mapKernel(&a);
        return proc;
    }

    fn mapKernel(self: *u8) void {
        _ = self;
    }
};

pub fn main() !void {
    _ = Process.init();
}
