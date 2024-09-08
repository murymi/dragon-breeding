pub fn portByteIn(port: u16) u8 {
    return asm volatile(
        \\ in %[a], %al
        : [ret] "={al}" (->u8),
        : [a] "{dx}" (port)
    );
}

pub fn portByteOut(port: u16, data: u8) void {
    _ = asm volatile(
        \\ out %[a], %[b]
        : [ret] "={al}" (->u8),
        : [a] "{al}" (data),
        [b] "{dx}" (port)
    );
}
