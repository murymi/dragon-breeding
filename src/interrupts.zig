const io = @import("io.zig");
const vga = @import("vga.zig");
const std = @import("std");

pub const primary_command_port = 0x20;
pub const primary_data_port = 0x21;


pub const secondary_command_port = 0xa0;
pub const secondary_data_port = 0xa1;

pub const IDTGate = packed struct(u64) {
    offset_low: u16,
    selector: u16,
    zero: u8 = 0,
    flags: u8,
    offset_high: u16,

    pub fn init(offset: u32, flags: u8, selector: u8) IDTGate {
        const item = IDTGate{
            .offset_low = @intCast(offset & 0xffff),
            .offset_high = @intCast(offset >> 16),
            .flags = flags,
            .selector = selector
        };
        return item;
    }
};


extern fn my_test_func() void;

pub export fn vulga() void {
    vga.print("Vulga language\n", .{});
}

pub var interrupt_descriptor_table: [256]IDTGate = undefined;


pub fn initIdt() void {
    inline for(0..32) |i| {
        const stub = getInterruptStub(i);
        interrupt_descriptor_table[i] =  IDTGate.init(@intFromPtr(&stub), 0x8e, 0x08);
    }

    // icw1
    io.portByteOut(primary_command_port, 0x11);
    io.portByteOut(secondary_command_port, 0x11);

    // icw2
    io.portByteOut(primary_data_port, 0x20);
    io.portByteOut(secondary_data_port, 0x28);

    // icw3
    io.portByteOut(primary_data_port, 0x04);
    io.portByteOut(secondary_data_port, 0x02);

    // icw4
    io.portByteOut(primary_data_port, 0x1);
    io.portByteOut(secondary_data_port, 0x1);

    // ocw1
    io.portByteOut(primary_data_port, 0x0);
    io.portByteOut(secondary_data_port, 0x0);


    inline for(32..48) |i| {
        const stub = getInterruptStub(i);
        interrupt_descriptor_table[i] =  IDTGate.init(@intFromPtr(&stub), 0x8e, 0x08);
    }

    loadIdt(&idt);



    //vga.print("{}\n{}\n{}\n{}\n", .{
    //    isr0 & 0xff, isr1 & 0xff, isr2 & 0xff, isr3 & 0xff
    //});

    //vga.print("TESTER: {}, {}\n", .{
    //    isr_pointers[0], tester
    //});

    //my_test_func();

}

fn loadIdt(addr: *IDT) void {

    idt.base = interrupt_descriptor_table[0..].ptr;
    idt.limit = (interrupt_descriptor_table.len * @sizeOf(IDTGate)) - 1;


    asm volatile (
        \\ lidt (%%eax)
        : 
        : [a] "{eax}" (@intFromPtr(addr))
    );

    //@as(*fn()void, @ptrFromInt((
    //    @as(u32, @intCast(addr.base[0].offset_high)) << 16) | addr.base[0].offset_low
    //))();


    //vga.print("Diff: {any}\n", .{end_isr_pointers - isr_pointers});

    rt_loadedIDTSuccess();

    enable();

    vga.print("Enabled interrupts\n", .{});
}

pub fn enable() void {
    asm volatile("sti");
}

const IDT = packed struct {
    limit: u16,
    base: [*]IDTGate
};

var idt:IDT = undefined;





//const arch = @import("arch.zig");
//const syscalls = @import("syscalls.zig");
//const irq = @import("irq.zig");
//const idt = @import("idt.zig");

//extern fn irq2Handler(ctx: *CpuState) usize;
//extern fn isr2Handler(ctx: *CpuState) usize;

///
/// The main handler for all exceptions and interrupts. This will then go and call the correct
/// handler for an ISR or IRQ.
///
/// Arguments:
///     IN ctx: *arch.CpuState - Pointer to the exception context containing the contents
///                              of the registers at the time of a exception.
///
export fn handler(ctx: *CpuState) usize {
    //vga.print("==============oya===================\n", .{});
    //vga.print("{any}\n", .{ctx.*});

    if (ctx.int_num < 32 //or ctx.int_num == syscalls.INTERRUPT
    ) {
        return isrHandler(ctx);
    } else {
        return irqHandler(ctx);
    }
}

///
/// The common assembly that all exceptions and interrupts will call.
///
export fn commonStub() callconv(.Naked) void {
    asm volatile (
        \\pusha
        \\push  %%ds
        \\push  %%es
        \\push  %%fs
        \\push  %%gs
        \\mov %%cr3, %%eax
        \\push %%eax
        \\mov   $0x10, %%ax
        \\mov   %%ax, %%ds
        \\mov   %%ax, %%es
        \\mov   %%ax, %%fs
        \\mov   %%ax, %%gs
        \\mov   %%esp, %%eax
        \\push  %%eax
        \\call  handler
        \\mov   %%eax, %%esp
    );

    // Pop off the new cr3 then check if it's the same as the previous cr3
    // If so don't change cr3 to avoid a TLB flush
    asm volatile (
        \\pop   %%eax
        \\mov   %%cr3, %%ebx
        \\cmp   %%eax, %%ebx
        \\je    same_cr3
        \\mov   %%eax, %%cr3
        \\same_cr3:
        \\pop   %%gs
        \\pop   %%fs
        \\pop   %%es
        \\pop   %%ds
        \\popa
    );
    // The Tss.esp0 value is the stack pointer used when an interrupt occurs. This should be the current process' stack pointer
    // So skip the rest of the CpuState, set Tss.esp0 then un-skip the last few fields of the CpuState
    asm volatile (
        \\add   $0x1C, %%esp
        \\.extern gdt.main_tss_entry
        \\mov   %%esp, (gdt.main_tss_entry + 4)
        \\sub   $0x14, %%esp
        \\iret
    );
}

pub const InterruptHandler = fn () callconv(.Naked) void;


///
/// Generate the function that is the entry point for each exception/interrupt. This will then be
/// used as the handler for the corresponding IDT entry.
///
/// Arguments:
///     IN interrupt_num: u32 - The interrupt number to generate the function for.
///
/// Return: idt.InterruptHandler
///     The stub function that is called for each interrupt/exception.
///
pub fn getInterruptStub(comptime interrupt_num: u32) InterruptHandler {
    return struct {
        fn func() callconv(.Naked) void {
            asm volatile (
                \\ cli
            );

            // These interrupts don't push an error code onto the stack, so will push a zero.
            if (interrupt_num != 8 and !(interrupt_num >= 10 and interrupt_num <= 14) and interrupt_num != 17) {
                asm volatile (
                    \\ pushl $0
                );
            }

            asm volatile (
                \\ pushl %[nr]
                \\ jmp commonStub
                :
                : [nr] "n" (interrupt_num),
            );
        }
    }.func;
}


/// The interrupt context that is given to a interrupt handler. It contains most of the registers
/// and the interrupt number and error code (if there is one).
pub const CpuState = packed struct {
    // Page directory
    cr3: usize,
    // Extra segments
    gs: u32,
    fs: u32,
    es: u32,
    ds: u32,

    // Destination, source, base pointer
    edi: u32,
    esi: u32,
    ebp: u32,
    esp: u32,

    // General registers
    ebx: u32,
    edx: u32,
    ecx: u32,
    eax: u32,

    // Interrupt number and error code
    int_num: u32,
    error_code: u32,

    // Instruction pointer, code segment and flags
    eip: u32,
    cs: u32,
    eflags: u32,
    user_esp: u32,
    user_ss: u32,

    pub fn empty() CpuState {
        return .{
            .cr3 = undefined,
            .gs = undefined,
            .fs = undefined,
            .es = undefined,
            .ds = undefined,
            .edi = undefined,
            .esi = undefined,
            .ebp = undefined,
            .esp = undefined,
            .ebx = undefined,
            .edx = undefined,
            .ecx = undefined,
            .eax = undefined,
            .int_num = undefined,
            .error_code = undefined,
            .eip = undefined,
            .cs = undefined,
            .eflags = undefined,
            .user_esp = undefined,
            .user_ss = undefined,
        };
    }
};



export fn irqHandler(ctx: *CpuState) usize {
    // Get the IRQ index, by getting the interrupt number and subtracting the offset.
    if (ctx.int_num < 32) {
        vga.print("Not an IRQ number: {}\n", .{ctx.int_num});
    }

    sendEndOfInterrupt(@truncate(ctx.int_num));

    if(ctx.int_num == 32) {
        keyboardHandle();
    }

    //vga.print("SENT EOF\n", .{});

    const ret_esp = @intFromPtr(ctx);
    return ret_esp;
}

export fn isrHandler(ctx: *CpuState) usize {
    // Get the interrupt number
    //const isr_num = ctx.int_num;

    const ret_esp = @intFromPtr(ctx);

    return ret_esp;
}

//pub fn isValidIsr(isr_num: u32) bool {
//    return isr_num < NUMBER_OF_ENTRIES or isr_num == syscalls.INTERRUPT;
//}

///
/// Check that the IDT table was loaded properly by getting the previously loaded table and
/// compare the limit and base address.
///
fn rt_loadedIDTSuccess() void {
    const loaded_idt = sidt();
    if (idt.limit != loaded_idt.limit) {
        vga.print("FAILURE: IDT not loaded properly: 0x{X} != 0x{X}\n", .{ idt.limit, loaded_idt.limit });
    }
    if (idt.base != loaded_idt.base) {
        vga.print("FAILURE: IDT not loaded properly: {any} != {any}\n", .{ idt.base, loaded_idt.base });
    }
    vga.print("Tested loading IDT\n", .{});
}


///
/// Get the previously loaded IDT from the CPU.
///
/// Return: idt.IdtPtr
///     The previously loaded IDT from the CPU.
///
pub fn sidt() IDT {
    var idt_ptr = IDT{ .limit = 0, .base = @ptrFromInt(16) };
    asm volatile ("sidt %[tab]"
        : [tab] "=m" (idt_ptr),
    );
    return idt_ptr;
}


pub fn sendEndOfInterrupt(irq_num: u8) void {
        io.portByteOut(primary_command_port, 0x20);

    if (irq_num < 40) {
        io.portByteOut(secondary_command_port, 0x20);
    }

   // sendCommandMaster(OCW2_END_OF_INTERRUPT);
}

pub fn keyboardHandle() void {
    const status = io.portByteIn(0x64);

    if((status & 0x64) == 0) {
        vga.print("Waa\n", .{});
    }
    const scancode = io.portByteIn(0x60);

    vga.print("Scancode: 0x{x}\n", .{scancode});
}