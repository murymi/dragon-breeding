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
        const item = IDTGate{ .offset_low = @intCast(offset & 0xffff), .offset_high = @intCast(offset >> 16), .flags = flags, .selector = selector };
        return item;
    }
};

extern fn my_test_func() void;

pub export fn vulga() void {
    vga.print("Vulga language\n", .{});
}

pub var interrupt_descriptor_table: [256]sys.IDTEntry = undefined;

///
/// Send data to the master PIC. This will send it to the master data port.
///
/// Arguments:
///     IN data: u8 - The data to send.
///
inline fn sendDataMaster(data: u8) void {
    io.portByteOut(0x21, data);
}

///
/// Send data to the salve PIC. This will send it to the salve data port.
///
/// Arguments:
///     IN data: u8 - The data to send.
///
inline fn sendDataSlave(data: u8) void {
    io.portByteOut(0xa1, data);
}

///
/// Force the CPU to wait for an I/O operation to compete. Use port 0x80 as this is unused.
///
pub fn ioWait() void {
    io.portByteOut(0x80, @as(u8, 0));
}

pub fn initIdt() void {
    //inline for (0..32) |i| {
    //    const stub = getInterruptStub(i);
    //interrupt_descriptor_table[i] = IDTGate.init(@intFromPtr(&stub), 0x8e, 0x08);
    //}

    const selector = sys.SegmentSelector{ .index = 1, .requested_privilege_level = 0, .table_indicator = .gdt };
    inline for (sys.exceptions, 0..) |e, i| {
        const proc = sys.interruptProcedure(e, "handler");

        const gate: sys.IDTEntry = switch (e.kind) {
            .fault => .{ .interrupt = sys.InterruptGate.init(&proc, selector, 0, .@"32", true) },
            .trap => .{ .trap = sys.TrapGate.init(&proc, selector, 0, .@"32", true) },
            .interrupt => .{ .interrupt = sys.InterruptGate.init(&proc, selector, 0, .@"32", true) },
            .fault_trap => .{ .interrupt = sys.InterruptGate.init(&proc, selector, 0, .@"32", true) },
            .abort => .{ .interrupt = sys.InterruptGate.init(&proc, selector, 0, .@"32", true) },
            else => .{ .interrupt = sys.InterruptGate.init(&proc, selector, 0, .@"32", true) },
        };

        interrupt_descriptor_table[i] = gate;
        //_ = gate;
        //const entry = sys.
    }

    //    // icw1
    //    io.portByteOut(primary_command_port, 0x11);
    //    io.portByteOut(secondary_command_port, 0x11);
    //
    //    // icw2
    //    io.portByteOut(primary_data_port, 0x20);
    //    io.portByteOut(secondary_data_port, 0x28);
    //
    //    // icw3
    //    io.portByteOut(primary_data_port, 0x04);
    //    io.portByteOut(secondary_data_port, 0x02);
    //
    //    // icw4
    //    io.portByteOut(primary_data_port, 0x1);
    //    io.portByteOut(secondary_data_port, 0x1);
    //
    //    // ocw1
    //    io.portByteOut(primary_data_port, 0x0);
    //    io.portByteOut(secondary_data_port, 0x0);

    remap();

    inline for (32..256) |i| {
        //const stub = getInterruptStub(i);
        const proc = sys.interruptProcedure(.{ .id = i }, "handler");
        const pric = if(i == 0x80) 3 else 0;
        const gate = sys.IDTEntry{ .interrupt = sys.InterruptGate.init(&proc, selector, pric, .@"32", true) };

        interrupt_descriptor_table[i] = gate;
        //IDTGate.init(@intFromPtr(&s), 0x8e, 0x08);
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
        : [a] "{eax}" (@intFromPtr(addr)),
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
    asm volatile ("sti");
}

const IDT = packed struct { limit: u16, base: [*]sys.IDTEntry };

var idt: IDT = undefined;

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
const sys = @import("sys/registers.zig");

export fn handler(ctx: *sys.cpuContext(.{})) usize {
    if (ctx.interrupt_number < 32 //or ctx.int_num == syscalls.INTERRUPT
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
        //\\push  %%ss
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
        //\\mov   %%ax, %%ss
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
        //\\pop %%ss
        \\popa
    );
    // The Tss.esp0 value is the stack pointer used when an interrupt occurs. This should be the current process' stack pointer
    // So skip the rest of the CpuState, set Tss.esp0 then un-skip the last few fields of the CpuState
    asm volatile (
    //\\add   $0x1C, %%esp
    //\\.extern gdt.main_tss_entry
    //\\mov   %%esp, (gdt.main_tss_entry + 4)
        \\add   $0x8, %%esp
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
            //asm volatile (
            //    \\ cli
            //);

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

const main = @import("main.zig");

export fn irqHandler(ctx: *sys.cpuContext(.{})) usize {
    //vga.print("==========================\n", .{});
    // Get the IRQ index, by getting the interrupt number and subtracting the offset.
    //var res
    const ret_esp = @intFromPtr(ctx);
    if (ctx.interrupt_number < 32) {
        vga.print("Not an IRQ number: {}\n", .{ctx.interrupt_number});
    }
    const irq_offset = ctx.interrupt_number - 32;

    if (!spuriousIrq(@truncate(irq_offset))) {
        if (ctx.interrupt_number == 33) {
            //keyboardHandle();
            onKeyEvent();
        }

        if (ctx.interrupt_number == 32) {
            //vga.print("[[[[ ]]]]\n", .{});
            //ret_esp = main.handleClock(ctx);
        }

        if(ctx.interrupt_number == 0x80) {
            vga.print("SYSCALLL....: {}\n", .{ctx.edi});
            ctx.eax = 90;
        }
        //ret_esp = handler(ctx);
        // Send the end of interrupt command
        sendEndOfInterrupt(@truncate(irq_offset));
    } else {}

    //sendEndOfInterrupt(@truncate(ctx.int_num));

    //vga.print("SENT EOF\n", .{});

    return ret_esp;
}

//const paging = @import("paging.zig");

export fn isrHandler(ctx: *sys.cpuContext(.{})) usize {
    // Get the interrupt number
    //const isr_num = ctx.int_num;

    if (ctx.interrupt_number == 14) {
        //_ = paging.pageFault(ctx);
        //while (true) {}
        const code:sys.PageFaultErrorCode = @bitCast(ctx.error_code);
        //sys.Cr1{}
        vga.print("Page Fault occurred: 0x{x}\n{}\n", .{sys.Cr2.read().value, code});
        while (true) {
            sys.haltProcessor();
        }
    } else {
        const code:sys.ErrorCode = @bitCast(ctx.error_code);

        vga.print("Other: 0x{x}, no: {}\n", .{code.index, ctx.interrupt_number});
        while (true) {
            sys.haltProcessor();
        }
    }

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

pub fn keyboardHandle() void {
    const status = io.portByteIn(0x64);

    if ((status & 0x64) == 0) {
        vga.print("Waa\n", .{});
    }
    const scancode = io.portByteIn(0x60);

    vga.print("Scancode: 0x{x}\n", .{scancode});
}

// ----------
// Port address for the PIC master and slave registers.
// ----------

/// The port address for issuing a command to the master PIC. This is a write only operation.
const MASTER_COMMAND_REG: u16 = 0x20;

/// The port address for reading one of the status register of the master PIC. This can be either
/// the In-Service Register (ISR) or the Interrupt Request Register (IRR). This is a read only
/// operation.
const MASTER_STATUS_REG: u16 = 0x20;

/// The port address for reading or writing to the data register of the master PIC. This can be
/// used in conjunction with the command register to set up the PIC. This can also be used to mask
/// the interrupt lines so interrupts can be issued to the CPU.
const MASTER_DATA_REG: u16 = 0x21;

/// The port address for issuing a command to the slave PIC. This is a write only operation.
const SLAVE_COMMAND_REG: u16 = 0xA0;

/// The port address for reading one of the status register of the slave PIC. This can be either
/// the In-Service Register (ISR) or the Interrupt Request Register (IRR). This is a read only
/// operation.
const SLAVE_STATUS_REG: u16 = 0xA0;

/// The port address for reading or writing to the data register of the status PIC. This can be
/// used in conjunction with the command register to set up the PIC. This can also be used to mask
/// the interrupt lines so interrupts can be issued to the CPU.
const SLAVE_DATA_REG: u16 = 0xA1;

// ----------
// Initialisation control word 1.
// ----------

/// Initialisation control word 1. Primary control word for initialising the PIC. If set, then the
/// PIC expects to receive a initialisation control word 4.
const ICW1_EXPECT_ICW4: u8 = 0x01;

/// If set, then there is only one PIC in the system. If not set, then PIC is cascaded with slave
/// PICs and initialisation control word 3 must be sent to the controller.
const ICW1_SINGLE_CASCADE_MODE: u8 = 0x02;

/// If set, then the internal CALL address is 4. If not set, then is 8. Usually ignored by x86. So
/// default is not set, 0.
const ICW1_CALL_ADDRESS_INTERVAL_4: u8 = 0x04;

/// If set, then operating in level triggered mode. If not set, then operating in edge triggered
/// mode.
const ICW1_LEVEL_TRIGGER_MODE: u8 = 0x08;

/// If set, then the PIC is to be initialised.
const ICW1_INITIALISATION: u8 = 0x10;

// ----------
// Initialisation control word 2.
// ----------

/// Initialisation control word 2. Map the base address of the interrupt vector table. The new port
/// map for the master PIC. IRQs 0-7 mapped to use interrupts 0x20-0x27.
const ICW2_MASTER_REMAP_OFFSET: u8 = 0x20;

/// The new port map for the slave PIC. IRQs 8-15 mapped to use interrupts 0x28-0x2F.
const ICW2_SLAVE_REMAP_OFFSET: u8 = 0x28;

// ----------
// Initialisation control word 3.
// ----------

/// Initialisation control word 3. For Telling the master and slave where the cascading. interrupts
/// are coming from. Tell the slave PIT to send interrupts to the master PIC on IRQ2.
const ICW3_SLAVE_IRQ_MAP_TO_MASTER: u8 = 0x02;

/// Tell the master PIT to receive interrupts from the slave PIC on IRQ2.
const ICW3_MASTER_IRQ_MAP_FROM_SLAVE: u8 = 0x04;

// ----------
// Initialisation control word 4.
// ----------

/// Initialisation control word 4. Tell the master and slave what mode to operate in. If set, then
/// in 80x86 mode. If not set, then in MCS-80/86 mode.
const ICW4_80x86_MODE: u8 = 0x01;

/// If set, then on last interrupt acknowledge pulse the PIC automatically performs end of
/// interrupt operation.
const ICW4_AUTO_END_OF_INTERRUPT: u8 = 0x02;

/// Only use if ICW4_BUFFER_MODE is set. If set, then selects master's buffer. If not set then uses
/// slave's buffer.
const ICW4_BUFFER_SELECT: u8 = 0x04;

/// If set, then PIC operates in buffered mode.
const ICW4_BUFFER_MODE: u8 = 0x08;

/// If set, then the the system had many cascaded PICs. Not supported in x86.
const ICW4_FULLY_NESTED_MODE: u8 = 0x10;

// ----------
// Operation control word 1.
// ----------

/// Operation control word 1. Interrupt masks for IRQ0 and IRQ8.
const OCW1_MASK_IRQ0_8: u8 = 0x01;

/// Operation control word 1. Interrupt masks for IRQ1 and IRQ9.
const OCW1_MASK_IRQ1_9: u8 = 0x02;

/// Operation control word 1. Interrupt masks for IRQ2 and IRQ10.
const OCW1_MASK_IRQ2_10: u8 = 0x04;

/// Operation control word 1. Interrupt masks for IRQ3 and IRQ11.
const OCW1_MASK_IRQ3_11: u8 = 0x08;

/// Operation control word 1. Interrupt masks for IRQ4 and IRQ12.
const OCW1_MASK_IRQ4_12: u8 = 0x10;

/// Operation control word 1. Interrupt masks for IRQ5 and IRQ13.
const OCW1_MASK_IRQ5_13: u8 = 0x20;

/// Operation control word 1. Interrupt masks for IRQ6 and IRQ14.
const OCW1_MASK_IRQ6_14: u8 = 0x40;

/// Operation control word 1. Interrupt masks for IRQ7 and IRQ15.
const OCW1_MASK_IRQ7_15: u8 = 0x80;

// ----------
// Operation control word 2.
// ----------

/// Operation control word 2. Primary commands for the PIC. Interrupt level 1 upon which the
/// controller must react. Interrupt level for the current interrupt.
const OCW2_INTERRUPT_LEVEL_1: u8 = 0x01;

/// Interrupt level 2 upon which the controller must react. Interrupt level for the current
/// interrupt
const OCW2_INTERRUPT_LEVEL_2: u8 = 0x02;

/// Interrupt level 3 upon which the controller must react. Interrupt level for the current
/// interrupt
const OCW2_INTERRUPT_LEVEL_3: u8 = 0x04;

/// The end of interrupt command code.
const OCW2_END_OF_INTERRUPT: u8 = 0x20;

/// Select command.
const OCW2_SELECTION: u8 = 0x40;

/// Rotation command.
const OCW2_ROTATION: u8 = 0x80;

// ----------
// Operation control word 3.
// ----------

/// Operation control word 3.
/// Read the Interrupt Request Register register
const OCW3_READ_IRR: u8 = 0x00;

/// Read the In Service Register register.
const OCW3_READ_ISR: u8 = 0x01;

/// If set, then bit 0 will be acted on, so read ISR or IRR. If not set, then no action taken.
const OCW3_ACT_ON_READ: u8 = 0x02;

/// If set, then poll command issued. If not set, then no pool command issued.
const OCW3_POLL_COMMAND_ISSUED: u8 = 0x04;

/// This must be set for all OCW 3.
const OCW3_DEFAULT: u8 = 0x08;

// Next bit must be zero.

/// If set, then the special mask is set. If not set, then resets special mask.
const OCW3_SPECIAL_MASK: u8 = 0x20;

/// If set, then bit 5 will be acted on, so setting the special mask. If not set, then no action it
/// taken.
const OCW3_ACK_ON_SPECIAL_MASK: u8 = 0x40;

// Last bit must be zero.

// ----------
// The IRQs
// ----------

/// The IRQ for the PIT.
pub const IRQ_PIT: u8 = 0x00;

/// The IRQ for the keyboard.
pub const IRQ_KEYBOARD: u8 = 0x01;

/// The IRQ for the cascade from master to slave.
pub const IRQ_CASCADE_FOR_SLAVE: u8 = 0x02;

/// The IRQ for the serial COM2/4.
pub const IRQ_SERIAL_PORT_2: u8 = 0x03;

/// The IRQ for the serial COM1/3.
pub const IRQ_SERIAL_PORT_1: u8 = 0x04;

/// The IRQ for the parallel port 2.
pub const IRQ_PARALLEL_PORT_2: u8 = 0x05;

/// The IRQ for the floppy disk.
pub const IRQ_DISKETTE_DRIVE: u8 = 0x06;

/// The IRQ for the parallel port 1.
pub const IRQ_PARALLEL_PORT_1: u8 = 0x07;

/// The IRQ for the CMOS real time clock (RTC).
pub const IRQ_REAL_TIME_CLOCK: u8 = 0x08;

/// The IRQ for the CGA vertical retrace.
pub const IRQ_CGA_VERTICAL_RETRACE: u8 = 0x09;

/// Reserved.
pub const IRQ_RESERVED1: u8 = 0x0A;

/// Reserved.
pub const IRQ_RESERVED2: u8 = 0x0B;

// The IRQ for the PS/2 mouse.
pub const IRQ_PS2_MOUSE: u8 = 0x0C;

/// The IRQ for the floating point unit/co-processor.
pub const IRQ_FLOATING_POINT_UNIT: u8 = 0x0D;

/// The IRQ for the primary hard drive controller.
pub const IRQ_PRIMARY_HARD_DISK_CONTROLLER: u8 = 0x0E;

/// The IRQ for the secondary hard drive controller.
pub const IRQ_SECONDARY_HARD_DISK_CONTROLLER: u8 = 0x0F;

/// Keep track of the number of spurious IRQs.
var spurious_irq_counter: u32 = 0;

///
/// Clear the mask bit for the provided IRQ. This will allow interrupts to triggering for this IRQ.
///
/// Arguments:
///     IN irq_num: u8 - The IRQ number unmask.
///
pub fn clearMask(irq_num: u8) void {
    const port: u16 = if (irq_num < 8) MASTER_DATA_REG else SLAVE_DATA_REG;
    const shift: u3 = @intCast(irq_num % 8);
    const value: u8 = io.portByteIn(port) & ~(@as(u8, 1) << shift);
    io.portByteOut(port, value);
}

///
/// Remap the PIC interrupt lines as initially they conflict with CPU exceptions which are reserved
/// by Intel up to 0x1F. So this will move the IRQs from 0x00-0x0F to 0x20-0x2F.
///
pub fn remap() void {
    //log.info("Init\n", .{});
    //defer log.info("Done\n", .{});

    // Initiate
    sendCommandMaster(ICW1_INITIALISATION | ICW1_EXPECT_ICW4);
    ioWait();
    sendCommandSlave(ICW1_INITIALISATION | ICW1_EXPECT_ICW4);
    ioWait();

    // Offsets
    sendDataMaster(ICW2_MASTER_REMAP_OFFSET);
    ioWait();
    sendDataSlave(ICW2_SLAVE_REMAP_OFFSET);
    ioWait();

    // IRQ lines
    sendDataMaster(ICW3_MASTER_IRQ_MAP_FROM_SLAVE);
    ioWait();
    sendDataSlave(ICW3_SLAVE_IRQ_MAP_TO_MASTER);
    ioWait();

    // 80x86 mode
    sendDataMaster(ICW4_80x86_MODE);
    ioWait();
    sendDataSlave(ICW4_80x86_MODE);
    ioWait();

    // Mask all interrupts
    sendDataMaster(0xFF);
    ioWait();
    sendDataSlave(0xFF);
    ioWait();

    // Clear the IRQ for the slave
    clearMask(IRQ_CASCADE_FOR_SLAVE);

    vga.print("remaped pic...\n", .{});
}

///
/// Send a command to the master PIC. This will send it to the master command port.
///
/// Arguments:
///     IN cmd: u8 - The command to send.
///
inline fn sendCommandMaster(cmd: u8) void {
    io.portByteOut(MASTER_COMMAND_REG, cmd);
}

///
/// Send a command to the salve PIC. This will send it to the salve command port.
///
/// Arguments:
///     IN cmd: u8 - The command to send.
///
inline fn sendCommandSlave(cmd: u8) void {
    io.portByteOut(SLAVE_COMMAND_REG, cmd);
}

pub fn spuriousIrq(irq_num: u8) bool {
    // Only for IRQ 7 and 15
    if (irq_num == 7) {
        // Read master ISR
        // Check the MSB is zero, if so, then is a spurious IRQ
        // This is (1 << irq_num) or (1 << 7) to check if it is set for this IRQ
        if ((readMasterIsr() & 0x80) == 0) {
            spurious_irq_counter += 1;
            return true;
        }
    } else if (irq_num == 15) {
        // Read slave ISR
        // Check the MSB is zero, if so, then is a spurious irq
        if ((readSlaveIsr() & 0x80) == 0) {
            // Need to send EOI to the master
            sendCommandMaster(OCW2_END_OF_INTERRUPT);
            spurious_irq_counter += 1;
            return true;
        }
    }

    return false;
}

///
/// Read the data from the master data register. This will read from the master data port.
///
/// Return: u8
///     The data that is stored in the master data register.
///
inline fn readDataMaster() u8 {
    return io.portByteIn(MASTER_DATA_REG);
}

///
/// Read the data from the salve data register. This will read from the salve data port.
///
/// Return: u8
///     The data that is stored in the salve data register.
///
inline fn readDataSlave() u8 {
    return io.portByteIn(SLAVE_DATA_REG);
}

///
/// Read the master interrupt request register (IRR).
///
/// Return: u8
///     The data that is stored in the master IRR.
///
inline fn readMasterIrr() u8 {
    sendCommandMaster(OCW3_DEFAULT | OCW3_ACT_ON_READ | OCW3_READ_IRR);
    return io.portByteIn(MASTER_STATUS_REG);
}

///
/// Read the slave interrupt request register (IRR).
///
/// Return: u8
///     The data that is stored in the slave IRR.
///
inline fn readSlaveIrr() u8 {
    sendCommandSlave(OCW3_DEFAULT | OCW3_ACT_ON_READ | OCW3_READ_IRR);
    return io.portByteIn(SLAVE_STATUS_REG);
}

///
/// Read the master in-service register (ISR).
///
/// Return: u8
///     The data that is stored in the master ISR.
///
inline fn readMasterIsr() u8 {
    sendCommandMaster(OCW3_DEFAULT | OCW3_ACT_ON_READ | OCW3_READ_ISR);
    return io.portByteIn(MASTER_STATUS_REG);
}

///
/// Read the slave in-service register (ISR).
///
/// Return: u8
///     The data that is stored in the slave ISR.
///
inline fn readSlaveIsr() u8 {
    sendCommandSlave(OCW3_DEFAULT | OCW3_ACT_ON_READ | OCW3_READ_ISR);
    return io.portByteIn(SLAVE_STATUS_REG);
}

///
/// Send the end of interrupt (EOI) signal to the PIC. If the IRQ was from the master, then will
/// send the EOI to the master only. If the IRQ came from the slave, then will send the EOI to both
/// the slave and master.
///
/// Arguments:
///     IN irq_num: u8 - The IRQ number to sent the EOI to.
///
pub fn sendEndOfInterrupt(irq_num: u8) void {
    if (irq_num >= 8) {
        sendCommandSlave(OCW2_END_OF_INTERRUPT);
    }

    sendCommandMaster(OCW2_END_OF_INTERRUPT);
}

pub const KeyPosition = enum(u7) {
    ESC,
    F1,
    F2,
    F3,
    F4,
    F5,
    F6,
    F7,
    F8,
    F9,
    F10,
    F11,
    F12,
    PRINT_SCREEN,
    SCROLL_LOCK,
    PAUSE,
    BACKTICK,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX,
    SEVEN,
    EIGHT,
    NINE,
    ZERO,
    HYPHEN,
    EQUALS,
    BACKSPACE,
    TAB,
    Q,
    W,
    E,
    R,
    T,
    Y,
    U,
    I,
    O,
    P,
    LEFT_BRACKET,
    RIGHT_BRACKET,
    ENTER,
    CAPS_LOCK,
    A,
    S,
    D,
    F,
    G,
    H,
    J,
    K,
    L,
    SEMICOLON,
    APOSTROPHE,
    HASH,
    LEFT_SHIFT,
    BACKSLASH,
    Z,
    X,
    C,
    V,
    B,
    N,
    M,
    COMMA,
    DOT,
    FORWARD_SLASH,
    RIGHT_SHIFT,
    LEFT_CTRL,
    SPECIAL,
    LEFT_ALT,
    SPACE,
    RIGHT_ALT,
    FN,
    SPECIAL2,
    RIGHT_CTRL,
    INSERT,
    HOME,
    PAGE_UP,
    DELETE,
    END,
    PAGE_DOWN,
    LEFT_ARROW,
    UP_ARROW,
    DOWN_ARROW,
    RIGHT_ARROW,
    NUM_LOCK,
    KEYPAD_SLASH,
    KEYPAD_ASTERISK,
    KEYPAD_MINUS,
    KEYPAD_7,
    KEYPAD_8,
    KEYPAD_9,
    KEYPAD_PLUS,
    KEYPAD_4,
    KEYPAD_5,
    KEYPAD_6,
    KEYPAD_1,
    KEYPAD_2,
    KEYPAD_3,
    KEYPAD_ENTER,
    KEYPAD_0,
    KEYPAD_DOT,
};

/// A keyboard action, either a press or release
pub const KeyAction = struct {
    /// The position of the key
    position: KeyPosition,
    /// Whether it was a release or press
    released: bool,
};

/// The initialised keyboard
//var keyboard: *Keyboard = undefined;
/// The number of keys pressed without a corresponding release
var pressed_keys: usize = 0;
/// If we're in the middle of a special key sequence (e.g. print_screen and arrow keys)
var special_sequence = false;
/// The number of release scan codes expected before registering a release event
/// This is used in special sequences since they end in a certain number of release scan codes
var expected_releases: usize = 0;
/// If a print_screen press is being processed
var on_print_screen = false;

///
/// Read a byte from the keyboard buffer
///
/// Return: u8
///     The byte waiting in the keyboard buffer
///
fn readKeyboardBuffer() u8 {
    return io.portByteIn(0x60);
}

///
/// Parse a keyboard scan code and return the associated keyboard action.
/// Some keys require a specific sequence of scan codes so this function acts as a state machine
///
/// Arguments:
///     IN scan_code: u8 - The scan code from the keyboard
///
/// Return: ?KeyAction
///     The keyboard action resulting from processing the scan code, or null if the scan code doesn't result in a finished keyboard action
///
fn parseScanCode(scan_code: u8) ?KeyAction {
    var released = false;
    // The print screen key requires special processing since it uses a unique byte sequence
    if (on_print_screen or scan_code >= 128) {
        released = true;
        if (special_sequence or on_print_screen) {
            // Special sequences are followed by a certain number of release scan codes that should be ignored. Update the expected number
            if (expected_releases >= 1) {
                expected_releases -= 1;
                return null;
            }
        } else {
            if (pressed_keys == 0) {
                // A special sequence is started by a lone key release scan code
                special_sequence = true;
                return null;
            }
        }
    }
    // Cut off the top bit, which denotes that the key was released
    const key_code: u7 = @truncate(scan_code);
    var key_pos: ?KeyPosition = null;
    if (special_sequence or on_print_screen) {
        if (!released) {
            // Most special sequences are followed by an extra key release byte
            expected_releases = 1;
        }
        switch (key_code) {
            72 => key_pos = KeyPosition.UP_ARROW,
            75 => key_pos = KeyPosition.LEFT_ARROW,
            77 => key_pos = KeyPosition.RIGHT_ARROW,
            80 => key_pos = KeyPosition.DOWN_ARROW,
            // First byte sent for the pause key
            29 => return null,
            42 => {
                // The print screen key is followed by five extra key release bytes
                key_pos = KeyPosition.PRINT_SCREEN;
                if (!released) {
                    on_print_screen = true;
                    expected_releases = 5;
                }
            },
            // Second and final byte sent for the pause key
            69 => {
                // The pause key is followed by two extra key release bytes
                key_pos = KeyPosition.PAUSE;
                if (!released) {
                    expected_releases = 2;
                }
            },
            82 => key_pos = KeyPosition.INSERT,
            71 => key_pos = KeyPosition.HOME,
            73 => key_pos = KeyPosition.PAGE_UP,
            83 => key_pos = KeyPosition.DELETE,
            79 => key_pos = KeyPosition.END,
            81 => key_pos = KeyPosition.PAGE_DOWN,
            53 => key_pos = KeyPosition.KEYPAD_SLASH,
            28 => key_pos = KeyPosition.KEYPAD_ENTER,
            56 => key_pos = KeyPosition.RIGHT_ALT,
            91 => key_pos = KeyPosition.SPECIAL,
            else => return null,
        }
    }
    key_pos = key_pos orelse switch (key_code) {
        1 => KeyPosition.ESC,
        // Number keys and second row
        2...28 => @enumFromInt(@intFromEnum(KeyPosition.ONE) + (key_code - 2)),
        29 => KeyPosition.LEFT_CTRL,
        30...40 => @enumFromInt(@intFromEnum(KeyPosition.A) + (key_code - 30)),
        41 => KeyPosition.BACKTICK,
        42 => KeyPosition.LEFT_SHIFT,
        43 => KeyPosition.HASH,
        44...54 => @enumFromInt(@intFromEnum(KeyPosition.Z) + (key_code - 44)),
        55 => KeyPosition.KEYPAD_ASTERISK,
        56 => KeyPosition.LEFT_ALT,
        57 => KeyPosition.SPACE,
        58 => KeyPosition.CAPS_LOCK,
        59...68 => @enumFromInt(@intFromEnum(KeyPosition.F1) + (key_code - 59)),
        69 => KeyPosition.NUM_LOCK,
        70 => KeyPosition.SCROLL_LOCK,
        71...73 => @enumFromInt(@intFromEnum(KeyPosition.KEYPAD_7) + (key_code - 71)),
        74 => KeyPosition.KEYPAD_MINUS,
        75...77 => @enumFromInt(@intFromEnum(KeyPosition.KEYPAD_4) + (key_code - 75)),
        78 => KeyPosition.KEYPAD_PLUS,
        79...81 => @enumFromInt(@intFromEnum(KeyPosition.KEYPAD_1) + (key_code - 79)),
        82 => KeyPosition.KEYPAD_0,
        83 => KeyPosition.KEYPAD_DOT,
        86 => KeyPosition.BACKSLASH,
        87 => KeyPosition.F11,
        88 => KeyPosition.F12,
        else => null,
    };
    if (key_pos) |k| {
        // If we're releasing a key decrement the number of keys pressed, else increment it
        if (!released) {
            pressed_keys += 1;
        } else {
            //pressed_keys -= 1;
            // Releasing a special key means we are no longer on that special key
            special_sequence = false;
        }
        //vga.print("============================\n", .{});
        return KeyAction{ .position = k, .released = released };
    }
    return null;
}

///
/// Register a keyboard action. Should only be called in response to a keyboard IRQ
///
/// Arguments:
///     IN ctx: *arch.CpuState - The state of the C        const proc: u32 = @intFromPtr(proc_entry_point);
///PU when the keyboard action occurred
///
/// Return: usize
///     The stack pointer value to use when returning from the interrupt
///
fn onKeyEvent() void {
    const scan_code = readKeyboardBuffer();
    if (parseScanCode(scan_code)) |action| {
        vga.print("{}", .{action});
        //if(!action.released) {
        //}
        //if (!keyboard.writeKey(action)) {
        //    log.warn("No room for keyboard action {}\n", .{action});
        //}
    }
    //return @ptrToInt(ctx);
}
