const std = @import("std");
const Builder = @import("std").Build;
const Target = @import("std").Target;
const CrossTarget = Target.Query;
//zig.CrossTarget;
const Feature = @import("std").Target.Cpu.Feature;

pub fn build(b: *Builder) void {
    const features = Target.x86.Feature;

    var disabled_features = Feature.Set.empty;
    var enabled_features = Feature.Set.empty;

    disabled_features.addFeature(@intFromEnum(features.mmx));
    disabled_features.addFeature(@intFromEnum(features.sse));
    disabled_features.addFeature(@intFromEnum(features.sse2));
    disabled_features.addFeature(@intFromEnum(features.avx));
    disabled_features.addFeature(@intFromEnum(features.avx2));
    enabled_features.addFeature(@intFromEnum(features.soft_float));

    const query = CrossTarget{ .cpu_arch = Target.Cpu.Arch.x86, .os_tag = Target.Os.Tag.freestanding, .abi = Target.Abi.none, .cpu_features_sub = disabled_features, .cpu_features_add = enabled_features };

    const optimize = b.standardOptimizeOption(.{});

    const kernel = b.addExecutable(.{
        .name = "kernel.elf",
        .root_source_file = b.path("src/main.zig"),
        .target = b.resolveTargetQuery(query),
        .optimize = optimize,
        .code_model = .kernel,
    });
    //kernel.addAssemblyFile(b.path("src/boot.s"));
    kernel.setLinkerScript(.{ .cwd_relative = "src/linker.ld" });
    b.installArtifact(kernel);
    var kernel_step = b.step("kernel", "Build the kernel");
    kernel_step.dependOn(&kernel.step);


    const iso_dir = b.fmt("{s}/iso", .{b.cache_root.path.?});
    const mkdir_command_args = &[_][]const u8{
        "mkdir", "-p", b.fmt("{s}/boot/grub", .{iso_dir}),
    };
    var mk_iso_dir_command = b.addSystemCommand(mkdir_command_args);
    mk_iso_dir_command.step.dependOn(b.getInstallStep());


    const move_bin_command_args = &[_][]const u8{
        "cp", b.fmt("{s}/{s}", .{b.exe_dir, kernel.name}), b.fmt("{s}/boot/kernel.elf", .{iso_dir}),
    };
    var move_bin_command = b.addSystemCommand(move_bin_command_args);
    move_bin_command.step.dependOn(&mk_iso_dir_command.step);


    const move_cfg_args = &[_][]const u8{
        "cp", "grub.cfg", b.fmt("{s}/boot/grub/grub.cfg", .{iso_dir})
    };
    var mov_cfg = b.addSystemCommand(move_cfg_args);
    mov_cfg.step.dependOn(&move_bin_command.step);


    const mk_rescue_command_args = &[_][]const u8{
        "grub-mkrescue", "-o", "os.iso", iso_dir
    };
    var rescue_command = b.addSystemCommand(mk_rescue_command_args);
    rescue_command.step.dependOn(&mov_cfg.step);
    

    const qemu_command_args = &[_][]const u8{
        "qemu-system-i386", "-cdrom", "os.iso"
    };

    var qemu_command = b.addSystemCommand(qemu_command_args);
    var qemu = b.step("qemu", "display on QEmu");
    qemu.dependOn(&qemu_command.step);
    qemu_command.step.dependOn(&rescue_command.step);

    const run_step = b.step("run", "Run the kernel");
    run_step.dependOn(qemu);

    const clean_step = b.step("clean", "refresh build");
    const clean_command_args = &[_][]const u8{
        "rm", "-r", "os.iso", ".zig-cache", "zig-out"
    };

    var cleaner = b.addSystemCommand(clean_command_args);
    clean_step.dependOn(&cleaner.step);


}
