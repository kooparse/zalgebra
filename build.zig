const std = @import("std");
const Builder = std.build.Builder;

pub const pkg = std.build.Pkg{
    .name = "zalgebra",
    .source = .{ .path = thisDir() ++ "/src/main.zig" },
};

pub fn build(b: *Builder) void {
    const mode = b.standardReleaseOptions();

    const tests = b.addTest("src/main.zig");
    tests.setBuildMode(mode);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&tests.step);
}

fn thisDir() []const u8 {
    return std.fs.path.dirname(@src().file) orelse ".";
}
