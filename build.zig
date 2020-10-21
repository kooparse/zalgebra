const std = @import("std");
const Builder = std.build.Builder;

pub fn build(b: *Builder) void {
    const mode = b.standardReleaseOptions();

    var lib = b.addStaticLibrary("zalgebra", "src/main.zig");
    lib.setBuildMode(mode);

    var tests = b.addTest("src/main.zig");
    tests.setBuildMode(mode);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&tests.step);

    b.default_step.dependOn(&lib.step);
    lib.install();
}
