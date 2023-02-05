const std = @import("std");
const za = @import("zalgebra");
const Vec3 = za.Vec3;
const Mat4 = za.Mat4;

pub fn main() !void {
    var projection = za.perspective(45.0, 800.0 / 600.0, 0.1, 100.0);
    var view = za.lookAt(Vec3.new(0.0, 0.0, -3.0), Vec3.zero(), Vec3.up());
    var model = Mat4.fromTranslate(Vec3.new(0.2, 0.5, 0.0));

    var mvp = Mat4.mul(projection, view.mul(model));
    _ = mvp;
}

test "simple test" {
    const vec1 = Vec3.new(1, 2, 3);
    try std.testing.expectEqual(vec1.z(), 3);

    var list = std.ArrayList(i32).init(std.testing.allocator);
    defer list.deinit(); // try commenting this out and see if zig detects the memory leak!
    try list.append(42);
    try std.testing.expectEqual(@as(i32, 42), list.pop());
}
