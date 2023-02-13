const std = @import("std");
const za = @import("zalgebra");
const Vec3 = za.Vec3;
const Mat4 = za.Mat4;

pub fn main() void {
    const projection = za.perspective(45.0, 800.0 / 600.0, 0.1, 100.0);
    const view = za.lookAt(Vec3.new(0.0, 0.0, -3.0), Vec3.zero(), Vec3.up());
    const model = Mat4.fromTranslate(Vec3.new(0.2, 0.5, 0.0));

    const mvp = Mat4.mul(projection, view.mul(model));
    mvp.debugPrint();
}

test "simple test" {
    const vec1 = Vec3.new(1, 2, 3);
    try std.testing.expectEqual(vec1.z(), 3);
}
