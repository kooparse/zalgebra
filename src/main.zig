//! Math utilities for graphics.
//!
const std = @import("std");
const expectEqual = std.testing.expectEqual;
const math = std.math;

pub usingnamespace @import("generic_vector.zig");
pub usingnamespace @import("mat3.zig");
pub usingnamespace @import("mat4.zig");
pub usingnamespace @import("quaternion.zig");

/// Convert degrees to radians.
pub fn toRadians(degrees: anytype) @TypeOf(degrees) {
    const T = @TypeOf(degrees);

    if (@typeInfo(T) != .float) {
        @compileError("Radians not implemented for " ++ @typeName(T));
    }

    return degrees * (math.pi / 180.0);
}

/// Convert radians to degrees.
pub fn toDegrees(radians: anytype) @TypeOf(radians) {
    const T = @TypeOf(radians);

    if (@typeInfo(T) != .float) {
        @compileError("Radians not implemented for " ++ @typeName(T));
    }

    return radians * (180.0 / math.pi);
}

/// Linear interpolation between two floats.
/// `t` is used to interpolate between `from` and `to`.
pub fn lerp(comptime T: type, from: T, to: T, t: T) T {
    if (@typeInfo(T) != .float) {
        @compileError("Lerp not implemented for " ++ @typeName(T));
    }

    return (1 - t) * from + t * to;
}

test "zalgebra.toRadians" {
    try expectEqual(toRadians(@as(f32, 0)), 0);
    try expectEqual(toRadians(@as(f32, 30)), 0.523598790);
    try expectEqual(toRadians(@as(f32, 45)), 0.78539818);
    try expectEqual(toRadians(@as(f32, 60)), 1.04719758);
    try expectEqual(toRadians(@as(f32, 90)), 1.57079637); //math.pi / 2
    try expectEqual(toRadians(@as(f32, 180)), 3.14159274); //math.pi
    try expectEqual(toRadians(@as(f32, 270)), 4.71238899);
    try expectEqual(toRadians(@as(f32, 360)), 6.28318548); //math.pi * 2
}

test "zalgebra.toDegrees" {
    try expectEqual(toDegrees(@as(f32, 0)), 0);
    try expectEqual(toDegrees(@as(f32, 0.5)), 28.6478900);
    try expectEqual(toDegrees(@as(f32, 1)), 57.2957801);
    try expectEqual(toDegrees(@as(f32, 1.57079637)), 90); //math.pi / 2
    try expectEqual(toDegrees(@as(f32, 3.14159274)), 180); //math.pi
    try expectEqual(toDegrees(@as(f32, 4.71238899)), 270);
    try expectEqual(toDegrees(@as(f32, 6.28318548)), 360); //math.pi * 2
}

test "zalgebra.lerp" {
    const from: f32 = 0;
    const to: f32 = 10;

    try expectEqual(lerp(f32, from, to, 0), 0);
    try expectEqual(lerp(f32, from, to, 0.5), 5);
    try expectEqual(lerp(f32, from, to, 1), 10);
}
