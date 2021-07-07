//! Math utilities for graphics.
//!
const std = @import("std");
const testing = std.testing;
const math = std.math;

pub usingnamespace @import("vec2.zig");
pub usingnamespace @import("vec3.zig");
pub usingnamespace @import("vec4.zig");
pub usingnamespace @import("mat4.zig");
pub usingnamespace @import("quaternion.zig");

/// Convert degrees to radians.
pub fn toRadians(degrees: anytype) @TypeOf(degrees) {
    const T = @TypeOf(degrees);

    return switch (@typeInfo(T)) {
        .Float => degrees * (math.pi / 180.0),
        else => @compileError("Radians not implemented for " ++ @typeName(T)),
    };
}

/// Convert radians to degrees.
pub fn toDegrees(radians: anytype) @TypeOf(radians) {
    const T = @TypeOf(radians);

    return switch (@typeInfo(T)) {
        .Float => radians * (180.0 / math.pi),
        else => @compileError("Degrees not implemented for " ++ @typeName(T)),
    };
}

/// Linear interpolation between two floats.
/// `t` is used to interpolate between `from` and `to`.
pub fn lerp(comptime T: type, from: T, to: T, t: T) T {
    return switch (@typeInfo(T)) {
        .Float => (1 - t) * from + t * to,
        else => @compileError("Lerp not implemented for " ++ @typeName(T)),
    };
}

test "zalgebra.toRadians" {
    try testing.expectEqual(toRadians(@as(f32, 90)), 1.57079637);
    try testing.expectEqual(toRadians(@as(f32, 45)), 0.785398185);
    try testing.expectEqual(toRadians(@as(f32, 360)), 6.28318548);
    try testing.expectEqual(toRadians(@as(f32, 0)), 0.0);
}

test "zalgebra.toDegrees" {
    try testing.expectEqual(toDegrees(@as(f32, 0.5)), 28.6478900);
    try testing.expectEqual(toDegrees(@as(f32, 1.0)), 57.2957801);
    try testing.expectEqual(toDegrees(@as(f32, 0.0)), 0.0);
}

test "zalgebra.lerp" {
    const from: f32 = 0;
    const to: f32 = 10;

    try testing.expectEqual(lerp(f32, from, to, 0), 0);
    try testing.expectEqual(lerp(f32, from, to, 0.5), 5);
    try testing.expectEqual(lerp(f32, from, to, 1), 10);
}
