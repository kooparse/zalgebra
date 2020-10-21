//! Math utilities for graphics.
//!
const std = @import("std");
const testing = std.testing;
const math = std.math;

pub usingnamespace @import("vec2.zig");
pub usingnamespace @import("vec3.zig");
pub usingnamespace @import("vec4.zig");
pub usingnamespace @import("mat4.zig");

/// Convert degrees to radians.
pub fn to_radians(degrees: anytype) @TypeOf(degrees) {
    const T = @TypeOf(degrees);

    return switch (T) {
        f32, f64 => degrees * (math.pi / 180.0),
        else => @compileError("Radians not implemented for " ++ @typeName(T)),
    };
}

test "zalgebra.to_radians" {
    testing.expectEqual(to_radians(@as(f32, 90)), 1.57079637);
    testing.expectEqual(to_radians(@as(f32, 45)), 0.785398185);
    testing.expectEqual(to_radians(@as(f32, 360)), 6.28318548);
    testing.expectEqual(to_radians(@as(f32, 0)), 0.0);
}

/// Convert radians to degrees.
pub fn to_degrees(radians: anytype) @TypeOf(radians) {
    const T = @TypeOf(radians);

    return switch (T) {
        f32, f64 => radians * (180.0 / math.pi),
        else => @compileError("Degrees not implemented for " ++ @typeName(T)),
    };
}

test "zalgebra.to_degrees" {
    testing.expectEqual(to_degrees(@as(f32, 0.5)), 28.6478900);
    testing.expectEqual(to_degrees(@as(f32, 1.0)), 57.2957801);
    testing.expectEqual(to_degrees(@as(f32, 0.0)), 0.0);
}
