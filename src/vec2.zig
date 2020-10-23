const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const testing = std.testing;

pub const vec2 = Vec2(f32);
pub const vec2_f64 = Vec2(f64);

/// A 2 dimensional vector.
pub fn Vec2(comptime T: type) type {
    if (T != f32 and T != f64) {
        @compileError("Vec2 not implemented for " ++ @typeName(T));
    }

    return packed struct {
        x: T,
        y: T,

        const Self = @This();

        /// Constract vector from given 3 components.
        pub fn new(x: T, y: T) Self {
            return .{ .x = x, .y = y };
        }

        pub fn zero() Self {
            return Self.new(0.0, 0.0);
        }

        pub fn up() Self {
            return Self.new(0.0, 1.0);
        }

        /// Return the angle in degrees between two vectors.
        pub fn get_angle(left: *const Self, right: *const Self) T {
            const dot_product = Self.dot(left.norm(), right.norm());
            return root.to_degrees(math.acos(dot_product));
        }

        /// Compute the length (magnitude) of given vector |a|.
        pub fn length(self: Self) T {
            return math.sqrt((self.x * self.x) + (self.y * self.y));
        }

        /// Construct new normalized vector from a given vector.
        pub fn norm(self: Self) Self {
            var l = length(self);
            return Self.new(self.x / l, self.y / l);
        }

        pub fn is_eq(left: Self, right: Self) bool {
            return left.x == right.x and left.y == right.y;
        }

        /// Substraction between two given vector.
        pub fn sub(left: Self, right: Self) Self {
            return Self.new(left.x - right.x, left.y - right.y);
        }

        /// Addition betwen two given vector.
        pub fn add(left: Self, right: Self) Self {
            return Self.new(left.x + right.x, left.y + right.y);
        }

        /// Multiply each components by the given scalar.
        pub fn scale(v: Self, scalar: T) Self {
            return Self.new(v.x * scalar, v.y * scalar);
        }

        /// Return the dot product between two given vector.
        pub fn dot(left: Self, right: Self) T {
            return (left.x * right.x) + (left.y * right.y);
        }

        /// Lerp between two vectors.
        pub fn lerp(left: *const Self, right: *const Self, t: T) Self {
            const x = root.lerp(T, left.x, right.x, t);
            const y = root.lerp(T, left.y, right.y, t);
            return Self.new(x, y);
        }
    };
}

test "zalgebra.Vec2.init" {
    var _vec_0 = vec2.new(1.5, 2.6);

    testing.expectEqual(_vec_0.x, 1.5);
    testing.expectEqual(_vec_0.y, 2.6);
}

test "zalgebra.Vec2.get_angle" {
    var _vec_0 = vec2.new(1., 0.);
    var _vec_1 = vec2.up();
    var _vec_2 = vec2.new(-1., 0.);
    var _vec_3 = vec2.new(1., 1.);

    testing.expectEqual(vec2.get_angle(&_vec_0, &_vec_1), 90.);
    testing.expectEqual(vec2.get_angle(&_vec_0, &_vec_2), 180.);
    testing.expectEqual(vec2.get_angle(&_vec_0, &_vec_3), 45.);
}

test "zalgebra.Vec2.is_eq" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(1., 2.);
    var _vec_2 = vec2.new(1.5, 2.);
    testing.expectEqual(vec2.is_eq(_vec_0, _vec_1), true);
    testing.expectEqual(vec2.is_eq(_vec_0, _vec_2), false);
}

test "zalgebra.Vec2.length" {
    var _vec_0 = vec2.new(1.5, 2.6);
    testing.expectEqual(_vec_0.length(), 3.00166606);
}

test "zalgebra.Vec2.normalize" {
    var _vec_0 = vec2.new(1.5, 2.6);
    testing.expectEqual(vec2.is_eq(_vec_0.norm(), vec2.new(0.499722480, 0.866185605)), true);
}

test "zalgebra.Vec2.sub" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(2., 2.);
    testing.expectEqual(vec2.is_eq(vec2.sub(_vec_0, _vec_1), vec2.new(-1., 0.)), true);
}

test "zalgebra.Vec2.add" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(2., 2.);
    testing.expectEqual(vec2.is_eq(vec2.add(_vec_0, _vec_1), vec2.new(3., 4.)), true);
}

test "zalgebra.Vec2.scale" {
    var _vec_0 = vec2.new(1., 2.);
    testing.expectEqual(vec2.is_eq(vec2.scale(_vec_0, 5.), vec2.new(5., 10.)), true);
}

test "zalgebra.Vec2.dot" {
    var _vec_0 = vec2.new(1.5, 2.6);
    var _vec_1 = vec2.new(2.5, 3.45);

    testing.expectEqual(vec2.dot(_vec_0, _vec_1), 12.7200002);
}

test "zalgebra.Vec2.lerp" {
    var _vec_0 = vec2.new(-10.0, 0.0);
    var _vec_1 = vec2.new(10.0, 10.0);

    testing.expectEqual(vec2.is_eq(vec2.lerp(&_vec_0, &_vec_1, 0.5), vec2.new(0.0, 5.0)), true);
}
