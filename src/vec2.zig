const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const testing = std.testing;

pub const vec2 = Vec2(f32);
pub const vec2_f64 = Vec2(f64);
pub const vec2_i32 = Vec2(i32);

/// A 2 dimensional vector.
pub fn Vec2(comptime T: type) type {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .Int) {
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

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return Self.new(val, val);
        }

        pub fn zero() Self {
            return Self.new(0.0, 0.0);
        }

        pub fn up() Self {
            return Self.new(0.0, 1.0);
        }

        /// Cast a type to another type. Only for integers and floats.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt
        pub fn cast(self: Self, dest: anytype) Vec2(dest) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest);

            if (source_info == .Float and dest_info == .Int) {
                const x = @floatToInt(dest, self.x);
                const y = @floatToInt(dest, self.y);
                return Vec2(dest).new(x, y);
            }

            if (source_info == .Int and dest_info == .Float) {
                const x = @intToFloat(dest, self.x);
                const y = @intToFloat(dest, self.y);
                return Vec2(dest).new(x, y);
            }

            return switch (dest_info) {
                .Float => {
                    const x = @floatCast(dest, self.x);
                    const y = @floatCast(dest, self.y);
                    return Vec2(dest).new(x, y);
                },
                .Int => {
                    const x = @intCast(dest, self.x);
                    const y = @intCast(dest, self.y);
                    return Vec2(dest).new(x, y);
                },
                else => panic(
                    "Error, given type should be integers or float.\n",
                    .{},
                ),
            };
        }

        /// Construct new vector from slice.
        pub fn from_slice(slice: []const T) Self {
            return Self.new(slice[0], slice[1]);
        }

        /// Transform vector to array.
        pub fn to_array(self: Self) [2]T {
            return .{ self.x, self.y };
        }

        /// Return the angle in degrees between two vectors.
        pub fn get_angle(left: Self, right: Self) T {
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
        pub fn lerp(left: Self, right: Self, t: T) Self {
            const x = root.lerp(T, left.x, right.x, t);
            const y = root.lerp(T, left.y, right.y, t);
            return Self.new(x, y);
        }

        /// Construct a new vector from the min components between two vectors.
        pub fn min(left: Self, right: Self) Self {
            return Self.new(
                math.min(left.x, right.x),
                math.min(left.y, right.y),
            );
        }

        /// Construct a new vector from the max components between two vectors.
        pub fn max(left: Self, right: Self) Self {
            return Self.new(
                math.max(left.x, right.x),
                math.max(left.y, right.y),
            );
        }
    };
}

test "zalgebra.Vec2.init" {
    var _vec_0 = vec2.new(1.5, 2.6);

    testing.expectEqual(_vec_0.x, 1.5);
    testing.expectEqual(_vec_0.y, 2.6);
}

test "zalgebra.Vec2.set" {
    var _vec_0 = vec2.new(2.5, 2.5);
    var _vec_1 = vec2.set(2.5);
    testing.expectEqual(vec2.is_eq(_vec_0, _vec_1), true);
}

test "zalgebra.Vec2.get_angle" {
    var _vec_0 = vec2.new(1., 0.);
    var _vec_1 = vec2.up();
    var _vec_2 = vec2.new(-1., 0.);
    var _vec_3 = vec2.new(1., 1.);

    testing.expectEqual(vec2.get_angle(_vec_0, _vec_1), 90.);
    testing.expectEqual(vec2.get_angle(_vec_0, _vec_2), 180.);
    testing.expectEqual(vec2.get_angle(_vec_0, _vec_3), 45.);
}

test "zalgebra.Vec2.to_array" {
    const _vec_0 = vec2.up().to_array();
    const _vec_1 = [_]f32{ 0, 1 };

    testing.expectEqual(std.mem.eql(f32, &_vec_0, &_vec_1), true);
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

    testing.expectEqual(vec2.is_eq(vec2.lerp(_vec_0, _vec_1, 0.5), vec2.new(0.0, 5.0)), true);
}

test "zalgebra.Vec2.min" {
    var _vec_0 = vec2.new(10.0, -2.0);
    var _vec_1 = vec2.new(-10.0, 5.0);

    testing.expectEqual(vec2.is_eq(vec2.min(_vec_0, _vec_1), vec2.new(-10.0, -2.0)), true);
}

test "zalgebra.Vec2.max" {
    var _vec_0 = vec2.new(10.0, -2.0);
    var _vec_1 = vec2.new(-10.0, 5.0);

    testing.expectEqual(vec2.is_eq(vec2.max(_vec_0, _vec_1), vec2.new(10.0, 5.0)), true);
}

test "zalgebra.Vec2.from_slice" {
    const array = [2]f32{ 2, 4 };
    testing.expectEqual(vec2.is_eq(vec2.from_slice(&array), vec2.new(2, 4)), true);
}

test "zalgebra.Vec2.cast" {
    const a = vec2_i32.new(3, 6);
    const b = Vec2(usize).new(3, 6);

    testing.expectEqual(
        Vec2(usize).is_eq(a.cast(usize), b),
        true,
    );

    const c = vec2.new(3.5, 6.5);
    const d = vec2_f64.new(3.5, 6.5);

    testing.expectEqual(
        vec2_f64.is_eq(c.cast(f64), d),
        true,
    );

    const e = vec2_i32.new(3, 6);
    const f = vec2.new(3.0, 6.0);

    testing.expectEqual(
        vec2.is_eq(e.cast(f32), f),
        true,
    );

    const g = vec2.new(3.0, 6.0);
    const h = vec2_i32.new(3, 6);

    testing.expectEqual(
        vec2_i32.is_eq(g.cast(i32), h),
        true,
    );
}
