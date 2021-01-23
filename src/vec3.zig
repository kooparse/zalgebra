const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const assert = std.debug.assert;
const testing = std.testing;

pub const vec3 = Vec3(f32);
pub const vec3_f64 = Vec3(f64);
pub const vec3_i32 = Vec3(i32);

/// A 3 dimensional vector.
pub fn Vec3(comptime T: type) type {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .Int) {
        @compileError("Vec3 not implemented for " ++ @typeName(T));
    }

    return packed struct {
        x: T,
        y: T,
        z: T,

        const Self = @This();

        /// Constract vector from given 3 components.
        pub fn new(x: T, y: T, z: T) Self {
            return Self{
                .x = x,
                .y = y,
                .z = z,
            };
        }

        /// Return component from given index.
        pub fn at(self: *const Self, index: i32) T {
            assert(index <= 2);

            if (index == 0) {
                return self.x;
            } else if (index == 1) {
                return self.y;
            } else {
                return self.z;
            }
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return Self.new(val, val, val);
        }

        /// Shorthand for writing vec3.new(0, 0, 0).
        pub fn zero() Self {
            return Self.new(0., 0., 0.);
        }

        /// Shorthand for writing vec3.new(1, 1, 1).
        pub fn one() Self {
            return Self.new(1., 1., 1.);
        }

        /// Shorthand for writing vec3.new(0, 1, 0).
        pub fn up() Self {
            return Self.new(0., 1., 0.);
        }

        /// Shorthand for writing vec3.new(0, -1, 0).
        pub fn down() Self {
            return Self.new(0., -1., 0.);
        }

        /// Shorthand for writing vec3.new(1, 0, 0).
        pub fn right() Self {
            return Self.new(1., 0., 0.);
        }

        /// Shorthand for writing vec3.new(-1, 0, 0).
        pub fn left() Self {
            return Self.new(-1., 0., 0.);
        }

        /// Shorthand for writing vec3.new(0, 0, -1).
        pub fn back() Self {
            return Self.new(0., 0., -1);
        }

        /// Shorthand for writing vec3.new(0, 0, 1).
        pub fn forward() Self {
            return Self.new(0., 0., 1);
        }

        /// Cast a type to another type. Only for integers and floats.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt
        pub fn cast(self: Self, dest: anytype) Vec3(dest) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest);

            if (source_info == .Float and dest_info == .Int) {
                const x = @floatToInt(dest, self.x);
                const y = @floatToInt(dest, self.y);
                const z = @floatToInt(dest, self.z);
                return Vec3(dest).new(x, y, z);
            }

            if (source_info == .Int and dest_info == .Float) {
                const x = @intToFloat(dest, self.x);
                const y = @intToFloat(dest, self.y);
                const z = @intToFloat(dest, self.z);
                return Vec3(dest).new(x, y, z);
            }

            return switch (dest_info) {
                .Float => {
                    const x = @floatCast(dest, self.x);
                    const y = @floatCast(dest, self.y);
                    const z = @floatCast(dest, self.z);
                    return Vec3(dest).new(x, y, z);
                },
                .Int => {
                    const x = @intCast(dest, self.x);
                    const y = @intCast(dest, self.y);
                    const z = @intCast(dest, self.z);
                    return Vec3(dest).new(x, y, z);
                },
                else => panic(
                    "Error, given type should be integers or float.\n",
                    .{},
                ),
            };
        }

        /// Construct new vector from slice.
        pub fn from_slice(slice: []const T) Self {
            return Self.new(slice[0], slice[1], slice[2]);
        }

        /// Transform vector to array.
        pub fn to_array(self: Self) [3]T {
            return .{ self.x, self.y, self.z };
        }

        /// Return the angle in degrees between two vectors.
        pub fn get_angle(lhs: Self, rhs: Self) T {
            const dot_product = Self.dot(lhs.norm(), rhs.norm());
            return root.to_degrees(math.acos(dot_product));
        }

        /// Compute the length (magnitude) of given vector |a|.
        pub fn length(self: Self) T {
            return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z));
        }

        /// Construct new normalized vector from a given vector.
        pub fn norm(self: Self) Self {
            var l = length(self);
            return Self.new(self.x / l, self.y / l, self.z / l);
        }

        pub fn is_eq(lhs: Self, rhs: Self) bool {
            return lhs.x == rhs.x and lhs.y == rhs.y and lhs.z == rhs.z;
        }

        /// Substraction between two given vector.
        pub fn sub(lhs: Self, rhs: Self) Self {
            return Self.new(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
        }

        /// Addition betwen two given vector.
        pub fn add(lhs: Self, rhs: Self) Self {
            return Self.new(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }

        /// Multiply each components by the given scalar.
        pub fn scale(v: Self, scalar: T) Self {
            return Self.new(v.x * scalar, v.y * scalar, v.z * scalar);
        }

        /// Compute the cross product from two vector.
        pub fn cross(lhs: Self, rhs: Self) Self {
            return Self.new(
                (lhs.y * rhs.z) - (lhs.z * rhs.y),
                (lhs.z * rhs.x) - (lhs.x * rhs.z),
                (lhs.x * rhs.y) - (lhs.y * rhs.x),
            );
        }

        /// Return the dot product between two given vector.
        pub fn dot(lhs: Self, rhs: Self) T {
            return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z);
        }

        /// Lerp between two vectors.
        pub fn lerp(lhs: Self, rhs: Self, t: T) Self {
            const x = root.lerp(T, lhs.x, rhs.x, t);
            const y = root.lerp(T, lhs.y, rhs.y, t);
            const z = root.lerp(T, lhs.z, rhs.z, t);
            return Self.new(x, y, z);
        }

        /// Construct a new vector from the min components between two vectors.
        pub fn min(lhs: Self, rhs: Self) Self {
            return Self.new(
                math.min(lhs.x, rhs.x),
                math.min(lhs.y, rhs.y),
                math.min(lhs.z, rhs.z),
            );
        }

        /// Construct a new vector from the max components between two vectors.
        pub fn max(lhs: Self, rhs: Self) Self {
            return Self.new(
                math.max(lhs.x, rhs.x),
                math.max(lhs.y, rhs.y),
                math.max(lhs.z, rhs.z),
            );
        }
    };
}

test "zalgebra.Vec3.init" {
    var _vec_0 = vec3.new(1.5, 2.6, 3.7);

    testing.expectEqual(_vec_0.x, 1.5);
    testing.expectEqual(_vec_0.y, 2.6);
    testing.expectEqual(_vec_0.z, 3.7);
}

test "zalgebra.Vec3.set" {
    var _vec_0 = vec3.new(2.5, 2.5, 2.5);
    var _vec_1 = vec3.set(2.5);
    testing.expectEqual(vec3.is_eq(_vec_0, _vec_1), true);
}

test "zalgebra.Vec3.get_angle" {
    var _vec_0 = vec3.new(1., 0., 0.);
    var _vec_1 = vec3.up();
    var _vec_2 = vec3.new(-1., 0., 0.);
    var _vec_3 = vec3.new(1., 1., 0.);

    testing.expectEqual(vec3.get_angle(_vec_0, _vec_1), 90.);
    testing.expectEqual(vec3.get_angle(_vec_0, _vec_2), 180.);
    testing.expectEqual(vec3.get_angle(_vec_0, _vec_3), 45.);
}

test "zalgebra.Vec3.to_array" {
    const _vec_0 = vec3.up().to_array();
    const _vec_1 = [_]f32{ 0, 1, 0 };

    testing.expectEqual(std.mem.eql(f32, &_vec_0, &_vec_1), true);
}

test "zalgebra.Vec3.is_eq" {
    var _vec_0 = vec3.new(1., 2., 3.);
    var _vec_1 = vec3.new(1., 2., 3.);
    var _vec_2 = vec3.new(1.5, 2., 3.);
    testing.expectEqual(vec3.is_eq(_vec_0, _vec_1), true);
    testing.expectEqual(vec3.is_eq(_vec_0, _vec_2), false);
}

test "zalgebra.Vec3.length" {
    var _vec_0 = vec3.new(1.5, 2.6, 3.7);
    testing.expectEqual(_vec_0.length(), 4.7644519);
}

test "zalgebra.Vec3.normalize" {
    var _vec_0 = vec3.new(1.5, 2.6, 3.7);
    testing.expectEqual(vec3.is_eq(_vec_0.norm(), vec3.new(0.314831584, 0.545708060, 0.776584625)), true);
}

test "zalgebra.Vec3.sub" {
    var _vec_0 = vec3.new(1., 2., 3.);
    var _vec_1 = vec3.new(2., 2., 3.);
    testing.expectEqual(vec3.is_eq(vec3.sub(_vec_0, _vec_1), vec3.new(-1., 0., 0.)), true);
}

test "zalgebra.Vec3.add" {
    var _vec_0 = vec3.new(1., 2., 3.);
    var _vec_1 = vec3.new(2., 2., 3.);
    testing.expectEqual(vec3.is_eq(vec3.add(_vec_0, _vec_1), vec3.new(3., 4., 6.)), true);
}

test "zalgebra.Vec3.scale" {
    var _vec_0 = vec3.new(1., 2., 3.);
    testing.expectEqual(vec3.is_eq(vec3.scale(_vec_0, 5.), vec3.new(5., 10., 15.)), true);
}

test "zalgebra.Vec3.cross" {
    var _vec_0 = vec3.new(1.5, 2.6, 3.7);
    var _vec_1 = vec3.new(2.5, 3.45, 1.0);
    var _vec_2 = vec3.new(1.5, 2.6, 3.7);

    var _cross_product_0 = vec3.cross(_vec_0, _vec_2);
    var _cross_product_1 = vec3.cross(_vec_0, _vec_1);

    testing.expectEqual(vec3.is_eq(_cross_product_0, vec3.new(0., 0., 0.)), true);
    testing.expectEqual(vec3.is_eq(_cross_product_1, vec3.new(-10.1650009, 7.75, -1.32499980)), true);
}

test "zalgebra.Vec3.dot" {
    var _vec_0 = vec3.new(1.5, 2.6, 3.7);
    var _vec_1 = vec3.new(2.5, 3.45, 1.0);

    testing.expectEqual(vec3.dot(_vec_0, _vec_1), 16.42);
}

test "zalgebra.Vec3.lerp" {
    var _vec_0 = vec3.new(-10.0, 0.0, -10.0);
    var _vec_1 = vec3.new(10.0, 10.0, 10.0);

    testing.expectEqual(vec3.is_eq(vec3.lerp(_vec_0, _vec_1, 0.5), vec3.new(0.0, 5.0, 0.0)), true);
}

test "zalgebra.Vec3.min" {
    var _vec_0 = vec3.new(10.0, -2.0, 0.0);
    var _vec_1 = vec3.new(-10.0, 5.0, 0.0);

    testing.expectEqual(vec3.is_eq(vec3.min(_vec_0, _vec_1), vec3.new(-10.0, -2.0, 0.0)), true);
}

test "zalgebra.Vec3.max" {
    var _vec_0 = vec3.new(10.0, -2.0, 0.0);
    var _vec_1 = vec3.new(-10.0, 5.0, 0.0);

    testing.expectEqual(vec3.is_eq(vec3.max(_vec_0, _vec_1), vec3.new(10.0, 5.0, 0.0)), true);
}

test "zalgebra.Vec3.at" {
    const t = vec3.new(10.0, -2.0, 0.0);

    testing.expectEqual(t.at(0), 10.0);
    testing.expectEqual(t.at(1), -2.0);
    testing.expectEqual(t.at(2), 0.0);
}

test "zalgebra.Vec3.from_slice" {
    const array = [3]f32{ 2, 1, 4 };
    testing.expectEqual(vec3.is_eq(vec3.from_slice(&array), vec3.new(2, 1, 4)), true);
}

test "zalgebra.Vec3.cast" {
    const a = vec3_i32.new(3, 6, 2);
    const b = Vec3(usize).new(3, 6, 2);

    testing.expectEqual(
        Vec3(usize).is_eq(a.cast(usize), b),
        true,
    );

    const c = vec3.new(3.5, 6.5, 2.0);
    const d = vec3_f64.new(3.5, 6.5, 2);

    testing.expectEqual(
        vec3_f64.is_eq(c.cast(f64), d),
        true,
    );

    const e = vec3_i32.new(3, 6, 2);
    const f = vec3.new(3.0, 6.0, 2.0);

    testing.expectEqual(
        vec3.is_eq(e.cast(f32), f),
        true,
    );

    const g = vec3.new(3.0, 6.0, 2.0);
    const h = vec3_i32.new(3, 6, 2);

    testing.expectEqual(
        vec3_i32.is_eq(g.cast(i32), h),
        true,
    );
}
