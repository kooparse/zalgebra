const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const panic = std.debug.panic;
const testing = std.testing;

pub const Vec4 = Vector4(f32);
pub const Vec4_f64 = Vector4(f64);
pub const Vec4_i32 = Vector4(i32);

/// A 4 dimensional vector.
pub fn Vector4(comptime T: type) type {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .Int) {
        @compileError("Vector4 not implemented for " ++ @typeName(T));
    }

    return struct {
        x: T,
        y: T,
        z: T,
        w: T,

        const Self = @This();

        /// Constract vector from given 3 components.
        pub fn new(x: T, y: T, z: T, w: T) Self {
            return Self{
                .x = x,
                .y = y,
                .z = z,
                .w = w,
            };
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return Self.new(val, val, val, val);
        }

        pub fn zero() Self {
            return Self.new(0, 0, 0, 0);
        }

        pub fn one() Self {
            return Self.new(1, 1, 1, 1);
        }

        /// Cast a type to another type. Only for integers and floats.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt
        pub fn cast(self: Self, dest: anytype) Vector4(dest) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest);

            if (source_info == .Float and dest_info == .Int) {
                const x = @floatToInt(dest, self.x);
                const y = @floatToInt(dest, self.y);
                const z = @floatToInt(dest, self.z);
                const w = @floatToInt(dest, self.w);
                return Vector4(dest).new(x, y, z, w);
            }

            if (source_info == .Int and dest_info == .Float) {
                const x = @intToFloat(dest, self.x);
                const y = @intToFloat(dest, self.y);
                const z = @intToFloat(dest, self.z);
                const w = @intToFloat(dest, self.w);
                return Vector4(dest).new(x, y, z, w);
            }

            return switch (dest_info) {
                .Float => {
                    const x = @floatCast(dest, self.x);
                    const y = @floatCast(dest, self.y);
                    const z = @floatCast(dest, self.z);
                    const w = @floatCast(dest, self.w);
                    return Vector4(dest).new(x, y, z, w);
                },
                .Int => {
                    const x = @intCast(dest, self.x);
                    const y = @intCast(dest, self.y);
                    const z = @intCast(dest, self.z);
                    const w = @intCast(dest, self.w);
                    return Vector4(dest).new(x, y, z, w);
                },
                else => panic(
                    "Error, given type should be integers or float.\n",
                    .{},
                ),
            };
        }

        /// Construct new vector from slice.
        pub fn fromSlice(slice: []const T) Self {
            return Self.new(slice[0], slice[1], slice[2], slice[3]);
        }

        /// Transform vector to array.
        pub fn toArray(self: Self) [4]T {
            return .{ self.x, self.y, self.z, self.w };
        }

        /// Compute the length (magnitude) of given vector |a|.
        pub fn length(self: Self) T {
            return math.sqrt(
                (self.x * self.x) +
                    (self.y * self.y) +
                    (self.z * self.z) +
                    (self.w * self.w),
            );
        }

        /// Compute the distance between two points.
        pub fn distance(a: Self, b: Self) T {
            return math.sqrt(
                math.pow(T, b.x - a.x, 2) +
                    math.pow(T, b.y - a.y, 2) +
                    math.pow(T, b.z - a.z, 2) +
                    math.pow(T, b.w - a.w, 2),
            );
        }

        /// Construct new normalized vector from a given vector.
        pub fn norm(self: Self) Self {
            var l = length(self);
            return Self.new(self.x / l, self.y / l, self.z / l, self.w / l);
        }

        pub fn is_eq(left: Self, right: Self) bool {
            return left.x == right.x and
                left.y == right.y and
                left.z == right.z and
                left.w == right.w;
        }

        /// Substraction between two given vector.
        pub fn sub(left: Self, right: Self) Self {
            return Self.new(
                left.x - right.x,
                left.y - right.y,
                left.z - right.z,
                left.w - right.w,
            );
        }

        /// Addition betwen two given vector.
        pub fn add(left: Self, right: Self) Self {
            return Self.new(
                left.x + right.x,
                left.y + right.y,
                left.z + right.z,
                left.w + right.w,
            );
        }

        /// Multiply each components by the given scalar.
        pub fn scale(v: Self, scalar: T) Self {
            return Self.new(
                v.x * scalar,
                v.y * scalar,
                v.z * scalar,
                v.w * scalar,
            );
        }

        /// Return the dot product between two given vector.
        pub fn dot(left: Self, right: Self) T {
            return (left.x * right.x) + (left.y * right.y) + (left.z * right.z) + (left.w * right.w);
        }

        /// Lerp between two vectors.
        pub fn lerp(left: Self, right: Self, t: T) Self {
            const x = root.lerp(T, left.x, right.x, t);
            const y = root.lerp(T, left.y, right.y, t);
            const z = root.lerp(T, left.z, right.z, t);
            const w = root.lerp(T, left.w, right.w, t);
            return Self.new(x, y, z, w);
        }

        /// Construct a new vector from the min components between two vectors.
        pub fn min(left: Self, right: Self) Self {
            return Self.new(
                math.min(left.x, right.x),
                math.min(left.y, right.y),
                math.min(left.z, right.z),
                math.min(left.w, right.w),
            );
        }

        /// Construct a new vector from the max components between two vectors.
        pub fn max(left: Self, right: Self) Self {
            return Self.new(
                math.max(left.x, right.x),
                math.max(left.y, right.y),
                math.max(left.z, right.z),
                math.max(left.w, right.w),
            );
        }
    };
}

test "zalgebra.Vec4.init" {
    var _vec_0 = Vec4.new(1.5, 2.6, 3.7, 4.7);

    try testing.expectEqual(_vec_0.x, 1.5);
    try testing.expectEqual(_vec_0.y, 2.6);
    try testing.expectEqual(_vec_0.z, 3.7);
    try testing.expectEqual(_vec_0.w, 4.7);
}

test "zalgebra.Vec4.is_eq" {
    var _vec_0 = Vec4.new(1, 2, 3, 4);
    var _vec_1 = Vec4.new(1, 2, 3, 4);
    var _vec_2 = Vec4.new(1, 2, 3, 5);
    try testing.expectEqual(Vec4.is_eq(_vec_0, _vec_1), true);
    try testing.expectEqual(Vec4.is_eq(_vec_0, _vec_2), false);
}

test "zalgebra.Vec4.set" {
    var _vec_0 = Vec4.new(2.5, 2.5, 2.5, 2.5);
    var _vec_1 = Vec4.set(2.5);
    try testing.expectEqual(Vec4.is_eq(_vec_0, _vec_1), true);
}

test "zalgebra.Vec2.toArray" {
    const _vec_0 = Vec4.new(0, 1, 0, 1).toArray();
    const _vec_1 = [_]f32{ 0, 1, 0, 1 };

    try testing.expectEqual(std.mem.eql(f32, &_vec_0, &_vec_1), true);
}

test "zalgebra.Vec4.length" {
    var _vec_0 = Vec4.new(1.5, 2.6, 3.7, 4.7);
    try testing.expectEqual(_vec_0.length(), 6.69253301);
}

test "zalgebra.Vec4.distance" {
    var a = Vec4.new(0, 0, 0, 0);
    var b = Vec4.new(-1, 0, 0, 0);
    var c = Vec4.new(0, 5, 0, 0);

    try testing.expectEqual(Vec4.distance(a, b), 1);
    try testing.expectEqual(Vec4.distance(a, c), 5);
}

test "zalgebra.Vec4.normalize" {
    var _vec_0 = Vec4.new(1.5, 2.6, 3.7, 4.0);
    try testing.expectEqual(Vec4.is_eq(
        _vec_0.norm(),
        Vec4.new(0.241121411, 0.417943745, 0.594766139, 0.642990410),
    ), true);
}

test "zalgebra.Vec4.sub" {
    var _vec_0 = Vec4.new(1, 2, 3, 6);
    var _vec_1 = Vec4.new(2, 2, 3, 5);
    try testing.expectEqual(Vec4.is_eq(
        Vec4.sub(_vec_0, _vec_1),
        Vec4.new(-1, 0, 0, 1),
    ), true);
}

test "zalgebra.Vec4.add" {
    var _vec_0 = Vec4.new(1, 2, 3, 5);
    var _vec_1 = Vec4.new(2, 2, 3, 6);
    try testing.expectEqual(Vec4.is_eq(
        Vec4.add(_vec_0, _vec_1),
        Vec4.new(3, 4, 6, 11),
    ), true);
}

test "zalgebra.Vec4.scale" {
    var _vec_0 = Vec4.new(1, 2, 3, 4);
    try testing.expectEqual(Vec4.is_eq(
        Vec4.scale(_vec_0, 5),
        Vec4.new(5, 10, 15, 20),
    ), true);
}

test "zalgebra.Vec4.dot" {
    var _vec_0 = Vec4.new(1.5, 2.6, 3.7, 5);
    var _vec_1 = Vec4.new(2.5, 3.45, 1.0, 1);

    try testing.expectEqual(Vec4.dot(_vec_0, _vec_1), 21.4200000);
}

test "zalgebra.Vec4.lerp" {
    var _vec_0 = Vec4.new(-10.0, 0.0, -10.0, -10.0);
    var _vec_1 = Vec4.new(10.0, 10.0, 10.0, 10.0);

    try testing.expectEqual(Vec4.is_eq(
        Vec4.lerp(_vec_0, _vec_1, 0.5),
        Vec4.new(0.0, 5.0, 0.0, 0.0),
    ), true);
}

test "zalgebra.Vec4.min" {
    var _vec_0 = Vec4.new(10.0, -2.0, 0.0, 1.0);
    var _vec_1 = Vec4.new(-10.0, 5.0, 0.0, 1.01);

    try testing.expectEqual(Vec4.is_eq(
        Vec4.min(_vec_0, _vec_1),
        Vec4.new(-10.0, -2.0, 0.0, 1.0),
    ), true);
}

test "zalgebra.Vec4.max" {
    var _vec_0 = Vec4.new(10.0, -2.0, 0.0, 1.0);
    var _vec_1 = Vec4.new(-10.0, 5.0, 0.0, 1.01);

    try testing.expectEqual(Vec4.is_eq(
        Vec4.max(_vec_0, _vec_1),
        Vec4.new(10.0, 5.0, 0.0, 1.01),
    ), true);
}

test "zalgebra.Vec2.fromSlice" {
    const array = [4]f32{ 2, 4, 3, 6 };
    try testing.expectEqual(Vec4.is_eq(
        Vec4.fromSlice(&array),
        Vec4.new(2, 4, 3, 6),
    ), true);
}

test "zalgebra.Vec4.cast" {
    const a = Vec4_i32.new(3, 6, 2, 0);
    const b = Vector4(usize).new(3, 6, 2, 0);

    try testing.expectEqual(
        Vector4(usize).is_eq(a.cast(usize), b),
        true,
    );

    const c = Vec4.new(3.5, 6.5, 2.0, 0);
    const d = Vec4_f64.new(3.5, 6.5, 2, 0.0);

    try testing.expectEqual(
        Vec4_f64.is_eq(c.cast(f64), d),
        true,
    );

    const e = Vec4_i32.new(3, 6, 2, 0);
    const f = Vec4.new(3.0, 6.0, 2.0, 0.0);

    try testing.expectEqual(
        Vec4.is_eq(e.cast(f32), f),
        true,
    );

    const g = Vec4.new(3.0, 6.0, 2.0, 0.0);
    const h = Vec4_i32.new(3, 6, 2, 0);

    try testing.expectEqual(
        Vec4_i32.is_eq(g.cast(i32), h),
        true,
    );
}
