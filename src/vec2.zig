const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const testing = std.testing;
const panic = std.debug.panic;

pub const Vec2 = Vector2(f32);
pub const Vec2_f64 = Vector2(f64);
pub const Vec2_i32 = Vector2(i32);

/// A 2 dimensional vector.
pub fn Vector2(comptime T: type) type {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .Int) {
        @compileError("Vector2 not implemented for " ++ @typeName(T));
    }

    return struct {
        x: T,
        y: T,

        const Self = @This();

        /// Construct vector from given 2 components.
        pub fn new(x: T, y: T) Self {
            return .{ .x = x, .y = y };
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return Self.new(val, val);
        }

        pub fn zero() Self {
            return Self.new(0, 0);
        }

        pub fn one() Self {
            return Self.new(1, 1);
        }

        pub fn up() Self {
            return Self.new(0, 1);
        }

        /// Cast a type to another type. Only for integers and floats.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt.
        pub fn cast(self: Self, dest: anytype) Vector2(dest) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest);

            if (source_info == .Float and dest_info == .Int) {
                const x = @floatToInt(dest, self.x);
                const y = @floatToInt(dest, self.y);
                return Vector2(dest).new(x, y);
            }

            if (source_info == .Int and dest_info == .Float) {
                const x = @intToFloat(dest, self.x);
                const y = @intToFloat(dest, self.y);
                return Vector2(dest).new(x, y);
            }

            return switch (dest_info) {
                .Float => {
                    const x = @floatCast(dest, self.x);
                    const y = @floatCast(dest, self.y);
                    return Vector2(dest).new(x, y);
                },
                .Int => {
                    const x = @intCast(dest, self.x);
                    const y = @intCast(dest, self.y);
                    return Vector2(dest).new(x, y);
                },
                else => panic(
                    "Error, given type should be integer or float.\n",
                    .{},
                ),
            };
        }

        /// Construct new vector from slice.
        pub fn fromSlice(slice: []const T) Self {
            return Self.new(slice[0], slice[1]);
        }

        /// Transform vector to array.
        pub fn toArray(self: Self) [2]T {
            return .{ self.x, self.y };
        }

        /// Return the angle in degrees between two vectors.
        pub fn getAngle(left: Self, right: Self) T {
            const dot_product = Self.dot(left.norm(), right.norm());
            return root.toDegrees(math.acos(dot_product));
        }

        /// Compute the length (magnitude) of given vector |a|.
        pub fn length(self: Self) T {
            return math.sqrt((self.x * self.x) + (self.y * self.y));
        }

        /// Compute the distance between two points.
        pub fn distance(a: Self, b: Self) T {
            return math.sqrt(
                math.pow(T, b.x - a.x, 2) + math.pow(T, b.y - a.y, 2),
            );
        }

        /// Construct new normalized vector from a given vector.
        pub fn norm(self: Self) Self {
            var l = length(self);
            return Self.new(self.x / l, self.y / l);
        }

        pub fn eql(left: Self, right: Self) bool {
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
    var a = Vec2.new(1.5, 2.6);

    try testing.expectEqual(a.x, 1.5);
    try testing.expectEqual(a.y, 2.6);
}

test "zalgebra.Vec2.set" {
    var a = Vec2.new(2.5, 2.5);
    var b = Vec2.set(2.5);
    try testing.expectEqual(Vec2.eql(a, b), true);
}

test "zalgebra.Vec2.getAngle" {
    var a = Vec2.new(1, 0);
    var b = Vec2.up();
    var c = Vec2.new(-1, 0);
    var d = Vec2.new(1, 1);

    try testing.expectEqual(Vec2.getAngle(a, b), 90);
    try testing.expectEqual(Vec2.getAngle(a, c), 180);
    try testing.expectEqual(Vec2.getAngle(a, d), 45);
}

test "zalgebra.Vec2.toArray" {
    const a = Vec2.up().toArray();
    const b = [_]f32{ 0, 1 };

    try testing.expectEqual(std.mem.eql(f32, &a, &b), true);
}

test "zalgebra.Vec2.eql" {
    var a = Vec2.new(1, 2);
    var b = Vec2.new(1, 2);
    var c = Vec2.new(1.5, 2);
    try testing.expectEqual(Vec2.eql(a, b), true);
    try testing.expectEqual(Vec2.eql(a, c), false);
}

test "zalgebra.Vec2.length" {
    var a = Vec2.new(1.5, 2.6);
    try testing.expectEqual(a.length(), 3.00166606);
}

test "zalgebra.Vec2.distance" {
    var a = Vec2.new(0, 0);
    var b = Vec2.new(-1, 0);
    var c = Vec2.new(0, 5);

    try testing.expectEqual(Vec2.distance(a, b), 1);
    try testing.expectEqual(Vec2.distance(a, c), 5);
}

test "zalgebra.Vec2.normalize" {
    var a = Vec2.new(1.5, 2.6);
    try testing.expectEqual(Vec2.eql(a.norm(), Vec2.new(0.499722480, 0.866185605)), true);
}

test "zalgebra.Vec2.sub" {
    var a = Vec2.new(1, 2);
    var b = Vec2.new(2, 2);
    try testing.expectEqual(Vec2.eql(Vec2.sub(a, b), Vec2.new(-1, 0)), true);
}

test "zalgebra.Vec2.add" {
    var a = Vec2.new(1, 2);
    var b = Vec2.new(2, 2);
    try testing.expectEqual(Vec2.eql(Vec2.add(a, b), Vec2.new(3, 4)), true);
}

test "zalgebra.Vec2.scale" {
    var a = Vec2.new(1, 2);
    try testing.expectEqual(Vec2.eql(Vec2.scale(a, 5), Vec2.new(5, 10)), true);
}

test "zalgebra.Vec2.dot" {
    var a = Vec2.new(1.5, 2.6);
    var b = Vec2.new(2.5, 3.45);

    try testing.expectEqual(Vec2.dot(a, b), 12.7200002);
}

test "zalgebra.Vec2.lerp" {
    var a = Vec2.new(-10.0, 0.0);
    var b = Vec2.new(10.0, 10.0);

    try testing.expectEqual(Vec2.eql(Vec2.lerp(a, b, 0.5), Vec2.new(0.0, 5.0)), true);
}

test "zalgebra.Vec2.min" {
    var a = Vec2.new(10.0, -2.0);
    var b = Vec2.new(-10.0, 5.0);

    try testing.expectEqual(Vec2.eql(Vec2.min(a, b), Vec2.new(-10.0, -2.0)), true);
}

test "zalgebra.Vec2.max" {
    var a = Vec2.new(10.0, -2.0);
    var b = Vec2.new(-10.0, 5.0);

    try testing.expectEqual(Vec2.eql(Vec2.max(a, b), Vec2.new(10.0, 5.0)), true);
}

test "zalgebra.Vec2.fromSlice" {
    const array = [2]f32{ 2, 4 };
    try testing.expectEqual(Vec2.eql(Vec2.fromSlice(&array), Vec2.new(2, 4)), true);
}

test "zalgebra.Vec2.cast" {
    const a = Vec2_i32.new(3, 6);
    const b = Vector2(usize).new(3, 6);

    try testing.expectEqual(
        Vector2(usize).eql(a.cast(usize), b),
        true,
    );

    const c = Vec2.new(3.5, 6.5);
    const d = Vec2_f64.new(3.5, 6.5);

    try testing.expectEqual(
        Vec2_f64.eql(c.cast(f64), d),
        true,
    );

    const e = Vec2_i32.new(3, 6);
    const f = Vec2.new(3.0, 6.0);

    try testing.expectEqual(
        Vec2.eql(e.cast(f32), f),
        true,
    );

    const g = Vec2.new(3.0, 6.0);
    const h = Vec2_i32.new(3, 6);

    try testing.expectEqual(
        Vec2_i32.eql(g.cast(i32), h),
        true,
    );
}
