const math = @import("std").math;
const testing = @import("std").testing;

pub const vec2 = Vec2(f32);

/// A 2 dimensional vector.
pub fn Vec2(comptime T: type) type {
    return packed struct {
        x: T,
        y: T,

        const Self = @This();

        /// Constract vector from given 3 components.
        pub fn new(x: T, y: T) Self {
            return .{ .x = x, .y = y };
        }

        pub fn zeros() Self {
            return Self{ 0.0, 0.0 };
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

        pub fn isEq(left: Self, right: Self) bool {
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
    };
}

test "zalgebra.Vec2.init" {
    var _vec_0 = vec2.new(1.5, 2.6);

    testing.expectEqual(_vec_0.x, 1.5);
    testing.expectEqual(_vec_0.y, 2.6);
}

test "zalgebra.Vec2.isEq" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(1., 2.);
    var _vec_2 = vec2.new(1.5, 2.);
    testing.expectEqual(vec2.isEq(_vec_0, _vec_1), true);
    testing.expectEqual(vec2.isEq(_vec_0, _vec_2), false);
}

test "zalgebra.Vec2.length" {
    var _vec_0 = vec2.new(1.5, 2.6);
    testing.expectEqual(_vec_0.length(), 3.00166606);
}

test "zalgebra.Vec2.normalize" {
    var _vec_0 = vec2.new(1.5, 2.6);
    testing.expectEqual(vec2.isEq(_vec_0.norm(), vec2.new(0.499722480, 0.866185605)), true);
}

test "zalgebra.Vec2.sub" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(2., 2.);
    testing.expectEqual(vec2.isEq(vec2.sub(_vec_0, _vec_1), vec2.new(-1., 0.)), true);
}

test "zalgebra.Vec2.add" {
    var _vec_0 = vec2.new(1., 2.);
    var _vec_1 = vec2.new(2., 2.);
    testing.expectEqual(vec2.isEq(vec2.add(_vec_0, _vec_1), vec2.new(3., 4.)), true);
}

test "zalgebra.Vec2.scale" {
    var _vec_0 = vec2.new(1., 2.);
    testing.expectEqual(vec2.isEq(vec2.scale(_vec_0, 5.), vec2.new(5., 10.)), true);
}

test "zalgebra.Vec2.dot" {
    var _vec_0 = vec2.new(1.5, 2.6);
    var _vec_1 = vec2.new(2.5, 3.45);

    testing.expectEqual(vec2.dot(_vec_0, _vec_1), 12.7200002);
}
