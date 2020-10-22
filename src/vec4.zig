const math = @import("std").math;
const testing = @import("std").testing;

pub const vec4 = Vec4(f32);
pub const vec4_f64 = Vec4(f64);

/// A 4 dimensional vector.
pub fn Vec4(comptime T: type) type {
    if (T!= f32 and T != f64) {
        @compileError("Vec4 not implemented for " ++ @typeName(T));
    }

    return packed struct {
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

        pub fn zero() Self {
            return Self.new(0., 0., 0., 0.);
        }

        /// Compute the length (magnitude) of given vector |a|.
        pub fn length(self: Self) T {
            return math.sqrt((self.x * self.x) + (self.y * self.y) + (self.z * self.z) + (self.w * self.w));
        }

        /// Construct new normalized vector from a given vector.
        pub fn norm(self: Self) Self {
            var l = length(self);
            return Self.new(self.x / l, self.y / l, self.z / l, self.w / l);
        }

        pub fn is_eq(left: Self, right: Self) bool {
            return left.x == right.x and left.y == right.y and left.z == right.z and left.w == right.w;
        }

        /// Substraction between two given vector.
        pub fn sub(left: Self, right: Self) Self {
            return Self.new(left.x - right.x, left.y - right.y, left.z - right.z, left.w - right.w);
        }

        /// Addition betwen two given vector.
        pub fn add(left: Self, right: Self) Self {
            return Self.new(left.x + right.x, left.y + right.y, left.z + right.z, left.w + right.w);
        }

        /// Multiply each components by the given scalar.
        pub fn scale(v: Self, scalar: T) Self {
            return Self.new(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
        }

        /// Return the dot product between two given vector.
        pub fn dot(left: Self, right: Self) T {
            return (left.x * right.x) + (left.y * right.y) + (left.z * right.z) + (left.w * right.w);
        }
    };
}

test "zalgebra.Vec4.init" {
    var _vec_0 = vec4.new(1.5, 2.6, 3.7, 4.7);

    testing.expectEqual(_vec_0.x, 1.5);
    testing.expectEqual(_vec_0.y, 2.6);
    testing.expectEqual(_vec_0.z, 3.7);
    testing.expectEqual(_vec_0.w, 4.7);
}

test "zalgebra.Vec4.is_eq" {
    var _vec_0 = vec4.new(1., 2., 3., 4.);
    var _vec_1 = vec4.new(1., 2., 3., 4.);
    var _vec_2 = vec4.new(1., 2., 3., 5.);
    testing.expectEqual(vec4.is_eq(_vec_0, _vec_1), true);
    testing.expectEqual(vec4.is_eq(_vec_0, _vec_2), false);
}

test "zalgebra.Vec4.length" {
    var _vec_0 = vec4.new(1.5, 2.6, 3.7, 4.7);
    testing.expectEqual(_vec_0.length(), 6.69253301);
}

test "zalgebra.Vec4.normalize" {
    var _vec_0 = vec4.new(1.5, 2.6, 3.7, 4.0);
    testing.expectEqual(vec4.is_eq(_vec_0.norm(), vec4.new(0.241121411, 0.417943745, 0.594766139, 0.642990410)), true);
}

test "zalgebra.Vec4.sub" {
    var _vec_0 = vec4.new(1., 2., 3., 6.);
    var _vec_1 = vec4.new(2., 2., 3., 5.);
    testing.expectEqual(vec4.is_eq(vec4.sub(_vec_0, _vec_1), vec4.new(-1., 0., 0., 1.)), true);
}

test "zalgebra.Vec4.add" {
    var _vec_0 = vec4.new(1., 2., 3., 5.);
    var _vec_1 = vec4.new(2., 2., 3., 6.);
    testing.expectEqual(vec4.is_eq(vec4.add(_vec_0, _vec_1), vec4.new(3., 4., 6., 11.)), true);
}

test "zalgebra.Vec4.scale" {
    var _vec_0 = vec4.new(1., 2., 3., 4.);
    testing.expectEqual(vec4.is_eq(vec4.scale(_vec_0, 5.), vec4.new(5., 10., 15., 20.)), true);
}

test "zalgebra.Vec4.dot" {
    var _vec_0 = vec4.new(1.5, 2.6, 3.7, 5.);
    var _vec_1 = vec4.new(2.5, 3.45, 1.0, 1.);

    testing.expectEqual(vec4.dot(_vec_0, _vec_1), 21.4200000);
}
