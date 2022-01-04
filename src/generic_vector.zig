const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const meta = std.meta;
const expectEqual = std.testing.expectEqual;
const panic = std.debug.panic;

pub const Vec2 = GenericVector(2, f32);
pub const Vec2_f64 = GenericVector(2, f64);
pub const Vec2_i32 = GenericVector(2, i32);
pub const Vec2_usize = GenericVector(2, usize);

pub const Vec3 = GenericVector(3, f32);
pub const Vec3_f64 = GenericVector(3, f64);
pub const Vec3_i32 = GenericVector(3, i32);
pub const Vec3_usize = GenericVector(3, usize);

pub const Vec4 = GenericVector(4, f32);
pub const Vec4_f64 = GenericVector(4, f64);
pub const Vec4_i32 = GenericVector(4, i32);
pub const Vec4_usize = GenericVector(4, usize);

/// A generic vector.
pub fn GenericVector(comptime dimensions: comptime_int, comptime T: type) type {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .Int) {
        @compileError("Vectors not implemented for " ++ @typeName(T));
    }

    if (dimensions < 2 or dimensions > 4) {
        @compileError("Dimensions must be 2, 3 or 4!");
    }

    const Vector = meta.Vector(dimensions, T);

    return extern struct {
        const Self = @This();

        usingnamespace switch (dimensions) {
            3 => extern struct {
                pub usingnamespace Self;

                /// Shorthand for (0, 0, -1).
                pub fn back() Vector {
                    return [3]T{ 0, 0, -1 };
                }

                /// Shorthand for (0, 0, 1).
                pub fn forward() Vector {
                    return [3]T{ 0, 0, 1 };
                }

                /// Construct the cross product (as vector) from two vectors.
                pub fn cross(lhs: Vector, rhs: Vector) Vector {
                    return [3]T{
                        (lhs[1] * rhs[2]) - (lhs[2] * rhs[1]),
                        (lhs[2] * rhs[0]) - (lhs[0] * rhs[2]),
                        (lhs[0] * rhs[1]) - (lhs[1] * rhs[0]),
                    };
                }
            },
            else => Self,
        };

        /// Construct new vector.
        pub fn new(values: [dimensions]T) Vector {
            return values;
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Vector {
            var result: [dimensions]T = undefined;
            for (result) |_, i| {
                result[i] = val;
            }
            return result;
        }

        /// Shorthand for (0..).
        pub fn zero() Vector {
            return set(0);
        }

        /// Shorthand for (1..).
        pub fn one() Vector {
            return set(1);
        }

        /// Shorthand for (0, 1).
        pub fn up() Vector {
            switch (dimensions) {
                2 => return [_]T{ 0, 1 },
                3 => return [_]T{ 0, 1, 0 },
                4 => return [_]T{ 0, 1, 0, 0 },
                else => unreachable,
            }
        }

        /// Shorthand for (0, -1).
        pub fn down() Vector {
            switch (dimensions) {
                2 => return [_]T{ 0, -1 },
                3 => return [_]T{ 0, -1, 0 },
                4 => return [_]T{ 0, -1, 0, 0 },
                else => unreachable,
            }
        }

        /// Shorthand for (1, 0).
        pub fn right() Vector {
            switch (dimensions) {
                2 => return [_]T{ 1, 0 },
                3 => return [_]T{ 1, 0, 0 },
                4 => return [_]T{ 1, 0, 0, 0 },
                else => unreachable,
            }
        }

        /// Shorthand for (-1, 0).
        pub fn left() Vector {
            switch (dimensions) {
                2 => return [_]T{ -1, 0 },
                3 => return [_]T{ -1, 0, 0 },
                4 => return [_]T{ -1, 0, 0, 0 },
                else => unreachable,
            }
        }

        /// Negate the given vector.
        pub fn negate(vector: Vector) Vector {
            return scale(vector, -1);
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt.
        pub fn cast(self: Vector, dest: anytype) meta.Vector(dimensions, dest) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest);
            var result: [dimensions]dest = undefined;

            if (source_info == .Float and dest_info == .Int) {
                for (result) |_, i| {
                    result[i] = @floatToInt(dest, self[i]);
                }
                return result;
            }

            if (source_info == .Int and dest_info == .Float) {
                for (result) |_, i| {
                    result[i] = @intToFloat(dest, self[i]);
                }
                return result;
            }

            return switch (dest_info) {
                .Float => {
                    for (result) |_, i| {
                        result[i] = @floatCast(dest, self[i]);
                    }
                    return result;
                },
                .Int => {
                    for (result) |_, i| {
                        result[i] = @intCast(dest, self[i]);
                    }
                    return result;
                },
                else => panic(
                    "Error, dest type should be integer or float.\n",
                    .{},
                ),
            };
        }

        /// Construct new vector from slice.
        pub fn fromSlice(slice: []const T) Vector {
            return slice[0..dimensions].*;
        }

        /// Return the angle (in degrees) between two vectors.
        pub fn getAngle(left_vector: Vector, right_vector: Vector) T {
            const dot_product = dot(norm(left_vector), norm(right_vector));
            return root.toDegrees(math.acos(dot_product));
        }

        /// Return the length (magnitude) of given vector.
        pub fn length(vector: Vector) T {
            return @sqrt(dot(vector, vector));
        }

        /// Return the distance between two points.
        pub fn distance(a: Vector, b: Vector) T {
            return length(b - a);
        }

        /// Construct new normalized vector from a given one.
        pub fn norm(vector: Vector) Vector {
            const l = @splat(dimensions, length(vector));
            return vector / l;
        }

        /// Return true if two vectors are equals.
        pub fn eql(left_vector: Vector, right_vector: Vector) bool {
            return @reduce(.And, left_vector == right_vector);
        }

        /// Construct vector from the max components in two vectors
        pub fn max(a: Vector, b: Vector) Vector {
            return @maximum(a, b);
        }

        /// Construct vector from the min components in two vectors
        pub fn min(a: Vector, b: Vector) Vector {
            return @minimum(a, b);
        }

        /// Construct new vector after multiplying each components by the given scalar
        pub fn scale(vector: Vector, scalar: T) Vector {
            return vector * @splat(dimensions, scalar);
        }

        /// Return the dot product between two given vector.
        pub fn dot(left_vector: Vector, right_vector: Vector) T {
            return @reduce(.Add, left_vector * right_vector);
        }

        /// Linear interpolation between two vectors
        pub fn lerp(left_vector: Vector, right_vector: Vector, t: T) Vector {
            var result: [dimensions]T = undefined;
            for (result) |_, i| {
                result[i] = root.lerp(T, left_vector[i], right_vector[i], t);
            }
            return result;
        }
    };
}

test "zalgebra.Vectors.add" {
    // Vec2
    const a = Vec2.new(.{ 1, 1 });
    const b = Vec2.set(1);
    try expectEqual(a + b, Vec2.set(2));

    // Vec3
    const c = Vec3.new(.{ 1, 1, 1 });
    const d = Vec3.set(1);
    try expectEqual(c + d, Vec3.set(2));

    // Vec4
    const e = Vec4.new(.{ 1, 1, 1, 1 });
    const f = Vec4.set(1);
    try expectEqual(e + f, Vec4.set(2));
}

test "zalgebra.Vectors.set" {
    // Vec2
    const a = [2]f32{ 2.5, 2.5 };
    const b = Vec2.set(2.5);

    try expectEqual(Vec2.eql(a, b), true);

    // Vec3
    const c = [3]f32{ 2.5, 2.5, 2.5 };
    const d = Vec3.set(2.5);
    try expectEqual(Vec3.eql(c, d), true);

    // Vec4
    const e = [4]f32{ 2.5, 2.5, 2.5, 2.5 };
    const f = Vec4.set(2.5);
    try expectEqual(Vec4.eql(e, f), true);
}

test "zalgebra.Vectors.negate" {
    // Vec2
    const a = Vec2.set(5);
    const b = Vec2.set(-5);
    try expectEqual(Vec2.eql(Vec2.negate(a), b), true);

    // Vec3
    const c = Vec3.set(5);
    const d = Vec3.set(-5);
    try expectEqual(Vec3.eql(Vec3.negate(c), d), true);

    // Vec4
    const e = Vec4.set(5);
    const f = Vec4.set(-5);
    try expectEqual(Vec4.eql(Vec4.negate(e), f), true);
}

test "zalgebra.Vectors.getAngle" {
    // Vec2
    const a = Vec2.right();
    const b = Vec2.up();
    const c = Vec2.left();
    const d = Vec2.set(1);

    try expectEqual(Vec2.getAngle(a, b), 90);
    try expectEqual(Vec2.getAngle(a, c), 180);
    try expectEqual(Vec2.getAngle(a, d), 45);

    // Vec3
    const e = Vec3.right();
    const f = Vec3.up();
    const g = Vec3.left();
    const h = [3]f32{ 1, 1, 0 };

    try expectEqual(Vec3.getAngle(e, f), 90);
    try expectEqual(Vec3.getAngle(e, g), 180);
    try expectEqual(Vec3.getAngle(e, h), 45);

    // Vec4
    const i = Vec4.right();
    const j = Vec4.up();
    const k = Vec4.left();
    const l = [4]f32{ 1, 1, 0, 0 };

    try expectEqual(Vec4.getAngle(i, j), 90);
    try expectEqual(Vec4.getAngle(i, k), 180);
    try expectEqual(Vec4.getAngle(i, l), 45);
}

test "zalgebra.Vectors.eql" {
    // Vec2
    const a = [2]f32{ 1, 2 };
    const b = [2]f32{ 1, 2 };
    const c = [2]f32{ 1.5, 2 };

    try expectEqual(Vec2.eql(a, b), true);
    try expectEqual(Vec2.eql(a, c), false);

    // Vec3
    const d = [3]f32{ 1, 2, 3 };
    const e = [3]f32{ 1, 2, 3 };
    const f = [3]f32{ 1.5, 2, 3 };

    try expectEqual(Vec3.eql(d, e), true);
    try expectEqual(Vec3.eql(d, f), false);

    // Vec4
    const g = [4]f32{ 1, 2, 3, 4 };
    const h = [4]f32{ 1, 2, 3, 4 };
    const i = [4]f32{ 1.5, 2, 3, 4 };

    try expectEqual(Vec4.eql(g, h), true);
    try expectEqual(Vec4.eql(g, i), false);
}

test "zalgebra.Vectors.length" {
    // Vec2
    const a = [2]f32{ 1.5, 2.6 };
    try expectEqual(Vec2.length(a), 3.00166606);

    // Vec3
    const b = [3]f32{ 1.5, 2.6, 3.7 };
    try expectEqual(Vec3.length(b), 4.7644519);

    // Vec4
    const c = [4]f32{ 1.5, 2.6, 3.7, 4.7 };
    try expectEqual(Vec4.length(c), 6.69253301);
}

test "zalgebra.Vectors.distance" {
    // Vec2
    const a = Vec2.zero();
    const b = Vec2.left();
    const c = [2]f32{ 0, 5 };

    try expectEqual(Vec2.distance(a, b), 1);
    try expectEqual(Vec2.distance(a, c), 5);

    // Vec3
    const d = Vec3.zero();
    const e = Vec3.left();
    const f = [3]f32{ 0, 5, 0 };

    try expectEqual(Vec3.distance(d, e), 1);
    try expectEqual(Vec3.distance(d, f), 5);

    // Vec4
    const g = Vec4.zero();
    const h = Vec4.left();
    const i = [4]f32{ 0, 5, 0, 0 };

    try expectEqual(Vec4.distance(g, h), 1);
    try expectEqual(Vec4.distance(g, i), 5);
}

test "zalgebra.Vectors.normalize" {
    // Vec2
    const a = [2]f32{ 1.5, 2.6 };
    try expectEqual(Vec2.eql(Vec2.norm(a), [2]f32{ 0.499722480, 0.866185605 }), true);

    // Vec3
    const b = [3]f32{ 1.5, 2.6, 3.7 };
    try expectEqual(Vec3.eql(Vec3.norm(b), [3]f32{ 0.314831584, 0.545708060, 0.776584625 }), true);

    // Vec4
    const c = [4]f32{ 1.5, 2.6, 3.7, 4.0 };
    try expectEqual(Vec4.eql(Vec4.norm(c), [4]f32{ 0.241121411, 0.417943745, 0.594766139, 0.642990410 }), true);
}

test "zalgebra.Vectors.scale" {
    // Vec2
    const a = [2]f32{ 1, 2 };
    try expectEqual(Vec2.eql(Vec2.scale(a, 5), [2]f32{ 5, 10 }), true);

    // Vec3
    const b = [3]f32{ 1, 2, 3 };
    try expectEqual(Vec3.eql(Vec3.scale(b, 5), [3]f32{ 5, 10, 15 }), true);

    // Vec4
    const c = [4]f32{ 1, 2, 3, 4 };
    try expectEqual(Vec4.eql(Vec4.scale(c, 5), [4]f32{ 5, 10, 15, 20 }), true);
}

test "zalgebra.Vectors.dot" {
    // Vec2
    const a = [2]f32{ 1.5, 2.6 };
    const b = [2]f32{ 2.5, 3.45 };
    try expectEqual(Vec2.dot(a, b), 12.7200002);

    // Vec3
    const c = [3]f32{ 1.5, 2.6, 3.7 };
    const d = [3]f32{ 2.5, 3.45, 1.0 };
    try expectEqual(Vec3.dot(c, d), 16.42);

    // Vec4
    const e = [4]f32{ 1.5, 2.6, 3.7, 5 };
    const f = [4]f32{ 2.5, 3.45, 1.0, 1 };
    try expectEqual(Vec4.dot(e, f), 21.4200000);
}

test "zalgebra.Vectors.lerp" {
    // Vec2
    const a = [2]f32{ -10.0, 0.0 };
    const b = Vec2.set(10.0);
    try expectEqual(Vec2.eql(Vec2.lerp(a, b, 0.5), [2]f32{ 0.0, 5.0 }), true);

    // Vec3
    const c = [3]f32{ -10.0, 0.0, -10.0 };
    const d = Vec3.set(10.0);
    try expectEqual(Vec3.eql(Vec3.lerp(c, d, 0.5), [3]f32{ 0.0, 5.0, 0.0 }), true);

    // Vec4
    const e = [4]f32{ -10.0, 0.0, -10.0, -10.0 };
    const f = Vec4.set(10.0);
    try expectEqual(Vec4.eql(Vec4.lerp(e, f, 0.5), [4]f32{ 0.0, 5.0, 0.0, 0.0 }), true);
}

test "zalgebra.Vectors.min" {
    // Vec2
    const a = [2]f32{ 10.0, -2.0 };
    const b = [2]f32{ -10.0, 5.0 };
    try expectEqual(Vec2.eql(Vec2.min(a, b), [2]f32{ -10.0, -2.0 }), true);

    // Vec3
    const c = [3]f32{ 10.0, -2.0, 0.0 };
    const d = [3]f32{ -10.0, 5.0, 0.0 };
    try expectEqual(Vec3.eql(Vec3.min(c, d), [3]f32{ -10.0, -2.0, 0.0 }), true);

    // Vec4
    const e = [4]f32{ 10.0, -2.0, 0.0, 1.0 };
    const f = [4]f32{ -10.0, 5.0, 0.0, 1.01 };
    try expectEqual(Vec4.eql(Vec4.min(e, f), [4]f32{ -10.0, -2.0, 0.0, 1.0 }), true);
}

test "zalgebra.Vectors.max" {
    // Vec2
    const a = [2]f32{ 10.0, -2.0 };
    const b = [2]f32{ -10.0, 5.0 };
    try expectEqual(Vec2.eql(Vec2.max(a, b), [2]f32{ 10.0, 5.0 }), true);

    // Vec3
    const c = [3]f32{ 10.0, -2.0, 0.0 };
    const d = [3]f32{ -10.0, 5.0, 0.0 };
    try expectEqual(Vec3.eql(Vec3.max(c, d), [3]f32{ 10.0, 5.0, 0.0 }), true);

    // Vec4
    const e = [4]f32{ 10.0, -2.0, 0.0, 1.0 };
    const f = [4]f32{ -10.0, 5.0, 0.0, 1.01 };
    try expectEqual(Vec4.eql(Vec4.max(e, f), [4]f32{ 10.0, 5.0, 0.0, 1.01 }), true);
}

test "zalgebra.Vectors.fromSlice" {
    // Vec2
    const a = [2]f32{ 2, 4 };
    try expectEqual(Vec2.eql(Vec2.fromSlice(&a), [2]f32{ 2, 4 }), true);

    // Vec3
    const b = [3]f32{ 2, 4, 3 };
    try expectEqual(Vec3.eql(Vec3.fromSlice(&b), [3]f32{ 2, 4, 3 }), true);

    // Vec4
    const c = [4]f32{ 2, 4, 3, 6 };
    try expectEqual(Vec4.eql(Vec4.fromSlice(&c), [4]f32{ 2, 4, 3, 6 }), true);
}

test "zalgebra.Vectors.cast" {
    // Vec2
    const a = [2]i32{ 3, 6 };
    const b = [2]usize{ 3, 6 };
    try expectEqual(Vec2_usize.eql(Vec2_i32.cast(a, usize), b), true);

    const c = [2]f32{ 3.5, 6.5 };
    const d = [2]f64{ 3.5, 6.5 };
    try expectEqual(Vec2_f64.eql(Vec2.cast(c, f64), d), true);

    const e = [2]i32{ 3, 6 };
    const f = [2]f32{ 3.0, 6.0 };
    try expectEqual(Vec2.eql(Vec2_i32.cast(e, f32), f), true);

    const g = [2]f32{ 3.0, 6.0 };
    const h = [2]i32{ 3, 6 };
    try expectEqual(Vec2_i32.eql(Vec2.cast(g, i32), h), true);

    // Vec3
    const i = [3]i32{ 3, 6, 2 };
    const j = [3]usize{ 3, 6, 2 };
    try expectEqual(Vec3_usize.eql(Vec3_i32.cast(i, usize), j), true);

    const k = [3]f32{ 3.5, 6.5, 2.0 };
    const l = [3]f64{ 3.5, 6.5, 2.0 };
    try expectEqual(Vec3_f64.eql(Vec3.cast(k, f64), l), true);

    const m = [3]i32{ 3, 6, 2 };
    const n = [3]f32{ 3.0, 6.0, 2.0 };
    try expectEqual(Vec3.eql(Vec3_i32.cast(m, f32), n), true);

    const o = [3]f32{ 3.0, 6.0, 2.0 };
    const p = [3]i32{ 3, 6, 2 };
    try expectEqual(Vec3_i32.eql(Vec3.cast(o, i32), p), true);

    // Vec4
    const q = [4]i32{ 3, 6, 2, 0 };
    const r = [4]usize{ 3, 6, 2, 0 };
    try expectEqual(Vec4_usize.eql(Vec4_i32.cast(q, usize), r), true);

    const s = [4]f32{ 3.5, 6.5, 2.0, 0 };
    const t = [4]f64{ 3.5, 6.5, 2, 0.0 };
    try expectEqual(Vec4_f64.eql(Vec4.cast(s, f64), t), true);

    const u = [4]i32{ 3, 6, 2, 0 };
    const v = [4]f32{ 3.0, 6.0, 2.0, 0.0 };
    try expectEqual(Vec4.eql(Vec4_i32.cast(u, f32), v), true);

    const w = [4]f32{ 3.0, 6.0, 2.0, 0.0 };
    const x = [4]i32{ 3, 6, 2, 0 };
    try expectEqual(Vec4_i32.eql(Vec4.cast(w, i32), x), true);
}

test "zalgebra.Vectors.cross" {
    // Only for Vec3
    const a = [3]f32{ 1.5, 2.6, 3.7 };
    const b = [3]f32{ 2.5, 3.45, 1.0 };
    const c = [3]f32{ 1.5, 2.6, 3.7 };

    const result_1 = Vec3.cross(a, c);
    const result_2 = Vec3.cross(a, b);

    try expectEqual(Vec3.eql(result_1, Vec3.zero()), true);
    try expectEqual(Vec3.eql(result_2, [3]f32{ -10.1650009, 7.75, -1.32499980 }), true);
}
