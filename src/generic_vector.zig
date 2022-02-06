const std = @import("std");
const root = @import("main.zig");
const math = std.math;
const meta = std.meta;
const expectEqual = std.testing.expectEqual;
const expect = std.testing.expect;
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

    return struct {
        const Self = @This();
        data: meta.Vector(dimensions, T),

        usingnamespace switch (dimensions) {
            2 => extern struct {
                /// Construct new vector.
                pub fn new(vx: T, vy: T) Self {
                    return .{ .data = [2]T{ vx, vy } };
                }
            },
            3 => extern struct {
                /// Construct new vector.
                pub fn new(vx: T, vy: T, vz: T) Self {
                    return .{ .data = [3]T{ vx, vy, vz } };
                }

                pub fn z(self: Self) T {
                    return self.data[2];
                }

                /// Shorthand for (0, 0, 1).
                pub fn forward() Self {
                    return new(0, 0, 1);
                }

                /// Shorthand for (0, 0, -1).
                pub fn back() Self {
                    return forward().negate();
                }

                /// Construct the cross product (as vector) from two vectors.
                pub fn cross(lhs: Self, rhs: Self) Self {
                    const lx = lhs.x();
                    const ly = lhs.y();
                    const lz = lhs.z();

                    const rx = rhs.x();
                    const ry = rhs.y();
                    const rz = rhs.z();

                    const vx = (ly * rz) - (lz * ry);
                    const vy = (lz * rx) - (lx * rz);
                    const vz = (lx * ry) - (ly * rx);
                    return new(vx, vy, vz);
                }
            },
            4 => extern struct {
                /// Construct new vector.
                pub fn new(vx: T, vy: T, vz: T, vw: T) Self {
                    return .{ .data = [4]T{ vx, vy, vz, vw } };
                }

                /// Shorthand for (0, 0, 1, 0).
                pub fn forward() Self {
                    return new(0, 0, 1, 0);
                }

                /// Shorthand for (0, 0, -1, 0).
                pub fn back() Self {
                    return forward().negate();
                }

                pub fn z(self: Self) T {
                    return self.data[2];
                }

                pub fn w(self: Self) T {
                    return self.data[3];
                }
            },
            else => {},
        };

        pub fn x(self: Self) T {
            return self.data[0];
        }

        pub fn y(self: Self) T {
            return self.data[1];
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            const result = @splat(dimensions, val);
            return .{ .data = result };
        }

        /// Shorthand for (0..).
        pub fn zero() Self {
            return set(0);
        }

        /// Shorthand for (1..).
        pub fn one() Self {
            return set(1);
        }

        /// Shorthand for (0, 1).
        pub fn up() Self {
            return switch (dimensions) {
                2 => Self.new(0, 1),
                3 => Self.new(0, 1, 0),
                4 => Self.new(0, 1, 0, 0),
                else => unreachable,
            };
        }

        /// Shorthand for (0, -1).
        pub fn down() Self {
            return up().negate();
        }

        /// Shorthand for (1, 0).
        pub fn right() Self {
            return switch (dimensions) {
                2 => Self.new(1, 0),
                3 => Self.new(1, 0, 0),
                4 => Self.new(1, 0, 0, 0),
                else => unreachable,
            };
        }

        /// Shorthand for (-1, 0).
        pub fn left() Self {
            return right().negate();
        }

        /// Negate the given vector.
        pub fn negate(vector: Self) Self {
            return vector.scale(-1);
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt.
        pub fn cast(self: Self, dest_type: anytype) GenericVector(dimensions, dest_type) {
            const source_info = @typeInfo(T);
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .Float and dest_info != .Int) {
                panic("Error, dest type should be integer or float.\n", .{});
            }

            var result: [dimensions]dest_type = undefined;

            if (source_info == .Float and dest_info == .Int) {
                for (result) |_, i| {
                    result[i] = @floatToInt(dest_type, self.data[i]);
                }
                return .{ .data = result };
            }

            if (source_info == .Int and dest_info == .Float) {
                for (result) |_, i| {
                    result[i] = @intToFloat(dest_type, self.data[i]);
                }
                return .{ .data = result };
            }

            if (source_info == .Float and dest_info == .Float) {
                for (result) |_, i| {
                    result[i] = @floatCast(dest_type, self.data[i]);
                }
                return .{ .data = result };
            }

            if (source_info == .Int and dest_info == .Int) {
                for (result) |_, i| {
                    result[i] = @intCast(dest_type, self.data[i]);
                }
                return .{ .data = result };
            }
        }

        /// Construct new vector from slice.
        pub fn fromSlice(slice: []const T) Self {
            const result = slice[0..dimensions].*;
            return .{ .data = result };
        }

        /// Transform vector to array.
        pub fn toArray(self: Self) [dimensions]T {
            return self.data;
        }

        /// Return the angle (in degrees) between two vectors.
        pub fn getAngle(left_vector: Self, right_vector: Self) T {
            const dot_product = dot(norm(left_vector), norm(right_vector));
            return root.toDegrees(math.acos(dot_product));
        }

        /// Return the length (magnitude) of given vector.
        pub fn length(vector: Self) T {
            return @sqrt(vector.dot(vector));
        }

        /// Return the distance between two points.
        pub fn distance(a: Self, b: Self) T {
            return length(b.sub(a));
        }

        /// Construct new normalized vector from a given one.
        pub fn norm(vector: Self) Self {
            const l = vector.length();
            if (l == 0) {
                return vector;
            }
            const result = vector.data / @splat(dimensions, l);
            return .{ .data = result };
        }

        /// Return true if two vectors are equals.
        pub fn eql(left_vector: Self, right_vector: Self) bool {
            return @reduce(.And, left_vector.data == right_vector.data);
        }

        /// Substraction between two given vector.
        pub fn sub(lhs: Self, rhs: Self) Self {
            const result = lhs.data - rhs.data;
            return .{ .data = result };
        }

        /// Addition betwen two given vector.
        pub fn add(lhs: Self, rhs: Self) Self {
            const result = lhs.data + rhs.data;
            return .{ .data = result };
        }

        /// Construct vector from the max components in two vectors
        pub fn max(a: Self, b: Self) Self {
            const result = @maximum(a.data, b.data);
            return .{ .data = result };
        }

        /// Construct vector from the min components in two vectors
        pub fn min(a: Self, b: Self) Self {
            const result = @minimum(a.data, b.data);
            return .{ .data = result };
        }

        /// Construct new vector after multiplying each components by the given scalar
        pub fn scale(vector: Self, scalar: T) Self {
            const result = vector.data * @splat(dimensions, scalar);
            return .{ .data = result };
        }

        /// Return the dot product between two given vector.
        pub fn dot(left_vector: Self, right_vector: Self) T {
            return @reduce(.Add, left_vector.data * right_vector.data);
        }

        /// Linear interpolation between two vectors
        pub fn lerp(left_vector: Self, right_vector: Self, t: T) Self {
            var result: [dimensions]T = undefined;
            for (result) |_, i| {
                result[i] = root.lerp(T, left_vector.data[i], right_vector.data[i], t);
            }
            return .{ .data = result };
        }
    };
}

test "zalgebra.Vectors.eql" {
    // Vec2
    const a = Vec2.new(1, 2);
    const b = Vec2.new(1, 2);
    const c = Vec2.new(1.5, 2);

    try expectEqual(Vec2.eql(a, b), true);
    try expectEqual(Vec2.eql(a, c), false);

    // Vec3
    const d = Vec3.new(1, 2, 3);
    const e = Vec3.new(1, 2, 3);
    const f = Vec3.new(1.5, 2, 3);

    try expectEqual(Vec3.eql(d, e), true);
    try expectEqual(Vec3.eql(d, f), false);

    // Vec4
    const g = Vec4.new(1, 2, 3, 4);
    const h = Vec4.new(1, 2, 3, 4);
    const i = Vec4.new(1.5, 2, 3, 4);

    try expectEqual(Vec4.eql(g, h), true);
    try expectEqual(Vec4.eql(g, i), false);
}

test "zalgebra.Vectors.set" {
    // Vec2
    const a = Vec2.new(2.5, 2.5);
    const b = Vec2.set(2.5);
    try expectEqual(a, b);

    // Vec3
    const c = Vec3.new(2.5, 2.5, 2.5);
    const d = Vec3.set(2.5);
    try expectEqual(c, d);

    // Vec4
    const e = Vec4.new(2.5, 2.5, 2.5, 2.5);
    const f = Vec4.set(2.5);
    try expectEqual(e, f);
}

test "zalgebra.Vectors.add" {
    // Vec2
    const a = Vec2.one();
    const b = Vec2.one();
    try expectEqual(a.add(b), Vec2.set(2));

    // Vec3
    const c = Vec3.one();
    const d = Vec3.one();
    try expectEqual(c.add(d), Vec3.set(2));

    // Vec4
    const e = Vec4.one();
    const f = Vec4.one();
    try expectEqual(e.add(f), Vec4.set(2));
}

test "zalgebra.Vectors.negate" {
    // Vec2
    const a = Vec2.set(5);
    const a_negated = Vec2.set(-5);
    try expectEqual(a.negate(), a_negated);

    // Vec3
    const b = Vec3.set(5);
    const b_negated = Vec3.set(-5);
    try expectEqual(b.negate(), b_negated);

    // Vec4
    const c = Vec4.set(5);
    const c_negated = Vec4.set(-5);
    try expectEqual(c.negate(), c_negated);
}

test "zalgebra.Vectors.getAngle" {
    // Vec2
    const a = Vec2.right();
    const b = Vec2.up();
    const c = Vec2.left();
    const d = Vec2.one();

    try expectEqual(a.getAngle(b), 90);
    try expectEqual(a.getAngle(c), 180);
    try expectEqual(a.getAngle(d), 45);

    // Vec3
    const e = Vec3.right();
    const f = Vec3.up();
    const g = Vec3.left();
    const h = Vec3.new(1, 1, 0);

    try expectEqual(e.getAngle(f), 90);
    try expectEqual(e.getAngle(g), 180);
    try expectEqual(e.getAngle(h), 45);

    // Vec4
    const i = Vec4.right();
    const j = Vec4.up();
    const k = Vec4.left();
    const l = Vec4.new(1, 1, 0, 0);

    try expectEqual(i.getAngle(j), 90);
    try expectEqual(i.getAngle(k), 180);
    try expectEqual(i.getAngle(l), 45);
}

test "zalgebra.Vec3.toArray" {
    const a = Vec3.up().toArray();
    const b = [_]f32{ 0, 1, 0 };

    try expect(std.mem.eql(f32, &a, &b));
}

test "zalgebra.Vectors.length" {
    // Vec2
    const a = Vec2.new(1.5, 2.6);
    try expectEqual(a.length(), 3.00166606);

    // Vec3
    const b = Vec3.new(1.5, 2.6, 3.7);
    try expectEqual(b.length(), 4.7644519);

    // Vec4
    const c = Vec4.new(1.5, 2.6, 3.7, 4.7);
    try expectEqual(c.length(), 6.69253301);
}

test "zalgebra.Vectors.distance" {
    // Vec2
    const a = Vec2.zero();
    const b = Vec2.left();
    const c = Vec2.new(0, 5);

    try expectEqual(a.distance(b), 1);
    try expectEqual(a.distance(c), 5);

    // Vec3
    const d = Vec3.zero();
    const e = Vec3.left();
    const f = Vec3.new(0, 5, 0);

    try expectEqual(d.distance(e), 1);
    try expectEqual(d.distance(f), 5);

    // Vec4
    const g = Vec4.zero();
    const h = Vec4.left();
    const i = Vec4.new(0, 5, 0, 0);

    try expectEqual(g.distance(h), 1);
    try expectEqual(g.distance(i), 5);
}

test "zalgebra.Vectors.normalize" {
    // Vec2
    const a = Vec2.new(1.5, 2.6);
    const a_normalized = Vec2.new(0.499722480, 0.866185605);
    try expectEqual(a.norm(), a_normalized);

    // Vec3
    const b = Vec3.new(1.5, 2.6, 3.7);
    const b_normalized = Vec3.new(0.314831584, 0.545708060, 0.776584625);
    try expectEqual(b.norm(), b_normalized);

    // Vec4
    const c = Vec4.new(1.5, 2.6, 3.7, 4.0);
    const c_normalized = Vec4.new(0.241121411, 0.417943745, 0.594766139, 0.642990410);
    try expectEqual(c.norm(), c_normalized);
}

test "zalgebra.Vectors.scale" {
    // Vec2
    const a = Vec2.new(1, 2);
    const a_scaled = Vec2.new(5, 10);
    try expectEqual(a.scale(5), a_scaled);

    // Vec3
    const b = Vec3.new(1, 2, 3);
    const b_scaled = Vec3.new(5, 10, 15);
    try expectEqual(b.scale(5), b_scaled);

    // Vec4
    const c = Vec4.new(1, 2, 3, 4);
    const c_scaled = Vec4.new(5, 10, 15, 20);
    try expectEqual(c.scale(5), c_scaled);
}

test "zalgebra.Vectors.dot" {
    // Vec2
    const a = Vec2.new(1.5, 2.6);
    const b = Vec2.new(2.5, 3.45);
    try expectEqual(a.dot(b), 12.7200002);

    // Vec3
    const c = Vec3.new(1.5, 2.6, 3.7);
    const d = Vec3.new(2.5, 3.45, 1.0);
    try expectEqual(c.dot(d), 16.42);

    // Vec4
    const e = Vec4.new(1.5, 2.6, 3.7, 5);
    const f = Vec4.new(2.5, 3.45, 1.0, 1);
    try expectEqual(e.dot(f), 21.4200000);
}

test "zalgebra.Vectors.lerp" {
    // Vec2
    const a = Vec2.new(-10, 0);
    const b = Vec2.set(10);
    try expectEqual(Vec2.lerp(a, b, 0.5), Vec2.new(0, 5));

    // Vec3
    const c = Vec3.new(-10, 0, -10);
    const d = Vec3.set(10);
    try expectEqual(Vec3.lerp(c, d, 0.5), Vec3.new(0, 5, 0));

    // Vec4
    const e = Vec4.new(-10, 0, -10, -10);
    const f = Vec4.set(10);
    try expectEqual(Vec4.lerp(e, f, 0.5), Vec4.new(0, 5, 0, 0));
}

test "zalgebra.Vectors.min" {
    // Vec2
    const a = Vec2.new(10, -2);
    const b = Vec2.new(-10, 5);
    const a_b_minimum = Vec2.new(-10, -2);
    try expectEqual(Vec2.min(a, b), a_b_minimum);

    // Vec3
    const c = Vec3.new(10, -2, 0);
    const d = Vec3.new(-10, 5, 0);
    const c_d_minimum = Vec3.new(-10, -2, 0);
    try expectEqual(Vec3.min(c, d), c_d_minimum);

    // Vec4
    const e = Vec4.new(10, -2, 0, 1);
    const f = Vec4.new(-10, 5, 0, 1.01);
    const e_f_minimum = Vec4.new(-10, -2, 0, 1);
    try expectEqual(Vec4.min(e, f), e_f_minimum);
}

test "zalgebra.Vectors.max" {
    // Vec2
    const a = Vec2.new(10, -2);
    const b = Vec2.new(-10, 5);
    const a_b_maximum = Vec2.new(10, 5);
    try expectEqual(Vec2.max(a, b), a_b_maximum);

    // Vec3
    const c = Vec3.new(10, -2, 0);
    const d = Vec3.new(-10, 5, 0);
    const c_d_maximum = Vec3.new(10, 5, 0);
    try expectEqual(Vec3.max(c, d), c_d_maximum);

    // Vec4
    const e = Vec4.new(10, -2, 0, 1);
    const f = Vec4.new(-10, 5, 0, 1.01);
    const e_f_maximum = Vec4.new(10, 5, 0, 1.01);
    try expectEqual(Vec4.max(e, f), e_f_maximum);
}

test "zalgebra.Vectors.fromSlice" {
    // Vec2
    const a = [2]f32{ 2, 4 };
    try expectEqual(Vec2.fromSlice(&a), Vec2.new(2, 4));

    // Vec3
    const b = [3]f32{ 2, 4, 3 };
    try expectEqual(Vec3.fromSlice(&b), Vec3.new(2, 4, 3));

    // Vec4
    const c = [4]f32{ 2, 4, 3, 6 };
    try expectEqual(Vec4.fromSlice(&c), Vec4.new(2, 4, 3, 6));
}

test "zalgebra.Vectors.cast" {
    // Vec2
    const a = Vec2_i32.new(3, 6);
    const a_usize = Vec2_usize.new(3, 6);
    try expectEqual(a.cast(usize), a_usize);

    const b = Vec2.new(3.5, 6.5);
    const b_f64 = Vec2_f64.new(3.5, 6.5);
    try expectEqual(b.cast(f64), b_f64);

    const c = Vec2_i32.new(3, 6);
    const c_f32 = Vec2.new(3, 6);
    try expectEqual(c.cast(f32), c_f32);

    const d = Vec2.new(3, 6);
    const d_i32 = Vec2_i32.new(3, 6);
    try expectEqual(d.cast(i32), d_i32);

    // Vec3
    const e = Vec3_i32.new(3, 6, 2);
    const e_usize = Vec3_usize.new(3, 6, 2);
    try expectEqual(e.cast(usize), e_usize);

    const f = Vec3.new(3.5, 6.5, 2);
    const f_f64 = Vec3_f64.new(3.5, 6.5, 2);
    try expectEqual(f.cast(f64), f_f64);

    const g = Vec3_i32.new(3, 6, 2);
    const g_f32 = Vec3.new(3, 6, 2);
    try expectEqual(g.cast(f32), g_f32);

    const h = Vec3.new(3, 6, 2);
    const h_i32 = Vec3_i32.new(3, 6, 2);
    try expectEqual(h.cast(i32), h_i32);

    // Vec4
    const i = Vec4_i32.new(3, 6, 2, 0);
    const i_usize = Vec4_usize.new(3, 6, 2, 0);
    try expectEqual(i.cast(usize), i_usize);

    const j = Vec4.new(3.5, 6.5, 2, 0);
    const j_f64 = Vec4_f64.new(3.5, 6.5, 2, 0);
    try expectEqual(j.cast(f64), j_f64);

    const k = Vec4_i32.new(3, 6, 2, 0);
    const k_f32 = Vec4.new(3, 6, 2, 0);
    try expectEqual(k.cast(f32), k_f32);

    const l = Vec4.new(3, 6, 2, 0);
    const l_i32 = Vec4_i32.new(3, 6, 2, 0);
    try expectEqual(l.cast(i32), l_i32);
}

test "zalgebra.Vectors.cross" {
    // Only for Vec3
    const a = Vec3.new(1.5, 2.6, 3.7);
    const b = Vec3.new(2.5, 3.45, 1.0);
    const c = Vec3.new(1.5, 2.6, 3.7);

    const result_1 = Vec3.cross(a, c);
    const result_2 = Vec3.cross(a, b);

    try expectEqual(result_1, Vec3.zero());
    try expectEqual(result_2, Vec3.new(-10.1650009, 7.75, -1.32499980));
}
