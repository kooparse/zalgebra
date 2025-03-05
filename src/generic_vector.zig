const std = @import("std");
const root = @import("main.zig");
const math = std.math;
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
    if (@typeInfo(T) != .float and @typeInfo(T) != .int) {
        @compileError("Vectors not implemented for " ++ @typeName(T));
    }

    if (dimensions < 2 or dimensions > 4) {
        @compileError("Dimensions must be 2, 3 or 4!");
    }

    return extern struct {
        const Self = @This();
        const Data = @Vector(dimensions, T);

        data: Data,

        pub const Component = switch (dimensions) {
            2 => enum { x, y },
            3 => enum { x, y, z },
            4 => enum { x, y, z, w },
            else => unreachable,
        };

        pub usingnamespace switch (dimensions) {
            2 => extern struct {
                /// Construct new vector.
                pub inline fn new(vx: T, vy: T) Self {
                    return .{ .data = [2]T{ vx, vy } };
                }

                /// Rotate vector by angle (in degrees)
                pub fn rotate(self: Self, angle_in_degrees: T) Self {
                    const sin_theta = @sin(root.toRadians(angle_in_degrees));
                    const cos_theta = @cos(root.toRadians(angle_in_degrees));
                    return .{ .data = [2]T{
                        cos_theta * self.x() - sin_theta * self.y(),
                        sin_theta * self.x() + cos_theta * self.y(),
                    } };
                }

                pub inline fn toVec3(self: Self, vz: T) GenericVector(3, T) {
                    return GenericVector(3, T).fromVec2(self, vz);
                }

                pub inline fn toVec4(self: Self, vz: T, vw: T) GenericVector(4, T) {
                    return GenericVector(4, T).fromVec2(self, vz, vw);
                }

                pub inline fn fromVec3(vec3: GenericVector(3, T)) Self {
                    return Self.new(vec3.x(), vec3.y());
                }

                pub inline fn fromVec4(vec4: GenericVector(4, T)) Self {
                    return Self.new(vec4.x(), vec4.y());
                }
            },
            3 => extern struct {
                /// Construct new vector.
                pub inline fn new(vx: T, vy: T, vz: T) Self {
                    return .{ .data = [3]T{ vx, vy, vz } };
                }

                pub inline fn z(self: Self) T {
                    return self.data[2];
                }

                pub inline fn zMut(self: *Self) *T {
                    return &self.data[2];
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
                pub fn cross(first_vector: Self, second_vector: Self) Self {
                    const x1 = first_vector.x();
                    const y1 = first_vector.y();
                    const z1 = first_vector.z();

                    const x2 = second_vector.x();
                    const y2 = second_vector.y();
                    const z2 = second_vector.z();

                    const result_x = (y1 * z2) - (z1 * y2);
                    const result_y = (z1 * x2) - (x1 * z2);
                    const result_z = (x1 * y2) - (y1 * x2);
                    return new(result_x, result_y, result_z);
                }

                pub inline fn toVec2(self: Self) GenericVector(2, T) {
                    return GenericVector(2, T).fromVec3(self);
                }

                pub inline fn toVec4(self: Self, vw: T) GenericVector(4, T) {
                    return GenericVector(4, T).fromVec3(self, vw);
                }

                pub inline fn fromVec2(vec2: GenericVector(2, T), vz: T) Self {
                    return Self.new(vec2.x(), vec2.y(), vz);
                }

                pub inline fn fromVec4(vec4: GenericVector(4, T)) Self {
                    return Self.new(vec4.x(), vec4.y(), vec4.z());
                }
            },
            4 => extern struct {
                /// Construct new vector.
                pub inline fn new(vx: T, vy: T, vz: T, vw: T) Self {
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

                pub inline fn z(self: Self) T {
                    return self.data[2];
                }

                pub inline fn w(self: Self) T {
                    return self.data[3];
                }

                pub inline fn zMut(self: *Self) *T {
                    return &self.data[2];
                }

                pub inline fn wMut(self: *Self) *T {
                    return &self.data[3];
                }

                pub inline fn toVec2(self: Self) GenericVector(2, T) {
                    return GenericVector(2, T).fromVec4(self);
                }

                pub inline fn toVec3(self: Self) GenericVector(3, T) {
                    return GenericVector(3, T).fromVec4(self);
                }

                pub inline fn fromVec2(vec2: GenericVector(2, T), vz: T, vw: T) Self {
                    return Self.new(vec2.x(), vec2.y(), vz, vw);
                }

                pub inline fn fromVec3(vec3: GenericVector(3, T), vw: T) Self {
                    return Self.new(vec3.x(), vec3.y(), vec3.z(), vw);
                }
            },
            else => unreachable,
        };

        pub inline fn x(self: Self) T {
            return self.data[0];
        }

        pub inline fn y(self: Self) T {
            return self.data[1];
        }

        pub inline fn xMut(self: *Self) *T {
            return &self.data[0];
        }

        pub inline fn yMut(self: *Self) *T {
            return &self.data[1];
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            const result: Data = @splat(val);
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
        pub fn negate(self: Self) Self {
            return self.scale(-1);
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @floatFromInt, @intFromFloat.
        pub fn cast(self: Self, comptime dest_type: type) GenericVector(dimensions, dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .float and dest_info != .int) {
                panic("Error, dest type should be integer or float.\n", .{});
            }

            var result: [dimensions]dest_type = undefined;

            for (result, 0..) |_, i| {
                result[i] = math.lossyCast(dest_type, self.data[i]);
            }
            return .{ .data = result };
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
        pub fn getAngle(first_vector: Self, second_vector: Self) T {
            const dot_product = dot(norm(first_vector), norm(second_vector));
            return root.toDegrees(math.acos(dot_product));
        }

        /// Return the length (magnitude) of given vector.
        /// √[x^2 + y^2 + z^2 ...]
        pub fn length(self: Self) T {
            return @sqrt(self.dot(self));
        }

        /// Return the distance between two points.
        /// √[(x1 - x2)^2 + (y1 - y2)^2 + (z1 - z2)^2 ...]
        pub fn distance(first_vector: Self, second_vector: Self) T {
            return length(first_vector.sub(second_vector));
        }

        /// Construct new normalized vector from a given one.
        pub fn norm(self: Self) Self {
            const l = self.length();
            if (l == 0) {
                return self;
            }
            const result = self.data / @as(Data, @splat(l));
            return .{ .data = result };
        }

        /// Return true if two vectors are equals.
        pub fn eql(first_vector: Self, second_vector: Self) bool {
            return @reduce(.And, first_vector.data == second_vector.data);
        }

        /// Substraction between two given vector.
        pub fn sub(first_vector: Self, second_vector: Self) Self {
            const result = first_vector.data - second_vector.data;
            return .{ .data = result };
        }

        /// Addition betwen two given vector.
        pub fn add(first_vector: Self, second_vector: Self) Self {
            const result = first_vector.data + second_vector.data;
            return .{ .data = result };
        }

        /// Component wise multiplication betwen two given vector.
        pub fn mul(first_vector: Self, second_vector: Self) Self {
            const result = first_vector.data * second_vector.data;
            return .{ .data = result };
        }

        /// Construct vector from the max components in two vectors
        pub fn max(first_vector: Self, second_vector: Self) Self {
            const result = @max(first_vector.data, second_vector.data);
            return .{ .data = result };
        }

        /// Construct vector from the min components in two vectors
        pub fn min(first_vector: Self, second_vector: Self) Self {
            const result = @min(first_vector.data, second_vector.data);
            return .{ .data = result };
        }

        /// Construct new vector after multiplying each components by a given scalar
        pub fn scale(self: Self, scalar: T) Self {
            const result = self.data * @as(Data, @splat(scalar));
            return .{ .data = result };
        }

        /// Return the dot product between two given vector.
        /// (x1 * x2) + (y1 * y2) + (z1 * z2) ...
        pub fn dot(first_vector: Self, second_vector: Self) T {
            return @reduce(.Add, first_vector.data * second_vector.data);
        }

        /// Linear interpolation between two vectors
        pub fn lerp(first_vector: Self, second_vector: Self, t: T) Self {
            const from = first_vector.data;
            const to = second_vector.data;

            const result = from + (to - from) * @as(Data, @splat(t));
            return .{ .data = result };
        }

        pub inline fn swizzle2(self: Self, comptime vx: Component, comptime vy: Component) GenericVector(2, T) {
            return GenericVector(2, T).new(self.data[@intFromEnum(vx)], self.data[@intFromEnum(vy)]);
        }

        pub inline fn swizzle3(self: Self, comptime vx: Component, comptime vy: Component, comptime vz: Component) GenericVector(3, T) {
            return GenericVector(3, T).new(self.data[@intFromEnum(vx)], self.data[@intFromEnum(vy)], self.data[@intFromEnum(vz)]);
        }

        pub inline fn swizzle4(self: Self, comptime vx: Component, comptime vy: Component, comptime vz: Component, comptime vw: Component) GenericVector(4, T) {
            return GenericVector(4, T).new(self.data[@intFromEnum(vx)], self.data[@intFromEnum(vy)], self.data[@intFromEnum(vz)], self.data[@intFromEnum(vw)]);
        }
    };
}

test "zalgebra.Vectors.eql" {
    // Vec2
    {
        const a = Vec2.new(1, 2);
        const b = Vec2.new(1, 2);
        const c = Vec2.new(1.5, 2);

        try expectEqual(Vec2.eql(a, b), true);
        try expectEqual(Vec2.eql(a, c), false);
    }

    // Vec3
    {
        const a = Vec3.new(1, 2, 3);
        const b = Vec3.new(1, 2, 3);
        const c = Vec3.new(1.5, 2, 3);

        try expectEqual(Vec3.eql(a, b), true);
        try expectEqual(Vec3.eql(a, c), false);
    }

    // Vec4
    {
        const a = Vec4.new(1, 2, 3, 4);
        const b = Vec4.new(1, 2, 3, 4);
        const c = Vec4.new(1.5, 2, 3, 4);

        try expectEqual(Vec4.eql(a, b), true);
        try expectEqual(Vec4.eql(a, c), false);
    }
}

test "zalgebra.Vectors.set" {
    // Vec2
    {
        const a = Vec2.new(2.5, 2.5);
        const b = Vec2.set(2.5);
        try expectEqual(a, b);
    }

    // Vec3
    {
        const a = Vec3.new(2.5, 2.5, 2.5);
        const b = Vec3.set(2.5);
        try expectEqual(a, b);
    }

    // Vec4
    {
        const a = Vec4.new(2.5, 2.5, 2.5, 2.5);
        const b = Vec4.set(2.5);
        try expectEqual(a, b);
    }
}

test "zalgebra.Vectors.add" {
    // Vec2
    {
        const a = Vec2.one();
        const b = Vec2.one();
        try expectEqual(a.add(b), Vec2.set(2));
    }

    // Vec3
    {
        const a = Vec3.one();
        const b = Vec3.one();
        try expectEqual(a.add(b), Vec3.set(2));
    }

    // Vec4
    {
        const a = Vec4.one();
        const b = Vec4.one();
        try expectEqual(a.add(b), Vec4.set(2));
    }
}

test "zalgebra.Vectors.negate" {
    // Vec2
    {
        const a = Vec2.set(5);
        const a_negated = Vec2.set(-5);
        try expectEqual(a.negate(), a_negated);
    }

    // Vec3
    {
        const a = Vec3.set(5);
        const a_negated = Vec3.set(-5);
        try expectEqual(a.negate(), a_negated);
    }

    // Vec4
    {
        const a = Vec4.set(5);
        const a_negated = Vec4.set(-5);
        try expectEqual(a.negate(), a_negated);
    }
}

test "zalgebra.Vectors.getAngle" {
    // Vec2
    {
        const a = Vec2.right();
        const b = Vec2.up();
        const c = Vec2.left();
        const d = Vec2.one();

        try expectEqual(a.getAngle(a), 0);
        try expectEqual(a.getAngle(b), 90);
        try expectEqual(a.getAngle(c), 180);
        try expectEqual(a.getAngle(d), 45);
    }

    // Vec3
    {
        const a = Vec3.right();
        const b = Vec3.up();
        const c = Vec3.left();
        const d = Vec3.new(1, 1, 0);

        try expectEqual(a.getAngle(a), 0);
        try expectEqual(a.getAngle(b), 90);
        try expectEqual(a.getAngle(c), 180);
        try expectEqual(a.getAngle(d), 45);
    }

    // Vec4
    {
        const a = Vec4.right();
        const b = Vec4.up();
        const c = Vec4.left();
        const d = Vec4.new(1, 1, 0, 0);

        try expectEqual(a.getAngle(a), 0);
        try expectEqual(a.getAngle(b), 90);
        try expectEqual(a.getAngle(c), 180);
        try expectEqual(a.getAngle(d), 45);
    }
}

test "zalgebra.Vectors.toArray" {
    //Vec2
    {
        const a = Vec2.up().toArray();
        const b = [_]f32{ 0, 1 };

        try std.testing.expectEqualSlices(f32, &a, &b);
    }

    //Vec3
    {
        const a = Vec3.up().toArray();
        const b = [_]f32{ 0, 1, 0 };

        try std.testing.expectEqualSlices(f32, &a, &b);
    }

    //Vec4
    {
        const a = Vec4.up().toArray();
        const b = [_]f32{ 0, 1, 0, 0 };

        try std.testing.expectEqualSlices(f32, &a, &b);
    }
}

test "zalgebra.Vectors.length" {
    // Vec2
    {
        const a = Vec2.new(1.5, 2.6);
        try expectEqual(a.length(), 3.00166606);
    }

    // Vec3
    {
        const a = Vec3.new(1.5, 2.6, 3.7);
        try expectEqual(a.length(), 4.7644519);
    }

    // Vec4
    {
        const a = Vec4.new(1.5, 2.6, 3.7, 4.7);
        try expectEqual(a.length(), 6.69253301);
    }
}

test "zalgebra.Vectors.distance" {
    // Vec2
    {
        const a = Vec2.zero();
        const b = Vec2.left();
        const c = Vec2.new(0, 5);

        try expectEqual(a.distance(b), 1);
        try expectEqual(a.distance(c), 5);
    }

    // Vec3
    {
        const a = Vec3.zero();
        const b = Vec3.left();
        const c = Vec3.new(0, 5, 0);

        try expectEqual(a.distance(b), 1);
        try expectEqual(a.distance(c), 5);
    }

    // Vec4
    {
        const a = Vec4.zero();
        const b = Vec4.left();
        const c = Vec4.new(0, 5, 0, 0);

        try expectEqual(a.distance(b), 1);
        try expectEqual(a.distance(c), 5);
    }
}

test "zalgebra.Vectors.normalize" {
    // Vec2
    {
        const a = Vec2.new(1.5, 2.6);
        const a_normalized = Vec2.new(0.499722480, 0.866185605);
        try expectEqual(a.norm(), a_normalized);
    }

    // Vec3
    {
        const a = Vec3.new(1.5, 2.6, 3.7);
        const a_normalized = Vec3.new(0.314831584, 0.545708060, 0.776584625);
        try expectEqual(a.norm(), a_normalized);
    }

    // Vec4
    {
        const a = Vec4.new(1.5, 2.6, 3.7, 4.0);
        const a_normalized = Vec4.new(0.241121411, 0.417943745, 0.594766139, 0.642990410);
        try expectEqual(a.norm(), a_normalized);
    }
}

test "zalgebra.Vectors.scale" {
    // Vec2
    {
        const a = Vec2.new(1, 2);
        const a_scaled = Vec2.new(5, 10);
        try expectEqual(a.scale(5), a_scaled);
    }

    // Vec3
    {
        const a = Vec3.new(1, 2, 3);
        const a_scaled = Vec3.new(5, 10, 15);
        try expectEqual(a.scale(5), a_scaled);
    }

    // Vec4
    {
        const a = Vec4.new(1, 2, 3, 4);
        const a_scaled = Vec4.new(5, 10, 15, 20);
        try expectEqual(a.scale(5), a_scaled);
    }
}

test "zalgebra.Vectors.dot" {
    // Vec2
    {
        const a = Vec2.new(1.5, 2.6);
        const b = Vec2.new(2.5, 3.45);
        try expectEqual(a.dot(b), 12.7200002);
    }

    // Vec3
    {
        const a = Vec3.new(1.5, 2.6, 3.7);
        const b = Vec3.new(2.5, 3.45, 1.0);
        try expectEqual(a.dot(b), 16.42);
    }

    // Vec4
    {
        const a = Vec4.new(1.5, 2.6, 3.7, 5);
        const b = Vec4.new(2.5, 3.45, 1.0, 1);
        try expectEqual(a.dot(b), 21.4200000);
    }
}

test "zalgebra.Vectors.lerp" {
    // Vec2
    {
        const a = Vec2.new(-10, 0);
        const b = Vec2.set(10);
        try expectEqual(Vec2.lerp(a, b, 0.5), Vec2.new(0, 5));
    }

    // Vec3
    {
        const a = Vec3.new(-10, 0, -10);
        const b = Vec3.set(10);
        try expectEqual(Vec3.lerp(a, b, 0.5), Vec3.new(0, 5, 0));
    }

    // Vec4
    {
        const a = Vec4.new(-10, 0, -10, -10);
        const b = Vec4.set(10);
        try expectEqual(Vec4.lerp(a, b, 0.5), Vec4.new(0, 5, 0, 0));
    }
}

test "zalgebra.Vectors.min" {
    // Vec2
    {
        const a = Vec2.new(10, -2);
        const b = Vec2.new(-10, 5);
        const minimum = Vec2.new(-10, -2);
        try expectEqual(Vec2.min(a, b), minimum);
    }

    // Vec3
    {
        const a = Vec3.new(10, -2, 0);
        const b = Vec3.new(-10, 5, 0);
        const minimum = Vec3.new(-10, -2, 0);
        try expectEqual(Vec3.min(a, b), minimum);
    }

    // Vec4
    {
        const a = Vec4.new(10, -2, 0, 1);
        const b = Vec4.new(-10, 5, 0, 1.01);
        const minimum = Vec4.new(-10, -2, 0, 1);
        try expectEqual(Vec4.min(a, b), minimum);
    }
}

test "zalgebra.Vectors.max" {
    // Vec2
    {
        const a = Vec2.new(10, -2);
        const b = Vec2.new(-10, 5);
        const maximum = Vec2.new(10, 5);
        try expectEqual(Vec2.max(a, b), maximum);
    }

    // Vec3
    {
        const a = Vec3.new(10, -2, 0);
        const b = Vec3.new(-10, 5, 0);
        const maximum = Vec3.new(10, 5, 0);
        try expectEqual(Vec3.max(a, b), maximum);
    }

    // Vec4
    {
        const a = Vec4.new(10, -2, 0, 1);
        const b = Vec4.new(-10, 5, 0, 1.01);
        const maximum = Vec4.new(10, 5, 0, 1.01);
        try expectEqual(Vec4.max(a, b), maximum);
    }
}

test "zalgebra.Vectors.fromSlice" {
    // Vec2
    {
        const slice = [_]f32{ 2, 4 };
        try expectEqual(Vec2.fromSlice(&slice), Vec2.new(2, 4));
    }

    // Vec3
    {
        const slice = [_]f32{ 2, 4, 3 };
        try expectEqual(Vec3.fromSlice(&slice), Vec3.new(2, 4, 3));
    }

    // Vec4
    {
        const slice = [_]f32{ 2, 4, 3, 6 };
        try expectEqual(Vec4.fromSlice(&slice), Vec4.new(2, 4, 3, 6));
    }
}

test "zalgebra.Vectors.cast" {
    // Vec2
    {
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
    }

    // Vec3
    {
        const a = Vec3_i32.new(3, 6, 2);
        const a_usize = Vec3_usize.new(3, 6, 2);
        try expectEqual(a.cast(usize), a_usize);

        const b = Vec3.new(3.5, 6.5, 2);
        const b_f64 = Vec3_f64.new(3.5, 6.5, 2);
        try expectEqual(b.cast(f64), b_f64);

        const c = Vec3_i32.new(3, 6, 2);
        const c_f32 = Vec3.new(3, 6, 2);
        try expectEqual(c.cast(f32), c_f32);

        const d = Vec3.new(3, 6, 2);
        const d_i32 = Vec3_i32.new(3, 6, 2);
        try expectEqual(d.cast(i32), d_i32);
    }

    // Vec4
    {
        const a = Vec4_i32.new(3, 6, 2, 0);
        const a_usize = Vec4_usize.new(3, 6, 2, 0);
        try expectEqual(a.cast(usize), a_usize);

        const b = Vec4.new(3.5, 6.5, 2, 0);
        const b_f64 = Vec4_f64.new(3.5, 6.5, 2, 0);
        try expectEqual(b.cast(f64), b_f64);

        const c = Vec4_i32.new(3, 6, 2, 0);
        const c_f32 = Vec4.new(3, 6, 2, 0);
        try expectEqual(c.cast(f32), c_f32);

        const d = Vec4.new(3, 6, 2, 0);
        const d_i32 = Vec4_i32.new(3, 6, 2, 0);
        try expectEqual(d.cast(i32), d_i32);
    }
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

test "vector conversions 2 -> 3 -> 4" {
    const v2 = Vec2.new(1, 2);
    var v3 = Vec3.fromVec2(v2, 3);
    var v4 = Vec4.fromVec3(v3, 4);

    try expectEqual(v3, Vec3.new(1, 2, 3));
    try expectEqual(v4, Vec4.new(1, 2, 3, 4));

    v3 = v2.toVec3(3);
    v4 = v2.toVec4(3, 4);

    try expectEqual(v3, Vec3.new(1, 2, 3));
    try expectEqual(v4, Vec4.new(1, 2, 3, 4));
}

test "vector conversions 4 -> 3 -> 2" {
    const v4 = Vec4.new(1, 2, 3, 4);
    var v3 = Vec3.fromVec4(v4);
    var v2 = Vec2.fromVec3(v3);

    try expectEqual(v3, Vec3.new(1, 2, 3));
    try expectEqual(v2, Vec2.new(1, 2));

    v3 = v4.toVec3();
    v2 = v4.toVec2();

    try expectEqual(v3, Vec3.new(1, 2, 3));
    try expectEqual(v2, Vec2.new(1, 2));
}

test "vector conversions 2 -> 4" {
    const v2 = Vec2.new(1, 2);
    var v4 = Vec4.fromVec2(v2, 3, 4);

    try expectEqual(v4, Vec4.new(1, 2, 3, 4));

    v4 = v2.toVec4(3, 4);

    try expectEqual(v4, Vec4.new(1, 2, 3, 4));
}

test "vector conversions 4 -> 2" {
    const v4 = Vec4.new(1, 2, 3, 4);
    var v2 = Vec2.fromVec4(v4);

    try expectEqual(v2, Vec2.new(1, 2));

    v2 = v4.toVec2();

    try expectEqual(v2, Vec2.new(1, 2));
}

// Just touching every component
// not testing every combination cause it's too many
test "swizzle2" {
    const v2 = Vec2.new(1, 2);
    const yx = v2.swizzle2(.y, .x);
    try expectEqual(Vec2.new(2, 1), yx);

    const v3 = Vec3.new(1, 2, 3);
    const zy = v3.swizzle2(.z, .y);
    const xx = v3.swizzle2(.x, .x);
    try expectEqual(Vec2.new(3, 2), zy);
    try expectEqual(Vec2.new(1, 1), xx);

    const v4 = Vec4.new(1, 2, 3, 4);
    const wz = v4.swizzle2(.w, .z);
    const xy = v4.swizzle2(.x, .y);
    try expectEqual(Vec2.new(4, 3), wz);
    try expectEqual(Vec2.new(1, 2), xy);
}

test "swizzle3" {
    const v2 = Vec2.new(1, 2);
    const yxy = v2.swizzle3(.y, .x, .y);
    try expectEqual(Vec3.new(2, 1, 2), yxy);

    const v3 = Vec3.new(1, 2, 3);
    const zyx = v3.swizzle3(.z, .y, .x);
    try expectEqual(Vec3.new(3, 2, 1), zyx);

    const v4 = Vec4.new(1, 2, 3, 4);
    const wzy = v4.swizzle3(.w, .z, .y);
    const xxx = v4.swizzle3(.x, .x, .x);
    try expectEqual(Vec3.new(4, 3, 2), wzy);
    try expectEqual(Vec3.new(1, 1, 1), xxx);
}

test "swizzle4" {
    const v2 = Vec2.new(1, 2);
    const yxyx = v2.swizzle4(.y, .x, .y, .x);
    try expectEqual(Vec4.new(2, 1, 2, 1), yxyx);

    const v3 = Vec3.new(1, 2, 3);
    const zyxz = v3.swizzle4(.z, .y, .x, .z);
    try expectEqual(Vec4.new(3, 2, 1, 3), zyxz);

    const v4 = Vec4.new(1, 2, 3, 4);
    const wzyx = v4.swizzle4(.w, .z, .y, .x);
    try expectEqual(Vec4.new(4, 3, 2, 1), wzyx);
}
