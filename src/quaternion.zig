const std = @import("std");
const math = std.math;
const testing = std.testing;
const assert = std.debug.assert;
const root = @import("main.zig");
usingnamespace @import("vec3.zig");
usingnamespace @import("vec4.zig");
usingnamespace @import("mat4.zig");

pub const quat = Quaternion(f32);
pub const quat_f64 = Quaternion(f64);

/// A Quaternion for 3D rotations.
pub fn Quaternion(comptime T: type) type {
    if (@typeInfo(T) != .Float) {
        @compileError("Quaternion not implemented for " ++ @typeName(T));
    }

    return struct {
        w: T,
        x: T,
        y: T,
        z: T,

        const Self = @This();

        /// Construct new quaternion from floats.
        pub fn new(w: T, x: T, y: T, z: T) Self {
            return .{
                .w = w,
                .x = x,
                .y = y,
                .z = z,
            };
        }

        /// Construct most basic quaternion.
        pub fn zero() Self {
            return Self.new(1, 0, 0, 0);
        }

        /// Construct new quaternion from slice.
        /// Note: Careful, the latest component `slice[3]` is the `W` component.
        pub fn from_slice(slice: []const T) Self {
            return Self.new(slice[3], slice[0], slice[1], slice[2]);
        }

        pub fn from_vec3(w: T, axis: Vec3(T)) Self {
            return .{
                .w = w,
                .x = axis.x,
                .y = axis.y,
                .z = axis.z,
            };
        }

        pub fn is_eq(left: Self, right: Self) bool {
            return (left.w == right.w and
                left.x == right.x and
                left.y == right.y and
                left.z == right.z);
        }

        pub fn norm(self: Self) Self {
            const l = length(self);
            assert(l != 0);

            return Self.new(self.w / l, self.x / l, self.y / l, self.z / l);
        }

        pub fn length(self: Self) T {
            return math.sqrt((self.w * self.w) +
                (self.x * self.x) +
                (self.y * self.y) +
                (self.z * self.z));
        }

        pub fn sub(left: Self, right: Self) Self {
            return Self.new(
                left.w - right.w,
                left.x - right.x,
                left.y - right.y,
                left.z - right.z,
            );
        }

        pub fn add(left: Self, right: Self) Self {
            return Self.new(
                left.w + right.w,
                left.x + right.x,
                left.y + right.y,
                left.z + right.z,
            );
        }

        pub fn mult(left: Self, right: Self) Self {
            var q: Self = undefined;

            q.x = (left.x * right.w) + (left.y * right.z) - (left.z * right.y) + (left.w * right.x);
            q.y = (-left.x * right.z) + (left.y * right.w) + (left.z * right.x) + (left.w * right.y);
            q.z = (left.x * right.y) - (left.y * right.x) + (left.z * right.w) + (left.w * right.z);
            q.w = (-left.x * right.x) - (left.y * right.y) - (left.z * right.z) + (left.w * right.w);

            return q;
        }

        pub fn scale(mat: Self, scalar: T) Self {
            var result: Self = undefined;
            result.w = mat.w * scalar;
            result.x = mat.x * scalar;
            result.y = mat.y * scalar;
            result.z = mat.z * scalar;

            return result;
        }

        /// Return the dot product between two quaternion.
        pub fn dot(left: Self, right: Self) T {
            return (left.x * right.x) + (left.y * right.y) + (left.z * right.z) + (left.w * right.w);
        }

        /// Convert given quaternion to rotation 4x4 matrix.
        /// Mostly taken from https://github.com/HandmadeMath/Handmade-Math.
        pub fn to_mat4(self: Self) Mat4(T) {
            var result: Mat4(T) = undefined;

            const normalized = self.norm();
            const xx = normalized.x * normalized.x;
            const yy = normalized.y * normalized.y;
            const zz = normalized.z * normalized.z;
            const xy = normalized.x * normalized.y;
            const xz = normalized.x * normalized.z;
            const yz = normalized.y * normalized.z;
            const wx = normalized.w * normalized.x;
            const wy = normalized.w * normalized.y;
            const wz = normalized.w * normalized.z;

            result.data[0][0] = 1.0 - 2.0 * (yy + zz);
            result.data[0][1] = 2.0 * (xy + wz);
            result.data[0][2] = 2.0 * (xz - wy);
            result.data[0][3] = 0.0;

            result.data[1][0] = 2.0 * (xy - wz);
            result.data[1][1] = 1.0 - 2.0 * (xx + zz);
            result.data[1][2] = 2.0 * (yz + wx);
            result.data[1][3] = 0.0;

            result.data[2][0] = 2.0 * (xz + wy);
            result.data[2][1] = 2.0 * (yz - wx);
            result.data[2][2] = 1.0 - 2.0 * (xx + yy);
            result.data[2][3] = 0.0;

            result.data[3][0] = 0.0;
            result.data[3][1] = 0.0;
            result.data[3][2] = 0.0;
            result.data[3][3] = 1.0;

            return result;
        }

        /// From Mike Day at Insomniac Games.
        /// For more details: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
        pub fn from_mat4(m: Mat4(T)) Self {
            var t: f32 = 0;
            var result: Self = undefined;

            if (m.data[2][2] < 0.0) {
                if (m.data[0][0] > m.data[1][1]) {
                    t = 1 + m.data[0][0] - m.data[1][1] - m.data[2][2];
                    result = Self.new(
                        m.data[1][2] - m.data[2][1],
                        t,
                        m.data[0][1] + m.data[1][0],
                        m.data[2][0] + m.data[0][2],
                    );
                } else {
                    t = 1 - m.data[0][0] + m.data[1][1] - m.data[2][2];
                    result = Self.new(
                        m.data[2][0] - m.data[0][2],
                        m.data[0][1] + m.data[1][0],
                        t,
                        m.data[1][2] + m.data[2][1],
                    );
                }
            } else {
                if (m.data[0][0] < -m.data[1][1]) {
                    t = 1 - m.data[0][0] - m.data[1][1] + m.data[2][2];
                    result = Self.new(
                        m.data[0][1] - m.data[1][0],
                        m.data[2][0] + m.data[0][2],
                        m.data[1][2] + m.data[2][1],
                        t,
                    );
                } else {
                    t = 1 + m.data[0][0] + m.data[1][1] + m.data[2][2];
                    result = Self.new(
                        t,
                        m.data[1][2] - m.data[2][1],
                        m.data[2][0] - m.data[0][2],
                        m.data[0][1] - m.data[1][0],
                    );
                }
            }

            return Self.scale(result, 0.5 / math.sqrt(t));
        }

        /// Convert all Euler angles to quaternion.
        pub fn from_euler_angle(axis: Vec3(T)) Self {
            const x = Self.from_axis(axis.x, vec3.new(1, 0, 0));
            const y = Self.from_axis(axis.y, vec3.new(0, 1, 0));
            const z = Self.from_axis(axis.z, vec3.new(0, 0, 1));

            return z.mult(y.mult(x));
        }

        /// Convert Euler angle around specified axis to quaternion.
        pub fn from_axis(degrees: T, axis: Vec3(T)) Self {
            const radians = root.to_radians(degrees);

            const rot_sin = math.sin(radians / 2.0);
            const quat_axis = axis.norm().scale(rot_sin);
            const w = math.cos(radians / 2.0);

            return Self.from_vec3(w, quat_axis);
        }

        /// Extract euler angles from quaternion.
        pub fn extract_rotation(self: Self) Vec3(T) {
            const yaw = math.atan2(T, 2.0 * (self.y * self.z + self.w * self.x), self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z);
            const pitch = math.asin(-2.0 * (self.x * self.z - self.w * self.y));
            const roll = math.atan2(T, 2.0 * (self.x * self.y + self.w * self.z), self.w * self.w + self.x * self.x - self.y * self.y - self.z * self.z);

            return Vec3(T).new(root.to_degrees(yaw), root.to_degrees(pitch), root.to_degrees(roll));
        }
    };
}

test "zalgebra.Quaternion.new" {
    const q = quat.new(1.5, 2.6, 3.7, 4.7);

    testing.expectEqual(q.w, 1.5);
    testing.expectEqual(q.x, 2.6);
    testing.expectEqual(q.y, 3.7);
    testing.expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.from_slice" {
    const array = [4]f32{ 2, 3, 4, 1 };
    testing.expectEqual(quat.is_eq(quat.from_slice(&array), quat.new(1, 2, 3, 4)), true);
}

test "zalgebra.Quaternion.from_vec3" {
    const q = quat.from_vec3(1.5, vec3.new(2.6, 3.7, 4.7));

    testing.expectEqual(q.w, 1.5);
    testing.expectEqual(q.x, 2.6);
    testing.expectEqual(q.y, 3.7);
    testing.expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.from_vec3" {
    const q1 = quat.from_vec3(1.5, vec3.new(2.6, 3.7, 4.7));
    const q2 = quat.from_vec3(1.5, vec3.new(2.6, 3.7, 4.7));
    const q3 = quat.from_vec3(1., vec3.new(2.6, 3.7, 4.7));

    testing.expectEqual(q1.is_eq(q2), true);
    testing.expectEqual(q1.is_eq(q3), false);
}

test "zalgebra.Quaternion.norm" {
    const q1 = quat.from_vec3(1., vec3.new(2., 2.0, 2.0));
    const q2 = quat.from_vec3(0.2773500978946686, vec3.new(0.5547001957893372, 0.5547001957893372, 0.5547001957893372));

    testing.expectEqual(q1.norm().is_eq(q2), true);
}

test "zalgebra.Quaternion.from_euler_angle" {
    const q1 = quat.from_euler_angle(vec3.new(10, 5, 45));
    const res_q1 = q1.extract_rotation();

    const q2 = quat.from_euler_angle(vec3.new(0, 55, 22));
    const res_q2 = q2.to_mat4().extract_rotation();

    testing.expectEqual(vec3.is_eq(res_q1, vec3.new(9.999999046325684, 5.000000476837158, 45)), true);
    testing.expectEqual(vec3.is_eq(res_q2, vec3.new(0, 47.245025634765625, 22)), true);
}

test "zalgebra.Quaternion.from_axis" {
    const q1 = quat.from_axis(45, vec3.new(0, 1, 0));
    const res_q1 = q1.extract_rotation();

    testing.expectEqual(vec3.is_eq(res_q1, vec3.new(0, 45.0000076, 0)), true);
}

test "zalgebra.Quaternion.extract_rotation" {
    const q1 = quat.from_vec3(0.5, vec3.new(0.5, 1, 0.3));
    const res_q1 = q1.extract_rotation();

    testing.expectEqual(vec3.is_eq(res_q1, vec3.new(129.6000213623047, 44.427005767822266, 114.41073608398438)), true);
}
