const std = @import("std");
const math = std.math;
const testing = std.testing;
const assert = std.debug.assert;
const root = @import("main.zig");
usingnamespace @import("vec3.zig");
usingnamespace @import("vec4.zig");
usingnamespace @import("mat4.zig");

pub const Quat = Quaternion(f32);
pub const Quat_f64 = Quaternion(f64);

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
        pub fn fromSlice(slice: []const T) Self {
            return Self.new(slice[3], slice[0], slice[1], slice[2]);
        }

        pub fn fromVec3(w: T, axis: Vector3(T)) Self {
            return .{
                .w = w,
                .x = axis.x,
                .y = axis.y,
                .z = axis.z,
            };
        }

        pub fn eql(left: Self, right: Self) bool {
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
        pub fn toMat4(self: Self) Mat4x4(T) {
            var result: Mat4x4(T) = undefined;

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
        pub fn fromMat4(m: Mat4x4(T)) Self {
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
        pub fn fromEulerAngle(axis: Vector3(T)) Self {
            const x = Self.fromAxis(axis.x, Vec3.new(1, 0, 0));
            const y = Self.fromAxis(axis.y, Vec3.new(0, 1, 0));
            const z = Self.fromAxis(axis.z, Vec3.new(0, 0, 1));

            return z.mult(y.mult(x));
        }

        /// Convert Euler angle around specified axis to quaternion.
        pub fn fromAxis(degrees: T, axis: Vector3(T)) Self {
            const radians = root.toRadians(degrees);

            const rot_sin = math.sin(radians / 2.0);
            const quat_axis = axis.norm().scale(rot_sin);
            const w = math.cos(radians / 2.0);

            return Self.fromVec3(w, quat_axis);
        }

        /// Extract euler angles from quaternion.
        pub fn extractRotation(self: Self) Vector3(T) {
            const yaw = math.atan2(
                T,
                2.0 * (self.y * self.z + self.w * self.x),
                self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z,
            );
            const pitch = math.asin(
                -2.0 * (self.x * self.z - self.w * self.y),
            );
            const roll = math.atan2(
                T,
                2.0 * (self.x * self.y + self.w * self.z),
                self.w * self.w + self.x * self.x - self.y * self.y - self.z * self.z,
            );

            return Vector3(T).new(root.toDegrees(yaw), root.toDegrees(pitch), root.toDegrees(roll));
        }

        /// Rotate the vector v using the sandwich product.
        /// Taken from "Foundations of Game Engine Development Vol. 1 Mathematics".
        pub fn rotateVec(self: Self, v: Vector3(T)) Vector3(T) {
            const q = self.norm();
            const b = Vector3(T).new(q.x, q.y, q.z);
            const b2 = b.x * b.x + b.y * b.y + b.z * b.z;

            return v.scale(q.w * q.w - b2).add(b.scale(v.dot(b) * 2.0)).add(b.cross(v).scale(q.w * 2.0));
        }
    };
}

test "zalgebra.Quaternion.new" {
    const q = Quat.new(1.5, 2.6, 3.7, 4.7);

    try testing.expectEqual(q.w, 1.5);
    try testing.expectEqual(q.x, 2.6);
    try testing.expectEqual(q.y, 3.7);
    try testing.expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.fromSlice" {
    const array = [4]f32{ 2, 3, 4, 1 };
    try testing.expectEqual(Quat.eql(Quat.fromSlice(&array), Quat.new(1, 2, 3, 4)), true);
}

test "zalgebra.Quaternion.fromVec3" {
    const q = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));

    try testing.expectEqual(q.w, 1.5);
    try testing.expectEqual(q.x, 2.6);
    try testing.expectEqual(q.y, 3.7);
    try testing.expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.fromVec3" {
    const q1 = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const q2 = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const q3 = Quat.fromVec3(1, Vec3.new(2.6, 3.7, 4.7));

    try testing.expectEqual(q1.eql(q2), true);
    try testing.expectEqual(q1.eql(q3), false);
}

test "zalgebra.Quaternion.norm" {
    const q1 = Quat.fromVec3(1, Vec3.new(2, 2.0, 2.0));
    const q2 = Quat.fromVec3(0.2773500978946686, Vec3.new(0.5547001957893372, 0.5547001957893372, 0.5547001957893372));

    try testing.expectEqual(q1.norm().eql(q2), true);
}

test "zalgebra.Quaternion.fromEulerAngle" {
    const q1 = Quat.fromEulerAngle(Vec3.new(10, 5, 45));
    const res_q1 = q1.extractRotation();

    const q2 = Quat.fromEulerAngle(Vec3.new(0, 55, 22));
    const res_q2 = q2.toMat4().extractRotation();

    try testing.expectEqual(Vec3.eql(res_q1, Vec3.new(9.999999046325684, 5.000000476837158, 45)), true);
    try testing.expectEqual(Vec3.eql(res_q2, Vec3.new(0, 47.245025634765625, 22)), true);
}

test "zalgebra.Quaternion.fromAxis" {
    const q1 = Quat.fromAxis(45, Vec3.new(0, 1, 0));
    const res_q1 = q1.extractRotation();

    try testing.expectEqual(Vec3.eql(res_q1, Vec3.new(0, 45.0000076, 0)), true);
}

test "zalgebra.Quaternion.extractRotation" {
    const q1 = Quat.fromVec3(0.5, Vec3.new(0.5, 1, 0.3));
    const res_q1 = q1.extractRotation();

    try testing.expectEqual(Vec3.eql(res_q1, Vec3.new(129.6000213623047, 44.427005767822266, 114.41073608398438)), true);
}

test "zalgebra.Quaternion.rotateVec" {
    const eps_value = comptime std.math.epsilon(f32);
    const q = Quat.fromEulerAngle(Vec3.new(45, 45, 45));
    const m = q.toMat4();

    const v = Vec3.new(0, 1, 0);
    const v1 = q.rotateVec(v);
    const v2 = m.multByVec4(Vec4.new(v.x, v.y, v.z, 1.0));

    try testing.expect(std.math.approxEqAbs(f32, v1.x, -1.46446585e-01, eps_value));
    try testing.expect(std.math.approxEqAbs(f32, v1.y, 8.53553473e-01, eps_value));
    try testing.expect(std.math.approxEqAbs(f32, v1.z, 0.5, eps_value));

    try testing.expect(std.math.approxEqAbs(f32, v1.x, v2.x, eps_value));
    try testing.expect(std.math.approxEqAbs(f32, v1.y, v2.y, eps_value));
    try testing.expect(std.math.approxEqAbs(f32, v1.z, v2.z, eps_value));
}
