const std = @import("std");
const root = @import("main.zig");
const meta = std.meta;
const generic_vector = @import("generic_vector.zig");
const mat4 = @import("mat4.zig");
const math = std.math;
const expectApproxEqRel = std.testing.expectApproxEqRel;
const expectEqual = std.testing.expectEqual;
const expect = std.testing.expect;
const assert = std.debug.assert;

const GenericVector = generic_vector.GenericVector;

const Vec3 = generic_vector.Vec3;
const Vec4 = generic_vector.Vec4;
const Mat4x4 = mat4.Mat4x4;

pub const Quat = Quaternion(f32);
pub const Quat_f64 = Quaternion(f64);

/// A Quaternion for 3D rotations.
pub fn Quaternion(comptime T: type) type {
    if (@typeInfo(T) != .Float) {
        @compileError("Quaternion not implemented for " ++ @typeName(T));
    }

    const Vector3 = GenericVector(3, T);

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

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return new(val, val, val, val);
        }

        /// Shorthand for (1, 0, 0, 0).
        pub fn zero() Self {
            return Self.new(1, 0, 0, 0);
        }

        /// Construct new quaternion from slice.
        /// Note: Careful, the latest component `slice[3]` is the `W` component.
        pub fn fromSlice(slice: []const T) Self {
            return Self.new(slice[3], slice[0], slice[1], slice[2]);
        }

        // Construct new quaternion from given `W` component and Vector3.
        pub fn fromVec3(w: T, axis: Vector3) Self {
            return .{
                .w = w,
                .x = axis.x(),
                .y = axis.y(),
                .z = axis.z(),
            };
        }

        /// Return true if two quaternions are equal.
        pub fn eql(left: Self, right: Self) bool {
            return meta.eql(left, right);
        }

        /// Construct new normalized quaternion from a given one.
        pub fn norm(self: Self) Self {
            const l = length(self);
            if (l == 0) {
                return self;
            }
            return Self.new(
                self.w / l,
                self.x / l,
                self.y / l,
                self.z / l,
            );
        }

        /// Return the length (magnitude) of quaternion.
        pub fn length(self: Self) T {
            return @sqrt(self.dot(self));
        }

        /// Substraction between two quaternions.
        pub fn sub(left: Self, right: Self) Self {
            return Self.new(
                left.w - right.w,
                left.x - right.x,
                left.y - right.y,
                left.z - right.z,
            );
        }

        /// Addition between two quaternions.
        pub fn add(left: Self, right: Self) Self {
            return Self.new(
                left.w + right.w,
                left.x + right.x,
                left.y + right.y,
                left.z + right.z,
            );
        }

        /// Quaternions' multiplication.
        /// Produce a new quaternion from given two quaternions.
        pub fn mult(left: Self, right: Self) Self {
            var q: Self = undefined;

            q.x = (left.x * right.w) + (left.y * right.z) - (left.z * right.y) + (left.w * right.x);
            q.y = (-left.x * right.z) + (left.y * right.w) + (left.z * right.x) + (left.w * right.y);
            q.z = (left.x * right.y) - (left.y * right.x) + (left.z * right.w) + (left.w * right.z);
            q.w = (-left.x * right.x) - (left.y * right.y) - (left.z * right.z) + (left.w * right.w);

            return q;
        }

        /// Multiply each component by the given scalar.
        pub fn scale(mat: Self, scalar: T) Self {
            var result: Self = undefined;
            result.w = mat.w * scalar;
            result.x = mat.x * scalar;
            result.y = mat.y * scalar;
            result.z = mat.z * scalar;

            return result;
        }

        /// Negate the given quaternion
        pub fn negate(self: Self) Self {
            return self.scale(-1);
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

            return Self.scale(result, 0.5 / @sqrt(t));
        }

        /// Convert all Euler angles (in degrees) to quaternion.
        pub fn fromEulerAngles(axis_in_degrees: Vector3) Self {
            const x = Self.fromAxis(axis_in_degrees.x(), Vector3.right());
            const y = Self.fromAxis(axis_in_degrees.y(), Vector3.up());
            const z = Self.fromAxis(axis_in_degrees.z(), Vector3.forward());

            return z.mult(y.mult(x));
        }

        /// Convert Euler angle around specified axis to quaternion.
        pub fn fromAxis(degrees: T, axis: Vector3) Self {
            const radians = root.toRadians(degrees);

            const rot_sin = @sin(radians / 2.0);
            const quat_axis = axis.norm().data * @splat(3, rot_sin);
            const w = @cos(radians / 2.0);

            return Self.fromVec3(w, .{ .data = quat_axis });
        }

        /// Extract euler angles (degrees) from quaternion.
        pub fn extractEulerAngles(self: Self) Vector3 {
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

            return Vector3.new(root.toDegrees(yaw), root.toDegrees(pitch), root.toDegrees(roll));
        }

        /// Get the rotation angle (degrees) and axis for a given quaternion.
        // Taken from https://github.com/raysan5/raylib/blob/master/src/raymath.h#L1755
        pub fn extractAxisAngle(self: Self) struct { axis: Vec3, angle: f32 } {
            var copy = self;
            if (math.fabs(copy.w) > 1.0) copy = copy.norm();

            var res_axis = Vec3.zero();
            var res_angle: f32 = 2.0 * math.acos(copy.w);
            var den: f32 = @sqrt(1.0 - copy.w * copy.w);

            if (den > 0.0001) {
                res_axis.data[0] = copy.x / den;
                res_axis.data[1] = copy.y / den;
                res_axis.data[2] = copy.z / den;
            } else {
                // This occurs when the angle is zero.
                // Not a problem: just set an arbitrary normalized axis.
                res_axis.data[0] = 1.0;
            }

            return .{
                .axis = res_axis,
                .angle = root.toDegrees(res_angle),
            };
        }

        /// Linear interpolation between two quaternions.
        pub fn lerp(left: Self, right: Self, t: f32) Self {
            const w = root.lerp(T, left.w, right.w, t);
            const x = root.lerp(T, left.x, right.x, t);
            const y = root.lerp(T, left.y, right.y, t);
            const z = root.lerp(T, left.z, right.z, t);
            return Self.new(w, x, y, z);
        }

        // Shortest path slerp between two quaternions.
        // Taken from "Physically Based Rendering, 3rd Edition, Chapter 2.9.2"
        // https://pbr-book.org/3ed-2018/Geometry_and_Transformations/Animating_Transformations#QuaternionInterpolation
        pub fn slerp(left: Self, right: Self, t: f32) Self {
            const ParallelThreshold: f32 = 0.9995;
            var cos_theta = dot(left, right);
            var right1 = right;

            // We need the absolute value of the dot product to take the shortest path
            if (cos_theta < 0.0) {
                cos_theta *= -1;
                right1 = right.negate();
            }

            if (cos_theta > ParallelThreshold) {
                // Use regular old lerp to avoid numerical instability
                return lerp(left, right1, t);
            } else {
                var theta = math.acos(math.clamp(cos_theta, -1, 1));
                var thetap = theta * t;
                var qperp = right1.sub(left.scale(cos_theta)).norm();
                return left.scale(@cos(thetap)).add(qperp.scale(@sin(thetap)));
            }
        }

        /// Rotate the Vector3 v using the sandwich product.
        /// Taken from "Foundations of Game Engine Development Vol. 1 Mathematics".
        pub fn rotateVec(self: Self, v: Vector3) Vector3 {
            const q = self.norm();
            const b = Vector3.new(q.x, q.y, q.z);
            const b2 = Vector3.dot(b, b);

            return v.scale(q.w * q.w - b2).add(b.scale(v.dot(b) * 2.0)).add(b.cross(v).scale(q.w * 2.0));
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt.
        pub fn cast(self: Self, dest_type: anytype) Quaternion(dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .Float) {
                std.debug.panic("Error, dest type should be float.\n", .{});
            }

            var result: Quaternion(dest_type) = undefined;
            result.w = @floatCast(dest_type, self.w);
            result.x = @floatCast(dest_type, self.x);
            result.y = @floatCast(dest_type, self.y);
            result.z = @floatCast(dest_type, self.z);
            return result;
        }
    };
}

test "zalgebra.Quaternion.new" {
    const q = Quat.new(1.5, 2.6, 3.7, 4.7);

    try expectEqual(q.w, 1.5);
    try expectEqual(q.x, 2.6);
    try expectEqual(q.y, 3.7);
    try expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.set" {
    const a = Quat.set(12);
    const b = Quat.new(12, 12, 12, 12);

    try expectEqual(a, b);
}

test "zalgebra.Quaternion.eql" {
    const q1 = Quat.new(1.5, 2.6, 3.7, 4.7);
    const q2 = Quat.new(1.5, 2.6, 3.7, 4.7);
    const q3 = Quat.new(2.6, 3.7, 4.8, 5.9);

    try expectEqual(Quat.eql(q1, q2), true);
    try expectEqual(Quat.eql(q1, q3), false);
}

test "zalgebra.Quaternion.fromSlice" {
    const array = [4]f32{ 2, 3, 4, 1 };
    try expectEqual(Quat.fromSlice(&array), Quat.new(1, 2, 3, 4));
}

test "zalgebra.Quaternion.fromVec3" {
    const q = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));

    try expectEqual(q.w, 1.5);
    try expectEqual(q.x, 2.6);
    try expectEqual(q.y, 3.7);
    try expectEqual(q.z, 4.7);
}

test "zalgebra.Quaternion.fromVec3" {
    const q1 = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const q2 = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const q3 = Quat.fromVec3(1, Vec3.new(2.6, 3.7, 4.7));

    try expectEqual(q1, q2);
    try expectEqual(Quat.eql(q1, q3), false);
}

test "zalgebra.Quaternion.norm" {
    const q1 = Quat.fromVec3(1, Vec3.new(2, 2.0, 2.0));
    const q2 = Quat.fromVec3(0.2773500978946686, Vec3.new(0.5547001957893372, 0.5547001957893372, 0.5547001957893372));

    try expectEqual(q1.norm(), q2);
}

test "zalgebra.Quaternion.fromEulerAngles" {
    const q1 = Quat.fromEulerAngles(Vec3.new(10, 5, 45));
    const res_q1 = q1.extractEulerAngles();

    const q2 = Quat.fromEulerAngles(Vec3.new(0, 55, 22));
    const res_q2 = q2.toMat4().extractEulerAngles();

    try expectEqual(res_q1, Vec3.new(9.999999046325684, 5.000000476837158, 45));
    try expectEqual(res_q2, Vec3.new(0, 47.2450294, 22));
}

test "zalgebra.Quaternion.fromAxis" {
    const q1 = Quat.fromAxis(45, Vec3.up());
    const res_q1 = q1.extractEulerAngles();

    try expectEqual(res_q1, Vec3.new(0, 45.0000076, 0));
}

test "zalgebra.Quaternion.extractAxisAngle" {
    const axis = Vec3.new(44, 120, 8).norm();
    const q1 = Quat.fromAxis(45, axis);
    const res = q1.extractAxisAngle();
    const eps_value = comptime math.epsilon(f32);

    try expect(math.approxEqRel(f32, axis.x(), res.axis.x(), eps_value) and
        math.approxEqRel(f32, axis.y(), res.axis.y(), eps_value) and
        math.approxEqRel(f32, axis.z(), res.axis.z(), eps_value));

    try expectApproxEqRel(@as(f32, 45.0000076), res.angle, eps_value);
}

test "zalgebra.Quaternion.extractEulerAngles" {
    const q1 = Quat.fromVec3(0.5, Vec3.new(0.5, 1, 0.3));
    const res_q1 = q1.extractEulerAngles();

    try expectEqual(res_q1, Vec3.new(129.6000213623047, 44.427005767822266, 114.4107360839843));
}

test "zalgebra.Quaternion.rotateVec" {
    const eps_value = comptime std.math.epsilon(f32);
    const q = Quat.fromEulerAngles(Vec3.set(45));
    const m = q.toMat4();

    const v = Vec3.up();
    const v1 = q.rotateVec(v);
    const v2 = m.multByVec4(Vec4.new(v.x(), v.y(), v.z(), 1.0));

    try expect(std.math.approxEqAbs(f32, v1.x(), -1.46446585e-01, eps_value));
    try expect(std.math.approxEqAbs(f32, v1.y(), 8.53553473e-01, eps_value));
    try expect(std.math.approxEqAbs(f32, v1.z(), 0.5, eps_value));

    try expect(std.math.approxEqAbs(f32, v1.x(), v2.data[0], eps_value));
    try expect(std.math.approxEqAbs(f32, v1.y(), v2.data[1], eps_value));
    try expect(std.math.approxEqAbs(f32, v1.z(), v2.data[2], eps_value));
}

test "zalgebra.Quaternion.lerp" {
    const eps_value = comptime std.math.epsilon(f32);
    var v1 = Quat.zero();
    var v2 = Quat.fromAxis(180, Vec3.up());
    try expectEqual(Quat.lerp(v1, v2, 1.0), v2);
    var v3 = Quat.lerp(v1, v2, 0.5);
    var v4 = Quat.new(4.99999970e-01, 0, 4.99999970e-01, 0);
    try expect(std.math.approxEqAbs(f32, v3.w, v4.w, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.x, v4.x, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.y, v4.y, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.z, v4.z, eps_value));
}

test "zalgebra.Quaternion.slerp" {
    const eps_value = comptime std.math.epsilon(f32);
    var v1 = Quat.zero();
    var v2 = Quat.fromAxis(180, Vec3.up());
    try expectEqual(Quat.slerp(v1, v2, 1.0), Quat.new(7.54979012e-08, 0, -1, 0));
    var v3 = Quat.slerp(v1, v2, 0.5);
    var v4 = Quat.new(7.071067e-01, 0, -7.071067e-01, 0);
    try expect(std.math.approxEqAbs(f32, v3.w, v4.w, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.x, v4.x, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.y, v4.y, eps_value));
    try expect(std.math.approxEqAbs(f32, v3.z, v4.z, eps_value));
}

test "zalgebra.Quaternion.cast" {
    const a = Quat.new(3.5, 4.5, 5.5, 6.5);
    const a_f64 = Quat_f64.new(3.5, 4.5, 5.5, 6.5);
    try expectEqual(a.cast(f64), a_f64);
    try expectEqual(a_f64.cast(f32), a);
}
