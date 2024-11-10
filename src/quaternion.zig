const std = @import("std");
const root = @import("main.zig");
const meta = std.meta;
const generic_vector = @import("generic_vector.zig");
const mat3 = @import("mat3.zig");
const mat4 = @import("mat4.zig");
const math = std.math;
const eps_value = math.floatEps(f32);
const expectApproxEqAbs = std.testing.expectApproxEqAbs;
const expectApproxEqRel = std.testing.expectApproxEqRel;
const expectEqual = std.testing.expectEqual;
const expect = std.testing.expect;
const assert = std.debug.assert;

const GenericVector = generic_vector.GenericVector;

const Vec3 = generic_vector.Vec3;
const Vec4 = generic_vector.Vec4;
const Mat3x3 = mat3.Mat3x3;
const Mat4x4 = mat4.Mat4x4;

pub const Quat = Quaternion(f32);
pub const Quat_f64 = Quaternion(f64);

/// A Quaternion for 3D rotations.
pub fn Quaternion(comptime T: type) type {
    if (@typeInfo(T) != .float) {
        @compileError("Quaternion not implemented for " ++ @typeName(T));
    }

    const Vector3 = GenericVector(3, T);

    return extern struct {
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

        /// Shorthand for (1, 0, 0, 0).
        pub fn identity() Self {
            return Self.new(1, 0, 0, 0);
        }

        /// Set all components to the same given value.
        pub fn set(val: T) Self {
            return Self.new(val, val, val, val);
        }

        /// Construct new quaternion from slice.
        /// Note: Careful, the latest component `slice[3]` is the `W` component.
        pub fn fromSlice(slice: []const T) Self {
            return Self.new(slice[3], slice[0], slice[1], slice[2]);
        }

        // Construct new quaternion from given `W` component and Vector3.
        pub fn fromVec3(w: T, axis: Vector3) Self {
            return Self.new(w, axis.x(), axis.y(), axis.z());
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
        pub fn mul(left: Self, right: Self) Self {
            const x = (left.x * right.w) + (left.y * right.z) - (left.z * right.y) + (left.w * right.x);
            const y = (-left.x * right.z) + (left.y * right.w) + (left.z * right.x) + (left.w * right.y);
            const z = (left.x * right.y) - (left.y * right.x) + (left.z * right.w) + (left.w * right.z);
            const w = (-left.x * right.x) - (left.y * right.y) - (left.z * right.z) + (left.w * right.w);

            return Self.new(w, x, y, z);
        }

        /// Multiply each component by the given scalar.
        pub fn scale(mat: Self, scalar: T) Self {
            const w = mat.w * scalar;
            const x = mat.x * scalar;
            const y = mat.y * scalar;
            const z = mat.z * scalar;

            return Self.new(w, x, y, z);
        }

        /// Negate the given quaternion
        pub fn negate(self: Self) Self {
            return self.scale(-1);
        }

        /// Return the dot product between two quaternion.
        pub fn dot(left: Self, right: Self) T {
            return (left.x * right.x) + (left.y * right.y) + (left.z * right.z) + (left.w * right.w);
        }

        /// Convert given quaternion to rotation 3x3 matrix.
        fn toMat3(self: Self) Mat3x3(T) {
            var result: Mat3x3(T) = undefined;

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

            result.data[0][0] = 1 - 2 * (yy + zz);
            result.data[0][1] = 2 * (xy + wz);
            result.data[0][2] = 2 * (xz - wy);

            result.data[1][0] = 2 * (xy - wz);
            result.data[1][1] = 1 - 2 * (xx + zz);
            result.data[1][2] = 2 * (yz + wx);

            result.data[2][0] = 2 * (xz + wy);
            result.data[2][1] = 2 * (yz - wx);
            result.data[2][2] = 1 - 2 * (xx + yy);

            return result;
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

            result.data[0][0] = 1 - 2 * (yy + zz);
            result.data[0][1] = 2 * (xy + wz);
            result.data[0][2] = 2 * (xz - wy);
            result.data[0][3] = 0;

            result.data[1][0] = 2 * (xy - wz);
            result.data[1][1] = 1 - 2 * (xx + zz);
            result.data[1][2] = 2 * (yz + wx);
            result.data[1][3] = 0;

            result.data[2][0] = 2 * (xz + wy);
            result.data[2][1] = 2 * (yz - wx);
            result.data[2][2] = 1 - 2 * (xx + yy);
            result.data[2][3] = 0;

            result.data[3][0] = 0;
            result.data[3][1] = 0;
            result.data[3][2] = 0;
            result.data[3][3] = 1;

            return result;
        }

        /// From Mike Day at Insomniac Games.
        /// For more details: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
        pub fn fromMat4(mat: Mat4x4(T)) Self {
            var t: T = undefined;
            var result: Self = undefined;

            if (mat.data[2][2] < 0) {
                if (mat.data[0][0] > mat.data[1][1]) {
                    t = 1 + mat.data[0][0] - mat.data[1][1] - mat.data[2][2];
                    result = Self.new(
                        mat.data[1][2] - mat.data[2][1],
                        t,
                        mat.data[0][1] + mat.data[1][0],
                        mat.data[2][0] + mat.data[0][2],
                    );
                } else {
                    t = 1 - mat.data[0][0] + mat.data[1][1] - mat.data[2][2];
                    result = Self.new(
                        mat.data[2][0] - mat.data[0][2],
                        mat.data[0][1] + mat.data[1][0],
                        t,
                        mat.data[1][2] + mat.data[2][1],
                    );
                }
            } else {
                if (mat.data[0][0] < -mat.data[1][1]) {
                    t = 1 - mat.data[0][0] - mat.data[1][1] + mat.data[2][2];
                    result = Self.new(
                        mat.data[0][1] - mat.data[1][0],
                        mat.data[2][0] + mat.data[0][2],
                        mat.data[1][2] + mat.data[2][1],
                        t,
                    );
                } else {
                    t = 1 + mat.data[0][0] + mat.data[1][1] + mat.data[2][2];
                    result = Self.new(
                        t,
                        mat.data[1][2] - mat.data[2][1],
                        mat.data[2][0] - mat.data[0][2],
                        mat.data[0][1] - mat.data[1][0],
                    );
                }
            }

            return result.scale(0.5 / @sqrt(t));
        }

        /// Convert all Euler angles (in degrees) to quaternion.
        pub fn fromEulerAngles(axis_in_degrees: Vector3) Self {
            const x = Self.fromAxis(axis_in_degrees.x(), Vector3.right());
            const y = Self.fromAxis(axis_in_degrees.y(), Vector3.up());
            const z = Self.fromAxis(axis_in_degrees.z(), Vector3.forward());

            return z.mul(y.mul(x));
        }

        /// Convert Euler angle around specified axis to quaternion.
        pub fn fromAxis(degrees: T, axis: Vector3) Self {
            const radians = root.toRadians(degrees);

            const rot_sin = @sin(radians / 2);
            const quat_axis = axis.norm().data * @as(@TypeOf(axis.data), @splat(rot_sin));
            const w = @cos(radians / 2);

            return Self.fromVec3(w, .{ .data = quat_axis });
        }

        /// Extract euler angles (degrees) from quaternion.
        pub fn extractEulerAngles(self: Self) Vector3 {
            const yaw = math.atan2(
                2 * (self.y * self.z + self.w * self.x),
                self.w * self.w - self.x * self.x - self.y * self.y + self.z * self.z,
            );
            const pitch = math.asin(
                -2 * (self.x * self.z - self.w * self.y),
            );
            const roll = math.atan2(
                2 * (self.x * self.y + self.w * self.z),
                self.w * self.w + self.x * self.x - self.y * self.y - self.z * self.z,
            );

            return Vector3.new(root.toDegrees(yaw), root.toDegrees(pitch), root.toDegrees(roll));
        }

        /// Get the rotation angle (degrees) and axis for a given quaternion.
        // Taken from https://github.com/raysan5/raylib/blob/master/src/raymath.h#L1755
        pub fn extractAxisAngle(self: Self) struct { axis: Vector3, angle: T } {
            var copy = self;
            if (@abs(copy.w) > 1) copy = copy.norm();

            var res_axis = Vector3.zero();
            const res_angle: T = 2 * math.acos(copy.w);
            const den: T = @sqrt(1 - copy.w * copy.w);

            if (den > 0.0001) {
                res_axis.data[0] = copy.x / den;
                res_axis.data[1] = copy.y / den;
                res_axis.data[2] = copy.z / den;
            } else {
                // This occurs when the angle is zero.
                // Not a problem: just set an arbitrary normalized axis.
                res_axis.data[0] = 1;
            }

            return .{
                .axis = res_axis,
                .angle = root.toDegrees(res_angle),
            };
        }

        /// Construct inverse quaternion
        pub fn inv(self: Self) Self {
            const res = Self.new(self.w, -self.x, -self.y, -self.z);
            return res.scale(1 / self.dot(self));
        }

        /// Linear interpolation between two quaternions.
        pub fn lerp(left: Self, right: Self, t: T) Self {
            const w = root.lerp(T, left.w, right.w, t);
            const x = root.lerp(T, left.x, right.x, t);
            const y = root.lerp(T, left.y, right.y, t);
            const z = root.lerp(T, left.z, right.z, t);
            return Self.new(w, x, y, z);
        }

        // Shortest path slerp between two quaternions.
        // Taken from "Physically Based Rendering, 3rd Edition, Chapter 2.9.2"
        // https://pbr-book.org/3ed-2018/Geometry_and_Transformations/Animating_Transformations#QuaternionInterpolation
        pub fn slerp(left: Self, right: Self, t: T) Self {
            const ParallelThreshold = 0.9995;
            var cos_theta = dot(left, right);
            var right1 = right;

            // We need the absolute value of the dot product to take the shortest path
            if (cos_theta < 0) {
                cos_theta *= -1;
                right1 = right.negate();
            }

            if (cos_theta > ParallelThreshold) {
                // Use regular old lerp to avoid numerical instability
                return lerp(left, right1, t);
            } else {
                const theta = math.acos(math.clamp(cos_theta, -1, 1));
                const thetap = theta * t;
                var qperp = right1.sub(left.scale(cos_theta)).norm();
                return left.scale(@cos(thetap)).add(qperp.scale(@sin(thetap)));
            }
        }

        /// Rotate the Vector3 v using the sandwich product.
        /// Taken from "Foundations of Game Engine Development Vol. 1 Mathematics".
        pub fn rotateVec(self: Self, v: Vector3) Vector3 {
            const q = self.norm();
            const b = Vector3.new(q.x, q.y, q.z);
            const b2 = b.dot(b);

            return v.scale(q.w * q.w - b2).add(b.scale(v.dot(b) * 2)).add(b.cross(v).scale(q.w * 2));
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @floatFromInt, @intFromFloat.
        pub fn cast(self: Self, comptime dest_type: type) Quaternion(dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .float) {
                std.debug.panic("Error, dest type should be float.\n", .{});
            }

            const w: dest_type = @floatCast(self.w);
            const x: dest_type = @floatCast(self.x);
            const y: dest_type = @floatCast(self.y);
            const z: dest_type = @floatCast(self.z);
            return Quaternion(dest_type).new(w, x, y, z);
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
    const a = Quat.new(1.5, 2.6, 3.7, 4.7);
    const b = Quat.new(1.5, 2.6, 3.7, 4.7);
    const c = Quat.new(2.6, 3.7, 4.8, 5.9);

    try expectEqual(Quat.eql(a, b), true);
    try expectEqual(Quat.eql(a, c), false);
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

    const a = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const b = Quat.fromVec3(1.5, Vec3.new(2.6, 3.7, 4.7));
    const c = Quat.fromVec3(1, Vec3.new(2.6, 3.7, 4.7));

    try expectEqual(a, b);
    try expectEqual(Quat.eql(a, c), false);
}

test "zalgebra.Quaternion.norm" {
    const a = Quat.fromVec3(1, Vec3.new(2, 2, 2));
    const b = Quat.fromVec3(0.2773500978946686, Vec3.new(0.5547001957893372, 0.5547001957893372, 0.5547001957893372));

    try expectEqual(a.norm(), b);
}

test "zalgebra.Quaternion.fromEulerAngles" {
    const a = Quat.fromEulerAngles(Vec3.new(10, 5, 45));
    const a_res = a.extractEulerAngles();

    const b = Quat.fromEulerAngles(Vec3.new(0, 55, 22));
    const b_res = b.toMat4().extractEulerAngles();

    try expectEqual(a_res, Vec3.new(9.999999046325684, 5.000000476837158, 45));
    try expectEqual(b_res, Vec3.new(0, 47.2450294, 22));
}

test "zalgebra.Quaternion.fromAxis" {
    const q = Quat.fromAxis(45, Vec3.up());
    const res_q = q.extractEulerAngles();

    try expectEqual(res_q, Vec3.new(0, 45.0000076, 0));
}

test "zalgebra.Quaternion.extractAxisAngle" {
    const axis = Vec3.new(44, 120, 8).norm();
    const q = Quat.fromAxis(45, axis);
    const res = q.extractAxisAngle();

    try expectApproxEqRel(axis.x(), res.axis.x(), eps_value);
    try expectApproxEqRel(axis.y(), res.axis.y(), eps_value);
    try expectApproxEqRel(axis.z(), res.axis.z(), eps_value);

    try expectApproxEqRel(res.angle, 45.0000076, eps_value);
}

test "zalgebra.Quaternion.extractEulerAngles" {
    const q = Quat.fromVec3(0.5, Vec3.new(0.5, 1, 0.3));
    const res_q = q.extractEulerAngles();

    try expectEqual(res_q, Vec3.new(129.6000213623047, 44.427005767822266, 114.4107360839843));
}

test "zalgebra.Quaternion.rotateVec" {
    const q = Quat.fromEulerAngles(Vec3.set(45));
    const m = q.toMat4();

    const v = Vec3.up();
    const v1 = q.rotateVec(v);
    const v2 = m.mulByVec4(Vec4.new(v.x(), v.y(), v.z(), 1));

    try expectApproxEqAbs(v1.x(), -1.46446585e-01, eps_value);
    try expectApproxEqAbs(v1.y(), 8.53553473e-01, eps_value);
    try expectApproxEqAbs(v1.z(), 0.5, eps_value);

    try expectApproxEqAbs(v1.x(), v2.data[0], eps_value);
    try expectApproxEqAbs(v1.y(), v2.data[1], eps_value);
    try expectApproxEqAbs(v1.z(), v2.data[2], eps_value);
}

test "zalgebra.Quaternion.lerp" {
    const a = Quat.identity();
    const b = Quat.fromAxis(180, Vec3.up());
    try expectEqual(Quat.lerp(a, b, 1), b);
    const c = Quat.lerp(a, b, 0.5);
    const d = Quat.new(0.5, 0, 0.5, 0);
    try expectApproxEqAbs(c.w, d.w, eps_value);
    try expectApproxEqAbs(c.x, d.x, eps_value);
    try expectApproxEqAbs(c.y, d.y, eps_value);
    try expectApproxEqAbs(c.z, d.z, eps_value);
}

test "zalgebra.Quaternion.slerp" {
    const a = Quat.identity();
    const b = Quat.fromAxis(180, Vec3.up());
    try expectEqual(Quat.slerp(a, b, 1), Quat.new(7.54979012e-08, 0, -1, 0));
    const c = Quat.slerp(a, b, 0.5);
    const d = Quat.new(1, 0, -1, 0).norm();
    try expectApproxEqAbs(c.w, d.w, eps_value);
    try expectApproxEqAbs(c.x, d.x, eps_value);
    try expectApproxEqAbs(c.y, d.y, eps_value);
    try expectApproxEqAbs(c.z, d.z, eps_value);
}

test "zalgebra.Quaternion.cast" {
    const a = Quat.new(3.5, 4.5, 5.5, 6.5);
    const a_f64 = Quat_f64.new(3.5, 4.5, 5.5, 6.5);
    try expectEqual(a.cast(f64), a_f64);
    try expectEqual(a_f64.cast(f32), a);
}

test "zalgebra.Quaternion.inv" {
    const out = Quat.new(7, 4, 5, 9).inv();
    const answ = Quat.new(0.0409357, -0.0233918, -0.0292398, -0.0526316);
    try expectApproxEqAbs(out.w, answ.w, eps_value);
    try expectApproxEqAbs(out.x, answ.x, eps_value);
    try expectApproxEqAbs(out.y, answ.y, eps_value);
    try expectApproxEqAbs(out.z, answ.z, eps_value);
}
