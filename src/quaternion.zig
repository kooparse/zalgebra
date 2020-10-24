const std = @import("std");
const assert = std.debug.assert;
const math = std.math;
const root = @import("main.zig");
usingnamespace @import("vec3.zig");
usingnamespace @import("vec4.zig");

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

        pub fn from_vec4(components: Vec4(T)) Self {
            return .{
                .w = components.w,
                .x = components.x,
                .y = components.y,
                .z = components.z,
            };
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

        pub fn norm(self: Self) T {
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
            var quat: Self = undefined;
            quat.x = (left.x * right.w) + (left.y * right.z) - (left.z * right.y) + (left.w * right.x);
            quat.y = (-left.x * right.z) + (left.y * right.w) + (left.z * right.x) + (left.w * right.y);
            quat.z = (left.x * right.y) - (left.y * right.x) + (left.z * right.w) + (left.w * right.z);
            quat.w = (-left.x * right.x) - (left.y * right.y) - (left.z * right.z) + (left.w * right.w);

            return quat;
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

        /// Convert Euler angles to quaternion.
        pub fn from_euler_angles(degrees: T, axis: Vec3(T)) Self {
            const radians = root.to_radians(degrees);

            const rot_sin = math.sin(radians / 2.0);
            const quat_axis = axis.norm().scale();
            const w = math.cos(radians / 2.0);

            return Self.from_vec3(w, quat_axis);
        }
    };
}
