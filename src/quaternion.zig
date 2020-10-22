const std = @import("std");
const math = std.math;
const root = @import("main.zig");
usingnamespace @import("vec3.zig");
usingnamespace @import("vec4.zig");

pub const quat = Quaternion(f32);

/// A Quaternion for 3D rotations.
pub fn Quaternion(comptime T: type) type {
    if (T != f32 and T != f64) {
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

        pub fn from_vec4(components: *const Vec4(T)) Self {
            return .{
                .w = components.w,
                .x = components.x,
                .y = components.y,
                .z = components.z,
            };
        }

        pub fn from_vec3(w: T, axis: *const Vec3(T)) Self {
            return .{
                .w = w,
                .x = axis.x,
                .y = axis.y,
                .z = axis.z,
            };
        }

        pub fn is_eq(left: *const Self, right: *const Self) bool {
            return (left.w == right.w and
                left.x == right.x and
                left.y == right.y and
                left.z == right.z);
        }

        pub fn norm(self: *const Self) T {
            const l = length(self);
            return Self.new(self.w / l, self.x / l, self.y / l, self.z / l);
        }

        pub fn length(self: *const Self) T {
            return math.sqrt((self.w * self.w) +
                (self.x * self.x) +
                (self.y * self.y) +
                (self.z * self.z));
        }

        pub fn sub(left: *const Self, right: *const Self) Self {
            return Self.new(
                left.w - right.w,
                left.x - right.x,
                left.y - right.y,
                left.z - right.z,
            );
        }

        pub fn add(left: *const Self, right: *const Self) Self {
            return Self.new(
                left.w + right.w,
                left.x + right.x,
                left.y + right.y,
                left.z + right.z,
            );
        }

        pub fn mult(left: *const Self, right: *const Self) Self {
            var quat: Self = undefined;
            quat.x = (left.x * right.w) + (left.y * right.z) - (left.z * right.y) + (left.w * right.x);
            quat.y = (-left.x * right.z) + (left.y * right.w) + (left.z * right.x) + (left.w * right.y);
            quat.z = (left.x * right.y) - (left.y * right.x) + (left.z * right.w) + (left.w * right.z);
            quat.w = (-left.x * right.x) - (left.y * right.y) - (left.z * right.z) + (left.w * right.w);

            return quat;
        }

        pub fn scale(mat: *const Self, scalar: T) Self {
            var result: Self = undefined;
            result.w = mat.w * scalar;
            result.x = mat.x * scalar;
            result.y = mat.y * scalar;
            result.z = mat.z * scalar;

            return result;
        }

        pub fn dot(left: *const Self, right: *const Self) T {
            return (left.x * right.x) + (left.y * right.y) + (left.z * right.z) + (left.w * right.w);
        }

        pub fn from_euler_angles(angle: T, axis: *const Vec3(T)) Self {
            const rot_sin = math.sin(angle / 2.0);
            const quat_axis = axis.norm().scale();
            const w = math.cos(angle / 2.0);

            return Self.from_vec3(w, quat_axis);
        }
    };
}
