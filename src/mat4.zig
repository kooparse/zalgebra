const math = @import("std").math;
const root = @import("main.zig");
usingnamespace @import("vec4.zig");
usingnamespace @import("vec3.zig");

pub const mat4 = Mat4(f32);

/// A column-major 4x4 matrix.
/// Note: Column-major means accessing data like m.data[COLUMN][ROW].
pub fn Mat4(comptime T: type) type {
    return struct {
        data: [4][4]T,

        const Self = @This();

        pub fn identity() Self {
            return Self{
                .data = .{
                    .{ 1., 0., 0., 0. },
                    .{ 0., 1., 0., 0. },
                    .{ 0., 0., 1., 0. },
                    .{ 0., 0., 0., 1. },
                },
            };
        }

        /// Return a pointer to the inner data of the matrix.
        pub fn get_data(mat: *const Self) *const T {
            return @ptrCast(*const T, &mat.data);
        }

        pub fn copy(src: *Self) Self {
            var dest = src;
            return dest;
        }

        pub fn multByVec4(mat: Self, v: Vec4(T)) Vec4(T) {
            var result: Vec4(T) = undefined;

            result.x = (mat.data[0][0] * v.x) + (mat.data[1][0] * v.y) + (mat.data[2][0] * v.z) + (mat.data[3][0] * v.w);
            result.y = (mat.data[0][1] * v.x) + (mat.data[1][1] * v.y) + (mat.data[2][1] * v.z) + (mat.data[3][1] * v.w);
            result.z = (mat.data[0][2] * v.x) + (mat.data[1][2] * v.y) + (mat.data[2][2] * v.z) + (mat.data[3][2] * v.w);
            result.w = (mat.data[0][3] * v.x) + (mat.data[1][3] * v.y) + (mat.data[2][3] * v.z) + (mat.data[3][3] * v.w);

            return result;
        }

        /// Construct 4x4 translation matrix by multiplying identity matrix and
        /// given translation vector.
        pub fn from_translate(axis: *const Vec3(T)) Self {
            var mat = Self.identity();

            mat.data[3][0] = axis.x;
            mat.data[3][1] = axis.y;
            mat.data[3][2] = axis.z;

            return mat;
        }

        pub fn translate(mat: Self, axis: Vec3(T)) Self {
            var trans_mat = Self.from_translate(&axis);
            return Self.mult(trans_mat, mat);
        }

        /// Construct a 4x4 matrix from given axis and angle (in degrees).
        pub fn from_rotation(angle_in_degrees: T, axis: Vec3(T)) Self {
            var mat = Self.identity();

            const norm_axis = axis.norm();

            const sin_theta = math.sin(root.to_radians(angle_in_degrees));
            const cos_theta = math.cos(root.to_radians(angle_in_degrees));
            const cos_value = 1.0 - cos_theta;

            mat.data[0][0] = (norm_axis.x * norm_axis.x * cos_value) + cos_theta;
            mat.data[0][1] = (norm_axis.x * norm_axis.y * cos_value) + (norm_axis.z * sin_theta);
            mat.data[0][2] = (norm_axis.x * norm_axis.z * cos_value) - (norm_axis.y * sin_theta);

            mat.data[1][0] = (norm_axis.y * norm_axis.x * cos_value) - (norm_axis.z * sin_theta);
            mat.data[1][1] = (norm_axis.y * norm_axis.y * cos_value) + cos_theta;
            mat.data[1][2] = (norm_axis.y * norm_axis.z * cos_value) + (norm_axis.x * sin_theta);

            mat.data[2][0] = (norm_axis.z * norm_axis.x * cos_value) + (norm_axis.y * sin_theta);
            mat.data[2][1] = (norm_axis.z * norm_axis.y * cos_value) - (norm_axis.x * sin_theta);
            mat.data[2][2] = (norm_axis.z * norm_axis.z * cos_value) + cos_theta;

            return mat;
        }

        pub fn rotate(mat: Self, angle_in_degrees: T, axis: Vec3(T)) Self {
            var rotation_mat = Self.from_rotation(angle_in_degrees, axis);
            return Self.mult(rotation_mat, mat);
        }

        pub fn from_scale(axis: Vec3(T)) Self {
            var mat = Self.identity();

            mat.data[0][0] = axis.x;
            mat.data[1][1] = axis.y;
            mat.data[2][2] = axis.z;

            return mat;
        }

        pub fn scale(mat: Self, axis: Vec3(T)) Self {
            var scale_mat = Self.from_scale(axis);
            return Self.mult(scale_mat, mat);
        }

        /// Construct a perspective 4x4 matrix.
        /// Note: Field of view is given in degrees.
        /// Also for more details https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml.
        pub fn perspective(fovy_in_degrees: T, aspect_ratio: T, near: T, far: T) Self {
            var mat: Self = undefined;

            const f = 1.0 / math.tan(fovy_in_degrees * 0.5);

            mat.data[0][0] = f / aspect_ratio;
            mat.data[1][1] = f;
            mat.data[2][2] = (near + far) / (near - far);
            mat.data[2][3] = -1;
            mat.data[3][2] = 2 * far * near / (near - far);

            return mat;
        }

        /// Right-handed look_at function.
        pub fn look_at(eye: Vec3(T), target: Vec3(T), up: Vec3(T)) Self {
            const f = Vec3(T).norm(Vec3(T).sub(target, eye));
            const s = Vec3(T).norm(Vec3(T).cross(f, up));
            const u = Vec3(T).cross(s, f);

            var mat: Self = undefined;
            mat.data[0][0] = s.x;
            mat.data[0][1] = u.x;
            mat.data[0][2] = -f.x;
            mat.data[0][3] = 0.0;

            mat.data[1][0] = s.y;
            mat.data[1][1] = u.y;
            mat.data[1][2] = -f.y;
            mat.data[1][3] = 0.0;

            mat.data[2][0] = s.z;
            mat.data[2][1] = u.z;
            mat.data[2][2] = -f.z;
            mat.data[2][3] = 0.0;

            mat.data[3][0] = -Vec3(T).dot(s, eye);
            mat.data[3][1] = -Vec3(T).dot(u, eye);
            mat.data[3][2] = Vec3(T).dot(f, eye);
            mat.data[3][3] = 1.0;

            return mat;
        }

        pub fn get_translation(self: *const Self) Vec3(T) {
            return Vec3(T).new(self.data[3][0], self.data[3][1], self.data[3][2]);
        }

        pub fn mult(left: Self, right: Self) Self {
            var mat = Self.identity();
            var columns: usize = 0;

            while (columns < 4) : (columns += 1) {
                var rows: usize = 0;
                while (rows < 4) : (rows += 1) {
                    var sum: T = 0.0;
                    var current_mat: usize = 0;

                    while (current_mat < 4) : (current_mat += 1) {
                        sum += left.data[current_mat][rows] * right.data[columns][current_mat];
                    }

                    mat.data[columns][rows] = sum;
                }
            }

            return mat;
        }

        /// Construct inverse 4x4 from given matrix.
        /// Note: This is not the most efficient way to do this.
        /// TODO: Make it more efficient.
        pub fn inv(mat: Self) Self {
            var inv_mat: Self = undefined;

            var s: [6]T = undefined;
            var c: [6]T = undefined;

            s[0] = mat.data[0][0] * mat.data[1][1] - mat.data[1][0] * mat.data[0][1];
            s[1] = mat.data[0][0] * mat.data[1][2] - mat.data[1][0] * mat.data[0][2];
            s[2] = mat.data[0][0] * mat.data[1][3] - mat.data[1][0] * mat.data[0][3];
            s[3] = mat.data[0][1] * mat.data[1][2] - mat.data[1][1] * mat.data[0][2];
            s[4] = mat.data[0][1] * mat.data[1][3] - mat.data[1][1] * mat.data[0][3];
            s[5] = mat.data[0][2] * mat.data[1][3] - mat.data[1][2] * mat.data[0][3];

            c[0] = mat.data[2][0] * mat.data[3][1] - mat.data[3][0] * mat.data[2][1];
            c[1] = mat.data[2][0] * mat.data[3][2] - mat.data[3][0] * mat.data[2][2];
            c[2] = mat.data[2][0] * mat.data[3][3] - mat.data[3][0] * mat.data[2][3];
            c[3] = mat.data[2][1] * mat.data[3][2] - mat.data[3][1] * mat.data[2][2];
            c[4] = mat.data[2][1] * mat.data[3][3] - mat.data[3][1] * mat.data[2][3];
            c[5] = mat.data[2][2] * mat.data[3][3] - mat.data[3][2] * mat.data[2][3];

            var idet: T = 1.0 / (s[0] * c[5] - s[1] * c[4] + s[2] * c[3] + s[3] * c[2] - s[4] * c[1] + s[5] * c[0]);

            inv_mat.data[0][0] =
                (mat.data[1][1] * c[5] - mat.data[1][2] * c[4] + mat.data[1][3] * c[3]) * idet;
            inv_mat.data[0][1] =
                (-mat.data[0][1] * c[5] + mat.data[0][2] * c[4] - mat.data[0][3] * c[3]) * idet;
            inv_mat.data[0][2] =
                (mat.data[3][1] * s[5] - mat.data[3][2] * s[4] + mat.data[3][3] * s[3]) * idet;
            inv_mat.data[0][3] =
                (-mat.data[2][1] * s[5] + mat.data[2][2] * s[4] - mat.data[2][3] * s[3]) * idet;

            inv_mat.data[1][0] =
                (-mat.data[1][0] * c[5] + mat.data[1][2] * c[2] - mat.data[1][3] * c[1]) * idet;
            inv_mat.data[1][1] =
                (mat.data[0][0] * c[5] - mat.data[0][2] * c[2] + mat.data[0][3] * c[1]) * idet;
            inv_mat.data[1][2] =
                (-mat.data[3][0] * s[5] + mat.data[3][2] * s[2] - mat.data[3][3] * s[1]) * idet;
            inv_mat.data[1][3] =
                (mat.data[2][0] * s[5] - mat.data[2][2] * s[2] + mat.data[2][3] * s[1]) * idet;

            inv_mat.data[2][0] =
                (mat.data[1][0] * c[4] - mat.data[1][1] * c[2] + mat.data[1][3] * c[0]) * idet;
            inv_mat.data[2][1] =
                (-mat.data[0][0] * c[4] + mat.data[0][1] * c[2] - mat.data[0][3] * c[0]) * idet;
            inv_mat.data[2][2] =
                (mat.data[3][0] * s[4] - mat.data[3][1] * s[2] + mat.data[3][3] * s[0]) * idet;
            inv_mat.data[2][3] =
                (-mat.data[2][0] * s[4] + mat.data[2][1] * s[2] - mat.data[2][3] * s[0]) * idet;

            inv_mat.data[3][0] =
                (-mat.data[1][0] * c[3] + mat.data[1][1] * c[1] - mat.data[1][2] * c[0]) * idet;
            inv_mat.data[3][1] =
                (mat.data[0][0] * c[3] - mat.data[0][1] * c[1] + mat.data[0][2] * c[0]) * idet;
            inv_mat.data[3][2] =
                (-mat.data[3][0] * s[3] + mat.data[3][1] * s[1] - mat.data[3][2] * s[0]) * idet;
            inv_mat.data[3][3] =
                (mat.data[2][0] * s[3] - mat.data[2][1] * s[1] + mat.data[2][2] * s[0]) * idet;

            return inv_mat;
        }

        /// Display the 4x4 matrix.
        pub fn fmt(self: Self) void {
            warn("\n", .{});
            warn("({}, {}, {}, {})\n", .{ self.data[0][0], self.data[1][0], self.data[2][0], self.data[3][0] });
            warn("({}, {}, {}, {})\n", .{ self.data[0][1], self.data[1][1], self.data[2][1], self.data[3][1] });
            warn("({}, {}, {}, {})\n", .{ self.data[0][2], self.data[1][2], self.data[2][2], self.data[3][2] });
            warn("({}, {}, {}, {})\n", .{ self.data[0][3], self.data[1][3], self.data[2][3], self.data[3][3] });
            warn("\n", .{});
        }
    };
}
