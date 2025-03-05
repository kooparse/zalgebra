const std = @import("std");
const math = std.math;
const meta = std.meta;
const mem = std.mem;
const expectEqual = std.testing.expectEqual;
const root = @import("main.zig");
const generic_vector = @import("generic_vector.zig");
const quat = @import("quaternion.zig");

const Vec3 = generic_vector.Vec3;
const Vec3_f64 = generic_vector.Vec3_f64;
const GenericVector = generic_vector.GenericVector;
const Quaternion = quat.Quaternion;
const Quat = quat.Quat;

pub const Mat4 = Mat4x4(f32);
pub const Mat4_f64 = Mat4x4(f64);
pub const perspective = Mat4.perspective;
pub const perspectiveReversedZ = Mat4.perspectiveReversedZ;
pub const orthographic = Mat4.orthographic;
pub const lookAt = Mat4.lookAt;

/// A column-major 4x4 matrix
/// Note: Column-major means accessing data like m.data[COLUMN][ROW].
pub fn Mat4x4(comptime T: type) type {
    if (@typeInfo(T) != .float) {
        @compileError("Mat4x4 not implemented for " ++ @typeName(T));
    }

    const Vector3 = GenericVector(3, T);
    const Vector4 = GenericVector(4, T);

    return extern struct {
        data: [4][4]T,

        const Self = @This();

        /// Shorthand for identity matrix.
        pub fn identity() Self {
            return .{
                .data = .{
                    .{ 1, 0, 0, 0 },
                    .{ 0, 1, 0, 0 },
                    .{ 0, 0, 1, 0 },
                    .{ 0, 0, 0, 1 },
                },
            };
        }

        /// Shorthand for matrix with all zeros.
        pub fn zero() Self {
            return Self.set(0);
        }

        /// Set all mat4 values to given value.
        pub fn set(value: T) Self {
            const data: [16]T = .{value} ** 16;
            return Self.fromSlice(&data);
        }

        /// Construct new 4x4 matrix from given slice.
        pub fn fromSlice(data: *const [16]T) Self {
            return .{
                .data = .{
                    data[0..4].*,
                    data[4..8].*,
                    data[8..12].*,
                    data[12..16].*,
                },
            };
        }

        /// Negate the given matrix.
        pub fn negate(self: Self) Self {
            var result = self;
            for (0..result.data.len) |column| {
                for (0..result.data[column].len) |row| {
                    result.data[column][row] = -result.data[column][row];
                }
            }
            return result;
        }

        /// Transpose the given matrix.
        pub fn transpose(self: Self) Self {
            var result = self;
            for (0..result.data.len) |column| {
                for (column..4) |row| {
                    std.mem.swap(T, &result.data[column][row], &result.data[row][column]);
                }
            }
            return result;
        }

        /// Return a pointer to the inner data of the matrix.
        pub fn getData(self: *const Self) *const T {
            return @ptrCast(&self.data);
        }

        /// Return true if two matrices are equals.
        pub fn eql(left: Self, right: Self) bool {
            return meta.eql(left, right);
        }

        pub fn mulByVec4(self: Self, v: Vector4) Vector4 {
            const x = (self.data[0][0] * v.x()) + (self.data[1][0] * v.y()) + (self.data[2][0] * v.z()) + (self.data[3][0] * v.w());
            const y = (self.data[0][1] * v.x()) + (self.data[1][1] * v.y()) + (self.data[2][1] * v.z()) + (self.data[3][1] * v.w());
            const z = (self.data[0][2] * v.x()) + (self.data[1][2] * v.y()) + (self.data[2][2] * v.z()) + (self.data[3][2] * v.w());
            const w = (self.data[0][3] * v.x()) + (self.data[1][3] * v.y()) + (self.data[2][3] * v.z()) + (self.data[3][3] * v.w());

            return Vector4.new(x, y, z, w);
        }

        /// Construct 4x4 translation matrix by multiplying identity matrix and
        /// given translation vector.
        pub fn fromTranslate(axis: Vector3) Self {
            var result = Self.identity();
            result.data[3][0] = axis.data[0];
            result.data[3][1] = axis.data[1];
            result.data[3][2] = axis.data[2];

            return result;
        }

        /// Make a translation between the given matrix and the given axis.
        pub fn translate(self: Self, axis: Vector3) Self {
            const trans_mat = Self.fromTranslate(axis);
            return Self.mul(trans_mat, self);
        }

        /// Get translation Vec3 from current matrix.
        pub fn extractTranslation(self: Self) Vector3 {
            return Vector3.new(self.data[3][0], self.data[3][1], self.data[3][2]);
        }

        /// Construct a 4x4 matrix from given axis and angle (in degrees).
        pub fn fromRotation(angle_in_degrees: T, axis: Vector3) Self {
            var result = Self.identity();

            const norm_axis = axis.norm();

            const sin_theta = @sin(root.toRadians(angle_in_degrees));
            const cos_theta = @cos(root.toRadians(angle_in_degrees));
            const cos_value = 1 - cos_theta;

            const x = norm_axis.x();
            const y = norm_axis.y();
            const z = norm_axis.z();

            result.data[0][0] = (x * x * cos_value) + cos_theta;
            result.data[0][1] = (x * y * cos_value) + (z * sin_theta);
            result.data[0][2] = (x * z * cos_value) - (y * sin_theta);

            result.data[1][0] = (y * x * cos_value) - (z * sin_theta);
            result.data[1][1] = (y * y * cos_value) + cos_theta;
            result.data[1][2] = (y * z * cos_value) + (x * sin_theta);

            result.data[2][0] = (z * x * cos_value) + (y * sin_theta);
            result.data[2][1] = (z * y * cos_value) - (x * sin_theta);
            result.data[2][2] = (z * z * cos_value) + cos_theta;

            return result;
        }

        pub fn rotate(self: Self, angle_in_degrees: T, axis: Vector3) Self {
            const rotation_mat = Self.fromRotation(angle_in_degrees, axis);
            return Self.mul(self, rotation_mat);
        }

        /// Construct a rotation matrix from euler angles (X * Y * Z).
        /// Order matters because matrix multiplication are NOT commutative.
        pub fn fromEulerAngles(euler_angle: Vector3) Self {
            const x = Self.fromRotation(euler_angle.x(), Vector3.right());
            const y = Self.fromRotation(euler_angle.y(), Vector3.up());
            const z = Self.fromRotation(euler_angle.z(), Vector3.forward());

            return z.mul(y.mul(x));
        }

        /// Ortho normalize given matrix.
        pub fn orthoNormalize(self: Self) Self {
            const column_1 = Vector3.new(self.data[0][0], self.data[0][1], self.data[0][2]).norm();
            const column_2 = Vector3.new(self.data[1][0], self.data[1][1], self.data[1][2]).norm();
            const column_3 = Vector3.new(self.data[2][0], self.data[2][1], self.data[2][2]).norm();

            var result = self;

            result.data[0][0] = column_1.x();
            result.data[0][1] = column_1.y();
            result.data[0][2] = column_1.z();

            result.data[1][0] = column_2.x();
            result.data[1][1] = column_2.y();
            result.data[1][2] = column_2.z();

            result.data[2][0] = column_3.x();
            result.data[2][1] = column_3.y();
            result.data[2][2] = column_3.z();

            return result;
        }

        /// Return the rotation as Euler angles in degrees.
        /// Taken from Mike Day at Insomniac Games (and `glm` as the same function).
        /// For more details: https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2012/07/euler-angles1.pdf
        pub fn extractEulerAngles(self: Self) Vector3 {
            const m = self.orthoNormalize();

            const theta_x = math.atan2(m.data[1][2], m.data[2][2]);
            const c2 = @sqrt(math.pow(T, m.data[0][0], 2) + math.pow(T, m.data[0][1], 2));
            const theta_y = math.atan2(-m.data[0][2], @sqrt(c2));
            const s1 = @sin(theta_x);
            const c1 = @cos(theta_x);
            const theta_z = math.atan2(s1 * m.data[2][0] - c1 * m.data[1][0], c1 * m.data[1][1] - s1 * m.data[2][1]);

            return Vector3.new(root.toDegrees(theta_x), root.toDegrees(theta_y), root.toDegrees(theta_z));
        }

        pub fn fromScale(axis: Vector3) Self {
            var result = Self.identity();

            result.data[0][0] = axis.x();
            result.data[1][1] = axis.y();
            result.data[2][2] = axis.z();

            return result;
        }

        pub fn scale(self: Self, axis: Vector3) Self {
            const scale_mat = Self.fromScale(axis);
            return Self.mul(scale_mat, self);
        }

        pub fn extractScale(self: Self) Vector3 {
            const scale_x = Vector3.new(self.data[0][0], self.data[0][1], self.data[0][2]);
            const scale_y = Vector3.new(self.data[1][0], self.data[1][1], self.data[1][2]);
            const scale_z = Vector3.new(self.data[2][0], self.data[2][1], self.data[2][2]);

            return Vector3.new(scale_x.length(), scale_y.length(), scale_z.length());
        }

        /// Construct a perspective 4x4 matrix.
        /// Note: Field of view is given in degrees.
        /// Also for more details https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml.
        pub fn perspective(fovy_in_degrees: T, aspect_ratio: T, z_near: T, z_far: T) Self {
            var result = Self.identity();

            const f = 1 / @tan(root.toRadians(fovy_in_degrees) * 0.5);

            result.data[0][0] = f / aspect_ratio;
            result.data[1][1] = f;
            result.data[2][2] = (z_near + z_far) / (z_near - z_far);
            result.data[2][3] = -1;
            result.data[3][2] = 2 * z_far * z_near / (z_near - z_far);
            result.data[3][3] = 0;

            return result;
        }

        /// Construct a perspective 4x4 matrix with reverse Z and infinite far plane.
        /// Note: Field of view is given in degrees.
        /// Also for more details https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml.
        /// For Reversed-Z details https://nlguillemot.wordpress.com/2016/12/07/reversed-z-in-opengl/
        pub fn perspectiveReversedZ(fovy_in_degrees: T, aspect_ratio: T, z_near: T) Self {
            var result = Self.identity();

            const f = 1 / @tan(root.toRadians(fovy_in_degrees) * 0.5);

            result.data[0][0] = f / aspect_ratio;
            result.data[1][1] = f;
            result.data[2][2] = 0;
            result.data[2][3] = -1;
            result.data[3][2] = z_near;
            result.data[3][3] = 0;

            return result;
        }

        /// Construct an orthographic 4x4 matrix.
        pub fn orthographic(left: T, right: T, bottom: T, top: T, z_near: T, z_far: T) Self {
            var result = Self.zero();

            result.data[0][0] = 2 / (right - left);
            result.data[1][1] = 2 / (top - bottom);
            result.data[2][2] = 2 / (z_near - z_far);
            result.data[3][3] = 1;

            result.data[3][0] = (left + right) / (left - right);
            result.data[3][1] = (bottom + top) / (bottom - top);
            result.data[3][2] = (z_far + z_near) / (z_near - z_far);

            return result;
        }

        /// Right-handed lookAt function.
        pub fn lookAt(eye: Vector3, target: Vector3, up: Vector3) Self {
            const f = Vector3.sub(target, eye).norm();
            const s = Vector3.cross(f, up).norm();
            const u = Vector3.cross(s, f);

            var result: Self = undefined;
            result.data[0][0] = s.x();
            result.data[0][1] = u.x();
            result.data[0][2] = -f.x();
            result.data[0][3] = 0;

            result.data[1][0] = s.y();
            result.data[1][1] = u.y();
            result.data[1][2] = -f.y();
            result.data[1][3] = 0;

            result.data[2][0] = s.z();
            result.data[2][1] = u.z();
            result.data[2][2] = -f.z();
            result.data[2][3] = 0;

            result.data[3][0] = -Vector3.dot(s, eye);
            result.data[3][1] = -Vector3.dot(u, eye);
            result.data[3][2] = Vector3.dot(f, eye);
            result.data[3][3] = 1;

            return result;
        }

        /// Matrices' multiplication.
        /// Produce a new matrix from given two matrices.
        pub fn mul(left: Self, right: Self) Self {
            var result = Self.identity();
            for (0..result.data.len) |column| {
                for (0..result.data[column].len) |row| {
                    var sum: T = 0;

                    for (0..left.data.len) |left_column| {
                        sum += left.data[left_column][row] * right.data[column][left_column];
                    }

                    result.data[column][row] = sum;
                }
            }
            return result;
        }

        fn detsubs(self: Self) [12]T {
            return .{
                self.data[0][0] * self.data[1][1] - self.data[1][0] * self.data[0][1],
                self.data[0][0] * self.data[1][2] - self.data[1][0] * self.data[0][2],
                self.data[0][0] * self.data[1][3] - self.data[1][0] * self.data[0][3],
                self.data[0][1] * self.data[1][2] - self.data[1][1] * self.data[0][2],
                self.data[0][1] * self.data[1][3] - self.data[1][1] * self.data[0][3],
                self.data[0][2] * self.data[1][3] - self.data[1][2] * self.data[0][3],

                self.data[2][0] * self.data[3][1] - self.data[3][0] * self.data[2][1],
                self.data[2][0] * self.data[3][2] - self.data[3][0] * self.data[2][2],
                self.data[2][0] * self.data[3][3] - self.data[3][0] * self.data[2][3],
                self.data[2][1] * self.data[3][2] - self.data[3][1] * self.data[2][2],
                self.data[2][1] * self.data[3][3] - self.data[3][1] * self.data[2][3],
                self.data[2][2] * self.data[3][3] - self.data[3][2] * self.data[2][3],
            };
        }

        /// Calculate determinant of the given 4x4 matrix.
        pub fn det(self: Self) T {
            const s = detsubs(self);
            return s[0] * s[11] - s[1] * s[10] + s[2] * s[9] + s[3] * s[8] - s[4] * s[7] + s[5] * s[6];
        }

        /// Construct inverse 4x4 from given matrix.
        /// Note: This is not the most efficient way to do this.
        /// TODO: Make it more efficient.
        pub fn inv(self: Self) Self {
            var inv_mat: Self = undefined;

            const s = detsubs(self);

            const determ = 1 / (s[0] * s[11] - s[1] * s[10] + s[2] * s[9] + s[3] * s[8] - s[4] * s[7] + s[5] * s[6]);

            inv_mat.data[0][0] = determ * (self.data[1][1] * s[11] - self.data[1][2] * s[10] + self.data[1][3] * s[9]);
            inv_mat.data[0][1] = determ * -(self.data[0][1] * s[11] - self.data[0][2] * s[10] + self.data[0][3] * s[9]);
            inv_mat.data[0][2] = determ * (self.data[3][1] * s[5] - self.data[3][2] * s[4] + self.data[3][3] * s[3]);
            inv_mat.data[0][3] = determ * -(self.data[2][1] * s[5] - self.data[2][2] * s[4] + self.data[2][3] * s[3]);

            inv_mat.data[1][0] = determ * -(self.data[1][0] * s[11] - self.data[1][2] * s[8] + self.data[1][3] * s[7]);
            inv_mat.data[1][1] = determ * (self.data[0][0] * s[11] - self.data[0][2] * s[8] + self.data[0][3] * s[7]);
            inv_mat.data[1][2] = determ * -(self.data[3][0] * s[5] - self.data[3][2] * s[2] + self.data[3][3] * s[1]);
            inv_mat.data[1][3] = determ * (self.data[2][0] * s[5] - self.data[2][2] * s[2] + self.data[2][3] * s[1]);

            inv_mat.data[2][0] = determ * (self.data[1][0] * s[10] - self.data[1][1] * s[8] + self.data[1][3] * s[6]);
            inv_mat.data[2][1] = determ * -(self.data[0][0] * s[10] - self.data[0][1] * s[8] + self.data[0][3] * s[6]);
            inv_mat.data[2][2] = determ * (self.data[3][0] * s[4] - self.data[3][1] * s[2] + self.data[3][3] * s[0]);
            inv_mat.data[2][3] = determ * -(self.data[2][0] * s[4] - self.data[2][1] * s[2] + self.data[2][3] * s[0]);

            inv_mat.data[3][0] = determ * -(self.data[1][0] * s[9] - self.data[1][1] * s[7] + self.data[1][2] * s[6]);
            inv_mat.data[3][1] = determ * (self.data[0][0] * s[9] - self.data[0][1] * s[7] + self.data[0][2] * s[6]);
            inv_mat.data[3][2] = determ * -(self.data[3][0] * s[3] - self.data[3][1] * s[1] + self.data[3][2] * s[0]);
            inv_mat.data[3][3] = determ * (self.data[2][0] * s[3] - self.data[2][1] * s[1] + self.data[2][2] * s[0]);

            return inv_mat;
        }

        /// Return 4x4 matrix from given all transform components; `translation`, `rotation` and `scale`.
        /// The final order is T * R * S.
        /// Note: `rotation` could be `Vec3` (Euler angles) or a `quat`.
        pub fn recompose(translation: Vector3, rotation: anytype, scalar: Vector3) Self {
            var r = switch (@TypeOf(rotation)) {
                Quaternion(T) => Quaternion(T).toMat4(rotation),
                Vector3 => Self.fromEulerAngles(rotation),
                else => @compileError("Recompose not implemented for " ++ @typeName(@TypeOf(rotation))),
            };

            r.data[0][0] *= scalar.x();
            r.data[0][1] *= scalar.x();
            r.data[0][2] *= scalar.x();
            r.data[1][0] *= scalar.y();
            r.data[1][1] *= scalar.y();
            r.data[1][2] *= scalar.y();
            r.data[2][0] *= scalar.z();
            r.data[2][1] *= scalar.z();
            r.data[2][2] *= scalar.z();

            r.data[3][0] = translation.x();
            r.data[3][1] = translation.y();
            r.data[3][2] = translation.z();

            return r;
        }

        /// Return `translation`, `rotation` and `scale` components from given matrix.
        /// For now, the rotation returned is a quaternion. If you want to get Euler angles
        /// from it, just do: `returned_quat.extractEulerAngles()`.
        /// Note: We ortho nornalize the given matrix before extracting the rotation.
        pub fn decompose(self: Self) struct { t: Vector3, r: Quaternion(T), s: Vector3 } {
            const t = self.extractTranslation();
            const s = self.extractScale();
            const r = Quaternion(T).fromMat4(self.orthoNormalize());

            return .{
                .t = t,
                .r = r,
                .s = s,
            };
        }

        /// Print the 4x4 to stderr.
        pub fn debugPrint(self: Self) void {
            const print = std.debug.print;

            print("({d}, {d}, {d}, {d})\n", .{
                self.data[0][0],
                self.data[1][0],
                self.data[2][0],
                self.data[3][0],
            });

            print("({d}, {d}, {d}, {d})\n", .{
                self.data[0][1],
                self.data[1][1],
                self.data[2][1],
                self.data[3][1],
            });

            print("({d}, {d}, {d}, {d})\n", .{
                self.data[0][2],
                self.data[1][2],
                self.data[2][2],
                self.data[3][2],
            });

            print("({d}, {d}, {d}, {d})\n", .{
                self.data[0][3],
                self.data[1][3],
                self.data[2][3],
                self.data[3][3],
            });
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @floatFromInt, @intFromFloat.
        pub fn cast(self: Self, comptime dest_type: type) Mat4x4(dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .float) {
                std.debug.panic("Error, dest type should be float.\n", .{});
            }

            var result: Mat4x4(dest_type) = undefined;
            for (0..result.data.len) |column| {
                for (0..result.data[column].len) |row| {
                    result.data[column][row] = @floatCast(self.data[column][row]);
                }
            }
            return result;
        }
    };
}

test "zalgebra.Mat4.eql" {
    const a = Mat4.identity();
    const b = Mat4.identity();
    const c = Mat4.zero();

    try expectEqual(Mat4.eql(a, b), true);
    try expectEqual(Mat4.eql(a, c), false);
}

test "zalgebra.Mat4.set" {
    const a = Mat4.set(12);
    const b = Mat4{
        .data = .{
            .{ 12, 12, 12, 12 },
            .{ 12, 12, 12, 12 },
            .{ 12, 12, 12, 12 },
            .{ 12, 12, 12, 12 },
        },
    };

    try expectEqual(a, b);
}

test "zalgebra.Mat4.negate" {
    const a = Mat4{
        .data = .{
            .{ 1, 2, 3, 4 },
            .{ 5, -6, 7, 8 },
            .{ 9, 10, 11, -12 },
            .{ 13, 14, 15, 16 },
        },
    };
    const a_negated = Mat4{
        .data = .{
            .{ -1, -2, -3, -4 },
            .{ -5, 6, -7, -8 },
            .{ -9, -10, -11, 12 },
            .{ -13, -14, -15, -16 },
        },
    };

    try expectEqual(a.negate(), a_negated);
}

test "zalgebra.Mat4.transpose" {
    const a = Mat4{
        .data = .{
            .{ 1, 2, 3, 4 },
            .{ 5, 6, 7, 8 },
            .{ 9, 10, 11, 12 },
            .{ 13, 14, 15, 16 },
        },
    };
    const b = Mat4{
        .data = .{
            .{ 1, 5, 9, 13 },
            .{ 2, 6, 10, 14 },
            .{ 3, 7, 11, 15 },
            .{ 4, 8, 12, 16 },
        },
    };

    try expectEqual(a.transpose(), b);
}

test "zalgebra.Mat4.fromSlice" {
    const data = [_]f32{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };
    const result = Mat4.fromSlice(&data);

    try expectEqual(result, Mat4.identity());
}

test "zalgebra.Mat4.fromTranslate" {
    const a = Mat4.fromTranslate(Vec3.new(2, 3, 4));

    try expectEqual(a, Mat4{
        .data = .{
            .{ 1, 0, 0, 0 },
            .{ 0, 1, 0, 0 },
            .{ 0, 0, 1, 0 },
            .{ 2, 3, 4, 1 },
        },
    });
}

test "zalgebra.Mat4.translate" {
    const a = Mat4.fromTranslate(Vec3.new(2, 3, 2));
    const result = Mat4.translate(a, Vec3.new(2, 3, 4));

    try expectEqual(result, Mat4{
        .data = .{
            .{ 1, 0, 0, 0 },
            .{ 0, 1, 0, 0 },
            .{ 0, 0, 1, 0 },
            .{ 4, 6, 6, 1 },
        },
    });
}

test "zalgebra.Mat4.fromScale" {
    const a = Mat4.fromScale(Vec3.new(2, 3, 4));

    try expectEqual(a, Mat4{
        .data = .{
            .{ 2, 0, 0, 0 },
            .{ 0, 3, 0, 0 },
            .{ 0, 0, 4, 0 },
            .{ 0, 0, 0, 1 },
        },
    });
}

test "zalgebra.Mat4.scale" {
    const a = Mat4.fromScale(Vec3.new(2, 3, 4));
    const result = Mat4.scale(a, Vec3.set(2));

    try expectEqual(result, Mat4{
        .data = .{
            .{ 4, 0, 0, 0 },
            .{ 0, 6, 0, 0 },
            .{ 0, 0, 8, 0 },
            .{ 0, 0, 0, 1 },
        },
    });
}

test "zalgebra.Mat4.det" {
    const a: Mat4 = .{
        .data = .{
            .{ 2, 0, 0, 4 },
            .{ 0, 2, 0, 0 },
            .{ 0, 0, 2, 0 },
            .{ 4, 0, 0, 2 },
        },
    };

    try expectEqual(a.det(), -48);
}

test "zalgebra.Mat4.inv" {
    const a: Mat4 = .{
        .data = .{
            .{ 2, 0, 0, 4 },
            .{ 0, 2, 0, 0 },
            .{ 0, 0, 2, 0 },
            .{ 4, 0, 0, 2 },
        },
    };

    try expectEqual(a.inv(), Mat4{
        .data = .{
            .{ -0.1666666716337204, 0, 0, 0.3333333432674408 },
            .{ 0, 0.5, 0, 0 },
            .{ 0, 0, 0.5, 0 },
            .{ 0.3333333432674408, 0, 0, -0.1666666716337204 },
        },
    });
}

test "zalgebra.Mat4.extractTranslation" {
    var a = Mat4.fromTranslate(Vec3.new(2, 3, 2));
    a = a.translate(Vec3.new(2, 3, 2));

    try expectEqual(a.extractTranslation(), Vec3.new(4, 6, 4));
}

test "zalgebra.Mat4.extractEulerAngles" {
    const a = Mat4.fromEulerAngles(Vec3.new(45, -5, 20));
    try expectEqual(a.extractEulerAngles(), Vec3.new(45.000003814697266, -4.99052524, 19.999998092651367));
}

test "zalgebra.Mat4.extractScale" {
    var a = Mat4.fromScale(Vec3.new(2, 4, 8));
    a = a.scale(Vec3.new(2, 4, 8));

    try expectEqual(a.extractScale(), Vec3.new(4, 16, 64));
}

test "zalgebra.Mat4.recompose" {
    const result = Mat4.recompose(
        Vec3.set(2),
        Vec3.new(45, 5, 0),
        Vec3.one(),
    );

    try expectEqual(result, Mat4{ .data = .{
        .{ 0.9961947202682495, 0, -0.08715573698282242, 0 },
        .{ 0.06162841245532036, 0.7071067690849304, 0.704416036605835, 0 },
        .{ 0.06162841245532036, -0.7071067690849304, 0.704416036605835, 0 },
        .{ 2, 2, 2, 1 },
    } });
}

test "zalgebra.Mat4.decompose" {
    const a = Mat4.recompose(
        Vec3.new(10, 5, 5),
        Vec3.new(45, 5, 0),
        Vec3.set(1),
    );

    const result = a.decompose();

    try expectEqual(result.t, Vec3.new(10, 5, 5));
    try expectEqual(result.s, Vec3.set(1));
    try expectEqual(result.r.extractEulerAngles(), Vec3.new(45, 5, 0.00000010712935250012379));
}

test "zalgebra.Mat4.cast" {
    const a = Mat4{ .data = .{
        .{ 0.9961947202682495, 0, -0.08715573698282242, 0 },
        .{ 0.06162841245532036, 0.7071067690849304, 0.704416036605835, 0 },
        .{ 0.06162841245532036, -0.7071067690849304, 0.704416036605835, 0 },
        .{ 2, 2, 2, 1 },
    } };
    const a_f64 = Mat4_f64{ .data = .{
        .{ 0.9961947202682495, 0, -0.08715573698282242, 0 },
        .{ 0.06162841245532036, 0.7071067690849304, 0.704416036605835, 0 },
        .{ 0.06162841245532036, -0.7071067690849304, 0.704416036605835, 0 },
        .{ 2, 2, 2, 1 },
    } };
    try expectEqual(a.cast(f64), a_f64);
    try expectEqual(a_f64.cast(f32), a);
}
