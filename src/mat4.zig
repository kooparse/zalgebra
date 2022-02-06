const std = @import("std");
const math = std.math;
const meta = std.meta;
const mem = std.mem;
const expectEqual = std.testing.expectEqual;
const print = std.debug.print;
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
pub const orthographic = Mat4.orthographic;
pub const lookAt = Mat4.lookAt;

/// A column-major 4x4 matrix
/// Note: Column-major means accessing data like m.data[COLUMN][ROW].
pub fn Mat4x4(comptime T: type) type {
    if (@typeInfo(T) != .Float) {
        @compileError("Mat4x4 not implemented for " ++ @typeName(T));
    }

    const Vector3 = GenericVector(3, T);
    const Vector4 = GenericVector(4, T);

    return struct {
        data: [4][4]T = mem.zeroes([4][4]T),

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
            return .{
                .data = .{
                    .{ 0, 0, 0, 0 },
                    .{ 0, 0, 0, 0 },
                    .{ 0, 0, 0, 0 },
                    .{ 0, 0, 0, 0 },
                },
            };
        }

        /// Set all mat4 values to given value.
        pub fn set(value: T) Self {
            var data: [16]T = undefined;
            mem.set(T, &data, value);
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
        pub fn negate(mat: Self) Self {
            var result = mat;
            for (result.data) |_, column| {
                for (result.data[column]) |_, row| {
                    result.data[column][row] = -result.data[column][row];
                }
            }
            return result;
        }

        /// Transpose the given matrix.
        pub fn transpose(mat: Self) Self {
            var result = mat;
            for (result.data) |_, column| {
                var row: usize = column;
                while (row < 4) : (row += 1) {
                    std.mem.swap(T, &result.data[column][row], &result.data[row][column]);
                }
            }
            return result;
        }

        /// Return a pointer to the inner data of the matrix.
        pub fn getData(mat: *const Self) *const T {
            return @ptrCast(*const T, &mat.data);
        }

        /// Return true if two matrices are equals.
        pub fn eql(left: Self, right: Self) bool {
            return meta.eql(left, right);
        }

        pub fn multByVec4(mat: Self, v: Vector4) Vector4 {
            const x = (mat.data[0][0] * v.x()) + (mat.data[1][0] * v.y()) + (mat.data[2][0] * v.z()) + (mat.data[3][0] * v.w());
            const y = (mat.data[0][1] * v.x()) + (mat.data[1][1] * v.y()) + (mat.data[2][1] * v.z()) + (mat.data[3][1] * v.w());
            const z = (mat.data[0][2] * v.x()) + (mat.data[1][2] * v.y()) + (mat.data[2][2] * v.z()) + (mat.data[3][2] * v.w());
            const w = (mat.data[0][3] * v.x()) + (mat.data[1][3] * v.y()) + (mat.data[2][3] * v.z()) + (mat.data[3][3] * v.w());

            return Vector4.new(x, y, z, w);
        }

        /// Construct 4x4 translation matrix by multiplying identity matrix and
        /// given translation vector.
        pub fn fromTranslate(axis: Vector3) Self {
            var mat = Self.identity();
            mat.data[3][0] = axis.data[0];
            mat.data[3][1] = axis.data[1];
            mat.data[3][2] = axis.data[2];

            return mat;
        }

        /// Make a translation between the given matrix and the given axis.
        pub fn translate(mat: Self, axis: Vector3) Self {
            const trans_mat = Self.fromTranslate(axis);
            return Self.mult(trans_mat, mat);
        }

        /// Get translation Vec3 from current matrix.
        pub fn extractTranslation(self: Self) Vector3 {
            return Vector3.new(self.data[3][0], self.data[3][1], self.data[3][2]);
        }

        /// Construct a 4x4 matrix from given axis and angle (in degrees).
        pub fn fromRotation(angle_in_degrees: T, axis: Vector3) Self {
            var mat = Self.identity();

            const norm_axis = axis.norm();

            const sin_theta = @sin(root.toRadians(angle_in_degrees));
            const cos_theta = @cos(root.toRadians(angle_in_degrees));
            const cos_value = 1.0 - cos_theta;

            const x = norm_axis.x();
            const y = norm_axis.y();
            const z = norm_axis.z();

            mat.data[0][0] = (x * x * cos_value) + cos_theta;
            mat.data[0][1] = (x * y * cos_value) + (z * sin_theta);
            mat.data[0][2] = (x * z * cos_value) - (y * sin_theta);

            mat.data[1][0] = (y * x * cos_value) - (z * sin_theta);
            mat.data[1][1] = (y * y * cos_value) + cos_theta;
            mat.data[1][2] = (y * z * cos_value) + (x * sin_theta);

            mat.data[2][0] = (z * x * cos_value) + (y * sin_theta);
            mat.data[2][1] = (z * y * cos_value) - (x * sin_theta);
            mat.data[2][2] = (z * z * cos_value) + cos_theta;

            return mat;
        }

        pub fn rotate(mat: Self, angle_in_degrees: T, axis: Vector3) Self {
            const rotation_mat = Self.fromRotation(angle_in_degrees, axis);
            return Self.mult(mat, rotation_mat);
        }

        /// Construct a rotation matrix from euler angles (X * Y * Z).
        /// Order matters because matrix multiplication are NOT commutative.
        pub fn fromEulerAngles(euler_angle: Vector3) Self {
            const x = Self.fromRotation(euler_angle.x(), Vector3.right());
            const y = Self.fromRotation(euler_angle.y(), Vector3.up());
            const z = Self.fromRotation(euler_angle.z(), Vector3.forward());

            return z.mult(y.mult(x));
        }

        /// Ortho normalize given matrix.
        pub fn orthoNormalize(mat: Self) Self {
            const column_1 = Vector3.new(mat.data[0][0], mat.data[0][1], mat.data[0][2]).norm();
            const column_2 = Vector3.new(mat.data[1][0], mat.data[1][1], mat.data[1][2]).norm();
            const column_3 = Vector3.new(mat.data[2][0], mat.data[2][1], mat.data[2][2]).norm();

            var result = mat;

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

            const theta_x = math.atan2(T, m.data[1][2], m.data[2][2]);
            const c2 = @sqrt(math.pow(f32, m.data[0][0], 2) + math.pow(f32, m.data[0][1], 2));
            const theta_y = math.atan2(T, -m.data[0][2], @sqrt(c2));
            const s1 = @sin(theta_x);
            const c1 = @cos(theta_x);
            const theta_z = math.atan2(T, s1 * m.data[2][0] - c1 * m.data[1][0], c1 * m.data[1][1] - s1 * m.data[2][1]);

            return Vector3.new(root.toDegrees(theta_x), root.toDegrees(theta_y), root.toDegrees(theta_z));
        }

        pub fn fromScale(axis: Vector3) Self {
            var mat = Self.identity();

            mat.data[0][0] = axis.x();
            mat.data[1][1] = axis.y();
            mat.data[2][2] = axis.z();

            return mat;
        }

        pub fn scale(mat: Self, axis: Vector3) Self {
            const scale_mat = Self.fromScale(axis);
            return Self.mult(scale_mat, mat);
        }

        pub fn extractScale(mat: Self) Vector3 {
            const scale_x = Vector3.new(mat.data[0][0], mat.data[0][1], mat.data[0][2]);
            const scale_y = Vector3.new(mat.data[1][0], mat.data[1][1], mat.data[1][2]);
            const scale_z = Vector3.new(mat.data[2][0], mat.data[2][1], mat.data[2][2]);

            return Vector3.new(scale_x.length(), scale_y.length(), scale_z.length());
        }

        /// Construct a perspective 4x4 matrix.
        /// Note: Field of view is given in degrees.
        /// Also for more details https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/gluPerspective.xml.
        pub fn perspective(fovy_in_degrees: T, aspect_ratio: T, z_near: T, z_far: T) Self {
            var mat: Self = Self.identity();

            const f = 1.0 / math.tan(root.toRadians(fovy_in_degrees) * 0.5);

            mat.data[0][0] = f / aspect_ratio;
            mat.data[1][1] = f;
            mat.data[2][2] = (z_near + z_far) / (z_near - z_far);
            mat.data[2][3] = -1;
            mat.data[3][2] = 2 * z_far * z_near / (z_near - z_far);
            mat.data[3][3] = 0;

            return mat;
        }

        /// Construct an orthographic 4x4 matrix.
        pub fn orthographic(left: T, right: T, bottom: T, top: T, z_near: T, z_far: T) Self {
            var mat = Self{};

            mat.data[0][0] = 2.0 / (right - left);
            mat.data[1][1] = 2.0 / (top - bottom);
            mat.data[2][2] = 2.0 / (z_near - z_far);
            mat.data[3][3] = 1.0;

            mat.data[3][0] = (left + right) / (left - right);
            mat.data[3][1] = (bottom + top) / (bottom - top);
            mat.data[3][2] = (z_far + z_near) / (z_near - z_far);

            return mat;
        }

        /// Right-handed lookAt function.
        pub fn lookAt(eye: Vector3, target: Vector3, up: Vector3) Self {
            const f = Vector3.sub(target, eye).norm();
            const s = Vector3.cross(f, up).norm();
            const u = Vector3.cross(s, f);

            var mat: Self = undefined;
            mat.data[0][0] = s.x();
            mat.data[0][1] = u.x();
            mat.data[0][2] = -f.x();
            mat.data[0][3] = 0;

            mat.data[1][0] = s.y();
            mat.data[1][1] = u.y();
            mat.data[1][2] = -f.y();
            mat.data[1][3] = 0;

            mat.data[2][0] = s.z();
            mat.data[2][1] = u.z();
            mat.data[2][2] = -f.z();
            mat.data[2][3] = 0;

            mat.data[3][0] = -Vector3.dot(s, eye);
            mat.data[3][1] = -Vector3.dot(u, eye);
            mat.data[3][2] = Vector3.dot(f, eye);
            mat.data[3][3] = 1;

            return mat;
        }

        /// Matrices' multiplication.
        /// Produce a new matrix from given two matrices.
        pub fn mult(left: Self, right: Self) Self {
            var result = Self.identity();
            for (result.data) |_, column| {
                for (result.data[column]) |_, row| {
                    var sum: T = 0;
                    var left_column: usize = 0;

                    while (left_column < 4) : (left_column += 1) {
                        sum += left.data[left_column][row] * right.data[column][left_column];
                    }

                    result.data[column][row] = sum;
                }
            }
            return result;
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

            const determ = 1.0 / (s[0] * c[5] - s[1] * c[4] + s[2] * c[3] + s[3] * c[2] - s[4] * c[1] + s[5] * c[0]);

            inv_mat.data[0][0] =
                (mat.data[1][1] * c[5] - mat.data[1][2] * c[4] + mat.data[1][3] * c[3]) * determ;
            inv_mat.data[0][1] =
                (-mat.data[0][1] * c[5] + mat.data[0][2] * c[4] - mat.data[0][3] * c[3]) * determ;
            inv_mat.data[0][2] =
                (mat.data[3][1] * s[5] - mat.data[3][2] * s[4] + mat.data[3][3] * s[3]) * determ;
            inv_mat.data[0][3] =
                (-mat.data[2][1] * s[5] + mat.data[2][2] * s[4] - mat.data[2][3] * s[3]) * determ;

            inv_mat.data[1][0] =
                (-mat.data[1][0] * c[5] + mat.data[1][2] * c[2] - mat.data[1][3] * c[1]) * determ;
            inv_mat.data[1][1] =
                (mat.data[0][0] * c[5] - mat.data[0][2] * c[2] + mat.data[0][3] * c[1]) * determ;
            inv_mat.data[1][2] =
                (-mat.data[3][0] * s[5] + mat.data[3][2] * s[2] - mat.data[3][3] * s[1]) * determ;
            inv_mat.data[1][3] =
                (mat.data[2][0] * s[5] - mat.data[2][2] * s[2] + mat.data[2][3] * s[1]) * determ;

            inv_mat.data[2][0] =
                (mat.data[1][0] * c[4] - mat.data[1][1] * c[2] + mat.data[1][3] * c[0]) * determ;
            inv_mat.data[2][1] =
                (-mat.data[0][0] * c[4] + mat.data[0][1] * c[2] - mat.data[0][3] * c[0]) * determ;
            inv_mat.data[2][2] =
                (mat.data[3][0] * s[4] - mat.data[3][1] * s[2] + mat.data[3][3] * s[0]) * determ;
            inv_mat.data[2][3] =
                (-mat.data[2][0] * s[4] + mat.data[2][1] * s[2] - mat.data[2][3] * s[0]) * determ;

            inv_mat.data[3][0] =
                (-mat.data[1][0] * c[3] + mat.data[1][1] * c[1] - mat.data[1][2] * c[0]) * determ;
            inv_mat.data[3][1] =
                (mat.data[0][0] * c[3] - mat.data[0][1] * c[1] + mat.data[0][2] * c[0]) * determ;
            inv_mat.data[3][2] =
                (-mat.data[3][0] * s[3] + mat.data[3][1] * s[1] - mat.data[3][2] * s[0]) * determ;
            inv_mat.data[3][3] =
                (mat.data[2][0] * s[3] - mat.data[2][1] * s[1] + mat.data[2][2] * s[0]) * determ;

            return inv_mat;
        }

        /// Return 4x4 matrix from given all transform components; `translation`, `rotation` and `scale`.
        /// The final order is T * R * S.
        /// Note: `rotation` could be `Vec3` (Euler angles) or a `quat`.
        pub fn recompose(translation: Vector3, rotation: anytype, scalar: Vector3) Self {
            const t = Self.fromTranslate(translation);
            const s = Self.fromScale(scalar);

            const r = switch (@TypeOf(rotation)) {
                Quaternion(T) => Quaternion(T).toMat4(rotation),
                Vector3 => Self.fromEulerAngles(rotation),
                else => @compileError("Recompose not implemented for " ++ @typeName(@TypeOf(rotation))),
            };

            return t.mult(r.mult(s));
        }

        /// Return `translation`, `rotation` and `scale` components from given matrix.
        /// For now, the rotation returned is a quaternion. If you want to get Euler angles
        /// from it, just do: `returned_quat.extractEulerAngles()`.
        /// Note: We ortho nornalize the given matrix before extracting the rotation.
        pub fn decompose(mat: Self) struct { t: Vector3, r: Quaternion(T), s: Vector3 } {
            const t = mat.extractTranslation();
            const s = mat.extractScale();
            const r = Quat.fromMat4(mat.orthoNormalize());

            return .{
                .t = t,
                .r = r,
                .s = s,
            };
        }

        /// Print the 4x4 to stderr.
        pub fn debugPrint(self: Self) void {
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
        /// It's like builtins: @intCast, @floatCast, @intToFloat, @floatToInt.
        pub fn cast(self: Self, dest_type: anytype) Mat4x4(dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .Float) {
                std.debug.panic("Error, dest type should be float.\n", .{});
            }

            var result: Mat4x4(dest_type) = undefined;
            for (result.data) |_, column| {
                for (result.data[column]) |_, row| {
                    result.data[column][row] = @floatCast(dest_type, self.data[column][row]);
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
