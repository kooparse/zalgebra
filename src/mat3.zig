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

pub const Mat3 = Mat3x3(f32);
pub const Mat3_f64 = Mat3x3(f64);

/// A column-major 3x3 matrix
/// Note: Column-major means accessing data like m.data[COLUMN][ROW].
pub fn Mat3x3(comptime T: type) type {
    if (@typeInfo(T) != .Float) {
        @compileError("Mat3x3 not implemented for " ++ @typeName(T));
    }

    const Vector3 = GenericVector(3, T);

    return extern struct {
        data: [3][3]T,

        const Self = @This();

        /// Shorthand for identity matrix.
        pub fn identity() Self {
            return .{
                .data = .{
                    .{ 1, 0, 0 },
                    .{ 0, 1, 0 },
                    .{ 0, 0, 1 },
                },
            };
        }

        /// Shorthand for matrix with all zeros.
        pub fn zero() Self {
            return Self.set(0);
        }

        /// Set all mat3 values to given value.
        pub fn set(value: T) Self {
            const data: [9]T = .{value} ** 9;
            return Self.fromSlice(&data);
        }

        /// Construct new 3x3 matrix from given slice.
        pub fn fromSlice(data: *const [9]T) Self {
            return .{
                .data = .{
                    data[0..3].*,
                    data[3..6].*,
                    data[6..9].*,
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
                for (column..result.data[column].len) |row| {
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

        pub fn mulByVec3(self: Self, v: Vector3) Vector3 {
            const x = (self.data[0][0] * v.x()) + (self.data[1][0] * v.y()) + (self.data[2][0] * v.z());
            const y = (self.data[0][1] * v.x()) + (self.data[1][1] * v.y()) + (self.data[2][1] * v.z());
            const z = (self.data[0][2] * v.x()) + (self.data[1][2] * v.y()) + (self.data[2][2] * v.z());

            return Vector3.new(x, y, z);
        }

        /// Construct a 3x3 matrix from given axis and angle (in degrees).
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

        /// Calculate determinant of the given 3x3 matrix.
        pub fn det(self: Self) T {
            var s: [3]T = undefined;
            s[0] = self.data[0][0] * (self.data[1][1] * self.data[2][2] - self.data[1][2] * self.data[2][1]);
            s[1] = self.data[0][1] * (self.data[1][0] * self.data[2][2] - self.data[1][2] * self.data[2][0]);
            s[2] = self.data[0][2] * (self.data[1][0] * self.data[2][1] - self.data[1][1] * self.data[2][0]);
            return s[0] - s[1] + s[2];
        }

        /// Construct inverse 3x3 from given matrix.
        /// Note: This is not the most efficient way to do this.
        /// TODO: Make it more efficient.
        pub fn inv(self: Self) Self {
            var inv_mat: Self = undefined;

            const determ = 1 / det(self);

            inv_mat.data[0][0] = determ * (self.data[1][1] * self.data[2][2] - self.data[1][2] * self.data[2][1]);
            inv_mat.data[0][1] = determ * -(self.data[0][1] * self.data[2][2] - self.data[0][2] * self.data[2][1]);
            inv_mat.data[0][2] = determ * (self.data[0][1] * self.data[1][2] - self.data[0][2] * self.data[1][1]);

            inv_mat.data[1][0] = determ * -(self.data[1][0] * self.data[2][2] - self.data[1][2] * self.data[2][0]);
            inv_mat.data[1][1] = determ * (self.data[0][0] * self.data[2][2] - self.data[0][2] * self.data[2][0]);
            inv_mat.data[1][2] = determ * -(self.data[0][0] * self.data[1][2] - self.data[0][2] * self.data[1][0]);

            inv_mat.data[2][0] = determ * (self.data[1][0] * self.data[2][1] - self.data[1][1] * self.data[2][0]);
            inv_mat.data[2][1] = determ * -(self.data[0][0] * self.data[2][1] - self.data[0][1] * self.data[2][0]);
            inv_mat.data[2][2] = determ * (self.data[0][0] * self.data[1][1] - self.data[0][1] * self.data[1][0]);

            return inv_mat;
        }

        /// Print the 3x3 to stderr.
        pub fn debugPrint(self: Self) void {
            const print = std.debug.print;

            print("({d}, {d}, {d})\n", .{
                self.data[0][0],
                self.data[1][0],
                self.data[2][0],
            });

            print("({d}, {d}, {d})\n", .{
                self.data[0][1],
                self.data[1][1],
                self.data[2][1],
            });

            print("({d}, {d}, {d})\n", .{
                self.data[0][2],
                self.data[1][2],
                self.data[2][2],
            });
        }

        /// Cast a type to another type.
        /// It's like builtins: @intCast, @floatCast, @floatFromInt, @intFromFloat.
        pub fn cast(self: Self, comptime dest_type: type) Mat3x3(dest_type) {
            const dest_info = @typeInfo(dest_type);

            if (dest_info != .Float) {
                std.debug.panic("Error, dest type should be float.\n", .{});
            }

            var result: Mat3x3(dest_type) = undefined;
            for (0..result.data.len) |column| {
                for (0..result.data[column].len) |row| {
                    result.data[column][row] = @floatCast(self.data[column][row]);
                }
            }
            return result;
        }
    };
}

test "zalgebra.Mat3.eql" {
    const a = Mat3.identity();
    const b = Mat3.identity();
    const c = Mat3.zero();

    try expectEqual(Mat3.eql(a, b), true);
    try expectEqual(Mat3.eql(a, c), false);
}

test "zalgebra.Mat3.set" {
    const a = Mat3.set(12);
    const b = Mat3{
        .data = .{
            .{ 12, 12, 12 },
            .{ 12, 12, 12 },
            .{ 12, 12, 12 },
        },
    };

    try expectEqual(a, b);
}

test "zalgebra.Mat3.negate" {
    const a = Mat3{
        .data = .{
            .{ 1, 2, 3 },
            .{ 5, -6, 7 },
            .{ 9, 10, 11 },
        },
    };
    const a_negated = Mat3{
        .data = .{
            .{ -1, -2, -3 },
            .{ -5, 6, -7 },
            .{ -9, -10, -11 },
        },
    };

    try expectEqual(a.negate(), a_negated);
}

test "zalgebra.Mat3.transpose" {
    const a = Mat3{
        .data = .{
            .{ 1, 2, 3 },
            .{ 5, 6, 7 },
            .{ 9, 10, 11 },
        },
    };
    const b = Mat3{
        .data = .{
            .{ 1, 5, 9 },
            .{ 2, 6, 10 },
            .{ 3, 7, 11 },
        },
    };

    try expectEqual(a.transpose(), b);
}

test "zalgebra.Mat3.fromSlice" {
    const data = [_]f32{ 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    const result = Mat3.fromSlice(&data);

    try expectEqual(result, Mat3.identity());
}

test "zalgebra.Mat3.fromScale" {
    const a = Mat3.fromScale(Vec3.new(2, 3, 4));

    try expectEqual(a, Mat3{
        .data = .{
            .{ 2, 0, 0 },
            .{ 0, 3, 0 },
            .{ 0, 0, 4 },
        },
    });
}

test "zalgebra.Mat3.scale" {
    const a = Mat3.fromScale(Vec3.new(2, 3, 4));
    const result = Mat3.scale(a, Vec3.set(2));

    try expectEqual(result, Mat3{
        .data = .{
            .{ 4, 0, 0 },
            .{ 0, 6, 0 },
            .{ 0, 0, 8 },
        },
    });
}

test "zalgebra.Mat3.det" {
    const a: Mat3 = .{
        .data = .{
            .{ 2, 0, 4 },
            .{ 0, 2, 0 },
            .{ 4, 0, 2 },
        },
    };

    try expectEqual(a.det(), -24);
}

test "zalgebra.Mat3.inv" {
    const a: Mat3 = .{
        .data = .{
            .{ 2, 0, 4 },
            .{ 0, 2, 0 },
            .{ 4, 0, 2 },
        },
    };

    try expectEqual(a.inv(), Mat3{
        .data = .{
            .{ -0.1666666716337204, 0, 0.3333333432674408 },
            .{ 0, 0.5, 0 },
            .{ 0.3333333432674408, 0, -0.1666666716337204 },
        },
    });
}

test "zalgebra.Mat3.extractEulerAngles" {
    const a = Mat3.fromEulerAngles(Vec3.new(45, -5, 20));
    try expectEqual(a.extractEulerAngles(), Vec3.new(45.000003814697266, -4.99052524, 19.999998092651367));
}

test "zalgebra.Mat3.extractScale" {
    var a = Mat3.fromScale(Vec3.new(2, 4, 8));
    a = a.scale(Vec3.new(2, 4, 8));

    try expectEqual(a.extractScale(), Vec3.new(4, 16, 64));
}

test "zalgebra.Mat3.cast" {
    const a = Mat3{ .data = .{
        .{ 0.9961947202682495, 0, -0.08715573698282242 },
        .{ 0.06162841245532036, 0.7071067690849304, 0.704416036605835 },
        .{ 0.06162841245532036, -0.7071067690849304, 0.704416036605835 },
    } };
    const a_f64 = Mat3_f64{ .data = .{
        .{ 0.9961947202682495, 0, -0.08715573698282242 },
        .{ 0.06162841245532036, 0.7071067690849304, 0.704416036605835 },
        .{ 0.06162841245532036, -0.7071067690849304, 0.704416036605835 },
    } };
    try expectEqual(a.cast(f64), a_f64);
    try expectEqual(a_f64.cast(f32), a);
}
