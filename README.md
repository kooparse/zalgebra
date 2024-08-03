# zalgebra

![CI](https://github.com/kooparse/zalgebra/workflows/CI/badge.svg)
<br/>
<br/>
Linear algebra library for games and computer graphics.

The goal is to become as complete and useful as the Unity one. I'm currently using it for my projects and will continue to update it as new needs are coming.

If you would like to contribute, don't hesitate! :)

Note: **Zig 0.13.x is required.**

## Examples

Example of usage is located at [example/](example/).

```zig
const za = @import("zalgebra");
const Vec3 = za.Vec3;
const Mat4 = za.Mat4;

pub fn main () void {
  const projection = za.perspective(45.0, 800.0 / 600.0, 0.1, 100.0);
  const view = za.lookAt(Vec3.new(0.0, 0.0, -3.), Vec3.zero(), Vec3.up());
  const model = Mat4.fromTranslate(Vec3.new(0.2, 0.5, 0.0));

  const mvp = Mat4.mul(projection, view.mul(model));
  mvp.debugPrint();
}
```

## Quick reference

### Aliases

| Type         | Description                              |
| ------------ | ---------------------------------------- |
| Vec2         | Two dimensional vector for `f32`         |
| Vec2_f64     | Two dimensional vector for `f64`         |
| Vec2_i32     | Two dimensional vector for `i32`         |
| Vec2_usize   | Two dimensional vector for `usize`       |
| Vec3         | Three dimensional vector for `f32`       |
| Vec3_f64     | Three dimensional vector for `f64`       |
| Vec3_i32     | Three dimensional vector for `i32`       |
| Vec3_usize   | Three dimensional vector for `usize`     |
| Vec4         | Four dimensional vector for `f32`        |
| Vec4_f64     | Four dimensional vector for `f64`        |
| Vec4_i32     | Four dimensional vector for `i32`        |
| Vec4_usize   | Four dimensional vector for `usize`      |
| Mat3         | 3x3 matrix for `f32`                     |
| Mat3_f64     | 3x3 matrix for `f64`                     |
| Mat4         | 4x4 matrix for `f32`                     |
| Mat4_f64     | 4x4 matrix for `f64`                     |
| Quat         | Quaternion for `f32`                     |
| Quat_f64     | Quaternion for `f64`                     |
| perspective  | Perspective function for `f32` 4x4 mat4  |
| orthographic | Orthographic function for `f32` 4x4 mat4 |
| lookAt       | LookAt function for `f32` 4x4 mat4       |

### Vectors

| Methods   | Description                                                                |
| --------- | -------------------------------------------------------------------------- |
| new       | Construct a vector from 2 to 4 components                                  |
| x         | Return first component                                                     |
| y         | Return second component                                                    |
| z         | Return third component (only for vec3, vec4)                               |
| w         | Return fourth component (only for vec4)                                    |
| at        | Return component from given index                                          |
| set       | Set all components to the same given value                                 |
| negate    | Scale all components by -1                                                 |
| cast      | Cast a type to another type                                                |
| fromSlice | Construct new vectors from slice                                           |
| zero      | Shorthand for `(0, 0, 0)`                                                  |
| one       | Shorthand for `(1, 1, 1)`                                                  |
| up        | Shorthand for `(0, 1, 0)`                                                  |
| down      | Shorthand for `(0, -1, 0)`                                                 |
| right     | Shorthand for `(1, 0, 0)`                                                  |
| left      | Shorthand for `(-1, 0, 0)`                                                 |
| forward   | Shorthand for `(0, 0, 1)` (only for vec3 and vec4)                         |
| back      | Shorthand for `(0, 0, -1)` (only for vec3 and vec4)                        |
| toArray   | Return an array of same size.                                              |
| getAngle  | Return angle in degrees between two vectors (only for vec2 and vec3)       |
| rotate    | Rotate vector by angle (in degrees)                                        |
| length    | Return the magnitude of the current vector                                 |
| distance  | Return the distance between two points                                     |
| norm      | Construct a new normalized vector based on the given one                   |
| eql       | Return `true` if two vectors are equals                                    |
| sub       | Construct new vector resulting from the substraction between two vectors   |
| add       | Construct new vector resulting from the addition between two vectors       |
| mul       | Construct new vector resulting from the multiplication between two vectors |
| scale     | Construct new vector after multiplying each components by a given scalar   |
| cross     | Construct the cross product (as vector) from two vectors (only for vec3)   |
| dot       | Return the dot product between two vectors                                 |
| lerp      | Linear interpolation between two vectors                                   |
| min       | Construct vector from the min components between two vectors               |
| max       | Construct vector from the max components between two vectors               |

### Matrices

Note: All matrices are column-major.

| Methods            | Description                                                                           |
| ------------------ | ------------------------------------------------------------------------------------- |
| identity           | Construct an identity matrix                                                          |
| set                | Set all matrix values to given value                                                  |
| fromSlice          | Construct new matrix from given slice of data                                         |
| getData            | Return a pointer to the inner data                                                    |
| transpose          | Return the transpose matrix                                                           |
| negate             | Scale all components by -1                                                            |
| cast               | Cast a type to another type                                                           |
| eql                | Return `true` if two matrices are equals                                              |
| mulByVec4          | Multiply a given vec4 by matrix (only for mat4)                                       |
| fromTranslate      | Construct a translation matrix                                                        |
| translate          | Construct a translation from the given matrix according to given axis (vec3)          |
| fromRotation       | Construct a rotation matrix                                                           |
| fromEulerAngles    | Construct a rotation matrix from pitch/yaw/roll in degrees (X _ Y _ Z)                |
| rotate             | Construct a rotation from the given matrix according to given axis (vec3)             |
| fromScale          | Construct a scale matrix                                                              |
| scale              | Construct a scale from the given matrix according to given axis (vec3)                |
| extractTranslation | Return a vector with proper translation                                               |
| orthoNormalize     | Ortho normalize the given matrix.                                                     |
| extractEulerAngles | Return a vector with Euler angles in degrees (pitch/yaw/roll)                         |
| extractScale       | Return a vector with proper scale                                                     |
| perspective        | Construct a perspective matrix from given fovy, aspect ratio, near/far inputs         |
| perspectiveReversedZ | Construct a perspective matrix with reverse Z and infinite far plane.               |
| orthographic       | Construct an orthographic matrix from given left, right, bottom, top, near/far inputs |
| lookAt             | Construct a right-handed lookAt matrix from given position (eye) and target           |
| mul                | Multiply two matrices                                                                 |
| inv                | Inverse the given matrix                                                              |
| recompose          | Return mat4 matrix from given `translation`, `rotation` and `scale` components        |
| decompose          | Return components `translation`, `rotation` and `scale` from given 4x4 matrix.        |
| debugPrint         | Print the matrix data for debug purpose                                               |

### Quaternions

| Methods           | Description                                                                                    |
| ----------------- | ---------------------------------------------------------------------------------------------- |
| new               | Construct new quat from given floats                                                           |
| identity          | Construct quat as `(1, 0, 0, 0)`                                                               |
| set               | Set all components to the same given value                                                     |
| cast              | Cast a type to another type                                                                    |
| fromSlice         | Construct new quaternion from slice                                                            |
| fromVec3          | Construct quaternion from vec3                                                                 |
| eql               | Return `true` if two quaternions are equal                                                     |
| norm              | Normalize given quaternion                                                                     |
| length            | Return the magniture of the given quaternion                                                   |
| inv               | Construct inverse quaternion                                                                   |
| sub               | Construct quaternion resulting from the subtraction of two given ones                          |
| add               | Construct quaternion resulting from the addition of two given ones                             |
| mul               | Construct quaternion resulting from the multiplication of two given ones                       |
| scale             | Construct new quaternion resulting from the multiplication of all components by a given scalar |
| dot               | Return the dot product between two quaternions                                                 |
| toMat4            | Convert given quat to rotation 4x4 matrix                                                      |
| fromEulerAngles   | Construct quaternion from Euler angles                                                         |
| fromAxis          | Construct quat from angle around specified axis                                                |
| extractAxisAngles | Get the rotation angle and axis for a given quaternion                                         |
| extractRotation   | Get euler angles from given quaternion                                                         |
| rotateVec         | Rotate given vector                                                                            |

### Utilities

| Methods   | Description                             |
| --------- | --------------------------------------- |
| toRadians | Convert degrees to radians              |
| toDegrees | Convert radians to degrees              |
| lerp      | Linear interpolation between two floats |

## Contributing to the project

Don’t be shy about shooting any questions you may have. If you are a beginner/junior, don’t hesitate, I will always encourage you. It’s a safe place here. Also, I would be very happy to receive any kind of pull requests, you will have (at least) some feedback/guidance rapidly.

Behind screens, there are human beings, living any sort of story. So be always kind and respectful, because we all sheer to learn new things.

## Thanks

This project is inspired by [Handmade Math](https://github.com/HandmadeMath/Handmade-Math), [nalgebra](https://nalgebra.org/) and [Unity](https://unity.com/).
