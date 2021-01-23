# zalgebra 
![CI](https://github.com/kooparse/zalgebra/workflows/CI/badge.svg)
<br/>
<br/>
Linear algebra library for games and computer graphics. 

The goal is to become as complete and useful as the Unity one. I'm currently using it for my projects and will continue to update it as new needs are coming. 

If you would like to contribute, don't hesitate! ;)

## Examples

```zig
const za = @import("zalgebra");
const vec3 = za.vec3;
const mat4 = za.mat4;

pub fn main () void {
  var projection = za.perspective(45.0, 800.0 / 600.0, 0.1, 100.0);
  var view = za.look_at(vec3.new(0., 0., -3.), vec3.new(0., 0., 0.), vec3.new(0., 1., 0.));
  var model = mat4.from_translation(vec3.new(0.2, 0.5, 0.0));
  
  var mvp = mat4.mult(projection, view.mult(model));
}
```

## Quick reference

### Aliases

Type | Description
------------ | -------------
vec2 | Two dimensional vector for `f32`
vec2_f64 | Two dimensional vector for `f64`
vec2_i32 | Two dimensional vector for `i32`
vec3 | Three dimensional vector for `f32`
vec3_f64 | Three dimensional vector for `f64`
vec3_i32 | Three dimensional vector for `i32`
vec4 | Four dimensional vector for `f32`
vec4_f64 | Four dimensional vector for `f64`
vec4_i32 | Four dimensional vector for `i32`
mat4 | 4x4 matrix for `f32`
mat4_f64 | 4x4 matrix for `f64`
quat | Quaternion for `f32`
quat_f64 | Quaternion for `f64`
perspective | Perspective function for `f32` 4x4 mat4
orthographic | Orthographic function for `f32` 4x4 mat4
look_at | LookAt function for `f32` 4x4 mat4

### Vectors

Methods | Description
------------ | -------------
new | Construct a vector from 2 to 4 components
at | Return component from given index
set | Set all components to the same given value
cast | Cast a type to another type
from_slice | Construct new vectors from slice
zero | Shorthand for `(0, 0, 0)`
one | Shorthand for `(1, 1, 1)` (only for vec3)
up | Shorthand for `(0, 1, 0)` (only for vec3)
down | Shorthand for `(0, -1, 0)` (only for vec3)
right | Shorthand for `(1, 0, 0)` (only for vec3)
left | Shorthand for `(-1, 0, 0)` (only for vec3)
forward | Shorthand for `(0, 0, 1)` (only for vec3)
back | Shorthand for `(0, 0, -1)` (only for vec3)
to_array | Return an array of same size.
get_angle | Return angle in degrees between two vectors (only for vec2 and vec3)
length | Return the magnitude of the current vector
norm | Construct a new normalized vector based on the given one
is_eq | Return `true` if two vectors are equals
sub | Construct new vector resulting from the substraction between two vectors
add | Construct new vector resulting from the addition between two vectors
scale | Construct new vector after multiplying each components by the given scalar
cross | Construct the cross product (as vector) from two vectors (only for vec3)
dot | Return the dot product between two vectors
lerp | Linear interpolation between two vectors
min | Construct vector from the min components between two vectors
max | Construct vector from the min components between two vectors

### Matrices
Note: All matrices are column-major.

Methods | Description
------------ | -------------
identity | Construct an identity matrix
from_slice | Construct new matrix from given slice of data
get_data | Return a pointer to the inner data
is_eq | Return `true` if two matrices are equals
mult_by_vec4 | Multiply a given vec4 by matrix (only for mat4)
from_translate | Construct a translation matrix
translate | Construct a translation from the given matrix according to given axis (vec3)
from_rotation | Construct a rotation matrix
from_euler_angle | Construct a rotation matrix from pitch/yaw/roll in degrees (X * Y * Z)
rotate | Construct a rotation from the given matrix according to given axis (vec3)
from_scale | Construct a scale matrix
scale | Construct a scale from the given matrix according to given axis (vec3)
extract_translation | Return a vector with proper translation
ortho_normalize | Ortho normalize the given matrix.
ectract_rotation | Return a vector with proper pitch/yaw/roll 
extract_scale | Return a vector with proper scale
perspective | Construct a perspective matrix from given fovy, aspect ratio, near/far inputs
orthographic| Construct an orthographic matrix from given left, right, bottom, top, near/far inputs
look_at | Construct a right-handed look_at matrix from given position (eye) and target
mult | Multiply two matrices
inv | Inverse the given matrix
recompose | Return matrix from given `translation`, `rotation` and `scale` components
decompose | Return components `translation`, `rotation` and `scale` from given matrix.
fmt | Display the matrix components for debug purpose

### Quaternions
Methods | Description
------------ | -------------
new| Construct new quat from given floats
zero| Construct quat as `(1, 0, 0, 0)`
from_slice | Construct new quaternion from slice
from_vec3 | Construct quaternion from vec3
is_eq | Return `true` if two quaternions are equal
norm | Normalize given quaternion
length | Return the magniture of the given quaternion
sub | Construct quaternion resulting from the subtraction of two given ones
add | Construct quaternion resulting from the addition of two given ones
mult | Construct quaternion resulting from the multiplication of two given ones
scale | Construct quaternion resulting from the multiplication of all components of the given quat
dot | Return the dot product between two quaternions
to_mat4 | Convert given quat to rotation 4x4 matrix
from_euler_angle | Construct quaternion from Euler angle
from_axis | Construct quat from angle around specified axis
extract_rotation | Get euler angles from given quaternion


### Utilities

Methods | Description
------------ | -------------
to_radians | Convert degrees to radians
to_degrees | Convert radians to degrees
lerp | Linear interpolation between two floats


## Contributing to the project

Don’t be shy about shooting any questions you may have. If you are a beginner/junior, don’t hesitate, I will always encourage you. It’s a safe place here. Also, I would be very happy to receive any kind of pull requests, you will have (at least) some feedback/guidance rapidly.

Behind screens, there are human beings, living any sort of story. So be always kind and respectful, because we all sheer to learn new things.


## Thanks
This project is inspired by [Handmade Math](https://github.com/HandmadeMath/Handmade-Math), [nalgebra](https://nalgebra.org/) and [Unity](https://unity.com/).
