# zalgebra
A linear algebra library written in Zig for real-time computer graphics.

## Documentation

### Vectors

Methods | Description
------------ | -------------
new | Construct a vector from 2-4 given components.
zeros | A vector with all components equals to zero.
length | Return the magnitude of the current vector.
norm | Construct a new normalized vector based on the given one.
is_eq | Return `true` if two vectors are equals.
sub | Construct new vector resulting from the substraction between two vectors.
add | Construct new vector resulting from the addition between two vectors.
scale | Construct new vector after multiplying each components by the given scalar.
cross | Construct the cross product (as vector) from two vectors (only for vec3).
dot | Return the dot product between two vector.

### Matrices
Note: All matrices are column-major.

Methods | Description
------------ | -------------
identity | Construct an identity matrix.
get_data | Return a pointer to the inner data.
is_eq | Return `true` if two matrices are equals.
mult_by_vec4 | Multiply a given vec4 by matrix (only for mat4).
from_translate | Construct a translation matrix from identity matrix.
translate | Construct a translation from the given matrix according to given axis (vec3).
from_rotate | Construct a rotation matrix from identity matrix.
rotate | Construct a rotation from the given matrix according to given axis (vec3).
from_scale | Construct a scale matrix from identity matrix.
scale | Construct a scale from the given matrix according to given axis (vec3).
perspective | Construct a perspective matrix from given fovy, aspect ratio, min/max inputs.
look_at | Construct a right-handed look_at matrix from given position (eye) and target.
mult | Multiply two matrices.
inv | Inverse the given matrix.
fmt | Display the matrix components for debug purpose.

### Utilities

Methods | Description
------------ | -------------
to_radians | Convert degrees to radians.
to_degrees | Convert radians to degrees.


## Current progress
- [x] vec2
- [x] vec3
- [x] vec4
- [ ] mat3
- [x] mat4 (still need tests)
- [ ] quaternion
