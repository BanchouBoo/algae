# Algae
Math library with a focus on game development, currently primarily focused on linear algebra but with plans to eventually include other things useful for game dev as I encounter needs for them such as fast RNG implementations, noise, 1D tranformations, etc.

Inspired by [zlm](https://github.com/ziglibs/zlm).

## Usage

### Linear Algebra
Current types available:
* Vec2
* Vec3
* Vec4
* Quaternion
* Matrices

Linear algebra is imported with a numeric type and has config options, example with default config options:

```zig
algae.lin_alg.WithType(f32, .{
    .use_degrees = true,
    .integer_division_behavior = .truncate,
    .auto_normalize_quaternions = true,
});
```

Matrices are generic on their rows and columns, and if the rows and columns are the same length it will have an `identity` constant. Additionally, there is a pre-defined 4x4 matrix type with various functions for creating translations, rotations, scaling, and perspective and orthographic projections.

Vectors can do arithmetic with other struct types and with arrays so long as the length and types match, as can matrices, example:

```zig
const vec = vec2(1, 2).add([_]f32{3, 4}); // returns vec2(4, 6)
```

### Float
Currently the only float operations available are `toRadians`, `toDegrees`, and `lerp`. They type used in the calculations will be inferred from the type of the first argument and, obviously, must be a float.
