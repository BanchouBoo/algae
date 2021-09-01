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
    .use_degrees = false, // angles will be passed into and returned as degrees if true
    .integer_division_behavior = .truncate, // how division operations work when the underlying type is an integer
    .auto_normalize_quaternions = true, // ensure quaternions are normalized for operations that require it if true
    .extern_types = true, // Vector and Matrix types will be extern structs instead of regular structs if true
});
```

Matrices are generic on their rows and columns, and if the rows and columns are the same length it will have an `identity` constant. Additionally, there is a pre-defined 4x4 matrix type with various functions for creating translations, rotations, scaling, and perspective and orthographic projections.

Vectors can do arithmetic with other struct types and with arrays so long as the length and types match, as can matrices, example:

```zig
const vec = vec2(1, 2).add([_]f32{3, 4}); // returns vec2(4, 6)
```

### Float
Offers the following:
* `toDegrees` and `toRadians` to convert angles
* `lerp`
* A variety of easing functions
