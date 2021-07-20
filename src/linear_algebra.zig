const std = @import("std");
const float = @import("float.zig");
const type_fields = std.meta.fields;

pub const LinearAlgebraConfig = struct {
    /// should functions expect angles to be given in degrees?
    use_degrees: bool = false, // TODO: split into separate functions instead?
    /// how should integers be divided?
    integer_division_behavior: enum {
        truncate,
        floor,
        exact,
    } = .truncate,
    /// automatically ensure quaternions are normalized for slerp functions and vector rotation
    auto_normalize_quaternions: bool = true,
    /// ensure types have a guaranteed memory layout
    extern_types: bool = true,
};

pub fn WithType(comptime Number: type, comptime settings: LinearAlgebraConfig) type {
    const number_typeinfo = @typeInfo(Number);
    // zig fmt: off
    const number_is_signed = if (number_typeinfo == .Int)
                                          number_typeinfo.Int.signedness == .signed
                                      else if (number_typeinfo == .Float)
                                          true
                                      else
                                          @compileError("Number type must be an integer or float!");
    // zig fmt: on
    const number_is_float = number_typeinfo == .Float;
    const epsilon = if (number_is_float) std.math.epsilon(Number) else 0;

    const divide = struct {
        pub fn execute(a: Number, b: Number) callconv(.Inline) Number {
            return if (number_is_float)
                return a / b
            else
                comptime switch (settings.integer_division_behavior) {
                    .truncate => @divTrunc(a, b),
                    .floor => @divFloor(a, b),
                    .exact => @divExact(a, b),
                };
        }
    }.execute;

    return struct {
        fn VectorMixin(comptime Vec: type) type {
            return struct {
                const VecRhsInfo = enum {
                    array,
                    struct_field,
                };

                fn GetVecArithRhsInfo(comptime T: type) VecRhsInfo {
                    const vector_len = type_fields(Vec).len;
                    var result: VecRhsInfo = undefined;
                    switch (@typeInfo(T)) {
                        .Array => |a| {
                            if (a.len != vector_len)
                                @compileError("Array length does not match vector length!");
                            switch (@typeInfo(a.child)) {
                                .Int, .Float => {
                                    result = .array;
                                },
                                else => @compileError("Invalid type for vector arithmetic!"),
                            }
                        },
                        .Struct => |s| {
                            if (s.fields.len != vector_len)
                                @compileError("Struct field count does not match vector length!");
                            result = .struct_field;
                        },
                        else => @compileError("Invalid type for vector arithmetic!"),
                    }
                    return result;
                }

                pub usingnamespace if (number_is_signed)
                    struct {
                        /// returns vector where every field is it's negative value
                        /// does not account for overflow
                        pub fn negative(self: Vec) Vec {
                            var result: Vec = undefined;
                            inline for (type_fields(Vec)) |field| {
                                @field(result, field.name) = -@field(self, field.name);
                            }
                            return result;
                        }

                        /// returns vector where every field is it's absolute value
                        /// does not account for overflow
                        pub fn abs(self: Vec) Vec {
                            var result: Vec = self;
                            inline for (type_fields(Vec)) |field| {
                                if (number_is_float)
                                    @field(result, field.name) = @fabs(@field(self, field.name))
                                else
                                    @field(result, field.name) = if (@field(self, field.name) < 0)
                                        -@field(self, field.name)
                                    else
                                        @field(self, field.name);
                            }
                            return result;
                        }
                    }
                else
                    struct {};

                /// check equality between every field in `a` and `b`
                pub fn equals(a: Vec, b: anytype) bool {
                    inline for (type_fields(Vec)) |field, i| {
                        const field_a = @field(a, field.name);
                        const field_b = comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };

                        if (number_is_float)
                            if (!std.math.approxEqAbs(Number, field_a, field_b, epsilon))
                                return false
                            else if (field_a != field_b)
                                return false;
                    }
                    return true;
                }

                /// add the fields of `a` and `b`
                pub fn add(a: Vec, b: anytype) Vec {
                    var result: Vec = undefined;
                    inline for (type_fields(Vec)) |field, i| {
                        @field(result, field.name) = @field(a, field.name) + comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };
                    }
                    return result;
                }

                /// subtract the fields of `b` from `a`
                pub fn sub(a: Vec, b: anytype) Vec {
                    var result: Vec = undefined;
                    inline for (type_fields(Vec)) |field, i| {
                        @field(result, field.name) = @field(a, field.name) - comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };
                    }
                    return result;
                }

                /// multiply all the fields of `a` by `b`
                pub fn mul(a: Vec, b: anytype) Vec {
                    var result: Vec = undefined;
                    inline for (type_fields(Vec)) |field, i| {
                        @field(result, field.name) = @field(a, field.name) * comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };
                    }
                    return result;
                }

                /// divide all the fields of `a` by `b`
                pub fn div(a: Vec, b: anytype) Vec {
                    var result: Vec = undefined;
                    inline for (type_fields(Vec)) |field, i| {
                        const field_a = @field(a, field.name);
                        const field_b = comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };

                        @field(result, field.name) = divide(field_a, field_b);
                    }
                    return result;
                }

                /// multiply all the fields of `vec` by `scalar`
                pub fn scale(vec: Vec, scalar: Number) Vec {
                    var result = vec;
                    inline for (type_fields(Vec)) |field| {
                        @field(result, field.name) *= scalar;
                    }
                    return result;
                }

                /// divide all the fields of `vec` by `scalar`
                pub fn divScale(vec: Vec, scalar: Number) Vec {
                    var result: Vec = undefined;
                    inline for (type_fields(Vec)) |field| {
                        @field(result, field.name) = divide(@field(vec, field.name), scalar);
                    }
                    return result;
                }

                /// returns the sum of the products of each field
                pub fn dot(a: Vec, b: anytype) Number {
                    var result: Number = 0;
                    inline for (type_fields(Vec)) |field, i| {
                        result += @field(a, field.name) * comptime switch (GetVecArithRhsInfo(@TypeOf(b))) {
                            .struct_field => @field(b, type_fields(@TypeOf(b))[i].name),
                            .array => b[i],
                        };
                    }
                    return result;
                }

                /// returns the squared length of the vector
                pub fn lengthSq(self: Vec) Number {
                    return self.dot(self);
                }

                pub usingnamespace if (number_is_float)
                    struct {
                        /// returns the length of the vector
                        pub fn length(self: Vec) Number {
                            return std.math.sqrt(self.lengthSq());
                        }

                        /// returns the vector scaled to have a length of 1, or a zero vector if length is 0
                        pub fn normalized(self: Vec) Vec {
                            const len = self.length();
                            var result = Vec.zero;
                            if (len != 0)
                                result = self.divScale(len);
                            return result;
                        }

                        /// returns the angle between `a` and `b`
                        pub fn angleTo(a: Vec, b: Vec) Number {
                            var angle = std.math.acos(divide(a.dot(b), a.length() * b.length()));
                            if (settings.use_degrees)
                                return float.toDegrees(angle)
                            else
                                return angle;
                        }

                        /// returns linear interpolation between `a` and `b`
                        pub fn lerp(a: Vec, b: Vec, t: Number) Vec {
                            return a.add(b.sub(a).scale(t));
                        }
                    }
                else
                    struct {};

                fn SwizzleTypeByElements(comptime i: usize) type {
                    return switch (i) {
                        1 => Number,
                        2 => Vec2,
                        3 => Vec3,
                        4 => Vec4,
                        else => @compileError("Swizzle takes between 1 and 4 fields!"),
                    };
                }

                /// swizzle vector fields into a new vector or scalar value
                /// rgba can be used as an alternative to xyzw
                /// 0 and 1 are also valid values
                pub fn swizzle(self: Vec, comptime fields: []const u8) SwizzleTypeByElements(fields.len) {
                    const T = SwizzleTypeByElements(fields.len);
                    var result: T = undefined;

                    if (fields.len > 1) {
                        inline for (fields) |field, i| {
                            @field(result, switch (i) {
                                0 => "x",
                                1 => "y",
                                2 => "z",
                                3 => "w",
                                else => unreachable,
                            }) = switch (field) {
                                '0' => 0,
                                '1' => 1,
                                'x', 'r' => @field(self, "x"),
                                'y', 'g' => @field(self, "y"),
                                'z', 'b' => @field(self, "z"),
                                'w', 'a' => @field(self, "w"),
                                else => @compileError("Invalid swizzle field `" ++ field ++ "`!"),
                            };
                        }
                    } else if (fields.len == 1) {
                        result = @field(self, fields);
                    } else {
                        @compileError("Swizzle must contain at least one field!");
                    }

                    return result;
                }
            };
        }

        const Vec2Mixin = struct {
            pub usingnamespace VectorMixin(Vec2);

            pub const zero = Vec2.new(0, 0);
            pub const one = Vec2.new(1, 1);
            pub const unitX = Vec2.new(1, 0);
            pub const unitY = Vec2.new(0, 1);
            pub usingnamespace if (number_is_signed)
                struct {
                    pub const right = unitX;
                    pub const left = right.negative();
                    pub const up = unitY;
                    pub const down = up.negative();
                }
            else
                struct {};

            pub fn new(x: Number, y: Number) Vec2 {
                return Vec2{
                    .x = x,
                    .y = y,
                };
            }

            pub fn format(self: Vec2, comptime fmt: []const u8, options: std.fmt.FormatOptions, stream: anytype) !void {
                _ = fmt;
                _ = options;
                try stream.print("Vec2({d:.2}, {d:.2})", .{ self.x, self.y });
            }
        };

        pub const Vec2 = if (settings.extern_types)
            extern struct {
                x: Number,
                y: Number,

                usingnamespace Vec2Mixin;
            }
        else
            struct {
                x: Number,
                y: Number,

                usingnamespace Vec2Mixin;
            };

        pub const Vec3Mixin = struct {
            pub usingnamespace VectorMixin(Vec3);

            pub const zero = Vec3.new(0, 0, 0);
            pub const one = Vec3.new(1, 1, 1);
            pub const unitX = Vec3.new(1, 0, 0);
            pub const unitY = Vec3.new(0, 1, 0);
            pub const unitZ = Vec3.new(0, 0, 1);
            pub usingnamespace if (number_is_signed)
                struct {
                    pub const right = unitX;
                    pub const left = right.negative();
                    pub const up = unitY;
                    pub const down = up.negative();
                    pub const forward = unitZ;
                    pub const backward = forward.negative();
                }
            else
                struct {};

            /// calculate a vector perpendicular to `a` and `b`
            pub fn cross(a: Vec3, b: Vec3) Vec3 {
                return Vec3.new(
                    (a.y * b.z) - (a.z * b.y),
                    (a.z * b.x) - (a.x * b.z),
                    (a.x * b.y) - (a.y * b.x),
                );
            }

            pub usingnamespace if (number_is_float)
                struct {
                    /// return `self` rotated by `quat`
                    pub fn rotated(self: Vec3, quaternion: Quaternion) Vec3 {
                        return quaternion.rotateVector(self);
                    }
                }
            else
                struct {};

            pub fn new(x: Number, y: Number, z: Number) Vec3 {
                return Vec3{
                    .x = x,
                    .y = y,
                    .z = z,
                };
            }

            pub fn format(self: Vec3, comptime fmt: []const u8, options: std.fmt.FormatOptions, stream: anytype) !void {
                _ = fmt;
                _ = options;
                try stream.print("Vec3({d:.2}, {d:.2}, {d:.2})", .{ self.x, self.y, self.z });
            }
        };

        pub const Vec3 = if (settings.extern_types)
            extern struct {
                x: Number,
                y: Number,
                z: Number,

                usingnamespace Vec3Mixin;
            }
        else
            struct {
                x: Number,
                y: Number,
                z: Number,

                usingnamespace Vec3Mixin;
            };

        pub const Vec4Mixin = struct {
            pub usingnamespace VectorMixin(Vec4);

            pub const zero = Vec4.new(0, 0, 0, 0);
            pub const one = Vec4.new(1, 1, 1, 1);
            pub const unitX = Vec4.new(1, 0, 0, 0);
            pub const unitY = Vec4.new(0, 1, 0, 0);
            pub const unitZ = Vec4.new(0, 0, 1, 0);
            pub const unitW = Vec4.new(0, 0, 0, 1);
            pub usingnamespace if (number_is_signed)
                struct {
                    pub const right = unitX;
                    pub const left = right.negative();
                    pub const up = unitY;
                    pub const down = up.negative();
                    pub const forward = unitZ;
                    pub const backward = forward.negative();
                }
            else
                struct {};

            pub fn new(x: Number, y: Number, z: Number, w: Number) Vec4 {
                return Vec4{
                    .x = x,
                    .y = y,
                    .z = z,
                    .w = w,
                };
            }

            pub fn format(self: Vec4, comptime fmt: []const u8, options: std.fmt.FormatOptions, stream: anytype) !void {
                _ = fmt;
                _ = options;
                try stream.print("Vec4({d:.2}, {d:.2}, {d:.2}, {d:.2})", .{ self.x, self.y, self.z, self.w });
            }
        };

        pub const Vec4 = if (settings.extern_types)
            extern struct {
                x: Number,
                y: Number,
                z: Number,
                w: Number,

                usingnamespace Vec4Mixin;
            }
        else
            struct {
                x: Number,
                y: Number,
                z: Number,
                w: Number,

                usingnamespace Vec4Mixin;
            };

        pub fn vec2(x: Number, y: Number) Vec2 {
            return Vec2.new(x, y);
        }

        pub fn vec3(x: Number, y: Number, z: Number) Vec3 {
            return Vec3.new(x, y, z);
        }

        pub fn vec4(x: Number, y: Number, z: Number, w: Number) Vec4 {
            return Vec4.new(x, y, z, w);
        }

        usingnamespace if (number_is_float)
            struct {
                pub fn quat(s: Number, x: Number, y: Number, z: Number) Quaternion {
                    return Quaternion.new(s, x, y, z);
                }

                pub const QuaternionMixin = struct {
                    pub const zero = quat(0, 0, 0, 0);
                    pub const identity = quat(1, 0, 0, 0);

                    pub const add = VectorMixin(Quaternion).add;
                    pub const sub = VectorMixin(Quaternion).sub;
                    pub const scale = VectorMixin(Quaternion).scale;
                    pub const divScale = VectorMixin(Quaternion).divScale;
                    pub const lengthSq = VectorMixin(Quaternion).lengthSq;
                    pub const length = VectorMixin(Quaternion).length;
                    pub const dot = VectorMixin(Quaternion).dot;
                    pub const normalized = VectorMixin(Quaternion).normalized;
                    pub const angleTo = VectorMixin(Quaternion).angleTo;
                    pub const lerp = VectorMixin(Quaternion).lerp;
                    pub const negative = VectorMixin(Quaternion).negative;
                    pub const equals = VectorMixin(Quaternion).equals;

                    pub fn mul(a: Quaternion, b: Quaternion) Quaternion {
                        var vector_a = vec3(a.x, a.y, a.z);
                        var vector_b = vec3(b.x, b.y, b.z);

                        var scalar_c = (a.s * b.s) - vector_a.dot(vector_b);
                        var vector_c = vector_b.scale(a.s)
                            .add(vector_a.scale(b.s))
                            .add(vector_a.cross(vector_b));

                        return quat(scalar_c, vector_c.x, vector_c.y, vector_c.z);
                    }

                    pub fn conjugate(self: Quaternion) Quaternion {
                        return quat(self.s, -self.x, -self.y, -self.z);
                    }

                    pub fn inverse(self: Quaternion) Quaternion {
                        return self.conjugate().divScale(self.lengthSq());
                    }

                    pub fn rotateVector(self: Quaternion, vector: Vec3) Vec3 {
                        const q = if (settings.auto_normalize_quaternions) self.normalized() else self;
                        var qv = vec3(q.x, q.y, q.z);
                        return vector.add(
                            qv.cross(
                                qv.cross(vector).add(vector.scale(q.s)),
                            ).scale(2.0),
                        );
                    }

                    pub fn lerpShortestPath(a: Quaternion, b: Quaternion, t: Number) Quaternion {
                        if (a.dot(b) < 0)
                            return a.lerp(b.negative(), t)
                        else
                            return a.lerp(b, t);
                    }

                    /// returns spherical linear interpolation between `a` and `b`
                    pub fn slerp(a: Quaternion, b: Quaternion, t: Number) Quaternion {
                        const qa = if (settings.auto_normalize_quaternions) a.normalized() else a;
                        const qb = if (settings.auto_normalize_quaternions) b.normalized() else b;

                        var dp = qa.dot(qb);

                        // if quaternions are similar enough, fall back to lerp
                        // apparently this can cause shaking at the ends of long bone chains?
                        if (dp > 1 - epsilon) {
                            return qa.lerpShortestPath(qb, t);
                        }

                        const theta = std.math.acos(dp);
                        return qa
                            .scale(@sin(theta * (1 - t)))
                            .add(qb.scale(theta * t))
                            .divScale(@sin(theta));
                    }

                    /// returns spherical linear interpolation on the shortest path between `a` and `b`
                    pub fn slerpShortestPath(a: Quaternion, b: Quaternion, t: Number) Quaternion {
                        const qa = if (settings.auto_normalize_quaternions) a.normalized() else a;
                        var qb = if (settings.auto_normalize_quaternions) b.normalized() else b;

                        var dp = qa.dot(qb);
                        // take shortest path
                        if (dp < 0) {
                            qb = qb.negative();
                            dp = -dp;
                        }

                        // if quaternions are similar enough, fall back to lerp
                        // apparently this can cause shaking at the ends of long bone chains?
                        if (dp > 1 - epsilon) {
                            return qa.lerp(qb, t);
                        }

                        const theta = std.math.acos(dp);
                        return qa
                            .scale(@sin(theta * (1 - t)))
                            .add(qb.scale(theta * t))
                            .divScale(@sin(theta));
                    }

                    pub fn right(self: Quaternion) Vec3 {
                        return vec3(
                            1 - (2 * (self.y * self.y + self.z * self.z)),
                            2 * (self.x * self.y - self.s * self.z),
                            2 * (self.s * self.y + self.x * self.z),
                        );
                    }

                    pub fn up(self: Quaternion) Vec3 {
                        return vec3(
                            2 * (self.x * self.y + self.s * self.z),
                            1 - (2 * (self.x * self.x + self.z * self.z)),
                            2 * (self.y * self.z - self.s * self.x),
                        );
                    }

                    pub fn forward(self: Quaternion) Vec3 {
                        return vec3(
                            2 * (self.x * self.z - self.s * self.y),
                            2 * (self.s * self.x + self.y * self.z),
                            1 - (2 * (self.x * self.x + self.y * self.y)),
                        );
                    }

                    pub fn new(s: Number, x: Number, y: Number, z: Number) Quaternion {
                        return Quaternion{
                            .s = s,
                            .x = x,
                            .y = y,
                            .z = z,
                        };
                    }

                    pub fn fromEulerXYZ(x: Number, y: Number, z: Number) Quaternion {
                        const _x = if (settings.use_degrees) float.toRadians(x) else x;
                        const _y = if (settings.use_degrees) float.toRadians(y) else y;
                        const _z = if (settings.use_degrees) float.toRadians(z) else z;
                        const half_x = _x * 0.5;
                        const half_y = _y * 0.5;
                        const half_z = _z * 0.5;
                        const sin1 = @sin(half_x);
                        const sin2 = @sin(half_y);
                        const sin3 = @sin(half_z);
                        const cos1 = @cos(half_x);
                        const cos2 = @cos(half_y);
                        const cos3 = @cos(half_z);

                        // zig fmt: off
                        return quat(
                            -sin1 * sin2 * sin3 + cos1 * cos2 * cos3,
                             sin1 * cos2 * cos3 + sin2 * sin3 * cos1,
                            -sin1 * sin3 * cos2 + sin2 * cos1 * cos3,
                             sin1 * sin2 * cos3 + sin3 * cos1 * cos2,
                        );
                        // zig fmt: on
                    }

                    pub fn fromEuler(euler: Vec3) Quaternion {
                        return fromEulerXYZ(euler.x, euler.y, euler.z);
                    }

                    pub fn fromAxisAngle(axis: Vec3, angle: Number) Quaternion {
                        const _angle = if (settings.use_degrees) float.toRadians(angle) else angle;
                        const sin_angle = @sin(_angle * 0.5);
                        const cos_angle = @cos(_angle * 0.5);
                        const vector = axis.scale(sin_angle);
                        return quat(cos_angle, vector.x, vector.y, vector.z);
                    }

                    pub fn format(self: Quaternion, comptime fmt: []const u8, options: std.fmt.FormatOptions, stream: anytype) !void {
                        _ = fmt;
                        _ = options;
                        try stream.print("quat({d:.2}, {d:.2}i, {d:.2}j, {d:.2}k)", .{ self.s, self.x, self.y, self.z });
                    }
                };

                pub const Quaternion = if (settings.extern_types)
                    extern struct {
                        s: Number,
                        x: Number,
                        y: Number,
                        z: Number,

                        usingnamespace QuaternionMixin;
                    }
                else
                    struct {
                        s: Number,
                        x: Number,
                        y: Number,
                        z: Number,

                        usingnamespace QuaternionMixin;
                    };
            }
        else
            struct {};


        fn MatrixMixin(comptime rows: usize, comptime columns: usize) type {
            return struct {
                const Self = Matrix(rows, columns);

                pub const row_len = rows;
                pub const col_len = columns;

                pub const zero = Self{
                    .fields = [1][columns]Number{[1]Number{0} ** columns} ** rows,
                };
                pub usingnamespace if (rows == columns)
                    struct {
                        pub const identity = mat: {
                            var matrix = zero;
                            @setEvalBranchQuota(10000);
                            for (matrix.fields) |*row, y| {
                                for (row) |*col, x| {
                                    var i = x + (y * rows);
                                    if (i % (rows + 1) == 0)
                                        col.* = 1;
                                }
                            }
                            break :mat matrix;
                        };
                    }
                else
                    struct {};

                pub usingnamespace if (rows == 4 and columns == 4)
                    Mat4x4Mixin
                else
                    struct {};

                pub fn new(fields: [rows][columns]Number) Self {
                    return Self{
                        .fields = fields,
                    };
                }

                const MatrixRhsInfo = struct {
                    rows: usize = 0,
                    cols: usize = 0,
                    access: enum {
                        matrix,
                        array_1d,
                        array_2d,
                        struct_field,
                    } = .matrix,
                };

                fn GetMatRhsInfo(comptime T: type) MatrixRhsInfo {
                    comptime var result = MatrixRhsInfo{};
                    switch (@typeInfo(T)) {
                        .Array => |a| {
                            result.rows = a.len;
                            result.access = .array_1d;
                            switch (@typeInfo(a.child)) {
                                .Array => |b| {
                                    switch (@typeInfo(b.child)) {
                                        .Int, .Float => {
                                            result.cols = b.len;
                                            result.access = .array_2d;
                                        },
                                        else => @compileError("Invalid array type for matrix rhs!"),
                                    }
                                },
                                .Int, .Float => {
                                    result.cols = 1;
                                },
                                else => @compileError("Invalid array type for matrix rhs!"),
                            }
                        },
                        .Struct => |s| {
                            if (@hasDecl(T, "row_len") and @hasDecl(T, "col_len") and @hasField(T, "fields")) {
                                result.rows = T.row_len;
                                result.cols = T.col_len;
                                result.access = .matrix;
                            } else {
                                for (s.fields) |field| {
                                    switch (@typeInfo(field.field_type)) {
                                        .Int, .Float => {},
                                        else => @compileError("Invalid struct type for matrix rhs!"),
                                    }
                                }
                                result.rows = s.fields.len;
                                result.cols = 1;
                                result.access = .struct_field;
                            }
                        },
                        else => @compileError("Invalid type for matrix rhs!"),
                    }

                    return result;
                }

                /// multiply `a` with `b`
                /// number of columns in `a` must equal number of rows in `b`
                /// `b` can be a 1d array (len == rows), a 2d array, or a struct (field count == rows)
                pub fn mul(a: Self, b: anytype) @TypeOf(b) {
                    const BType = @TypeOf(b);
                    const rhs_info = comptime GetMatRhsInfo(BType);
                    if (columns != rhs_info.rows) {
                        switch (result.access) {
                            .matrix => @compileError("Number of columns in matrix a must equal number of rows in matrix b!"),
                            .array_1d => @compileError("Number of columns in matrix a must equal length of array b!"),
                            .array_2d => @compileError("Number of columns in matrix a must equal number of rows of array b!"),
                            .struct_field => @compileError("Number of columns in matrix a must equal number of fields in struct b!"),
                        }
                    }

                    var result: BType = undefined;

                    // zig fmt: off
                    {comptime var row = 0; inline while (row < rhs_info.rows) : (row += 1) {
                        {comptime var col = 0; inline while (col < rhs_info.cols) : (col += 1) {
                            var sum: Number = 0;
                            {comptime var i = 0; inline while (i < Self.col_len) : (i += 1) {
                                switch (rhs_info.access) {
                                    .matrix => sum += a.fields[row][i] * b.fields[i][col],
                                    .array_1d => sum += a.fields[row][i] * b[i],
                                    .array_2d => sum += a.fields[row][i] * b[i][col],
                                    .struct_field => sum += a.fields[row][i] * @field(b, type_fields(BType)[i].name),
                                }
                            }}
                            switch (rhs_info.access) {
                                .matrix => result.fields[row][col] = sum,
                                .array_1d => result[row] = sum,
                                .array_2d => result[row][col] = sum,
                                .struct_field => @field(result, type_fields(BType)[row].name) = sum,
                            }
                        }}
                    }}
                    // zig fmt: on
                    return result;
                }

                pub fn equals(a: Self, b: anytype) bool {
                    const rhs_info = comptime GetMatRhsInfo(@TypeOf(b));
                    if (columns != rhs_info.cols or rows != rhs_info.rows) {
                        @compileError("Number of columns and rows must be the same between `a` and `b`!");
                    }

                    // zig fmt: off
                    {comptime var row = 0; inline while (row < rhs_info.rows) : (row += 1) {
                        {comptime var col = 0; inline while (col < rhs_info.cols) : (col += 1) {
                            const field_a = a.fields[row][col];
                            const field_b = comptime switch (rhs_info.access) {
                                .matrix => b.fields[row][col],
                                .array_2d => b[row][col],
                                .array_1d => b[row],
                                .struct_field => @field(b, type_fields(@TypeOf(b))[row].name),
                            };

                            if (number_is_float)
                                if (!std.math.approxEqAbs(Number, field_a, field_b, epsilon))
                                    return false
                                else if (field_a != field_b)
                                    return false;
                        }}
                    }}
                    // zig fmt: on
                    return true;
                }

                pub fn format(self: Self, comptime fmt: []const u8, options: std.fmt.FormatOptions, stream: anytype) !void {
                    _ = fmt;
                    _ = options;
                    try stream.print("Mat{d}x{d}{{\n", .{ rows, columns });

                    // zig fmt: off
                    {comptime var row = 0; inline while (row < rows) : (row += 1) {
                        {comptime var col = 0; inline while (col < columns) : (col += 1) {
                            try stream.print("{d:>6.2}, ", .{self.fields[row][col]});
                        }}
                        try stream.writeAll("\n");
                    }}
                    // zig fmt: on

                    try stream.writeAll("}");
                }
            };
        }

        pub fn Matrix(comptime rows: usize, comptime columns: usize) type {
            if (rows == 0 or columns == 0)
                @compileError("Matrix rows and columns must both be at least 1!");

            return if (settings.extern_types)
                extern struct {
                    fields: [rows][columns]Number,

                    usingnamespace MatrixMixin(rows, columns);
                }
            else
                struct {
                    fields: [rows][columns]Number,

                    usingnamespace MatrixMixin(rows, columns);
                };
        }

        pub const Mat4x4 = Matrix(4, 4);
        const Mat4x4Mixin = struct {
            pub fn translate(self: Mat4x4, v: Vec3) Mat4x4 {
                return self.mul(createTranslation(v));
            }

            pub fn scale(self: Mat4x4, v: Vec3) Mat4x4 {
                return self.mul(createScale(v));
            }

            pub fn rotate(self: Mat4x4, q: Quaternion) Mat4x4 {
                return self.mul(createRotation(q));
            }

            pub fn createTranslationXYZ(x: Number, y: Number, z: Number) Mat4x4 {
                var result = Mat4x4.identity;
                result.fields[3][0] = x;
                result.fields[3][1] = y;
                result.fields[3][2] = z;
                return result;
            }

            pub fn createTranslation(v: Vec3) Mat4x4 {
                return createTranslationXYZ(v.x, v.y, v.z);
            }

            pub fn createScaleXYZ(x: Number, y: Number, z: Number) Mat4x4 {
                var result = Mat4x4.identity;
                result.fields[0][0] = x;
                result.fields[1][1] = y;
                result.fields[2][2] = z;
                return result;
            }

            pub fn createScale(v: Vec3) Mat4x4 {
                return createScaleXYZ(v.x, v.y, v.z);
            }

            pub fn createUniformScale(s: Number) Mat4x4 {
                return createScaleXYZ(s, s, s);
            }

            pub fn createAxisAngle(axis: Vec3, angle: Number) Mat4x4 {
                return createRotation(Quaternion.fromAxisAngle(axis, angle));
            }

            pub fn createRotation(q: Quaternion) Mat4x4 {
                const x2 = q.x * q.x;
                const y2 = q.y * q.y;
                const z2 = q.z * q.z;
                const xy = q.x * q.y;
                const xz = q.x * q.z;
                const yz = q.y * q.z;
                const sx = q.s * q.x;
                const sy = q.s * q.y;
                const sz = q.s * q.z;
                return Mat4x4.new(.{
                    .{
                        1 - (2 * (y2 + z2)),
                        2 * (xy - sz),
                        2 * (sy + xz),
                        0,
                    },
                    .{
                        2 * (xy + sz),
                        1 - (2 * (x2 + z2)),
                        2 * (yz - sx),
                        0,
                    },
                    .{
                        2 * (xz - sy),
                        2 * (sx + yz),
                        1 - (2 * (x2 + y2)),
                        0,
                    },
                    .{ 0, 0, 0, 1 },
                });
            }

            /// creates an orthographic projection matrix
            pub fn createOrthographic(left: Number, right: Number, bottom: Number, top: Number, near: Number, far: Number) Mat4x4 {
                var result = Mat4x4.identity;
                result.fields[0][0] = 2 / (right - left);
                result.fields[1][1] = 2 / (top - bottom);
                result.fields[2][2] = -2 / (far - near);
                result.fields[3][0] = -((right + left) / (right - left));
                result.fields[3][1] = -((top + bottom) / (top - bottom));
                result.fields[3][2] = -((far + near) / (far - near));
                return result;
            }

            /// creates a perspective projection matrix
            pub fn createPerspective(fov: Number, aspect: Number, near: Number, far: Number) Mat4x4 {
                const _fov = if (settings.use_degrees) float.toRadians(fov) else fov;
                std.debug.assert(@fabs(aspect) > epsilon);
                std.debug.assert(!std.math.approxEqAbs(Number, near, far, epsilon));

                const tan_half_fov = std.math.tan(_fov / 2);

                var result = Mat4x4.zero;
                result.fields[0][0] = 1.0 / (aspect * tan_half_fov);
                result.fields[1][1] = 1.0 / (tan_half_fov);
                result.fields[2][2] = far / (far - near);
                result.fields[2][3] = 1;
                result.fields[3][2] = -(far * near) / (far - near);
                return result;
            }
        };
    };
}

test "vec2 arithmetic" {
    const expect = std.testing.expect;
    const lm = WithType(f32, .{});
    const a = lm.vec2(1, 2);
    const b = lm.vec2(3, 4);

    try expect(a.add([2]f32{ 3, 4 }).equals(lm.vec2(4, 6)));
    try expect(a.add(b).equals(lm.vec2(4, 6)));
    try expect(a.sub(b).equals(lm.vec2(-2, -2)));
    try expect(a.mul(b).equals(lm.vec2(3, 8)));
    try expect(a.div(b).equals(lm.vec2(1.0 / 3.0, 0.5)));
    try expect(a.scale(2).equals(lm.vec2(2, 4)));
    try expect(a.divScale(2).equals(lm.vec2(0.5, 1)));
    try expect(a.negative().equals(lm.vec2(-1, -2)));
    try expect(a.negative().abs().equals(a));

    try expect(std.math.approxEqAbs(
        f32,
        lm.vec2(0, 1).angleTo(lm.vec2(0, -1)),
        std.math.pi,
        0.001,
    ));

    try expect(a.dot(b) == 11.0);

    try expect(a.lengthSq() == 5.0);
    try expect(b.lengthSq() == 25.0);
}

test "vec3 arithmetic" {
    const expect = std.testing.expect;
    const lm = WithType(f32, .{});
    const a = lm.vec3(1, 2, 3);
    const b = lm.vec3(4, 5, 6);

    try expect(a.add([3]f32{ 4, 5, 6 }).equals(lm.vec3(5, 7, 9)));
    try expect(a.add(b).equals(lm.vec3(5, 7, 9)));
    try expect(a.sub(b).equals(lm.vec3(-3, -3, -3)));
    try expect(a.mul(b).equals(lm.vec3(4, 10, 18)));
    try expect(a.div(b).equals(lm.vec3(0.25, 0.4, 0.5)));
    try expect(a.scale(2).equals(lm.vec3(2, 4, 6)));
    try expect(a.divScale(2).equals(lm.vec3(0.5, 1, 1.5)));
    try expect(a.negative().equals(lm.vec3(-1, -2, -3)));
    try expect(a.negative().abs().equals(a));
    try expect(a.cross(b).equals(lm.vec3(-3, 6, -3)));

    try expect(std.math.approxEqAbs(
        f32,
        lm.vec3(0, 1, 0).angleTo(lm.vec3(0, -1, 0)),
        std.math.pi,
        0.001,
    ));

    try expect(a.dot(b) == 32.0);

    try expect(a.lengthSq() == 14.0);
    try expect(b.lengthSq() == 77.0);
}

test "i32 vec3 arithmetic" {
    const expect = std.testing.expect;
    const lm = WithType(i32, .{});
    const a = lm.vec3(2, 4, 6);
    const b = lm.vec3(8, 10, 12);

    try expect(a.add([3]i32{ 8, 10, 12 }).equals(lm.vec3(10, 14, 18)));
    try expect(a.add(b).equals(lm.vec3(10, 14, 18)));
    try expect(a.sub(b).equals(lm.vec3(-6, -6, -6)));
    try expect(a.mul(b).equals(lm.vec3(16, 40, 72)));
    try expect(b.div(a).equals(lm.vec3(4, 2, 2)));
    try expect(a.scale(2).equals(lm.vec3(4, 8, 12)));
    try expect(b.divScale(2).equals(lm.vec3(4, 5, 6)));
    try expect(a.negative().equals(lm.vec3(-2, -4, -6)));
    try expect(a.negative().abs().equals(a));
    try expect(a.cross(b).equals(lm.vec3(-12, 24, -12)));

    try expect(a.dot(b) == 128);

    try expect(a.lengthSq() == 56);
    try expect(b.lengthSq() == 308);
}

test "vec4 arithmetic" {
    const expect = std.testing.expect;
    const lm = WithType(f32, .{});
    const a = lm.vec4(1, 2, 3, 4);
    const b = lm.vec4(5, 6, 7, 8);

    try expect(a.add([4]f32{ 5, 6, 7, 8 }).equals(lm.vec4(6, 8, 10, 12)));
    try expect(a.add(b).equals(lm.vec4(6, 8, 10, 12)));
    try expect(a.sub(b).equals(lm.vec4(-4, -4, -4, -4)));
    try expect(a.mul(b).equals(lm.vec4(5, 12, 21, 32)));
    try expect(a.div(b).equals(lm.vec4(0.2, 1.0 / 3.0, 3.0 / 7.0, 0.5)));
    try expect(a.scale(2).equals(lm.vec4(2, 4, 6, 8)));
    try expect(a.divScale(2).equals(lm.vec4(0.5, 1, 1.5, 2)));
    try expect(a.negative().equals(lm.vec4(-1, -2, -3, -4)));
    try expect(a.negative().abs().equals(a));

    try expect(std.math.approxEqAbs(
        f32,
        lm.vec4(0, 1, 0, 0).angleTo(lm.vec4(0, -1, 0, 0)),
        std.math.pi,
        0.001,
    ));

    try expect(a.dot(b) == 70.0);

    try expect(a.lengthSq() == 30.0);
    try expect(b.lengthSq() == 174.0);
}
