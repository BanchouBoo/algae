const std = @import("std");
const math = std.math;

fn assertIsFloat(comptime T: type) void {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .ComptimeFloat)
        @compileError("Expected float type, found `" ++ @typeName(T) ++ "`!");
}

pub fn toRadians(degrees: anytype) @TypeOf(degrees) {
    assertIsFloat(@TypeOf(degrees));
    return degrees * math.pi / 180.0;
}

pub fn toDegrees(radians: anytype) @TypeOf(radians) {
    assertIsFloat(@TypeOf(radians));
    return radians * 180.0 / math.pi;
}

pub fn lerp(a: anytype, b: @TypeOf(a), t: @TypeOf(a)) @TypeOf(a) {
    assertIsFloat(@TypeOf(a));
    return a + ((b - a) * t);
}
