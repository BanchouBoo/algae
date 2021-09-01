const std = @import("std");
const math = std.math;

fn assertIsFloat(comptime T: type) void {
    if (@typeInfo(T) != .Float and @typeInfo(T) != .ComptimeFloat)
        @compileError("Expected float type, found `" ++ @typeName(T) ++ "`!");
}

pub fn toRadians(degrees: anytype) @TypeOf(degrees) {
    comptime assertIsFloat(@TypeOf(degrees));
    return degrees * math.pi / 180.0;
}

pub fn toDegrees(radians: anytype) @TypeOf(radians) {
    comptime assertIsFloat(@TypeOf(radians));
    return radians * 180.0 / math.pi;
}

pub fn lerp(a: anytype, b: @TypeOf(a), t: @TypeOf(a)) @TypeOf(a) {
    comptime assertIsFloat(@TypeOf(a));
    return a + ((b - a) * t);
}

fn pow(a: anytype, power_of: @TypeOf(a)) @TypeOf(a) {
    const T = @TypeOf(a);
    const _pow = std.math.pow;
    const pow_floor = @floor(power_of);
    if (power_of != pow_floor) {
        return lerp(_pow(T, a, pow_floor), _pow(T, a, @ceil(power_of)), power_of - pow_floor);
    } else {
        return _pow(T, a, power_of);
    }
}

pub fn easeStart(t: anytype, power_of: @TypeOf(t)) @TypeOf(t) {
    comptime assertIsFloat(@TypeOf(t));
    return pow(t, power_of);
}

pub fn easeEnd(t: anytype, power_of: @TypeOf(t)) @TypeOf(t) {
    comptime assertIsFloat(@TypeOf(t));
    return 1 - easeStart(1 - t, power_of);
}

pub fn easeStartEnd(t: anytype, power_of: @TypeOf(t)) @TypeOf(t) {
    comptime assertIsFloat(@TypeOf(t));
    return lerp(easeStart(t, power_of), easeEnd(t, power_of), t);
}

pub fn easeStartElastic(t: anytype, amplitude: f32, period: @TypeOf(t)) @TypeOf(t) {
    var a: f32 = amplitude;

    var s: f32 = period / 4.0;
    if (a < 1.0)
        a = 1.0
    else
        s = period * std.math.asin(1.0 / a) / std.math.tau;

    const _t = t - 1.0;
    return -(a * pow(@as(f32, 2.0), 10.0 * _t) * @sin((_t - s) * std.math.tau / period));
}

pub fn easeEndElastic(t: anytype, amplitude: f32, period: @TypeOf(t)) @TypeOf(t) {
    var a: f32 = amplitude;

    var s: f32 = period / 4.0;
    if (a < 1.0)
        a = 1.0
    else
        s = period * std.math.asin(1.0 / a) / std.math.tau;

    return a * pow(@as(f32, 2.0), -10.0 * t) * @sin((t - s) * std.math.tau / period) + 1.0;
}

pub fn easeStartEndElastic(t: anytype, amplitude: f32, period: @TypeOf(t)) @TypeOf(t) {
    var a: f32 = amplitude;

    var s: f32 = period / 4.0;
    if (a < 1.0)
        a = 1.0
    else
        s = period * std.math.asin(1.0 / a) / std.math.tau;

    if (t * 2 < 1) {
        return -0.5 * (a * pow(@as(f32, 2.0), 10.0 * (t * 2.0 - 1.0)) * @sin(((t * 2.0 - 1.0) - s) * std.math.tau / period));
    } else {
        return a * pow(@as(f32, 2.0), -10.0 * (t * 2.0 - 1.0)) * @sin(((t * 2.0) - s) * std.math.tau / period) * 0.5 + 1.0;
    }
}

pub fn easeStartBack(t: anytype, overshoot: @TypeOf(t)) @TypeOf(t) {
    return t * t * ((overshoot + 1) * t - overshoot);
}

pub fn easeEndBack(t: anytype, overshoot: @TypeOf(t)) @TypeOf(t) {
    const _t = t - 1;
    return _t * _t * ((overshoot + 1) * _t + overshoot) + 1;
}

pub fn easeStartEndBack(t: anytype, overshoot: @TypeOf(t)) @TypeOf(t) {
    const o: f32 = overshoot * 1.525;

    var _t = t * 2.0;
    if (_t < 1.0) {
        return 0.5 * (_t * _t * ((o + 1) * _t - o));
    } else {
        _t -= 2.0;
        return 0.5 * (_t * _t * ((o + 1) * _t + o) + 2.0);
    }
}

// TODO: add tests
