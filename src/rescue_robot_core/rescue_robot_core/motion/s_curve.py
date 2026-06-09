def clamp(value, min_value=-1.0, max_value=1.0):
    if value > max_value:
        return max_value

    if value < min_value:
        return min_value

    return value


def smootherstep(t):
    t = clamp(t, 0.0, 1.0)
    return (6.0 * t**5) - (15.0 * t**4) + (10.0 * t**3)


def interpolate(start_value, target_value, elapsed_time, ramp_time):
    if ramp_time <= 0.0:
        return target_value

    progress = clamp(elapsed_time / ramp_time, 0.0, 1.0)
    return start_value + (target_value - start_value) * smootherstep(progress)
