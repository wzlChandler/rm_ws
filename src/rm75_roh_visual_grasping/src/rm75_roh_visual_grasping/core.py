"""Pure geometry and validation used by the ROS grasping node."""

import math
from collections import deque


HAND_SLIDER_LIMITS = (0.01, 0.019, 0.019, 0.019, 0.019)
HAND_JOINT_NAMES = (
    ("th_proximal_link", "th_slider_link", "th_connecting_link", "th_distal_link"),
    ("if_slider_link", "if_slider_abpart_link", "if_proximal_link", "if_distal_link", "if_connecting_link"),
    ("mf_slider_link", "mf_slider_abpart_link", "mf_proximal_link", "mf_distal_link", "mf_connecting_link"),
    ("rf_slider_link", "rf_slider_abpart_link", "rf_proximal_link", "rf_distal_link", "rf_connecting_link"),
    ("lf_slider_link", "lf_slider_abpart_link", "lf_proximal_link", "lf_distal_link", "lf_connecting_link"),
)


def _norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def surface_point_to_sphere_center(surface_point, radius):
    """Move a visible surface point away from the camera along its sight ray."""
    distance = _norm(surface_point)
    if distance <= 0.0:
        raise ValueError("surface point must not be at the camera origin")
    scale = radius / distance
    return tuple(value * (1.0 + scale) for value in surface_point)


def quaternion_rotation_matrix(quaternion):
    x, y, z, w = quaternion
    length = math.sqrt(x * x + y * y + z * z + w * w)
    if length <= 0.0:
        raise ValueError("quaternion must not be zero")
    x, y, z, w = x / length, y / length, z / length, w / length
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def link_position_for_tcp(tcp_position, quaternion, tcp_offset):
    """Return the Link7 origin that places its fixed TCP at tcp_position."""
    rotation = quaternion_rotation_matrix(quaternion)
    rotated_offset = tuple(
        sum(rotation[row][column] * tcp_offset[column] for column in range(3))
        for row in range(3)
    )
    return tuple(tcp_position[index] - rotated_offset[index] for index in range(3))


def point_before_target_along_tool_z(target, quaternion, distance):
    """Move opposite the tool +Z approach direction by distance."""
    rotation = quaternion_rotation_matrix(quaternion)
    approach = tuple(rotation[row][2] for row in range(3))
    return tuple(target[index] - approach[index] * distance for index in range(3))


def point_in_workspace(point, bounds):
    return all(bounds[index][0] <= point[index] <= bounds[index][1] for index in range(3))


def cartesian_fraction_ok(fraction, minimum_fraction):
    return 0.0 <= fraction <= 1.0 and fraction >= minimum_fraction


def poses_are_close(position_a, quaternion_a, position_b, quaternion_b,
                    position_tolerance, orientation_tolerance):
    position_error = _norm(tuple(a - b for a, b in zip(position_a, position_b)))
    norm_a = _norm(quaternion_a)
    norm_b = _norm(quaternion_b)
    if norm_a <= 0.0 or norm_b <= 0.0:
        return False
    dot = abs(sum(a * b for a, b in zip(quaternion_a, quaternion_b)) / (norm_a * norm_b))
    orientation_error = 2.0 * math.acos(min(1.0, dot))
    return position_error <= position_tolerance and orientation_error <= orientation_tolerance


def hand_state_allows_lift(write_ok, error_code, positions, minimum_position):
    return bool(
        write_ok
        and error_code == 0
        and len(positions) == 6
        and all(position >= minimum_position for position in positions[:3])
    )


def decode_register_values(data, count):
    """Decode controller register data returned as values or big-endian bytes."""
    values = [int(value) for value in data]
    if len(values) == count:
        return values
    if len(values) != count * 2 or any(value < 0 or value > 0xFF for value in values):
        raise ValueError("register response length or byte value is invalid")
    return [
        (values[index] << 8) | values[index + 1]
        for index in range(0, len(values), 2)
    ]


def hand_joint_positions(actuator_values, angle_converter):
    """Convert six ROH-A001 actuator commands into named URDF joint positions."""
    if len(actuator_values) != 6:
        raise ValueError("ROH-A001 command must contain six actuator values")
    names = []
    positions = []
    for finger_index, actuator_value in enumerate(actuator_values[:5]):
        slider_limit = HAND_SLIDER_LIMITS[finger_index]
        slider = max(0.0, min(slider_limit, float(actuator_value) * slider_limit / 1000.0))
        angles = angle_converter(finger_index, slider)
        names.extend(HAND_JOINT_NAMES[finger_index])
        if finger_index == 0:
            positions.extend((angles[0], slider, angles[1], angles[2]))
        else:
            positions.extend((slider, angles[0], angles[1], angles[2], angles[3]))
    names.append("th_root_link")
    positions.append(float(actuator_values[5]) * 1.5708 / 1000.0)
    return names, positions


class StableTargetFilter:
    def __init__(self, sample_count, distance_threshold, max_age):
        if sample_count < 1:
            raise ValueError("sample_count must be positive")
        self.sample_count = sample_count
        self.distance_threshold = distance_threshold
        self.max_age = max_age
        self.samples = deque(maxlen=sample_count)
        self.last_stamp = None

    def clear(self):
        self.samples.clear()
        self.last_stamp = None

    def add(self, stamp, point):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return False
        self.samples.append((float(stamp), tuple(point)))
        self.last_stamp = float(stamp)
        return True

    def stable_target(self, now):
        if len(self.samples) != self.sample_count:
            return None
        if now - self.samples[-1][0] > self.max_age:
            return None
        points = [sample[1] for sample in self.samples]
        mean = tuple(sum(point[axis] for point in points) / len(points) for axis in range(3))
        if any(_norm(tuple(point[axis] - mean[axis] for axis in range(3))) > self.distance_threshold
               for point in points):
            return None
        return mean
