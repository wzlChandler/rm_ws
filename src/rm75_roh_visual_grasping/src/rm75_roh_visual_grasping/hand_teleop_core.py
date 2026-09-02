"""Hand landmark processing independent of ROS and MediaPipe."""

import math
import statistics


FINGER_CHAINS = (
    (0, 1, 2, 3, 4),
    (0, 5, 6, 7, 8),
    (0, 9, 10, 11, 12),
    (0, 13, 14, 15, 16),
    (0, 17, 18, 19, 20),
)


def _point(landmark):
    if hasattr(landmark, "x"):
        return (float(landmark.x), float(landmark.y), float(landmark.z))
    return tuple(float(value) for value in landmark[:3])


def joint_angle(first, middle, last):
    """Return the angle at ``middle`` in radians, from 0 to pi."""
    first = _point(first)
    middle = _point(middle)
    last = _point(last)
    left = tuple(first[index] - middle[index] for index in range(3))
    right = tuple(last[index] - middle[index] for index in range(3))
    left_length = math.sqrt(sum(value * value for value in left))
    right_length = math.sqrt(sum(value * value for value in right))
    if left_length == 0.0 or right_length == 0.0:
        raise ValueError("landmark vectors must have non-zero length")
    cosine = sum(left[index] * right[index] for index in range(3)) / (left_length * right_length)
    return math.acos(max(-1.0, min(1.0, cosine)))


def hand_features(landmarks):
    """Return five curl measures and one thumb-spread measure in [0, 1]."""
    if len(landmarks) != 21:
        raise ValueError("a MediaPipe hand must contain 21 landmarks")

    curls = []
    for chain in FINGER_CHAINS:
        bends = [
            math.pi - joint_angle(landmarks[chain[index]], landmarks[chain[index + 1]], landmarks[chain[index + 2]])
            for index in range(3)
        ]
        curls.append(max(0.0, min(1.0, sum(bends) / (1.5 * math.pi))))

    thumb_spread = joint_angle(landmarks[5], landmarks[0], landmarks[1]) / math.pi
    return curls + [max(0.0, min(1.0, thumb_spread))]


def feature_median(samples):
    if not samples:
        raise ValueError("at least one calibration sample is required")
    if any(len(sample) != 6 for sample in samples):
        raise ValueError("calibration samples must have six values")
    return [statistics.median(sample[index] for sample in samples) for index in range(6)]


class HandCalibration:
    def __init__(self, open_features, closed_features, minimum_range=0.02):
        if len(open_features) != 6 or len(closed_features) != 6:
            raise ValueError("calibration poses must have six values")
        self.open_features = list(open_features)
        self.closed_features = list(closed_features)
        ranges = [abs(closed - opened) for opened, closed in zip(open_features, closed_features)]
        if any(value < minimum_range for value in ranges):
            raise ValueError("calibration movement is too small; repeat with a fuller open and fist pose")

    def command(self, features):
        if len(features) != 6:
            raise ValueError("hand features must have six values")
        values = []
        for value, opened, closed in zip(features, self.open_features, self.closed_features):
            normalized = (value - opened) / (closed - opened)
            values.append(int(round(max(0.0, min(1.0, normalized)) * 1000.0)))
        return values


class CommandSmoother:
    def __init__(self, alpha=0.35):
        if not 0.0 < alpha <= 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha
        self.values = None

    def reset(self):
        self.values = None

    def update(self, target):
        if len(target) != 6:
            raise ValueError("hand command must have six values")
        target = [max(0, min(1000, int(value))) for value in target]
        if self.values is None:
            self.values = target
        else:
            self.values = [
                int(round(previous + self.alpha * (current - previous)))
                for previous, current in zip(self.values, target)
            ]
        return list(self.values)


class _OneEuroVectorSmoother:
    def __init__(self, min_cutoff=0.8, beta=0.4, derivative_cutoff=1.0):
        if min_cutoff <= 0.0 or beta < 0.0 or derivative_cutoff <= 0.0:
            raise ValueError("One Euro filter parameters must be positive")
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.derivative_cutoff = derivative_cutoff
        self.raw_values = None
        self.filtered_values = None
        self.filtered_derivatives = None
        self.last_timestamp = None

    @staticmethod
    def _alpha(cutoff, elapsed):
        rate = 2.0 * math.pi * cutoff * elapsed
        return rate / (rate + 1.0)

    def reset(self):
        self.raw_values = None
        self.filtered_values = None
        self.filtered_derivatives = None
        self.last_timestamp = None

    def update(self, target, timestamp):
        values = [float(value) for value in target]
        if self.filtered_values is None:
            self.raw_values = values
            self.filtered_values = values
            self.filtered_derivatives = [0.0] * len(values)
            self.last_timestamp = float(timestamp)
            return list(values)
        if len(values) != len(self.filtered_values):
            raise ValueError("filtered vector length cannot change")

        elapsed = max(1e-3, min(1.0, float(timestamp) - self.last_timestamp))
        derivative_alpha = self._alpha(self.derivative_cutoff, elapsed)
        output = []
        for index, value in enumerate(values):
            derivative = (value - self.raw_values[index]) / elapsed
            filtered_derivative = self.filtered_derivatives[index] + derivative_alpha * (
                derivative - self.filtered_derivatives[index]
            )
            cutoff = self.min_cutoff + self.beta * abs(filtered_derivative)
            value_alpha = self._alpha(cutoff, elapsed)
            filtered_value = self.filtered_values[index] + value_alpha * (
                value - self.filtered_values[index]
            )
            self.filtered_derivatives[index] = filtered_derivative
            self.filtered_values[index] = filtered_value
            output.append(filtered_value)
        self.raw_values = values
        self.last_timestamp = float(timestamp)
        return output


class OneEuroSmoother:
    """Adaptive low-pass filter for six normalized actuator commands."""

    def __init__(self, min_cutoff=0.8, beta=0.4, derivative_cutoff=1.0):
        self.filter = _OneEuroVectorSmoother(min_cutoff, beta, derivative_cutoff)

    def reset(self):
        self.filter.reset()

    def update(self, target, timestamp):
        if len(target) != 6:
            raise ValueError("hand command must have six values")
        normalized = [max(0.0, min(1.0, float(value) / 1000.0)) for value in target]
        filtered = self.filter.update(normalized, timestamp)
        return [int(round(value * 1000.0)) for value in filtered]


class LandmarkSmoother:
    """Adaptive temporal filter for all 21 MediaPipe hand landmarks."""

    def __init__(self, min_cutoff=1.0, beta=0.3, derivative_cutoff=1.0):
        self.filter = _OneEuroVectorSmoother(min_cutoff, beta, derivative_cutoff)

    def reset(self):
        self.filter.reset()

    def update(self, landmarks, timestamp):
        if len(landmarks) != 21:
            raise ValueError("a MediaPipe hand must contain 21 landmarks")
        flattened = [coordinate for landmark in landmarks for coordinate in _point(landmark)]
        filtered = self.filter.update(flattened, timestamp)
        return [tuple(filtered[index:index + 3]) for index in range(0, len(filtered), 3)]


def apply_hysteresis(current, previous, deadband):
    if previous is None:
        return list(current)
    return [
        value if abs(value - old_value) >= deadband else old_value
        for value, old_value in zip(current, previous)
    ]


def command_changed(current, previous, deadband):
    if previous is None:
        return True
    return any(abs(value - old_value) >= deadband for value, old_value in zip(current, previous))
