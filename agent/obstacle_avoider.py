"""
Reactive obstacle avoidance — Virtual Force Field (VFF).

Robot frame: +x forward, +y left.
LiDAR angles: angle_min + i * angle_increment (rad), counter-clockwise.

Returns (linear_x, angular_z) given a LaserScan.
"""

import math


class ObstacleAvoider:
    def __init__(
        self,
        repulse_radius: float = 1.2,   # m — rays closer than this push back
        base_speed: float = 0.35,       # m/s forward cruise
        angular_gain: float = 1.4,      # rad/s per rad of heading error
        min_front_clear: float = 0.5,   # m — stop if front sector closer (near-miss ≤ 0.3 m)
    ):
        self.repulse_radius = repulse_radius
        self.base_speed = base_speed
        self.angular_gain = angular_gain
        self.min_front_clear = min_front_clear

    def compute(self, ranges, angle_min, angle_increment) -> tuple[float, float]:
        """
        Returns (linear_x, angular_z) clamped to DerpBot limits.
        """
        fx, fy = 1.0, 0.0  # attractive force: straight ahead

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r >= self.repulse_radius:
                continue
            theta = angle_min + i * angle_increment
            magnitude = (self.repulse_radius - r) / self.repulse_radius
            # repulsive vector points away from obstacle (opposite of ray direction)
            fx -= magnitude * math.cos(theta)
            fy -= magnitude * math.sin(theta)

        heading_error = math.atan2(fy, fx)
        angular_z = self.angular_gain * heading_error

        # scale forward speed: full when heading forward, zero/reverse when turned hard
        forward_factor = math.cos(heading_error)
        linear_x = self.base_speed * max(0.0, forward_factor)

        # emergency brake: front sector (±30°) too close
        front_min = self._front_min(ranges, angle_min, angle_increment, half_angle=math.pi / 6)
        if front_min < self.min_front_clear:
            linear_x = 0.0

        # clamp to DerpBot limits
        linear_x = max(-0.5, min(0.5, linear_x))
        angular_z = max(-2.0, min(2.0, angular_z))

        return linear_x, angular_z

    @staticmethod
    def _front_min(ranges, angle_min, angle_increment, half_angle: float) -> float:
        """Min range in the front ±half_angle sector."""
        best = float("inf")
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            theta = angle_min + i * angle_increment
            # normalise to (-pi, pi]
            theta = (theta + math.pi) % (2 * math.pi) - math.pi
            if abs(theta) <= half_angle:
                best = min(best, r)
        return best
