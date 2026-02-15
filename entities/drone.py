import math
import pygame

from bendyruler import AP_OABendyRuler, Vector2
from config import DRONE_COLOR, wrap_180


class Drone:
    """Leader/defender drone model with yaw-limited kinematic movement."""

    def __init__(self, x, y, use_oa=True, color=DRONE_COLOR):
        # Pose and rendering
        self.pos = Vector2(x, y)
        self.vel = Vector2(0, 0)
        self.yaw = 0.0
        self.color = color
        self.use_oa = use_oa

        # Movement tuning
        self.max_speed = 5.0
        self.acceleration = 8.0
        self.turn_rate = 180.0

        # Optional obstacle avoidance planner for the leader
        if self.use_oa:
            self.bendy = AP_OABendyRuler()
            self.last_oa_target = Vector2(x, y)
        else:
            self.bendy = None
            self.last_oa_target = None

    def configure_physics(self, max_speed, acceleration, turn_rate):
        """Override movement tuning values."""
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.turn_rate = turn_rate

    def update(self, dt, target_pos, obstacles=None, override_yaw=None):
        # 1) Determine intermediate target (OA waypoint vs direct target)
        if self.use_oa and obstacles:
            oa_target = self.bendy.update(self.pos, target_pos, self.vel, obstacles)
            self.last_oa_target = oa_target
        else:
            oa_target = target_pos

        # 2) Compute desired yaw (formation-controlled or path-following)
        if override_yaw is not None:
            # Defender mode: face commanded direction (typically toward leader)
            target_yaw = override_yaw
        else:
            # Leader mode: face OA target / movement direction
            dx = oa_target.x - self.pos.x
            dy = oa_target.y - self.pos.y
            dist = math.hypot(dx, dy)
            if dist > 0.1:
                target_yaw = math.degrees(math.atan2(dy, dx))
            else:
                target_yaw = self.yaw

        diff = wrap_180(target_yaw - self.yaw)
        delta_yaw = max(-self.turn_rate * dt, min(self.turn_rate * dt, diff))
        self.yaw = wrap_180(self.yaw + delta_yaw)

        # 3) Kinematic arrive behavior toward final target_pos
        rad = math.radians(self.yaw)

        # Distance to final goal (used for smooth deceleration)
        final_dx = target_pos.x - self.pos.x
        final_dy = target_pos.y - self.pos.y
        final_dist = math.hypot(final_dx, final_dy)

        # Speed ramp near goal to reduce overshoot
        desired_speed = self.max_speed
        if final_dist < 2.0:
            desired_speed = self.max_speed * (final_dist / 2.0)
            if final_dist < 0.1:
                desired_speed = 0

        # Defender mode translates directly toward slot target.
        # Leader mode translates along current yaw.
        if override_yaw is not None:
            move_angle = math.atan2(final_dy, final_dx)
            target_vel_x = math.cos(move_angle) * desired_speed
            target_vel_y = math.sin(move_angle) * desired_speed
        else:
            target_vel_x = math.cos(rad) * desired_speed
            target_vel_y = math.sin(rad) * desired_speed

        # Acceleration-limited velocity integration
        dv_x = target_vel_x - self.vel.x
        dv_y = target_vel_y - self.vel.y
        dv_len = math.hypot(dv_x, dv_y)

        if dv_len > 0:
            scale = min(dv_len, self.acceleration * dt) / dv_len
            self.vel.x += dv_x * scale
            self.vel.y += dv_y * scale

        # Position integration
        self.pos.x += self.vel.x * dt
        self.pos.y += self.vel.y * dt

    def draw(self, surface, camera):
        """Draw a triangular drone glyph oriented by current yaw."""
        cx, cy = camera.world_to_screen(self.pos)
        size = camera.scale_len(0.5)
        rad = math.radians(self.yaw)
        p1 = (cx + math.cos(rad) * size, cy + math.sin(rad) * size)
        p2 = (cx + math.cos(rad + 2.4) * size, cy + math.sin(rad + 2.4) * size)
        p3 = (cx + math.cos(rad - 2.4) * size, cy + math.sin(rad - 2.4) * size)

        pygame.draw.polygon(surface, self.color, [p1, p2, p3])
        pygame.draw.polygon(surface, (255, 255, 255), [p1, p2, p3], 1)
