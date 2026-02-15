import itertools
import math
import pygame

from bendyruler import Vector2
from config import *
from entities.drone import Drone

# Hybrid formation controller: chase, recon, and trap-lockdown states.

# --- Constants & Tuning ---
NUM_DEFENDERS = 4
DEFENDER_RADIUS = 0.5
COLOR_ACTIVE = (255, 140, 0)     # Orange: active chase mode
COLOR_NEUTRAL = (180, 180, 180)  # Gray: passive recon mode

# U-shape pursuit parameters
U_BASE_DIST = 4.0         # Rear offset from leader
U_WIDTH_SPACING = 2.5     # Lateral spacing
U_DEPTH_FACTOR = 2.0      # Parabolic depth of U shape
HIJACK_SUCCESS_DIST = 2.0  # Distance that triggers trap entry
LOCKDOWN_EXIT_DIST = 5.0   # Hysteresis distance for trap exit

# Trap mode parameters
TRAP_RADIUS = 3.4
TRAP_MAX_ARC_SHIFT = 30.0  # Maximum arc oscillation in degrees
TRAP_ARC_SPEED = 2.8       # Oscillation angular speed (rad/sec)

# Recon cycle timing
TIME_CHASE_INIT = 2.0      # Initial chase phase duration
TIME_CHASE_NORMAL = 6.0    # Duration of later chase phases
TIME_RECON_WINDOW = 1.5    # Observation window at cycle end
MAX_RECON_ATTEMPTS = 2


class SmartUShapeFormation:
    """
    Controls defenders using three coordinated states:
    1) U-shape pursuit: push leader toward hijack target.
    2) Recon mode: temporarily passive to observe trajectory.
    3) Trap mode (lockdown): surround leader near the target.
    """

    def __init__(self, leader_start_pos):
        # Runtime state
        self.defenders = []
        self.hijack_target = Vector2(0, 0)
        self.success = False

        # State machine fields
        self.timer = 0.0
        self.lockdown_active = False
        self.assigned_roles = []  # Ordered defender list for 1:1 slot assignment in trap mode

        # Recon logic fields
        self.is_recon_mode = False
        self.recon_attempts_done = 0
        self.observed_rays = []
        self.estimated_destination = None

        # Leader kinematic tracking
        self.last_leader_pos = Vector2(leader_start_pos.x, leader_start_pos.y)

        # Initialize defenders
        for _ in range(NUM_DEFENDERS):
            d = Drone(leader_start_pos.x, leader_start_pos.y, use_oa=False, color=COLOR_ACTIVE)
            d.configure_physics(max_speed=15.0, acceleration=60.0, turn_rate=720.0)
            self.defenders.append(d)

    def set_hijack_target(self, target_vec):
        """Fully reset strategy state for a new hijack target."""
        self.hijack_target = target_vec
        self.observed_rays.clear()
        self.estimated_destination = None
        self.recon_attempts_done = 0
        self.timer = 0.0
        self.lockdown_active = False
        self.assigned_roles = []

    def update(self, dt, leader):
        """Main update: state transitions, then execute active movement strategy."""
        self.timer += dt

        # 1) Leader kinematic analysis (speed and heading)
        dx = leader.pos.x - self.last_leader_pos.x
        dy = leader.pos.y - self.last_leader_pos.y
        self.last_leader_pos = Vector2(leader.pos.x, leader.pos.y)
        speed = math.hypot(dx, dy)
        move_dir = Vector2(dx / speed, dy / speed) if speed > 0.001 else Vector2(0, 0)

        # 2) State machine transitions
        dist_to_hijack = math.hypot(
            leader.pos.x - self.hijack_target.x,
            leader.pos.y - self.hijack_target.y,
        )

        # Transition into/out of trap lockdown with hysteresis
        if not self.lockdown_active and dist_to_hijack < HIJACK_SUCCESS_DIST:
            self._enter_lockdown(leader)
        elif self.lockdown_active and dist_to_hijack > LOCKDOWN_EXIT_DIST:
            self._exit_lockdown()

        # Recon only runs outside lockdown
        if not self.lockdown_active:
            self._update_recon_logic(speed, move_dir)

        self.success = self.lockdown_active

        # 3) Execute active strategy
        if self.lockdown_active:
            return self._execute_trap_mode(dt, leader)
        return self._execute_u_shape_mode(dt, leader)

    # =========================================================
    # STATE MANAGEMENT HELPERS
    # =========================================================

    def _enter_lockdown(self, leader):
        """Activate trap mode and compute optimal drone-to-slot assignment."""
        self.lockdown_active = True
        self.success = True

        # Cross-like slot layout: +90, 0, -90, 180 degrees around leader
        base_angles = [math.radians(90), math.radians(0), math.radians(-90), math.radians(180)]
        phase = self.timer * TRAP_ARC_SPEED
        max_shift = math.radians(TRAP_MAX_ARC_SHIFT)

        # Minimize crossing during activation
        self.assigned_roles = self._assign_roles_lsa(leader, TRAP_RADIUS, phase, base_angles, max_shift)

    def _exit_lockdown(self):
        """Release trap mode once exit hysteresis condition is met."""
        self.lockdown_active = False
        self.success = False
        self.assigned_roles = []

    def _update_recon_logic(self, speed, move_dir):
        """Run periodic recon windows and record observations on recon end edges."""
        if self.recon_attempts_done >= MAX_RECON_ATTEMPTS:
            self.is_recon_mode = False
            return

        chase_dur = TIME_CHASE_INIT if self.recon_attempts_done == 0 else TIME_CHASE_NORMAL
        cycle_len = chase_dur + TIME_RECON_WINDOW
        local_time = self.timer % cycle_len

        prev_recon = self.is_recon_mode
        self.is_recon_mode = local_time > chase_dur

        # Detect falling edge of recon window
        if prev_recon and not self.is_recon_mode:
            self.recon_attempts_done += 1
            if speed > 0.02:
                self._record_observation(self.last_leader_pos, move_dir)

    def _record_observation(self, pos, direction):
        """Triangulate destination estimate from two consecutive motion rays."""
        self.observed_rays.append((pos, direction))
        if len(self.observed_rays) >= 2:
            inter = self._calculate_intersection(self.observed_rays[-2], self.observed_rays[-1])
            if inter:
                # Exponential-like averaging for stability
                if self.estimated_destination is None:
                    self.estimated_destination = inter
                else:
                    self.estimated_destination.x = (self.estimated_destination.x + inter.x) * 0.5
                    self.estimated_destination.y = (self.estimated_destination.y + inter.y) * 0.5

    # =========================================================
    # MOVEMENT STRATEGIES
    # =========================================================

    def _execute_trap_mode(self, dt, leader):
        """
        Trap behavior:
        Defenders oscillate on opposing arcs to create a dynamic opening.
        """
        obstacles_data = []
        center_x, center_y = leader.pos.x, leader.pos.y
        phase = self.timer * TRAP_ARC_SPEED

        # Base angles: up, right, down, left
        base_angles = [math.radians(90), math.radians(0), math.radians(-90), math.radians(180)]
        max_shift = math.radians(TRAP_MAX_ARC_SHIFT)

        for slot_idx, d in enumerate(self.assigned_roles):
            base_ang = base_angles[slot_idx]

            # Counter-phase oscillation between upper pair and lower pair
            shift_dir = 1.0 if slot_idx in (0, 1) else -1.0
            arc_shift = max_shift * math.sin(phase) * shift_dir

            ang = base_ang + arc_shift
            tx = center_x + math.cos(ang) * TRAP_RADIUS
            ty = center_y + math.sin(ang) * TRAP_RADIUS

            # Always face toward center
            t_yaw = math.degrees(math.atan2(center_y - ty, center_x - tx))

            d.update(dt, Vector2(tx, ty), obstacles=None, override_yaw=t_yaw)
            obstacles_data.append({"pos": d.pos, "radius": DEFENDER_RADIUS})

        return obstacles_data

    def _execute_u_shape_mode(self, dt, leader):
        """
        Standard pursuit behavior:
        Build U-shape behind leader based on hijack target direction.
        """
        if self.is_recon_mode:
            # Passive during recon: no collision obstacles emitted
            self._move_drones_u_shape(dt, leader, active=False)
            return []

        # Active chase mode: defenders are obstacles for OA
        self._move_drones_u_shape(dt, leader, active=True)
        return [{"pos": d.pos, "radius": DEFENDER_RADIUS} for d in self.defenders]

    def _move_drones_u_shape(self, dt, leader, active):
        """Move defenders on a U curve behind the leader."""
        # Direction vector from leader to hijack target
        vx = self.hijack_target.x - leader.pos.x
        vy = self.hijack_target.y - leader.pos.y
        dist = math.hypot(vx, vy)

        # Normalize forward/right vectors
        fx, fy = (vx / dist, vy / dist) if dist > 0.01 else (1.0, 0.0)
        rx, ry = -fy, fx

        # Rear pivot point for U-shape construction
        center_x = leader.pos.x - (fx * U_BASE_DIST)
        center_y = leader.pos.y - (fy * U_BASE_DIST)
        center_idx = (NUM_DEFENDERS - 1) / 2.0

        for i, d in enumerate(self.defenders):
            # Parabolic slot offset: depth proportional to lateral offset squared
            idx = i - center_idx
            lat_offset = idx * U_WIDTH_SPACING
            depth_offset = (idx ** 2) * U_DEPTH_FACTOR

            tx = center_x + (rx * lat_offset) + (fx * depth_offset)
            ty = center_y + (ry * lat_offset) + (fy * depth_offset)

            # Face toward leader
            t_yaw = math.degrees(math.atan2(leader.pos.y - ty, leader.pos.x - tx))

            d.color = COLOR_ACTIVE if active else COLOR_NEUTRAL
            d.update(dt, Vector2(tx, ty), obstacles=None, override_yaw=t_yaw)

    # =========================================================
    # UTILITIES (MATH & ASSIGNMENT)
    # =========================================================

    def _calculate_intersection(self, ray1, ray2):
        """2D ray-ray intersection with finite forward distance filtering."""
        p1, d1 = ray1
        p2, d2 = ray2
        det = d1.x * d2.y - d1.y * d2.x
        if abs(det) < 0.001:
            return None

        dx = p2.x - p1.x
        dy = p2.y - p1.y
        t = (dx * d2.y - dy * d2.x) / det
        u = (dx * d1.y - dy * d1.x) / det

        # t < 500 avoids unstable far-away intersections
        if t > 0 and u > 0 and t < 500:
            return Vector2(p1.x + t * d1.x, p1.y + t * d1.y)
        return None

    def _assign_roles_lsa(self, leader, radius, phase, base_angles, max_arc_shift):
        """
        Solve drone-to-slot assignment by minimizing sum of squared distances.
        Brute force permutations are acceptable for N=4.
        """
        # 1) Build theoretical slot targets for current trap phase
        slot_targets = []
        for slot_idx, base_ang in enumerate(base_angles):
            shift_dir = 1.0 if slot_idx in (0, 1) else -1.0
            ang = base_ang + (max_arc_shift * math.sin(phase) * shift_dir)

            tx = leader.pos.x + math.cos(ang) * radius
            ty = leader.pos.y + math.sin(ang) * radius
            slot_targets.append(Vector2(tx, ty))

        # 2) Find minimum-cost permutation
        drones = self.defenders
        n = len(drones)
        best_perm = None
        best_cost = float("inf")

        for perm in itertools.permutations(range(n)):
            current_cost = 0.0
            for slot_j in range(n):
                d = drones[perm[slot_j]]
                t = slot_targets[slot_j]
                current_cost += (d.pos.x - t.x) ** 2 + (d.pos.y - t.y) ** 2

                if current_cost >= best_cost:
                    break

            if current_cost < best_cost:
                best_cost = current_cost
                best_perm = perm

        return [drones[best_perm[i]] for i in range(n)]

    # =========================================================
    # DEBUG DRAWING
    # =========================================================

    def draw(self, surface, camera):
        """Draw target/recon/debug markers and all defender drones."""
        # 1) Target marker
        tgt_color = (0, 255, 0) if self.success else (255, 0, 255)
        h_scr = camera.world_to_screen(self.hijack_target)
        pygame.draw.circle(surface, tgt_color, (int(h_scr[0]), int(h_scr[1])), 8, 2)

        # 2) Recon label
        if self.is_recon_mode:
            font = pygame.font.SysFont("Impact", 20)
            txt = font.render(f"RECON #{self.recon_attempts_done + 1}", True, (200, 200, 200))
            l_scr = camera.world_to_screen(self.last_leader_pos)
            surface.blit(txt, (l_scr[0] + 15, l_scr[1] - 30))

        # 3) Estimated destination marker
        if self.estimated_destination:
            est_scr = camera.world_to_screen(self.estimated_destination)
            size = 8
            col = (255, 215, 0)  # Gold
            pygame.draw.line(surface, col, (est_scr[0] - size, est_scr[1] - size), (est_scr[0] + size, est_scr[1] + size), 2)
            pygame.draw.line(surface, col, (est_scr[0] + size, est_scr[1] - size), (est_scr[0] - size, est_scr[1] + size), 2)

            f_small = pygame.font.SysFont("Arial", 10)
            surface.blit(f_small.render("ESTIMATED", True, col), (est_scr[0] - 20, est_scr[1] + 10))

        # 4) Defenders
        for d in self.defenders:
            # Draw collision halo only when active (not in recon)
            if not self.is_recon_mode:
                center = camera.world_to_screen(d.pos)
                margin_px = int(camera.scale_len(DEFENDER_RADIUS + OA_MARGIN_MAX))
                if margin_px > 0:
                    s = pygame.Surface((margin_px * 2, margin_px * 2), pygame.SRCALPHA)
                    pygame.draw.circle(s, (255, 140, 0, 40), (margin_px, margin_px), margin_px)
                    surface.blit(s, (center[0] - margin_px, center[1] - margin_px))

            d.draw(surface, camera)
