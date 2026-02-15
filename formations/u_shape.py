import itertools
import math
import pygame

from bendyruler import Vector2
from config import *
from entities.drone import Drone

# U-shaped defender formation with near-target role swapping.

# --- Constants & Tuning ---
NUM_DEFENDERS = 4
DEFENDER_RADIUS = 0.5
COLOR_ACTIVE = (255, 140, 0)

# U-shape geometry
U_BASE_DIST = 4.0        # Rear offset from the leader
U_WIDTH_SPACING = 2.5    # Lateral spacing between drones
U_DEPTH_FACTOR = 2.0     # Parabolic depth coefficient
HIJACK_SUCCESS_DIST = 2.0

# Smart reassignment behavior
HP_PROXIMITY_THRESHOLD = 4.0  # Distance that enables reassignment
REASSIGN_COOLDOWN = 0.1       # Delay between role recomputations


class UShapeFormation:
    """
    Continuous U-shaped formation with dynamic role swapping.
    When Leader->HP rotates quickly near the target, drones swap slots
    instead of crossing the whole formation.
    """

    def __init__(self, leader_start_pos):
        # Runtime state
        self.defenders = []
        self.hijack_target = Vector2(0, 0)
        self.success = False

        # Dynamic role management
        self.reassign_timer = 0.0

        # Initialize defenders
        for _ in range(NUM_DEFENDERS):
            d = Drone(leader_start_pos.x, leader_start_pos.y, use_oa=False, color=COLOR_ACTIVE)
            d.configure_physics(max_speed=15.0, acceleration=60.0, turn_rate=720.0)
            self.defenders.append(d)

    def set_hijack_target(self, target_vec):
        """Set the point the formation tries to push the leader toward."""
        self.hijack_target = target_vec

    def update(self, dt, leader):
        """Update U-shape slots, optionally reassign roles, and move defenders."""
        self.reassign_timer -= dt

        # 1) Build orientation vectors from leader toward hijack point
        vx = self.hijack_target.x - leader.pos.x
        vy = self.hijack_target.y - leader.pos.y
        dist_to_hp = math.hypot(vx, vy)

        self.success = dist_to_hp < HIJACK_SUCCESS_DIST

        # Normalize forward/right vectors; use fallback direction near zero distance
        if dist_to_hp > 0.01:
            fx, fy = vx / dist_to_hp, vy / dist_to_hp
        else:
            fx, fy = 1.0, 0.0
        rx, ry = -fy, fx

        # 2) Compute ideal world-space slots for a perfect U shape
        center_x = leader.pos.x - (fx * U_BASE_DIST)
        center_y = leader.pos.y - (fy * U_BASE_DIST)
        center_idx = (NUM_DEFENDERS - 1) / 2.0

        ideal_slots = []
        for i in range(NUM_DEFENDERS):
            idx = i - center_idx
            lat_offset = idx * U_WIDTH_SPACING
            depth_offset = (idx ** 2) * U_DEPTH_FACTOR

            tx = center_x + (rx * lat_offset) + (fx * depth_offset)
            ty = center_y + (ry * lat_offset) + (fy * depth_offset)
            ideal_slots.append(Vector2(tx, ty))

        # 3) Reassign roles near target to minimize long cross-formation moves
        if dist_to_hp < HP_PROXIMITY_THRESHOLD and self.reassign_timer <= 0:
            self._optimize_assignments(ideal_slots)
            self.reassign_timer = REASSIGN_COOLDOWN

        # 4) Move defenders toward assigned slots
        obstacles_data = []
        for i, d in enumerate(self.defenders):
            target = ideal_slots[i]

            # Face the leader for intimidation/containment effect
            t_yaw = math.degrees(math.atan2(leader.pos.y - target.y, leader.pos.x - target.x))

            d.update(dt, target, obstacles=None, override_yaw=t_yaw)
            obstacles_data.append({"pos": d.pos, "radius": DEFENDER_RADIUS})

        return obstacles_data

    def _optimize_assignments(self, target_slots):
        """
        Reorder self.defenders to minimize total travel distance to target slots.
        Uses brute force permutations (4! = 24), which is fast for N=4.
        """
        drones = self.defenders
        n = len(drones)

        best_perm = None
        best_cost = float("inf")

        # perm[i] indicates which drone is assigned to slot i
        for perm in itertools.permutations(range(n)):
            current_cost = 0.0
            for slot_idx in range(n):
                drone_idx = perm[slot_idx]
                d = drones[drone_idx]
                t = target_slots[slot_idx]
                # Squared distance is enough for minimization and avoids sqrt cost
                dist_sq = (d.pos.x - t.x) ** 2 + (d.pos.y - t.y) ** 2
                current_cost += dist_sq

                # Early pruning
                if current_cost >= best_cost:
                    break

            if current_cost < best_cost:
                best_cost = current_cost
                best_perm = perm

        self.defenders = [drones[i] for i in best_perm]

    def draw(self, surface, camera):
        """Render hijack marker and defenders with active collision halos."""
        # Target marker
        tgt_color = (0, 255, 0) if self.success else (255, 0, 255)
        h_scr = camera.world_to_screen(self.hijack_target)
        pygame.draw.circle(surface, tgt_color, (int(h_scr[0]), int(h_scr[1])), 8, 2)

        # Defenders
        for d in self.defenders:
            center = camera.world_to_screen(d.pos)
            margin_px = int(camera.scale_len(DEFENDER_RADIUS + OA_MARGIN_MAX))
            if margin_px > 0:
                s = pygame.Surface((margin_px * 2, margin_px * 2), pygame.SRCALPHA)
                pygame.draw.circle(s, (255, 140, 0, 40), (margin_px, margin_px), margin_px)
                surface.blit(s, (center[0] - margin_px, center[1] - margin_px))
            d.draw(surface, camera)
