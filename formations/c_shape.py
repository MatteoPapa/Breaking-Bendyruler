import math
import pygame
from config import *
from bendyruler import Vector2
from entities.drone import Drone

# --- Settings ---
NUM_DEFENDERS = 4          
DEFENDER_COLOR = (3, 3, 252) 
DEFENDER_RADIUS = 0.5
FORMATION_RADIUS = 5.0
HIJACK_SUCCESS_DIST = 2.0  # Distanza per attivare il success state

class CShapeFormation:
    def __init__(self, leader_start_pos):
        self.defenders = []
        self.hijack_target = Vector2(0, 0)
        self.success = False

        for _ in range(NUM_DEFENDERS):
            d = Drone(leader_start_pos.x, leader_start_pos.y, use_oa=False, color=DEFENDER_COLOR)
            d.configure_physics(max_speed=12.0, acceleration=40.0, turn_rate=360.0)
            self.defenders.append(d)

    def set_hijack_target(self, target_vec):
        self.hijack_target = target_vec

    def update(self, dt, leader):
        obstacles_data = []

        # Check Successo (Visuale)
        dist_to_hijack = math.hypot(leader.pos.x - self.hijack_target.x, leader.pos.y - self.hijack_target.y)
        self.success = dist_to_hijack < HIJACK_SUCCESS_DIST

        # 1. Calcolo angoli
        dx = self.hijack_target.x - leader.pos.x
        dy = self.hijack_target.y - leader.pos.y
        base_angle = math.atan2(dy, dx) + math.pi 

        # 2. Calcolo Arco
        arc_width = min(math.radians(45) * (NUM_DEFENDERS - 1), math.radians(300))
        start_angle = base_angle - (arc_width / 2.0)
        step_angle = arc_width / (NUM_DEFENDERS - 1) if NUM_DEFENDERS > 1 else 0

        # 3. Aggiornamento droni
        for i, defender in enumerate(self.defenders):
            theta = start_angle + (i * step_angle)
            tx = leader.pos.x + math.cos(theta) * FORMATION_RADIUS
            ty = leader.pos.y + math.sin(theta) * FORMATION_RADIUS
            t_yaw = math.degrees(math.atan2(leader.pos.y - ty, leader.pos.x - tx))

            defender.update(dt, Vector2(tx, ty), obstacles=None, override_yaw=t_yaw)
            obstacles_data.append({'pos': defender.pos, 'radius': DEFENDER_RADIUS})

        return obstacles_data

    def draw(self, surface, camera):
        # Colore Target: Verde se successo, Magenta altrimenti
        tgt_color = (0, 255, 0) if self.success else (255, 0, 255)
        
        h_scr = camera.world_to_screen(self.hijack_target)
        pygame.draw.circle(surface, tgt_color, (int(h_scr[0]), int(h_scr[1])), 8, 2)
        pygame.draw.line(surface, tgt_color, (h_scr[0]-10, h_scr[1]), (h_scr[0]+10, h_scr[1]))
        pygame.draw.line(surface, tgt_color, (h_scr[0], h_scr[1]-10), (h_scr[0], h_scr[1]+10))

        for defender in self.defenders:
            center = camera.world_to_screen(defender.pos)
            margin_px = int(camera.scale_len(DEFENDER_RADIUS + OA_MARGIN_MAX))
            if margin_px > 0:
                s = pygame.Surface((margin_px*2, margin_px*2), pygame.SRCALPHA)
                pygame.draw.circle(s, (255, 0, 0, 30), (margin_px, margin_px), margin_px)
                surface.blit(s, (center[0]-margin_px, center[1]-margin_px))
            defender.draw(surface, camera)
