import math
import pygame
from bendyruler import AP_OABendyRuler, Vector2
from config import wrap_180, DRONE_COLOR

class Drone:
    def __init__(self, x, y, use_oa=True, color=DRONE_COLOR):
        self.pos = Vector2(x, y)
        self.vel = Vector2(0, 0)
        self.yaw = 0.0
        self.color = color
        self.use_oa = use_oa

        # Physics defaults
        self.max_speed = 5.0
        self.acceleration = 8.0
        self.turn_rate = 180.0
        
        # Inizializza BendyRuler se richiesto
        if self.use_oa:
            self.bendy = AP_OABendyRuler()
            self.last_oa_target = Vector2(x, y)
        else:
            self.bendy = None
            self.last_oa_target = None

    def configure_physics(self, max_speed, acceleration, turn_rate):
        """Helper per configurare parametri fisici del drone."""
        self.max_speed = max_speed
        self.acceleration = acceleration
        self.turn_rate = turn_rate

    def update(self, dt, target_pos, obstacles=None, override_yaw=None):
        # 1. Determinazione target intermedio (OA vs Diretto)
        if self.use_oa and obstacles:
            oa_target = self.bendy.update(self.pos, target_pos, self.vel, obstacles)
            self.last_oa_target = oa_target
        else:
            oa_target = target_pos # Vai dritto
        
        # 2. Gestione Yaw
        if override_yaw is not None:
            # Modalità Difensore: Guarda dove gli viene detto (il Leader)
            target_yaw = override_yaw
        else:
            # Modalità Leader: Guarda dove va
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

        # 3. Fisica di Movimento (Kinematic Arrive)
        rad = math.radians(self.yaw)
        
        # Calcolo distanza dal target finale fisico
        final_dx = target_pos.x - self.pos.x
        final_dy = target_pos.y - self.pos.y
        final_dist = math.hypot(final_dx, final_dy)

        # Gestione velocità desiderata (Rampa di decelarazione)
        desired_speed = self.max_speed
        if final_dist < 2.0:
            desired_speed = self.max_speed * (final_dist / 2.0)
            if final_dist < 0.1: desired_speed = 0

        # Se siamo difensori (override_yaw attivo), ci muoviamo indipendentemente da dove guardiamo
        # Se siamo leader, ci muoviamo nella direzione dello yaw
        if override_yaw is not None:
            move_angle = math.atan2(final_dy, final_dx)
            target_vel_x = math.cos(move_angle) * desired_speed
            target_vel_y = math.sin(move_angle) * desired_speed
        else:
            target_vel_x = math.cos(rad) * desired_speed
            target_vel_y = math.sin(rad) * desired_speed
        
        # Integrazione accelerazione
        dv_x = target_vel_x - self.vel.x
        dv_y = target_vel_y - self.vel.y
        dv_len = math.hypot(dv_x, dv_y)
        
        if dv_len > 0:
            scale = min(dv_len, self.acceleration * dt) / dv_len
            self.vel.x += dv_x * scale
            self.vel.y += dv_y * scale

        # Integrazione posizione
        self.pos.x += self.vel.x * dt
        self.pos.y += self.vel.y * dt

    def draw(self, surface, camera):
        cx, cy = camera.world_to_screen(self.pos)
        size = camera.scale_len(0.5) 
        rad = math.radians(self.yaw)
        p1 = (cx + math.cos(rad) * size, cy + math.sin(rad) * size)
        p2 = (cx + math.cos(rad + 2.4) * size, cy + math.sin(rad + 2.4) * size)
        p3 = (cx + math.cos(rad - 2.4) * size, cy + math.sin(rad - 2.4) * size)
        
        pygame.draw.polygon(surface, self.color, [p1, p2, p3])
        pygame.draw.polygon(surface, (255,255,255), [p1, p2, p3], 1)
