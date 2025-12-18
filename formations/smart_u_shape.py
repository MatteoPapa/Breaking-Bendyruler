import math
import pygame
import itertools
from config import *
from bendyruler import Vector2
from entities.drone import Drone

# --- Constants & Tuning ---
NUM_DEFENDERS = 4
DEFENDER_RADIUS = 0.5
COLOR_ACTIVE = (255, 140, 0)     # Arancio: Modalità Attacco
COLOR_NEUTRAL = (180, 180, 180)  # Grigio: Modalità Recon/Passiva

# Formazione U-Shape (Inseguimento)
U_BASE_DIST = 4.0        # Distanza posteriore dal leader
U_WIDTH_SPACING = 2.5    # Larghezza laterale della formazione
U_DEPTH_FACTOR = 2.0     # Curvatura della U
HIJACK_SUCCESS_DIST = 2.0 # Distanza di innesco trappola
LOCKDOWN_EXIT_DIST = 5.0  # Isteresi: distanza di rottura trappola

# Formazione Trap (Cattura)
TRAP_RADIUS = 3.4
TRAP_MAX_ARC_SHIFT = 30.0 # Gradi di oscillazione max
TRAP_ARC_SPEED = 2.8      # Velocità oscillazione (rad/sec)

# Timing Recon Algorithm
TIME_CHASE_INIT = 2.0     # Durata primo inseguimento
TIME_CHASE_NORMAL = 6.0   # Durata inseguimenti successivi
TIME_RECON_WINDOW = 1.5   # Finestra temporale di osservazione (Recon)
MAX_RECON_ATTEMPTS = 2

class SmartUShapeFormation:
    """
    Gestisce un team di droni difensori che alternano tra:
    1. U-Shape Pursuit: Spingono il leader verso un target di hijack.
    2. Recon Mode: Si "spengono" temporaneamente per osservare la traiettoria del leader e stimarne la destinazione.
    3. Trap Mode (Lockdown): Circondano il leader quando è vicino al target per tenerlo bloccato.
    """

    def __init__(self, leader_start_pos):
        self.defenders = []
        self.hijack_target = Vector2(0, 0)
        self.success = False
        
        # State Management
        self.timer = 0.0
        self.lockdown_active = False
        self.assigned_roles = []  # Lista ordinata droni per assignment 1:1 nel lockdown
        
        # Recon Logic
        self.is_recon_mode = False
        self.recon_attempts_done = 0
        self.observed_rays = [] 
        self.estimated_destination = None
        
        # Physics State
        self.last_leader_pos = Vector2(leader_start_pos.x, leader_start_pos.y)

        # Init Drones
        for _ in range(NUM_DEFENDERS):
            d = Drone(leader_start_pos.x, leader_start_pos.y, use_oa=False, color=COLOR_ACTIVE)
            d.configure_physics(max_speed=15.0, acceleration=60.0, turn_rate=720.0)
            self.defenders.append(d)

    def set_hijack_target(self, target_vec):
        """Reset completo della strategia per un nuovo target."""
        self.hijack_target = target_vec
        self.observed_rays.clear()
        self.estimated_destination = None
        self.recon_attempts_done = 0
        self.timer = 0.0
        self.lockdown_active = False
        self.assigned_roles = []

    def update(self, dt, leader):
        """Main Loop: Determina stato -> Calcola posizioni -> Muove droni."""
        self.timer += dt
        
        # 1. Analisi Cinematica Leader (Velocità e Direzione)
        dx = leader.pos.x - self.last_leader_pos.x
        dy = leader.pos.y - self.last_leader_pos.y
        self.last_leader_pos = Vector2(leader.pos.x, leader.pos.y)
        speed = math.hypot(dx, dy)
        move_dir = Vector2(dx/speed, dy/speed) if speed > 0.001 else Vector2(0,0)

        # 2. Gestione Stati (State Machine)
        dist_to_hijack = math.hypot(leader.pos.x - self.hijack_target.x,
                                    leader.pos.y - self.hijack_target.y)
        
        # Check Transizione: Trap Mode (Lockdown)
        if not self.lockdown_active and dist_to_hijack < HIJACK_SUCCESS_DIST:
            self._enter_lockdown(leader)
        elif self.lockdown_active and dist_to_hijack > LOCKDOWN_EXIT_DIST:
            self._exit_lockdown()

        # Check Transizione: Recon Mode (Solo se non in Lockdown)
        if not self.lockdown_active:
            self._update_recon_logic(speed, move_dir)

        self.success = self.lockdown_active # Flag visivo

        # 3. Esecuzione Strategia
        if self.lockdown_active:
            return self._execute_trap_mode(dt, leader)
        else:
            return self._execute_u_shape_mode(dt, leader)

    # =========================================================
    # STATE MANAGEMENT HELPER METHODS
    # =========================================================

    def _enter_lockdown(self, leader):
        """Attiva la trappola e calcola l'assegnazione ottimale degli slot."""
        self.lockdown_active = True
        self.success = True
        
        # Configurazione slot a croce (+90, 0, -90, 180)
        base_angles = [math.radians(90), math.radians(0), math.radians(-90), math.radians(180)]
        phase = self.timer * TRAP_ARC_SPEED
        max_shift = math.radians(TRAP_MAX_ARC_SHIFT)
        
        # Linear Sum Assignment per minimizzare incroci all'attivazione
        self.assigned_roles = self._assign_roles_lsa(leader, TRAP_RADIUS, phase, base_angles, max_shift)

    def _exit_lockdown(self):
        """Rilascia la trappola (isteresi superata)."""
        self.lockdown_active = False
        self.success = False
        self.assigned_roles = []

    def _update_recon_logic(self, speed, move_dir):
        """Gestisce il timer ciclico per attivare la modalità osservazione."""
        if self.recon_attempts_done >= MAX_RECON_ATTEMPTS:
            self.is_recon_mode = False
            return

        chase_dur = TIME_CHASE_INIT if self.recon_attempts_done == 0 else TIME_CHASE_NORMAL
        cycle_len = chase_dur + TIME_RECON_WINDOW
        local_time = self.timer % cycle_len
        
        prev_recon = self.is_recon_mode
        self.is_recon_mode = local_time > chase_dur # True se siamo nella finestra finale del ciclo
        
        # Edge Detection: Fine fase Recon
        if prev_recon and not self.is_recon_mode:
            self.recon_attempts_done += 1
            # Campionamento posizione solo se il leader si muove
            if speed > 0.02:
                self._record_observation(self.last_leader_pos, move_dir)

    def _record_observation(self, pos, direction):
        """Triangola la destinazione basandosi su due raggi consecutivi."""
        self.observed_rays.append((pos, direction))
        if len(self.observed_rays) >= 2:
            inter = self._calculate_intersection(self.observed_rays[-2], self.observed_rays[-1])
            if inter:
                # Media mobile semplice sulla stima
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
        Logica Trappola:
        I droni oscillano su archi opposti per creare un buco dinamico.
        """
        obstacles_data = []
        center_x, center_y = leader.pos.x, leader.pos.y
        phase = self.timer * TRAP_ARC_SPEED
        
        # Angoli base (Alto, Destra, Basso, Sinistra)
        base_angles = [math.radians(90), math.radians(0), math.radians(-90), math.radians(180)]
        max_shift = math.radians(TRAP_MAX_ARC_SHIFT)

        for slot_idx, d in enumerate(self.assigned_roles):
            base_ang = base_angles[slot_idx]
            
            # Oscillazione in controfase: Slot 0,1 (+sin) vs Slot 2,3 (-sin)
            shift_dir = 1.0 if slot_idx in (0, 1) else -1.0
            arc_shift = max_shift * math.sin(phase) * shift_dir
            
            ang = base_ang + arc_shift
            tx = center_x + math.cos(ang) * TRAP_RADIUS
            ty = center_y + math.sin(ang) * TRAP_RADIUS

            # Yaw sempre verso il centro (intimidazione)
            t_yaw = math.degrees(math.atan2(center_y - ty, center_x - tx))

            d.update(dt, Vector2(tx, ty), obstacles=None, override_yaw=t_yaw)
            obstacles_data.append({'pos': d.pos, 'radius': DEFENDER_RADIUS})
            
        return obstacles_data

    def _execute_u_shape_mode(self, dt, leader):
        """
        Logica Inseguimento Standard:
        Calcola vettore verso Hijack Target e posiziona la U dietro al leader.
        """
        if self.is_recon_mode:
            # In recon i droni sono passivi (niente collisione logica)
            self._move_drones_u_shape(dt, leader, active=False)
            return []
        else:
            # In chase i droni sono ostacoli attivi
            self._move_drones_u_shape(dt, leader, active=True)
            # Ritorna posizioni per OA
            return [{'pos': d.pos, 'radius': DEFENDER_RADIUS} for d in self.defenders]

    def _move_drones_u_shape(self, dt, leader, active):
        # Vettore Direzione: Da Leader a Target
        vx = self.hijack_target.x - leader.pos.x
        vy = self.hijack_target.y - leader.pos.y
        dist = math.hypot(vx, vy)
        
        # Normalizzazione (Forward e Right vectors)
        fx, fy = (vx/dist, vy/dist) if dist > 0.01 else (1.0, 0.0)
        rx, ry = -fy, fx

        # Punto pivot posteriore
        center_x = leader.pos.x - (fx * U_BASE_DIST)
        center_y = leader.pos.y - (fy * U_BASE_DIST)
        center_idx = (NUM_DEFENDERS - 1) / 2.0

        for i, d in enumerate(self.defenders):
            # Calcolo offset parabolico
            idx = i - center_idx
            lat_offset = idx * U_WIDTH_SPACING
            depth_offset = (idx ** 2) * U_DEPTH_FACTOR # Equazione parabola y = x^2
            
            tx = center_x + (rx * lat_offset) + (fx * depth_offset)
            ty = center_y + (ry * lat_offset) + (fy * depth_offset)
            
            # Yaw verso il leader
            t_yaw = math.degrees(math.atan2(leader.pos.y - ty, leader.pos.x - tx))
            
            d.color = COLOR_ACTIVE if active else COLOR_NEUTRAL
            d.update(dt, Vector2(tx, ty), obstacles=None, override_yaw=t_yaw)

    # =========================================================
    # UTILITIES (Math & Algo)
    # =========================================================

    def _calculate_intersection(self, ray1, ray2):
        """Intersezione 2D raggio-raggio."""
        p1, d1 = ray1
        p2, d2 = ray2
        det = d1.x * d2.y - d1.y * d2.x
        if abs(det) < 0.001: return None
        
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        t = (dx * d2.y - dy * d2.x) / det
        u = (dx * d1.y - dy * d1.x) / det
        
        if t > 0 and u > 0 and t < 500: # t < 500 evita intersezioni all'infinito
            return Vector2(p1.x + t*d1.x, p1.y + t*d1.y)
        return None

    def _assign_roles_lsa(self, leader, radius, phase, base_angles, max_arc_shift):
        """
        Risolve l'assegnazione Drone->Slot minimizzando la somma delle distanze al quadrato.
        Usa brute-force permutazioni (OK per N=4). Per N elevati usare scipy.linear_sum_assignment.
        """
        # 1. Calcola target slot teorici
        slot_targets = []
        for slot_idx, base_ang in enumerate(base_angles):
            # Replica logica oscillazione per prevedere dove sarà lo slot
            shift_dir = 1.0 if slot_idx in (0, 1) else -1.0
            ang = base_ang + (max_arc_shift * math.sin(phase) * shift_dir)
            
            tx = leader.pos.x + math.cos(ang) * radius
            ty = leader.pos.y + math.sin(ang) * radius
            slot_targets.append(Vector2(tx, ty))

        # 2. Trova permutazione a costo minimo
        drones = self.defenders
        n = len(drones)
        best_perm = None
        best_cost = float("inf")

        for perm in itertools.permutations(range(n)):
            current_cost = 0.0
            for slot_j in range(n):
                d = drones[perm[slot_j]]
                t = slot_targets[slot_j]
                current_cost += (d.pos.x - t.x)**2 + (d.pos.y - t.y)**2
                
                if current_cost >= best_cost: break # Early exit optimization
            
            if current_cost < best_cost:
                best_cost = current_cost
                best_perm = perm

        return [drones[best_perm[i]] for i in range(n)]

    # =========================================================
    # DEBUG DRAWING
    # =========================================================

    def draw(self, surface, camera):
        # 1. Target Marker
        tgt_color = (0, 255, 0) if self.success else (255, 0, 255)
        h_scr = camera.world_to_screen(self.hijack_target)
        pygame.draw.circle(surface, tgt_color, (int(h_scr[0]), int(h_scr[1])), 8, 2)

        # 2. Recon UI Info
        if self.is_recon_mode:
            font = pygame.font.SysFont("Impact", 20)
            txt = font.render(f"RECON #{self.recon_attempts_done + 1}", True, (200, 200, 200))
            l_scr = camera.world_to_screen(self.last_leader_pos)
            surface.blit(txt, (l_scr[0] + 15, l_scr[1] - 30))

        # 3. Estimated Destination (Visual Debug)
        if self.estimated_destination:
            est_scr = camera.world_to_screen(self.estimated_destination)
            size = 8
            col = (255, 215, 0) # Gold
            # X Marker
            pygame.draw.line(surface, col, (est_scr[0]-size, est_scr[1]-size), (est_scr[0]+size, est_scr[1]+size), 2)
            pygame.draw.line(surface, col, (est_scr[0]+size, est_scr[1]-size), (est_scr[0]-size, est_scr[1]+size), 2)
            
            f_small = pygame.font.SysFont("Arial", 10)
            surface.blit(f_small.render("ESTIMATED", True, col), (est_scr[0] - 20, est_scr[1] + 10))

        # 4. Drones Render
        for d in self.defenders:
            # Disegna alone di collisione solo se attivo (non in recon)
            if not self.is_recon_mode:
                center = camera.world_to_screen(d.pos)
                margin_px = int(camera.scale_len(DEFENDER_RADIUS + OA_MARGIN_MAX))
                if margin_px > 0:
                    s = pygame.Surface((margin_px*2, margin_px*2), pygame.SRCALPHA)
                    pygame.draw.circle(s, (255, 140, 0, 40), (margin_px, margin_px), margin_px)
                    surface.blit(s, (center[0]-margin_px, center[1]-margin_px))
            
            d.draw(surface, camera)
