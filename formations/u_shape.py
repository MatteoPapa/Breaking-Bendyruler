import math
import pygame
import itertools
from config import *
from bendyruler import Vector2
from entities.drone import Drone

# --- Constants & Tuning ---
NUM_DEFENDERS = 4
DEFENDER_RADIUS = 0.5
COLOR_ACTIVE = (255, 140, 0)

# Geometria U-Shape
U_BASE_DIST = 4.0        # Distanza posteriore
U_WIDTH_SPACING = 2.5    # Larghezza laterale
U_DEPTH_FACTOR = 2.0     # Curvatura

# Logica Smart Reassignment
HP_PROXIMITY_THRESHOLD = 4.0 # Distanza entro cui attivare il riassegnamento intelligente
REASSIGN_COOLDOWN = 0.1      # Secondi tra un ricalcolo ruoli (evita flickering)

class UShapeFormation:
    """
    Gestisce una formazione a U continua.
    Caratteristica chiave: Dynamic Role Swapping.
    Quando il vettore Leader->HP ruota velocemente (vicino all'HP), 
    i droni si scambiano i ruoli invece di attraversare tutta la formazione.
    """

    def __init__(self, leader_start_pos):
        self.defenders = []
        self.hijack_target = Vector2(0, 0)
        
        # Gestione ruoli dinamici
        self.reassign_timer = 0.0
        
        # Init Drones
        for _ in range(NUM_DEFENDERS):
            d = Drone(leader_start_pos.x, leader_start_pos.y, use_oa=False, color=COLOR_ACTIVE)
            d.configure_physics(max_speed=15.0, acceleration=60.0, turn_rate=720.0)
            self.defenders.append(d)

    def set_hijack_target(self, target_vec):
        self.hijack_target = target_vec

    def update(self, dt, leader):
        self.reassign_timer -= dt
        
        # 1. Calcolo Vettori Base (Orientamento U-Shape)
        # Vettore dal Leader verso l'Hijack Point
        vx = self.hijack_target.x - leader.pos.x
        vy = self.hijack_target.y - leader.pos.y
        dist_to_hp = math.hypot(vx, vy)
        
        # Normalizzazione (Forward e Right vectors)
        # Se siamo sopra l'HP, manteniamo l'ultima direzione valida per stabilità
        if dist_to_hp > 0.01:
            fx, fy = vx / dist_to_hp, vy / dist_to_hp
        else:
            fx, fy = 1.0, 0.0 
        rx, ry = -fy, fx

        # 2. Calcolo Posizioni Slot Ideali (World Space)
        # Dove dovrebbero essere i droni in una U perfetta in questo istante?
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

        # 3. Dynamic Role Reassignment (Il cervello del sistema)
        # Se siamo vicini all'HP, l'angolo cambia troppo in fretta.
        # Invece di forzare il drone[0] ad andare allo slot[0] (che magari ora è opposto),
        # ricalcoliamo chi è meglio posizionato per andare dove.
        if dist_to_hp < HP_PROXIMITY_THRESHOLD and self.reassign_timer <= 0:
            self._optimize_assignments(ideal_slots)
            self.reassign_timer = REASSIGN_COOLDOWN

        # 4. Movimento Droni
        # self.defenders è ora ordinato in modo che defender[i] vada a ideal_slots[i]
        obstacles_data = []
        for i, d in enumerate(self.defenders):
            target = ideal_slots[i]
            
            # Yaw verso il leader per intimidazione
            t_yaw = math.degrees(math.atan2(leader.pos.y - target.y, leader.pos.x - target.x))
            
            d.update(dt, target, obstacles=None, override_yaw=t_yaw)
            obstacles_data.append({'pos': d.pos, 'radius': DEFENDER_RADIUS})

        return obstacles_data

    def _optimize_assignments(self, target_slots):
        """
        Riassegna l'ordine della lista self.defenders.
        Usa un algoritmo combinatorio (Brute Force su 4! = 24 casi) per trovare
        la combinazione che minimizza la somma delle distanze percorse.
        """
        drones = self.defenders
        n = len(drones)
        
        best_perm = None
        best_cost = float("inf")

        # Genera tutte le possibili permutazioni degli indici [0, 1, 2, 3]
        # perm[i] indica quale drone va nello slot i
        for perm in itertools.permutations(range(n)):
            current_cost = 0.0
            for slot_idx in range(n):
                drone_idx = perm[slot_idx]
                d = drones[drone_idx]
                t = target_slots[slot_idx]
                # Distanza quadrata (più veloce di sqrt e matematicamente equivalente per il min)
                dist_sq = (d.pos.x - t.x)**2 + (d.pos.y - t.y)**2
                current_cost += dist_sq
                
                # Pruning anticipato
                if current_cost >= best_cost:
                    break
            
            if current_cost < best_cost:
                best_cost = current_cost
                best_perm = perm

        # Applica la permutazione: riordina la lista self.defenders
        # in modo che il drone all'indice i sia quello destinato allo slot i
        new_order = [drones[i] for i in best_perm]
        self.defenders = new_order

    def draw(self, surface, camera):
        # Target Marker
        h_scr = camera.world_to_screen(self.hijack_target)
        pygame.draw.circle(surface, (0, 255, 0), (int(h_scr[0]), int(h_scr[1])), 8, 2)

        # Drones
        for d in self.defenders:
            # Alone Collisione
            center = camera.world_to_screen(d.pos)
            margin_px = int(camera.scale_len(DEFENDER_RADIUS + OA_MARGIN_MAX))
            if margin_px > 0:
                s = pygame.Surface((margin_px*2, margin_px*2), pygame.SRCALPHA)
                pygame.draw.circle(s, (255, 140, 0, 40), (margin_px, margin_px), margin_px)
                surface.blit(s, (center[0]-margin_px, center[1]-margin_px))
            d.draw(surface, camera)
