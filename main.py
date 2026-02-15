import pygame 
import math
import sys
import argparse

from config import *
from bendyruler import Vector2

# Core modules
from core.camera import Camera
from core.ui import Button, draw_hud
from entities.drone import Drone

# Optional formation modules (loaded only when available)
try:
    from formations.c_shape import CShapeFormation
    from formations.u_shape import UShapeFormation
    from formations.smart_u_shape import SmartUShapeFormation
except ImportError:
    pass

def main():
    # Parse startup options
    parser = argparse.ArgumentParser(description="BendyRuler Simulation")
    parser.add_argument("--formation", choices=["NONE", "CSHAPE", "USHAPE", "SMART_USHAPE"], default="NONE")
    args = parser.parse_args()
    current_mode = args.formation

    # Initialize pygame and core app state
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
    pygame.display.set_caption(f"BendyRuler Simulation - Mode: {current_mode}")
    clock = pygame.time.Clock()
    
    # Camera is responsible for world/screen conversions, pan and zoom
    camera = Camera(SCREEN_WIDTH, SCREEN_HEIGHT)
    
    # Mutable simulation state for the current scenario
    drone = None
    target_pos = None
    obstacles = []
    formation = None 

    def reset_scenario():
        """Reset leader, target, static obstacles, and optional formation mode."""
        nonlocal drone, target_pos, obstacles, formation
        start_pos = Vector2(10, 20)
        drone = Drone(start_pos.x, start_pos.y)
        target_pos = Vector2(60, 20)
        obstacles = []
        formation = None
        
        if current_mode == "CSHAPE" and 'CShapeFormation' in globals():
            formation = CShapeFormation(start_pos)
            formation.set_hijack_target(Vector2(40, 5)) 
        elif current_mode == "USHAPE" and 'UShapeFormation' in globals():
            formation = UShapeFormation(start_pos)
            formation.set_hijack_target(Vector2(40, 5))
        elif current_mode == "SMART_USHAPE" and 'SmartUShapeFormation' in globals():
            formation = SmartUShapeFormation(start_pos)
            formation.set_hijack_target(Vector2(40, 5))


    reset_scenario()
    btn_reset = Button(20, SCREEN_HEIGHT - 60, 100, 40, "RESET (R)", reset_scenario)

    running = True
    while running:
        # Frame timing and live input snapshot
        dt = clock.tick(FPS) / 1000.0
        mouse_pos = pygame.mouse.get_pos()
        keys = pygame.key.get_pressed()

        # Handle input events (window, keyboard, mouse)
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False
            
            # Keep camera/UI dimensions in sync with the window
            elif event.type == pygame.VIDEORESIZE:
                camera.width = event.w
                camera.height = event.h
                btn_reset.rect.y = event.h - 60
            
            # Keyboard shortcuts
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r: reset_scenario()
            
            # Mouse wheel zoom around cursor position
            elif event.type == pygame.MOUSEWHEEL:
                camera.handle_zoom(mouse_pos, event.y)
            
            # Mouse interactions (target, hijack point, obstacles, pan)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if btn_reset.check_click(mouse_pos): continue
                
                # Middle click starts camera panning
                if event.button == 2:
                    camera.is_panning = True
                    camera.last_mouse_pos = mouse_pos
                
                world_mouse = camera.screen_to_world(mouse_pos)
                
                # Left click sets leader target; Shift+Left sets hijack target
                if event.button == 1: 
                    if (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]):
                        if formation: formation.set_hijack_target(world_mouse)
                    else:
                        target_pos = world_mouse
                
                # Right click toggles static circular obstacles
                elif event.button == 3: 
                    clicked = None
                    for obs in obstacles:
                        if math.hypot(obs['pos'].x - world_mouse.x, obs['pos'].y - world_mouse.y) < obs['radius']:
                            clicked = obs; break
                    if clicked: obstacles.remove(clicked)
                    else: obstacles.append({'pos': world_mouse, 'radius': 2.0})

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 2: camera.is_panning = False
        
        # Apply panning delta while dragging
        if camera.is_panning:
            mx, my = mouse_pos
            camera.offset_x += mx - camera.last_mouse_pos[0]
            camera.offset_y += my - camera.last_mouse_pos[1]
            camera.last_mouse_pos = mouse_pos

        # Update formation first so defenders become dynamic obstacles for the leader
        dynamic_obstacles = []
        if formation:
            dynamic_obstacles = formation.update(dt, drone)
        
        # Update leader movement using static + dynamic obstacles
        drone.update(dt, target_pos, obstacles + dynamic_obstacles)

        # Render full frame
        screen.fill(BACKGROUND_COLOR)
        
        # World grid for spatial readability
        _draw_grid(screen, camera)

        # Static obstacles and their OA safety margins
        for obs in obstacles:
            center = camera.world_to_screen(obs['pos'])
            radius_px = int(camera.scale_len(obs['radius']))
            
            # Draw margin
            margin_px = int(camera.scale_len(obs['radius'] + OA_MARGIN_MAX))
            if margin_px > 0:
                s = pygame.Surface((margin_px*2, margin_px*2), pygame.SRCALPHA)
                pygame.draw.circle(s, (*MARGIN_COLOR_RGB, 40), (margin_px, margin_px), margin_px)
                screen.blit(s, (center[0]-margin_px, center[1]-margin_px))
            
            pygame.draw.circle(screen, OBSTACLE_COLOR, center, radius_px)
            pygame.draw.circle(screen, OBSTACLE_OUTLINE, center, radius_px, 2)

        # Formation, debug paths, and leader rendering
        if formation: formation.draw(screen, camera)
        _draw_debug_lines(screen, camera, drone, target_pos)
        drone.draw(screen, camera)
        
        # Overlay UI
        btn_reset.draw(screen, mouse_pos)
        draw_hud(screen, current_mode, drone)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

def _draw_grid(screen, camera):
    """Draw major grid lines in world space (every 5 meters)."""
    start_x = int((-camera.offset_x) / (PIXELS_PER_METER * camera.zoom))
    end_x = int((camera.width - camera.offset_x) / (PIXELS_PER_METER * camera.zoom)) + 1
    start_y = int((-camera.offset_y) / (PIXELS_PER_METER * camera.zoom))
    end_y = int((camera.height - camera.offset_y) / (PIXELS_PER_METER * camera.zoom)) + 1
    
    for x in range(start_x, end_x):
        if x % 5 == 0:
            p = camera.world_to_screen(Vector2(x, 0))
            pygame.draw.line(screen, GRID_COLOR, (p[0], 0), (p[0], camera.height), 1)
    for y in range(start_y, end_y):
        if y % 5 == 0:
            p = camera.world_to_screen(Vector2(0, y))
            pygame.draw.line(screen, GRID_COLOR, (0, p[1]), (camera.width, p[1]), 1)

def _draw_debug_lines(screen, camera, drone, target_pos):
    """Visualize direct path and selected OA intermediate target."""
    drone_scr = camera.world_to_screen(drone.pos)
    target_scr = camera.world_to_screen(target_pos)
    oa_target_scr = camera.world_to_screen(drone.last_oa_target)

    pygame.draw.line(screen, PATH_DIRECT_COLOR, drone_scr, target_scr, 1) 
    pygame.draw.line(screen, PATH_PROBE_COLOR, drone_scr, oa_target_scr, 3) 
    pygame.draw.circle(screen, PATH_PROBE_COLOR, (int(oa_target_scr[0]), int(oa_target_scr[1])), 4)
    pygame.draw.circle(screen, TARGET_COLOR, (int(target_scr[0]), int(target_scr[1])), 6)

if __name__ == "__main__":
    main()
