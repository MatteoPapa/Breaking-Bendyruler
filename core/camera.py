import pygame
from bendyruler import Vector2
from config import PIXELS_PER_METER

class Camera:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.zoom = 1.0
        self.offset_x = 0
        self.offset_y = 0
        self.is_panning = False
        self.last_mouse_pos = (0, 0)

    def world_to_screen(self, world_vec):
        sx = (world_vec.x * PIXELS_PER_METER * self.zoom) + self.offset_x
        sy = (world_vec.y * PIXELS_PER_METER * self.zoom) + self.offset_y
        return (sx, sy)

    def screen_to_world(self, screen_pos):
        mx, my = screen_pos
        wx = (mx - self.offset_x) / (PIXELS_PER_METER * self.zoom)
        wy = (my - self.offset_y) / (PIXELS_PER_METER * self.zoom)
        return Vector2(wx, wy)

    def scale_len(self, length_meters):
        return length_meters * PIXELS_PER_METER * self.zoom

    def handle_zoom(self, mouse_pos, direction):
        old_world_pos = self.screen_to_world(mouse_pos)
        zoom_speed = 0.1
        if direction > 0: self.zoom *= (1 + zoom_speed)
        else: self.zoom /= (1 + zoom_speed)
        self.zoom = max(0.1, min(self.zoom, 5.0))
        
        # Ricalcola offset per mantenere il mouse sopra lo stesso punto del mondo
        new_screen_x = (old_world_pos.x * PIXELS_PER_METER * self.zoom) + self.offset_x
        new_screen_y = (old_world_pos.y * PIXELS_PER_METER * self.zoom) + self.offset_y
        self.offset_x += mouse_pos[0] - new_screen_x
        self.offset_y += mouse_pos[1] - new_screen_y
