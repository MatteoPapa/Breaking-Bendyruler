import pygame
from bendyruler import Vector2
from config import PIXELS_PER_METER

class Camera:
    """Viewport transform helper for pan/zoom and world/screen conversion."""
    def __init__(self, width, height):
        # Viewport size in pixels
        self.width = width
        self.height = height

        # Zoom and pan transform
        self.zoom = 1.0
        self.offset_x = 0
        self.offset_y = 0

        # Panning interaction state
        self.is_panning = False
        self.last_mouse_pos = (0, 0)

    def world_to_screen(self, world_vec):
        """Convert world coordinates (meters) to screen coordinates (pixels)."""
        sx = (world_vec.x * PIXELS_PER_METER * self.zoom) + self.offset_x
        sy = (world_vec.y * PIXELS_PER_METER * self.zoom) + self.offset_y
        return (sx, sy)

    def screen_to_world(self, screen_pos):
        """Convert screen coordinates (pixels) back to world coordinates (meters)."""
        mx, my = screen_pos
        wx = (mx - self.offset_x) / (PIXELS_PER_METER * self.zoom)
        wy = (my - self.offset_y) / (PIXELS_PER_METER * self.zoom)
        return Vector2(wx, wy)

    def scale_len(self, length_meters):
        """Scale a world length (meters) into pixels at current zoom."""
        return length_meters * PIXELS_PER_METER * self.zoom

    def handle_zoom(self, mouse_pos, direction):
        """Zoom in/out while keeping the world point under the cursor fixed."""
        old_world_pos = self.screen_to_world(mouse_pos)
        zoom_speed = 0.1
        if direction > 0: self.zoom *= (1 + zoom_speed)
        else: self.zoom /= (1 + zoom_speed)
        self.zoom = max(0.1, min(self.zoom, 5.0))
        
        # Recompute pan offsets to preserve cursor anchor point
        new_screen_x = (old_world_pos.x * PIXELS_PER_METER * self.zoom) + self.offset_x
        new_screen_y = (old_world_pos.y * PIXELS_PER_METER * self.zoom) + self.offset_y
        self.offset_x += mouse_pos[0] - new_screen_x
        self.offset_y += mouse_pos[1] - new_screen_y
