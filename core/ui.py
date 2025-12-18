import pygame
from config import TEXT_COLOR

class Button:
    def __init__(self, x, y, w, h, text, callback):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback
        self.color = (60, 60, 60)
        self.hover_color = (80, 80, 80)
        self.font = pygame.font.SysFont("Arial", 16, bold=True)

    def draw(self, surface, mouse_pos):
        col = self.hover_color if self.rect.collidepoint(mouse_pos) else self.color
        pygame.draw.rect(surface, col, self.rect, border_radius=5)
        pygame.draw.rect(surface, (150, 150, 150), self.rect, 2, border_radius=5)
        txt_surf = self.font.render(self.text, True, TEXT_COLOR)
        txt_rect = txt_surf.get_rect(center=self.rect.center)
        surface.blit(txt_surf, txt_rect)

    def check_click(self, mouse_pos):
        if self.rect.collidepoint(mouse_pos):
            self.callback()
            return True
        return False

def draw_hud(screen, mode, drone):
    font = pygame.font.SysFont("Consolas", 14)
    lookahead = f"{drone.bendy._current_lookahead:.1f}m" if drone else "N/A"
    
    info_lines = [
        f"Mode: {mode}",
        f"Lookahead: {lookahead}",
        "----------------",
        "Left Click: Set Destination",
        "Shift+Click: Set Hijack Point",
        "Right Click: +/- Obstacle",
        "R: Reset"
    ]
    
    for i, line in enumerate(info_lines):
        screen.blit(font.render(line, True, TEXT_COLOR), (10, 10 + i * 18))
