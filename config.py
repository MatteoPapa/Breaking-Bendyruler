import math

# --- Simulation Settings ---
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 720
FPS = 60
PIXELS_PER_METER = 20.0

# --- Colors ---
BACKGROUND_COLOR = (30, 30, 30)
GRID_COLOR = (50, 50, 50)
TEXT_COLOR = (220, 220, 220)

DRONE_COLOR = (255, 69, 58)        
TARGET_COLOR = (50, 215, 75)       
OBSTACLE_COLOR = (90, 200, 250)    
OBSTACLE_OUTLINE = (255, 255, 255)

PATH_DIRECT_COLOR = (0, 100, 0)    
PATH_PROBE_COLOR = (255, 149, 0)   

MARGIN_COLOR_RGB = (255, 50, 50) 

# --- BendyRuler Constants (Matching ArduPilot defaults) ---
OA_BENDYRULER_LOOKAHEAD_DEFAULT = 15.0
OA_BENDYRULER_RATIO_DEFAULT = 1.5
OA_BENDYRULER_ANGLE_DEFAULT = 75.0
OA_BENDYRULER_BEARING_INC_XY = 5
OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO = 1.0
OA_BENDYRULER_LOOKAHEAD_STEP2_MIN = 2.0
OA_BENDYRULER_LOOKAHEAD_PAST_DEST = 2.0
OA_BENDYRULER_LOW_SPEED_SQUARED = (0.2 * 0.2)
OA_MARGIN_MAX = 2.0

# --- Helpers ---
def constrain_float(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def wrap_180(angle_deg):
    while angle_deg > 180:
        angle_deg -= 360
    while angle_deg < -180:
        angle_deg += 360
    return angle_deg
