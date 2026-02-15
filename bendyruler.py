# bendyruler.py
import math
import sys
from config import *

class Vector2:
    """Small 2D vector helper used by simulation and planner logic."""
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def length(self):
        return math.hypot(self.x, self.y)

    def length_squared(self):
        return self.x**2 + self.y**2

    def normalize(self):
        l = self.length()
        if l == 0: return Vector2(0, 0)
        return Vector2(self.x / l, self.y / l)

class AP_OABendyRuler:
    """2D adaptation of ArduPilot's BendyRuler obstacle avoidance strategy."""
    def __init__(self):
        # Tunable planner parameters
        self._lookahead = OA_BENDYRULER_LOOKAHEAD_DEFAULT
        self._bendy_ratio = OA_BENDYRULER_RATIO_DEFAULT
        self._bendy_angle = OA_BENDYRULER_ANGLE_DEFAULT
        self._margin_max = OA_MARGIN_MAX
        
        # Planner memory across frames
        self._current_lookahead = self._lookahead
        self._bearing_prev = sys.float_info.max
        self._destination_prev = Vector2(-1000, -1000)  # Arbitrary unlikely position

    def get_bearing_to(self, origin, dest):
        """Returns bearing in degrees from origin to dest"""
        dx = dest.x - origin.x
        dy = dest.y - origin.y # In standard math +Y is up, but screens are +Y down. 
                               # We treat this as a standard cartesian plane for math.
        return math.degrees(math.atan2(dy, dx))

    def get_distance(self, loc1, loc2):
        return math.hypot(loc1.x - loc2.x, loc1.y - loc2.y)

    def offset_bearing(self, origin, bearing_deg, distance):
        rad = math.radians(bearing_deg)
        new_x = origin.x + math.cos(rad) * distance
        new_y = origin.y + math.sin(rad) * distance
        return Vector2(new_x, new_y)

    def calc_avoidance_margin(self, start, end, obstacles):
        """
        Calculates minimum distance between a segment (start->end) and any obstacle.
        Obstacles are expected to be a list of dicts: {'pos': Vector2, 'radius': float}
        """
        margin_min = sys.float_info.max

        if not obstacles:
            return margin_min

        # Convert segment to vector
        seg_v = end - start
        seg_len_sq = seg_v.length_squared()

        for obs in obstacles:
            obs_pos = obs['pos']
            obs_radius = obs['radius']

            # Math for closest point on segment to obstacle center
            if seg_len_sq <= 0:
                dist_to_line = (obs_pos - start).length()
            else:
                t = ((obs_pos.x - start.x) * seg_v.x + (obs_pos.y - start.y) * seg_v.y) / seg_len_sq
                t = max(0, min(1, t))
                closest_point = start + Vector2(seg_v.x * t, seg_v.y * t)
                dist_to_line = (obs_pos - closest_point).length()

            # Margin is distance minus radius (how much clearance we have)
            margin = dist_to_line - obs_radius
            if margin < margin_min:
                margin_min = margin

        return margin_min

    def resist_bearing_change(self, destination, current_loc, active, bearing_test, 
                              lookahead_step1_dist, margin, proximity_only, obstacles):
        """Dampen sudden heading changes when a previous direction is still acceptable."""
        resisted_change = False
        dest_change = False
        
        # Check if destination changed (simple float comparison tolerance)
        if self.get_distance(destination, self._destination_prev) > 0.1:
            dest_change = True
            self._destination_prev = destination

        if active and not dest_change and self._bendy_ratio > 0:
            # Check difference between fresh bearing and previous
            if abs(wrap_180(self._bearing_prev - bearing_test)) > self._bendy_angle and self._bearing_prev != sys.float_info.max:
                
                # Check margin in last bearing's direction
                test_loc_prev = self.offset_bearing(current_loc, wrap_180(self._bearing_prev), lookahead_step1_dist)
                prev_margin = self.calc_avoidance_margin(current_loc, test_loc_prev, obstacles)

                if margin < (self._bendy_ratio * prev_margin):
                    # Don't change direction abruptly
                    return True, self._bearing_prev, prev_margin

        else:
            self._bearing_prev = sys.float_info.max
        
        if not resisted_change:
            self._bearing_prev = bearing_test
            
        return False, bearing_test, margin

    def update(self, current_loc, destination, ground_speed_vec, obstacles):
        """
        Main update loop. 
        Returns: (active, destination_new)
        """
        # BendyRuler evaluates paths from the vehicle's current position.
        
        bearing_to_dest = self.get_bearing_to(current_loc, destination)
        distance_to_dest = self.get_distance(current_loc, destination)

        # Ensure meaningful lookahead
        self._lookahead = max(self._lookahead, 1.0)

        # Dynamic lookahead adjustment
        self._current_lookahead = constrain_float(self._current_lookahead, self._lookahead * 0.5, self._lookahead)

        # Step 1 lookahead
        lookahead_step1_dist = min(self._current_lookahead, distance_to_dest + OA_BENDYRULER_LOOKAHEAD_PAST_DEST)

        # Step 2 lookahead
        lookahead_step2_dist = self._current_lookahead * OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO

        # Ground course
        if ground_speed_vec.length_squared() < OA_BENDYRULER_LOW_SPEED_SQUARED:
            # If no speed, assume facing destination for simplicity in this 2D sim, 
            # or use last known yaw. Here we use bearing to dest as default.
            ground_course_deg = bearing_to_dest
        else:
            ground_course_deg = math.degrees(math.atan2(ground_speed_vec.y, ground_speed_vec.x))

        return self.search_xy_path(current_loc, destination, ground_course_deg, 
                                   lookahead_step1_dist, lookahead_step2_dist, 
                                   bearing_to_dest, distance_to_dest, obstacles)

    def search_xy_path(self, current_loc, destination, ground_course_deg, 
                       lookahead_step1_dist, lookahead_step2_dist, 
                       bearing_to_dest, distance_to_dest, obstacles):
        """Search candidate bearings and choose the safest near-feasible waypoint."""
        # Best-candidate tracking across all tested bearings
        best_bearing = bearing_to_dest
        best_bearing_margin = -sys.float_info.max
        have_best_bearing = False
        best_margin = -sys.float_info.max
        best_margin_bearing = best_bearing

        iterations = int(170 / OA_BENDYRULER_BEARING_INC_XY)

        # Evaluate symmetric angular offsets around direct bearing to destination
        for i in range(iterations + 1):
            for bdir in range(2):
                # Skip duplicate 0 check
                if i == 0 and bdir > 0:
                    continue
                
                bearing_delta = i * OA_BENDYRULER_BEARING_INC_XY * (-1.0 if bdir == 0 else 1.0)
                bearing_test = wrap_180(bearing_to_dest + bearing_delta)

                test_loc = self.offset_bearing(current_loc, bearing_test, lookahead_step1_dist)
                
                margin = self.calc_avoidance_margin(current_loc, test_loc, obstacles)

                if margin > best_margin:
                    best_margin_bearing = bearing_test
                    best_margin = margin
                
                if margin > self._margin_max:
                    # Step 1 path segment is clear
                    if not have_best_bearing:
                        best_bearing = bearing_test
                        best_bearing_margin = margin
                        have_best_bearing = True
                    elif abs(wrap_180(ground_course_deg - bearing_test)) < abs(wrap_180(ground_course_deg - best_bearing)):
                        # Prefer bearing closer to current course
                        best_bearing = bearing_test
                        best_bearing_margin = margin
                    
                    # Step 2 checks whether we can continue from test_loc toward destination
                    test_bearings = [0.0, 45.0, -45.0]
                    bearing_to_dest2 = self.get_bearing_to(test_loc, destination)
                    dist_from_test_to_dest = self.get_distance(test_loc, destination)
                    
                    distance2 = constrain_float(lookahead_step2_dist, OA_BENDYRULER_LOOKAHEAD_STEP2_MIN, dist_from_test_to_dest)

                    for j in range(len(test_bearings)):
                        bearing_test2 = wrap_180(bearing_to_dest2 + test_bearings[j])
                        test_loc2 = self.offset_bearing(test_loc, bearing_test2, distance2)

                        margin2 = self.calc_avoidance_margin(test_loc, test_loc2, obstacles)

                        if margin2 > self._margin_max:
                            # Both stages clear; OA is active only if we deviated from straight path
                            active = (i != 0 or j != 0)
                            
                            final_bearing = bearing_test
                            final_margin = margin

                            # Resist bearing change logic
                            resisted, final_bearing, final_margin = self.resist_bearing_change(
                                destination, current_loc, active, bearing_test, 
                                lookahead_step1_dist, margin, False, obstacles
                            )

                            # Create new destination point
                            dest_new = self.offset_bearing(current_loc, final_bearing, min(distance_to_dest, lookahead_step1_dist))
                            
                            # Grow lookahead when planner successfully finds viable path
                            self._current_lookahead = min(self._lookahead, self._current_lookahead * 1.1)
                            
                            return dest_new

        # Fallback when no full two-stage path is clear: choose least-bad candidate
        chosen_bearing = 0.0
        chosen_distance = 0.0

        if have_best_bearing:
            chosen_bearing = best_bearing
            chosen_distance = max(lookahead_step1_dist + min(best_bearing_margin, 0), 0)
            self._current_lookahead = min(self._lookahead, self._current_lookahead * 1.05)
        else:
            chosen_bearing = best_margin_bearing
            chosen_distance = max(lookahead_step1_dist + min(best_margin, 0), 0)
            self._current_lookahead = max(self._lookahead * 0.5, self._current_lookahead * 0.9)
        
        dest_new = self.offset_bearing(current_loc, chosen_bearing, chosen_distance)
        return dest_new
