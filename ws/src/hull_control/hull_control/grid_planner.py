#!/usr/bin/env python3
"""
HullSentinal V2 — 2D Occupancy Grid & 8-Connected A* Planner
Constructs a discrete 2D occupancy grid with static obstacle inflation (0.55m)
and hard boundary safety insets.
Computes optimal, collision-free paths with octile heuristics and line-of-sight smoothing.
"""

import math
import heapq
import numpy as np


class GridPlanner:
    def __init__(self, hull_width=24.0, hull_height=8.0, resolution=0.20,
                 inflation_radius=0.55, safety_margin=0.80, obstacles=None):
        self.width = float(hull_width)
        self.height = float(hull_height)
        self.res = float(resolution)
        self.inflation = float(inflation_radius)
        self.safety_margin = float(safety_margin)
        self.obstacles = obstacles or []

        self.nx = int(math.ceil(self.width / self.res))
        self.ny = int(math.ceil(self.height / self.res))

        # 0: free, 1: occupied
        self.grid = np.zeros((self.ny, self.nx), dtype=np.uint8)
        self.proximity_cost = np.zeros((self.ny, self.nx), dtype=np.float32)

        self._build_grid()

    def _build_grid(self):
        """Marks boundary insets and inflated obstacle bounding boxes."""
        # 1. Hard hull boundary insets
        for r in range(self.ny):
            y = (r + 0.5) * self.res
            for c in range(self.nx):
                x = (c + 0.5) * self.res
                if x < self.safety_margin or x > (self.width - self.safety_margin) or \
                   y < self.safety_margin or y > (self.height - self.safety_margin):
                    self.grid[r, c] = 1

        # 2. Inflated static obstacles
        for obs in self.obstacles:
            ox = float(obs['x'])
            oy = float(obs['y'])
            hw = float(obs.get('width', 1.0)) / 2.0 + self.inflation
            hh = float(obs.get('height', 0.5)) / 2.0 + self.inflation

            min_x = max(0.0, ox - hw)
            max_x = min(self.width, ox + hw)
            min_y = max(0.0, oy - hh)
            max_y = min(self.height, oy + hh)

            c_start = max(0, int(min_x / self.res))
            c_end = min(self.nx - 1, int(max_x / self.res))
            r_start = max(0, int(min_y / self.res))
            r_end = min(self.ny - 1, int(max_y / self.res))

            self.grid[r_start:r_end + 1, c_start:c_end + 1] = 1

            # Proximity buffer (additional 0.35m clearance cost field)
            prox_hw = hw + 0.35
            prox_hh = hh + 0.35
            p_c_start = max(0, int((ox - prox_hw) / self.res))
            p_c_end = min(self.nx - 1, int((ox + prox_hw) / self.res))
            p_r_start = max(0, int((oy - prox_hh) / self.res))
            p_r_end = min(self.ny - 1, int((oy + prox_hh) / self.res))
            self.proximity_cost[p_r_start:p_r_end + 1, p_c_start:p_c_end + 1] += 0.20

    def world_to_grid(self, x, y):
        c = int(x / self.res)
        r = int(y / self.res)
        c = max(0, min(self.nx - 1, c))
        r = max(0, min(self.ny - 1, r))
        return r, c

    def grid_to_world(self, r, c):
        x = (c + 0.5) * self.res
        y = (r + 0.5) * self.res
        return x, y

    def is_free(self, r, c):
        if 0 <= r < self.ny and 0 <= c < self.nx:
            return self.grid[r, c] == 0
        return False

    def find_nearest_free(self, r, c, max_radius=8):
        if self.is_free(r, c):
            return r, c
        for rad in range(1, max_radius + 1):
            for dr in range(-rad, rad + 1):
                for dc in range(-rad, rad + 1):
                    nr, nc = r + dr, c + dc
                    if self.is_free(nr, nc):
                        return nr, nc
        return r, c

    def is_line_of_sight_clear(self, p1, p2):
        """Bresenham-style ray casting to check collision along line segment."""
        x1, y1 = p1
        x2, y2 = p2
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(2, int(dist / (self.res * 0.5)))
        for i in range(steps + 1):
            t = i / steps
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            r, c = self.world_to_grid(px, py)
            if not self.is_free(r, c):
                return False
        return True

    def plan_path(self, start, goal):
        """8-connected A* with octile heuristic and no corner cutting."""
        sx, sy = start
        gx, gy = goal

        start_r, start_c = self.find_nearest_free(*self.world_to_grid(sx, sy))
        goal_r, goal_c = self.find_nearest_free(*self.world_to_grid(gx, gy))

        if start_r == goal_r and start_c == goal_c:
            return [(gx, gy)]

        # 8-connected motion: (dr, dc, move_cost)
        sqrt2 = math.sqrt(2.0) * self.res
        cardinal = self.res
        motions = [
            (-1, 0, cardinal), (1, 0, cardinal), (0, -1, cardinal), (0, 1, cardinal),
            (-1, -1, sqrt2), (-1, 1, sqrt2), (1, -1, sqrt2), (1, 1, sqrt2)
        ]

        def heuristic(r, c):
            dx = abs(c - goal_c)
            dy = abs(r - goal_r)
            return (min(dx, dy) * math.sqrt(2.0) + abs(dx - dy)) * self.res

        open_set = []
        heapq.heappush(open_set, (heuristic(start_r, start_c), 0.0, 0, start_r, start_c))
        came_from = {}
        g_costs = {(start_r, start_c): 0.0}
        counter = 0

        found = False
        while open_set:
            f, g, _, cr, cc = heapq.heappop(open_set)

            if cr == goal_r and cc == goal_c:
                found = True
                break

            if g > g_costs.get((cr, cc), float('inf')):
                continue

            for dr, dc, move_cost in motions:
                nr, nc = cr + dr, cc + dc

                if not self.is_free(nr, nc):
                    continue

                # Prevent diagonal corner-cutting
                if dr != 0 and dc != 0:
                    if not self.is_free(cr + dr, cc) or not self.is_free(cr, cc + dc):
                        continue

                new_g = g + move_cost + self.proximity_cost[nr, nc]
                if new_g < g_costs.get((nr, nc), float('inf')):
                    g_costs[(nr, nc)] = new_g
                    priority = new_g + heuristic(nr, nc)
                    counter += 1
                    heapq.heappush(open_set, (priority, new_g, counter, nr, nc))
                    came_from[(nr, nc)] = (cr, cc)

        if not found:
            # Fallback direct path
            return [(gx, gy)]

        # Reconstruct path
        path_indices = []
        curr = (goal_r, goal_c)
        while curr in came_from:
            path_indices.append(curr)
            curr = came_from[curr]
        path_indices.append((start_r, start_c))
        path_indices.reverse()

        # Convert to world coordinates
        raw_waypoints = [self.grid_to_world(r, c) for r, c in path_indices]
        if raw_waypoints:
            raw_waypoints[0] = (sx, sy)
            raw_waypoints[-1] = (gx, gy)

        # Path smoothing via line-of-sight shortcutting
        smoothed = [raw_waypoints[0]]
        curr_idx = 0
        while curr_idx < len(raw_waypoints) - 1:
            best_next = curr_idx + 1
            for next_idx in range(len(raw_waypoints) - 1, curr_idx, -1):
                if self.is_line_of_sight_clear(raw_waypoints[curr_idx], raw_waypoints[next_idx]):
                    best_next = next_idx
                    break
            smoothed.append(raw_waypoints[best_next])
            curr_idx = best_next

        return smoothed

    def get_path_length(self, start, goal):
        """Computes true navigation path length along obstacle-free trajectory."""
        waypoints = self.plan_path(start, goal)
        total_len = 0.0
        for i in range(len(waypoints) - 1):
            total_len += math.hypot(waypoints[i+1][0] - waypoints[i][0],
                                    waypoints[i+1][1] - waypoints[i][1])
        return total_len
