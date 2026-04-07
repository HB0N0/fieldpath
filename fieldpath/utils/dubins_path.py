from shapely import LineString
from . import geom
from fieldpath.dep import dubins
import numpy as np

class DubinsPath:
    def __init__(self, turn_radius: float, step_size: float = 0.05):
        """New DubinsPath instance.
        
        :param turn_radius: Minimum turning radius of the vehicle
        """
        self.turn_radius = turn_radius
        self.step_size = step_size
        self.start = (0.0, 0.0, 0.0)  # (x, y, heading in degrees)
        self.end = (0.0, 0.0, 0.0)
        if self.turn_radius > 0:
            self.dubins = dubins.Dubins(turn_radius, step_size)

    def set_start(self, start):
        """Set start position from tuple (x, y, heading_deg) or LineString."""
        if isinstance(start, tuple) and len(start) == 3:
            self.start = start
        elif isinstance(start, LineString):
            pos = geom.pose_from_linestring(start, geom.LINE_POSITION_END)
            if pos:
                self.start = pos

    def set_end(self, end):
        """Set end position from tuple (x, y, heading_deg) or LineString."""
        if isinstance(end, tuple) and len(end) == 3:
            self.end = end
        elif isinstance(end, LineString):
            pos = geom.pose_from_linestring(end, geom.LINE_POSITION_START)
            if pos:
                self.end = pos

    def generate_path(self, start = None, end = None) -> LineString:
        """
        Generate the shortest Dubins path from start to end configuration.
        
        :return: LineString representing the Dubins path
        """
        if start:
            self.set_start(start)
        if end:
            self.set_end(end)

        # If no turn radius, return straight line
        if self.turn_radius <= 0:
            return LineString([self.start[:2], self.end[:2]])
        
        start_pose = (self.start[0], self.start[1], np.radians(self.start[2]))
        end_pose = (self.end[0], self.end[1], np.radians(self.end[2]))
        
        # Check distance between start and end
        # distance = dubins.dist(start_pose, end_pose)
        # pose_angle = start_pose[2] - end_pose[2]
        # if distance < (2 * self.turn_radius):
        #     print("DubinsPath: Start and end too close for dubins path.")
        #     # return self._single_arc_turn(start_pose, end_pose)
        #     # Too close for dubins path, turn in place

        dubins_path = self.dubins.dubins_path(start_pose, end_pose)
        
        return LineString(dubins_path)
    
    def _single_arc_turn(self, start_pose, end_pose) -> LineString:
        """
        Generate a path that turns the robot while start and end coordinates are the same, 
        using the turn radius

        1) drive straight for turn radius
        2) turn in an arc to reach the desired heading
        3) drive straight for turn radius

        :param start_pose: Tuple(x, y, heading_rad)
        :param end_pose: Tuple(x, y, heading_rad)
        :return: LineString representing single arc turn
        """
        path_points = []
        # Step 1: drive straight for turn radius
        x0, y0, heading0 = start_pose
        x_straight = x0 + self.turn_radius * np.cos(heading0)
        y_straight = y0 + self.turn_radius * np.sin(heading0)
        path_points.append((x0, y0))
        path_points.append((x_straight, y_straight))
        # Step 2: turn in an arc to reach the desired heading
        heading_diff = end_pose[2] - start_pose[2]

        num_steps = int(abs(heading_diff) / self.step_size) + 1
        for step in range(1, num_steps + 1):
            fraction = step / num_steps
            heading = heading0 + fraction * heading_diff
            x_arc = x_straight + self.turn_radius * np.cos(heading + np.pi/2 * np.sign(heading_diff))
            y_arc = y_straight + self.turn_radius * np.sin(heading + np.pi/2 * np.sign(heading_diff))
            path_points.append((x_arc, y_arc))
        # Step 3: drive straight for turn radius to end point
        x_end, y_end, heading_end = end_pose
        x_final = x_end - self.turn_radius * np.cos(heading_end)
        y_final = y_end - self.turn_radius * np.sin(heading_end)
        path_points.append((x_final, y_final))
        path_points.append((x_end, y_end))

        return LineString(path_points)


