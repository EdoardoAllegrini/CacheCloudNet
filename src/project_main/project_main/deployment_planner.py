import math
import numpy as np

# UNUSED (DEPRECATED)

def get_random_points_in_coverage(n_points, coverage, X, Y):
    # Get the indices of the points that are covered
    covered_indices = np.argwhere(coverage)
    
    # Randomly select 5 indices from the covered indices
    selected_indices = covered_indices[np.random.choice(covered_indices.shape[0], n_points, replace=False)]

    # Get the coordinates of the selected points
    random_points = [(round(X[i, j]), round(Y[i, j]), 0) for i, j in selected_indices]
    
    return random_points


class DeploymentPlanner():

    """
    Class to compute automatically the best initial placement of:
        - Base Station, 
        - Balloons, 
        - Sensors (random but ensuring each sensor is covered by at least one balloon).
    """

    def __init__(self, num_balloons: int, num_sensors: int, sensor_range: int) -> None:
        self.num_balloons = num_balloons
        self.num_sensors = num_sensors
        self.sensor_range = sensor_range
        self.coverage = None

    def compute(self, map_width, map_height):

        """
        Method used to compute automatically Base Station placement, Balloons placement and initial Sensors random placement.
        """

        base_station_position = (map_width // 2, map_height // 2, 0)
        balloons_position = self.place_balloons(map_width, map_height)
        sensors_position = get_random_points_in_coverage(self.num_sensors, self.coverage, self.X, self.Y)

        return base_station_position, balloons_position, sensors_position, self.coverage, self.X, self.Y

    def place_balloons(self, map_width, map_height):
        if map_height < self.sensor_range * 2 or map_width < self.sensor_range * 2:
            raise Exception("Map dimension wrong")
        
        # Calculate the radius of the circumcircle for the regular polygon
        if self.num_balloons == 1:
            return [(map_width / 2, map_height / 2)]
        
        # Circumcircle radius for a regular polygon with self.num_balloons sides
        circumcircle_radius = self.sensor_range / (2 * math.sin(math.pi / self.num_balloons))
        
        # List to store balloon positions
        balloon_positions = []
        
        # Calculate the center of the map
        center_x = map_width // 2
        center_y = map_height // 2
        
        # Calculate the positions of the balloons at the vertices of the regular polygon
        for i in range(self.num_balloons):
            angle = 2 * math.pi * i / self.num_balloons
            x = round(center_x + circumcircle_radius * math.cos(angle))
            y = round(center_y + circumcircle_radius * math.sin(angle))
            balloon_positions.append((x, y, 1))

        x = np.linspace(0, map_width, 500)
        y = np.linspace(0, map_height, 500)
        self.X, self.Y = np.meshgrid(x, y)
        self.coverage = np.zeros_like(self.X, dtype=bool)

        # Check if each point in the grid is within the range of any balloon
        for (bx, by, _) in balloon_positions:
            self.coverage |= (self.X - bx)**2 + (self.Y - by)**2 <= self.sensor_range**2

        # Return the list of balloon positions
        return balloon_positions
