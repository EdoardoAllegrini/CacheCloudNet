import os
import re
import math
import numpy as np
import xml.etree.ElementTree as ET

from typing import Callable
from launch_ros.actions import Node
from rosgraph_msgs.msg import Clock

WORLD_NAME = "iot_project_world"

def balloons_placement(n, field_width, field_height, sensor_range):
    if field_width < sensor_range * 2 or field_height < sensor_range * 2:
        raise ValueError("Field dimensions are too small for the sensor range.")

    center_x = field_width // 2
    center_y = field_height // 2

    # If there is only one balloon, place it at the center of the field
    if n == 1:
        return [(center_x, center_y, 0)]

    # Calculate the radius of the circumcircle for the regular polygon
    circumcircle_radius = sensor_range / (2 * math.sin(math.pi / n))

    # Calculate the positions of the balloons at the vertices of the regular polygon
    balloons_coords = []

    for i in range(n):
        angle = 2 * math.pi * i / n
        bx = round(center_x + circumcircle_radius * math.cos(angle))
        by = round(center_y + circumcircle_radius * math.sin(angle))

        balloons_coords.append((bx, by, 1.0))

    return balloons_coords


def compute_area_coverage(field_width, field_height, balloons_coords, sensor_range):
        # Generate a grid of points across the field
        x_points = np.linspace(0, field_width, 500)
        y_points = np.linspace(0, field_height, 500)
        grid_x, grid_y = np.meshgrid(x_points, y_points)

        # Initialize a grid to track coverage
        coverage_grid = np.zeros_like(grid_x, dtype=bool)

        # Check if each point in the grid is within the range of any balloon
        for (bx, by, _) in balloons_coords:
            coverage_grid |= (grid_x - bx)**2 + (grid_y - by)**2 <= sensor_range**2
        
        return coverage_grid, grid_x, grid_y


def sensors_placement(n, coverage_grid, grid_x, grid_y):
    sensor_coords = []
    sensor_indices = np.argwhere(coverage_grid)

    if len(sensor_indices) < n:
        raise ValueError("Not enough coverage to place all sensors.")

    # Randomly select n sensor positions from the covered area
    selected_indices = np.random.choice(len(sensor_indices), n, replace=False)
    for idx in selected_indices:
        sensor_x, sensor_y = sensor_indices[idx]
        # Round the coordinates
        rounded_x = round(grid_x[sensor_x, sensor_y])
        rounded_y = round(grid_y[sensor_x, sensor_y])
        sensor_coords.append((rounded_x, rounded_y, 0.0))

    return sensor_coords


def base_station_placement(field_width, field_height, offset=50, outside=True):
    # Place the base station at the center of the field
    bs_x = field_width // 2
    bs_y = field_height // 2
    bs_z = 0

    # Place the base station outside the field by the specified offset        
    if outside:
        bs_x = field_width + offset
        bs_y = field_height + offset

    return (bs_x, bs_y, bs_z)


def spawn_sdf(sdf_input, id: int = None, pos: tuple = (0, 0, 0), world_name: str = WORLD_NAME):
    """
    Function used to dynamically spawn a new sdf object at the start of the simulation.
    Use this to spawn multiple times the same object with different ids in your launch file.
    This function additionally modifies all the occurrences of the model name inside
    the SDF, keeping plugins such as OdometryPublisher consistent even without namespaces set.
    All occurrences of [MODEL_NAME] inside the sdf will be replaced with [MODEL_NAME]_[ID].
    A position can additionally be specified to choose where to place the given model.

        sdf_input:
            the path to the sdf_file to spawn (relative to pwd or absolute)
        id:
            the id of the new model, should be different by all the ids of object with the same sdf
        pos:
            the position where to spawn the model
        world_name:
            the name of the world to spawn the model to
    """
    with open(sdf_input) as sdf_file:
        sdf_string = sdf_file.read()
        model_name = None

        # use re to look for the model name and save it
        model_name_pattern = r"<model\s+name='([^']+)'"
        match = re.search(model_name_pattern, sdf_string)
        if match:
            model_name = match.group(1)

        if model_name is None:
            raise Exception(f"Error while trying to parse model name from file {sdf_input}")

        # replace all the occurrences of model_name with model_name_id,
        # and set the model name accordingly, if an id was given
        if id is None:
            model_name_with_id = model_name
        else:
            sdf_string = sdf_string.replace(model_name, f"{model_name}_{id}")
            model_name_with_id = f"{model_name} - {id}"

        return Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world',
                    world_name,
                    '-string',
                    sdf_string,
                    "-name",
                    model_name_with_id,
                    "-x",
                    str(pos[0]),
                    "-y",
                    str(pos[1]),
                    "-z",
                    str(pos[2])
                ],
                name='sim'
        )

def create_node(filepath: str, name: str, world: str, x: float = 0, y: float = 0, z: float = 0):
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File {filepath} not found.")

    try:
        with open(filepath, "r") as f:
            sdf = f.read()
    except IOError as e:
        raise IOError(f"Error reading file {filepath}: {e}")

    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world',
            world,
            '-string',
            sdf,
            '-name',
            name,
            '-x',
            str(x),
            '-y',
            str(y),
            '-z',
            str(z)
        ],
        name='sim'
    )

def create_multiple_entities(filepath: str, name: str, world: str, n: int, coords: list):
    if not all(isinstance(coord, tuple) for coord in coords):
        raise TypeError("All coordinates must be tuples.")
    
    if n != len(coords):
        raise ValueError("The number of coordinates must match the number of entities.")

    if not os.path.exists(filepath):
        raise FileNotFoundError(f"File {filepath} not found.")
    
    # Read and parse the SDF file
    tree = ET.parse(filepath)
    root_element = tree.getroot()
    nodes = []
    
    # Print initial XML tree
    ET.dump(root_element)
    
    for i, xyz in enumerate(coords):
        # Generate the code to not modify the original SDF file
        entity_root = ET.ElementTree(root_element)
        
        # Find the model element and modify its name
        model_element = entity_root.find(".//model[@name]")
        
        if model_element is None:
            raise Exception(f"Error while trying to parse model name from SDF string")

        model_name = model_element.get("name")

        assert model_name == name, "Model name in SDF file does not match the expected name."
        
        # Replace all occurrences of the model name with the new model name
        new_model_name = f"{model_name}_{i}"
        model_element.set("name", new_model_name)

        # Convert the modified XML tree back to a string
        sdf_string = ET.tostring(entity_root.getroot(), encoding="unicode")
        
        # Create a new node with the modified SDF string
        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world',
                    world,
                    '-string',
                    sdf_string,
                    '-name',
                    new_model_name,
                    '-x',
                    str(xyz[0]),
                    '-y',
                    str(xyz[1]),
                    '-z',
                    str(xyz[2])
                ],
                name='sim'
            )
        )

    return nodes


DEFAULT_SCHEDULER_RATE = 10


class EventScheduler():
    """
    Class used to schedule timed events following a ROS2 Clock interface.

        rate:
            specify the rate at which events are checked, set by default to 10 checks
            per simulation seconds
    """
    def __init__(self, rate=DEFAULT_SCHEDULER_RATE):
        self.current_time = 0
        self.rate = rate
        self.scheduled_events = []

    def routine(self, clock: Clock):
        """
        Main method to update the scheduler. This method should be called every time
        the simulation clock updates.

            clock:
                the new time in the simulation
        """
        new_time = clock.clock.sec + clock.clock.nanosec * 10**(-9)

        # Before running through events, check for rate. This allows
        # to run through all the events at a desired frequency, without
        # wasting resources
        if new_time - self.current_time > 1/self.rate:
            self.current_time = new_time
        else:
            return

        # Loop through all the scheduled events
        for event in self.scheduled_events:
            # If the event time has passed, execute it
            if event['time'] + event['scheduled_time'] < self.current_time:
                event['callback'](*event['args'])

                if event['repeat']:
                    event['scheduled_time'] = self.current_time
                else:
                    self.scheduled_events.remove(event)

    # Schedule a new event in the scheduler.
    def schedule_event(self, time: float, callback: Callable, repeat=True, args: list = []):
        """
        Schedule a new event in the EvenScheduler.

            time:
                timeout to the event in seconds
            callback:
                the function to execute when the time expires
            repeat:
                if True, the event is repeated at the frequency given by time
            args:
                argument to pass to the callback function
        """

        new_event = {
            'time': time,
            'callback': callback,
            'repeat': repeat,
            'args': args,
            'scheduled_time': self.current_time,
        }

        self.scheduled_events.append(new_event)
