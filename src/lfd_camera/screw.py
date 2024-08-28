import numpy as np
import matplotlib.pyplot as plt
import csv

class ObjectTray:
    def __init__(self, csv_file_path, geom_bbox, clearance_bbox):
        """
        Initializes the ObjectTray class with object locations and bounding boxes.

        Parameters:
        - csv_file_path: Path to the CSV file containing x, y, and angle values.
        - geom_bbox: Tuple with (x_min, x_max, y_min, y_max) for the object's geometric bounding box.
        - clearance_bbox: Tuple with (x_min, x_max, y_min, y_max) for the object's clearance bounding box.
        """
        self.csv_file_path = csv_file_path
        self.geom_bbox = self.create_bounding_box(geom_bbox[0],geom_bbox[1],geom_bbox[2], geom_bbox[3]) 
        self.clearance_bbox = self.create_bounding_box(clearance_bbox[0],clearance_bbox[1],clearance_bbox[2], clearance_bbox[3]) 
        self.objects = [] 
        
        # Load the object data from CSV
        self.load_data()

    def load_data(self):
        """
        Loads object data from the CSV file and stores it in the objects list.
        Each object is stored as a dictionary with keys: 'x', 'y', 'angle'.
        """
        try:
            with open(self.csv_file_path, mode='r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    # Parse the x, y, and angle values from the CSV
                    obj_data = {
                        'x': float(row['x']),
                        'y': float(row['y']),
                        'angle': float(row['angle']),
                        'geom_bbox': self.transform_bounding_box(self.geom_bbox, float(row['x']), float(row['y']), float(row['angle'])),
                        'clearance_bbox': self.transform_bounding_box(self.clearance_bbox, float(row['x']), float(row['y']), float(row['angle']))
                    }
                    self.objects.append(obj_data)
        except Exception as e:
            print(f"Error loading data from CSV: {e}")

    def create_bounding_box(self, x_min, x_max, y_min, y_max):
        """
        Returns the corner points of a bounding box given the edges.
        
        Parameters:
        - x_min: Minimum x value (left edge of the bounding box).
        - x_max: Maximum x value (right edge of the bounding box).
        - y_min: Minimum y value (bottom edge of the bounding box).
        - y_max: Maximum y value (top edge of the bounding box).
        
        Returns:
        - A list of 4 points representing the corners of the bounding box.
        Format: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        """
        
        # Define the four corners of the bounding box
        bottom_left = (x_min, y_min)
        bottom_right = (x_max, y_min)
        top_right = (x_max, y_max)
        top_left = (x_min, y_max)
        
        # Return the bounding box as a list of 4 points
        return [bottom_left, bottom_right, top_right, top_left]

    def transform_bounding_box(self, bbox, x_translation, y_translation, angle):
        """
        Transforms a 2D bounding box by applying rotation first and then translation.
        
        Parameters:
        - bbox: A list or array of 4 points (x, y) representing the bounding box. 
                Format: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        - x_translation: Translation value along the x-axis.
        - y_translation: Translation value along the y-axis.
        - angle: Rotation angle in degrees.
        
        Returns:
        - A transformed list of bounding box points after applying rotation and translation.
        """
        # Convert angle to radians
        angle_rad = np.radians(angle)

        # Create the rotation matrix
        rotation_matrix = np.array([
            [np.cos(angle_rad), -np.sin(angle_rad)],
            [np.sin(angle_rad),  np.cos(angle_rad)]
        ])
        
        # Apply rotation to each point first
        rotated_bbox = []
        for point in bbox:
            rotated_point = np.dot(rotation_matrix, point)
            rotated_bbox.append(rotated_point)
        
        # Apply translation to the rotated points
        translated_bbox = []
        for point in rotated_bbox:
            translated_point = np.array([point[0] + x_translation, point[1] + y_translation])
            translated_bbox.append(translated_point)
        
        return translated_bbox

    def do_bounding_boxes_overlap(self, bbox1, bbox2):
        """
        Checks if two 2D axis-aligned bounding boxes overlap.
        
        Parameters:
        - bbox1: List of 4 points (x, y) representing the first bounding box.
                Format: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        - bbox2: List of 4 points (x, y) representing the second bounding box.
                Format: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
        
        Returns:
        - True if the bounding boxes overlap, False otherwise.
        """
        
        # Extract min and max coordinates for each bounding box
        x1_min, x1_max = min(point[0] for point in bbox1), max(point[0] for point in bbox1)
        y1_min, y1_max = min(point[1] for point in bbox1), max(point[1] for point in bbox1)
        
        x2_min, x2_max = min(point[0] for point in bbox2), max(point[0] for point in bbox2)
        y2_min, y2_max = min(point[1] for point in bbox2), max(point[1] for point in bbox2)
        
        # Check for no overlap conditions:
        if x1_max < x2_min or x2_max < x1_min:
            return False  # Separated along the x-axis
        if y1_max < y2_min or y2_max < y1_min:
            return False  # Separated along the y-axis
        
        # If neither condition is met, the bounding boxes overlap
        return True
    
    def find_non_overlapping_object(self):
        """
        Loops through all the objects and finds the object whose clearance bounding box does not overlap
        with the geometric bounding box of any other objects.
        
        Returns:
        - The object whose clearance bounding box does not overlap with any other objects,
          or None if no such object is found.
        """
        for i, obj1 in enumerate(self.objects):
            is_overlapping = False
            for j, obj2 in enumerate(self.objects):
                if i != j:
                    if self.do_bounding_boxes_overlap(obj1['clearance_bbox'], obj2['geom_bbox']):
                        is_overlapping = True
                        break
            if not is_overlapping:
                return obj1
        return None