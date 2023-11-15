import time

import numpy as np
import open3d as o3d # pip install open3d==0.17.0


class CollisionDetector():

    '''
    This is a class to detect collision of objects within cartesian coordinate space.
    '''

    def __init__(self, whd: list, pose: list):

        '''
        This is a CollisionDetector class object.

        Parameters
        --------------------
        whd : list of floats
            List object containing the (width, height, depth) of the bounding box to check for collisions.

        pose : list of list of float
            List containing the list [center_x, center_y, center_z] and...
            list contraining the list [i, j, k] describing the initial orientation of the bounding box.

        Returns
        --------------------
        None
        '''

        
        self.bbox_whd = np.array(whd)
        self.updatePose(pose)
        
        self.added_geometries = set()  # Initialize an empty set to track added geometries
        
    def updatePose(self, pose):
        
        self.bbox_pose = np.array(pose) 
        self.rot_mat = np.eye(3)
        self.bbox = o3d.geometry.OrientedBoundingBox(self.bbox_pose[0], self.rot_mat, self.bbox_whd)

        # Update the line (bbox_vector) to match the new pose
        self.bbox_vector = self.drawLine(self.bbox_pose[0], self.bbox_pose[1], 2, [1, 0, 1])
        
        if self.bbox_pose[1][0] != 0.0 or self.bbox_pose[1][1] != 0.0:
            rotation_matrix = self.alignVectors(np.array([0, 0, 1]), self.bbox_pose[1])
            self.bbox = self.rotateBoundingBox(rotation_matrix)

        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.6, origin=[0, 0, 0])

        # If the visualizer is already initialized, update its geometries
        if hasattr(self, 'vis'):
            self.update_visualizer()

    def rotateBoundingBox(self, rotation_matrix):

        '''
        This is a method to rotate the bounding box by a rotation matrix.

        Parameters
        --------------------
        rotation_matrix : array-like
            Rotation matrix.

        Returns
        ---------------
        None
        '''

        self.bbox.rotate(rotation_matrix)

        return self.bbox

    def alignVectors(self, z_axis, target_vector):
    
        '''
        This is a method to generate a rotation matrix that aligns the z-axis with a target vector.

        Parameters
        -------------------------
        z_axis : array-like
            The current Z-axis vector (3D numpy array).
        target_vector : array-like
            The target vector to align with (3D numpy array).

        Returns
        --------------------
        rotation_matrix : array-like
            The 3x3 rotation matrix.
        '''
        
        # Ensure that the input vectors are normalized
        z_axis = z_axis / np.linalg.norm(z_axis)
        target_vector = target_vector / np.linalg.norm(target_vector)

        # Compute the rotation axis and angle using the cross product and dot product
        rotation_axis = np.cross(z_axis, target_vector)
        rotation_angle = np.arccos(np.dot(z_axis, target_vector))

        # Normalize the rotation axis
        rotation_axis /= np.linalg.norm(rotation_axis)

        # Create the rotation matrix using the Rodrigues formula
        cos_theta = np.cos(rotation_angle)
        sin_theta = np.sin(rotation_angle)
        cross_matrix = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]])

        rotation_matrix = cos_theta * np.identity(3) + sin_theta * cross_matrix + (1 - cos_theta) * np.outer(rotation_axis, rotation_axis)

        return rotation_matrix

    def inVolume(self, point):

        '''
        This is method to check if a point is within the bbox.

        Parameters
        --------------------
        point : array-like
            Array of x,y,z. 
        Returns
        --------------------
        in_volume : bool
            Returns True if point is in bounding box.
            Returns False if point is NOT in bounding box.
        '''
        
        self.point = point
        point_check = np.linalg.inv(self.bbox.R).dot(point - self.bbox.center)

        # Check if the point is within half extents in all three dimensions
        within_x = (-self.bbox.extent[0] / 2 <= point_check[0] <= self.bbox.extent[0] / 2)
        within_y = (-self.bbox.extent[1] / 2 <= point_check[1] <= self.bbox.extent[1] / 2)
        within_z = (-self.bbox.extent[2] / 2 <= point_check[2] <= self.bbox.extent[2] / 2)
        self.in_volume = within_x and within_y and within_z

        if self.in_volume:
            self.point_geom = o3d.geometry.TriangleMesh.create_sphere(radius = 0.25, resolution = 100)
            self.point_geom.paint_uniform_color([1,0,0])
            self.point_geom.translate(self.point)
        else:
            self.point_geom = o3d.geometry.TriangleMesh.create_sphere(radius = 0.25, resolution = 100)
            self.point_geom.paint_uniform_color([0,0,0])
            self.point_geom.translate(self.point)

        return self.in_volume
    
    def drawLine(self, origin, orientation, length=1.0, color=[1, 0, 0]):
        
        '''
        This is a method to draw a line with a specified origin, orientation, length, and color.

        Parameters
        --------------------
        origin : array-like
            The starting point of the line [x, y, z].
        orientation : array-like
            The orientation vector of the line [i, j, k].
        length : float
            The length of the line (default is 1.0).
        color : array-like
            The color of the line as an RGB triplet (default is red).

        Returns
        --------------------
        line : open3d LineSet
            An open3d LineSet representing the drawn line.
        '''

        # Create a line set
        line = o3d.geometry.LineSet()

        # Define the vertices
        vertices = np.array([origin, origin + length * np.array(orientation)])

        # Set the points
        line.points = o3d.utility.Vector3dVector(vertices)

        # Set the lines
        lines = [[0, 1]]
        line.lines = o3d.utility.Vector2iVector(lines)

        # Set the colors
        line.colors = o3d.utility.Vector3dVector([color])

        return line
    
    def initialize_visualizer(self):
        
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.add_geometry_to_vis(self.coordinate_frame)
        self.add_geometry_to_vis(self.bbox)
        self.add_geometry_to_vis(self.bbox_vector)
        
    def add_geometry_to_vis(self, geometry):
        
        if geometry not in self.added_geometries:
            self.vis.add_geometry(geometry)
            self.added_geometries.add(geometry)

    def update_visualizer(self):
        
        # Update the coordinate frame
        self.vis.update_geometry(self.coordinate_frame)

        # Update the point geometry if it exists
        if hasattr(self, 'point_geom'):
            self.add_geometry_to_vis(self.point_geom)

        # Handle the bounding box: remove the old one and add the new one
        if 'old_bbox' in self.__dict__ and self.old_bbox in self.added_geometries:
            self.vis.remove_geometry(self.old_bbox, reset_bounding_box=False)
            self.added_geometries.remove(self.old_bbox)
        self.add_geometry_to_vis(self.bbox)

        # Handle the bbox_vector: remove the old one and add the new one
        if 'old_bbox_vector' in self.__dict__ and self.old_bbox_vector in self.added_geometries:
            self.vis.remove_geometry(self.old_bbox_vector, reset_bounding_box=False)
            self.added_geometries.remove(self.old_bbox_vector)
        self.add_geometry_to_vis(self.bbox_vector)

        # Store the current bounding box and bbox_vector as old for the next update
        self.old_bbox = self.bbox
        self.old_bbox_vector = self.bbox_vector

    def visualize_bbox(self):
        
        '''
        This is a method to visualize the simplified scene.
        Specifically a representation of a bounding box and a point.
        '''
        
        if not hasattr(self, 'vis'):
            self.initialize_visualizer()

        # Update the visualization
        self.update_visualizer()
        
        # Allow user to interact for a brief moment
        self.vis.poll_events()
        self.vis.update_renderer()
        time.sleep(2)  # Adjust this duration as needed

def main():
    
    # Define objects to check for collisions
    collided = False
    objects = [[0,0,1]]
    
    # Define poses for experiment
    poses = [[5,5,5], [3,3,3], [0,0,1]]
    orientations = [[0,0,1], [0,0,1], [0,0,1]]
    
    # Update the pose of the bounding box
    x = CollisionDetector([3,2,1], [poses[0], orientations[0]])
    for i in range(len(poses)):
        
        # First pose and orientation already defined
        if i == 0:
            
            # Loop through all objects and check
            for object in objects:
                collided = x.inVolume(object)
                x.visualize_bbox()
                if collided: break
                
        else:
            
            # Update pose and orientation
            x.updatePose([poses[i], orientations[i]])
            
            for object in objects:
                collided = x.inVolume(objects[0])
                x.visualize_bbox()
                if collided: break
        
    print(f'Collided: {collided}\n Pose: {poses[i]}\n Orientation: {orientations[i]}')

def demo():

    x = CollisionDetector([3,2,1], [[0,0,5], [0,0,1]])
    x.inVolume(np.array([0,0,5]))
    x.visualize_bbox()

if __name__ == "__main__":
    main()
    # demo()