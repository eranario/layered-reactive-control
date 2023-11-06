import numpy as np
import cv2

# Define the 3D coordinates of points in the cube (assuming cube center is at the origin).
s = 50  # Side length of the cube
cube_points = np.float32([
    [s, s, -s],
    [-s, s, -s],
    [-s, -s, -s],
    [s, -s, -s],
    [s, s, s],
    [-s, s, s],
    [-s, -s, s],
    [s, -s, s]
])

# Camera matrix (assuming focal length is 1 and the image center is at (0,0) in camera coords)
focal_length = 800 # in pixels
center = (400, 400)
camera_matrix = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype=np.float32)

# Assuming no lens distortion
dist_coeffs = np.zeros((4, 1))

# Rotation vector and Translation vector
# rotation_vector = np.array([35, 45, 0], dtype=np.float32)
# translation_vector = np.array([0, 0, 3 * s], dtype=np.float32)

rotation_vector = np.array([0, 0, 0], dtype=np.float32)
translation_vector = np.array([0, 0, 5*s], dtype=np.float32)

# Project the 3D points to 2D using OpenCV function
projected_points, _ = cv2.projectPoints(cube_points, rotation_vector, translation_vector, camera_matrix, dist_coeffs)

# Convert points into a format that OpenCV expects
projected_points = projected_points.reshape(-1, 2)

# Create an empty image
image_size = (800, 800)
image = np.zeros((image_size[1], image_size[0], 3), dtype=np.uint8)

# List of sides as tuples of indices into the projected_points array
sides = [
    (0, 1, 2, 3),  # Back face
    (4, 5, 6, 7),  # Front face
    (0, 1, 5, 4),  # Top face
    (2, 3, 7, 6),  # Bottom face
    (0, 3, 7, 4),  # Right face
    (1, 2, 6, 5),  # Left face
]

# Sort the faces by the average Z value of the points in each face
sorted_faces = sorted(sides, key=lambda idx: np.mean([cube_points[i][2] for i in idx]), reverse=True)

# Draw the faces
for face in sorted_faces:
    pts = np.array([projected_points[i] for i in face], dtype=np.int32)
    cv2.fillConvexPoly(image, pts, (255, 255, 255))

# Display the image
cv2.imshow('Filled Cube', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
