def is_point_in_rect_prism(point, prism_min, prism_max):
    """
    Check if a point is within a 3D rectangular prism defined by min and max points.

    :param point: A tuple (x, y, z) representing the point to check.
    :param prism_min: A tuple (x, y, z) representing the minimum x, y, z of the prism.
    :param prism_max: A tuple (x, y, z) representing the maximum x, y, z of the prism.
    :return: True if the point is within the prism, False otherwise.
    """
    x, y, z = point
    min_x, min_y, min_z = prism_min
    max_x, max_y, max_z = prism_max
    
    return (min_x <= x <= max_x) and (min_y <= y <= max_y) and (min_z <= z <= max_z)

# Define a point
point_to_check = (5, 5, 5)

# Define the minimum and maximum corners of the prism
prism_minimum = (1, 1, 1)
prism_maximum = (10, 10, 10)

# Check if the point is within the prism
is_within_prism = is_point_in_rect_prism(point_to_check, prism_minimum, prism_maximum)
is_within_prism
