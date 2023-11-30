from neo_config import runExperiment

if __name__ == '__main__':
    
    realtime = False
    headless = False
    
    # sensor arguments: fov, fov_resolution, depth, depth_resolution, fixed: tuple, depth_limit: None
    sensor_args = [
        # [(85.2, 58), (1980,1080), (0, 10), 10, None, 1.0],
        [(85.2, 58), (1980,1080), (0, 10), 10, [0,0,2], 2.0]
    ]
    
    # see document to check which config is associated with what diagram
    config_test = [
        ('Sphere', 0.075, [0.3, 0.2, 0.50], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Sphere', 0.075, [0.3, -0.2, 0.50], [0, 0, 0, 0, 0, 0], 'Static_Sphere_2')
    ]
    config_1 = [
        ('Sphere', 0.05, [0.3, 0.0, 0.5], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1')
    ]
    config_2 = [
        ('Sphere', 0.05, [0.2, 0.2, 0.5], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Sphere', 0.05, [0.5, 0.2, 0.5], [0, 0, 0, 0, 0, 0], 'Static_Sphere_2'),
        ('Cylinder', (0.05, 0.6), [0.4, -0.2, 0.3], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1')
    ]
    config_3 = [
        ('Sphere', 0.05, [0.4, 0.4, 0.3], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Sphere', 0.05, [0.4, -0.4, 0.3], [0, 0, 0, 0, 0, 0], 'Static_Sphere_2'),
        ('Cylinder', (0.05, 0.4), [0.35, 0.02, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1')
    ]
    config_4 = [
        ('Sphere', 0.05, [0.4, 0.0, 0.3], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Cylinder', (0.05, 0.4), [0.2, 0.0, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1'),
        ('Cylinder', (0.05, 0.4), [0.6, 0.0, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_2')
    ]
    config_5 = [
        ('Cylinder', (0.05, 0.4), [0.2, 0.2, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1'),
        ('Cylinder', (0.05, 0.4), [0.6, -0.2, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_2')
    ]
    config_6 = [
        ('Sphere', 0.05, [0.6, -0.2, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Cylinder', (0.05, 0.4), [0.6, 0.2, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1'),
        ('Cylinder', (0.05, 0.4), [0.4, 0.05, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_2')
    ]
    config_7 = [
        ('Sphere', 0.05, [0.2, -0.3, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Cylinder', (0.05, 0.4), [0.4, 0.05, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1'),
        ('Cylinder', (0.05, 0.4), [0.2, 0.3, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_2')
    ]
    config_8 = [
        ('Cylinder', (0.05, 0.4), [0.25, 0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_1'),
        ('Cylinder', (0.05, 0.4), [0.25, -0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_2'),
        ('Cylinder', (0.05, 0.4), [0.6, 0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_3'),
        ('Cylinder', (0.05, 0.4), [0.6, -0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_4'),
        ('Cylinder', (0.05, 0.4), [0.4, 0.05, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Cylinder_5')
    ]
    config_9 = [
        ('Sphere', 0.05, [0.25, 0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_1'),
        ('Sphere', 0.05, [0.25, -0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_2'),
        ('Sphere', 0.05, [0.4, 0.05, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_3'),
        ('Sphere', 0.05, [0.6, 0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_4'),
        ('Sphere', 0.05, [0.6, -0.35, 0.2], [0, 0, 0, 0, 0, 0], 'Static_Sphere_5')
    ]
    runExperiment(realtime=realtime, headless=headless, \
        sensor_args=sensor_args, config_args=config_9)