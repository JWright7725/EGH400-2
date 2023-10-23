def obstacle_locations():
    """Define an array of obstacle locations within the nodal representation of the world frame"""
    # Obstacle 1
    obstacle1_x_min = 7
    obstacle1_x_max = 17
    obstacle1_y_min = 18
    obstacle1_y_max = 22
    obstacle1_z_min = 0
    obstacle1_z_max = 10

    # Obstacle 2
    obstacle2_x_min = 24
    obstacle2_x_max = 28
    obstacle2_y_min = 14
    obstacle2_y_max = 24
    obstacle2_z_min = 0
    obstacle2_z_max = 10

    obstacle_node_limits = [[(obstacle1_x_min, obstacle1_x_max),
                            (obstacle1_y_min, obstacle1_y_max),
                            (obstacle1_z_min, obstacle1_z_max)],
                            [(obstacle2_x_min, obstacle2_x_max),
                            (obstacle2_y_min, obstacle2_y_max),
                            (obstacle2_z_min, obstacle2_z_max)]]

    return obstacle_node_limits