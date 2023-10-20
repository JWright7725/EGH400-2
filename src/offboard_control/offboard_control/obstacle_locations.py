def obstacle_locations():
    """Define an array of obstacle locations within the nodal representation of the world frame"""
    obstacle_x_min = 10
    obstacle_x_max = 14
    obstacle_y_min = 16
    obstacle_y_max = 18
    obstacle_z_min = 0
    obstacle_z_max = 10
    obstacle_node_limits = [(obstacle_x_min, obstacle_x_max),
                            (obstacle_y_min, obstacle_y_max),
                            (obstacle_z_min, obstacle_z_max)]

    return obstacle_node_limits