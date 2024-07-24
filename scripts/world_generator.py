'''
    worldGenerator class for random (dynamic) world generation
    The generator will generate a gazebo world file which can be included in the start.launch
'''
import numpy as np
import os
class worldGenerator:
    def __init__(self, cfg):
        self.cfg = cfg
        self.write_world_file()

    def write_world_file(self):
        static_models = self.load_static_obstacles()
        world_models = self.create_world_file(static_models)
        curr_path = os.path.dirname(os.path.abspath(__file__))
        parent_path = os.path.dirname(curr_path)
        os.makedirs(os.path.join(parent_path, "worlds/generated_env"), exist_ok=True)
        with open(os.path.join(parent_path, "worlds/generated_env/generated_env.world"), "w") as f:
            f.write(world_models)

    def load_static_obstacles(self):
        static_obstacles = self.cfg["static_objects"]
        
        static_models = []
        for obstacle_type in static_obstacles:
            obstacle_info = static_obstacles[obstacle_type]
            num_obstacles = obstacle_info["num"]
            range_x = obstacle_info["range_x"]
            range_y = obstacle_info["range_y"]

            if (obstacle_type == "box"):
                range_z = obstacle_info["range_z"]
                width_x_range = obstacle_info["width_x"]
                width_y_range = obstacle_info["width_y"]
            else:
                range_z = [0.0, 0.0]
                radius_range = obstacle_info["radius"]


            obstacle_height_range = obstacle_info["height"]        

            for i in range(num_obstacles):
                ox = np.random.uniform(low=range_x[0], high=range_x[1])
                oy = np.random.uniform(low=range_y[0], high=range_y[1])
                oz = np.random.uniform(low=range_z[0], high=range_z[1])
                height = np.random.uniform(low=obstacle_height_range[0], high=obstacle_height_range[1])

                if (obstacle_type == "box"):
                    ob_size = (np.random.uniform(low=width_x_range[0], high=width_x_range[1]), np.random.uniform(low=width_y_range[0], high=width_y_range[1]))
                    static_models.append(
                            f"""
                            <model name='box_{i}'>
                            <static>true</static>
                            <pose>{ox} {oy} {oz+height/2.} 0 0 0</pose> <!-- X, Y, Z, Roll, Pitch, Yaw -->
                            <link name='link'>
                                <visual name='visual'>
                                <geometry>
                                    <box>
                                        <size>{ob_size[0]} {ob_size[1]} {height}</size> <!-- Width, Depth, Height -->
                                    </box>
                                </geometry>
                                </visual>
                            </link>
                            </model> 
                            """
                    )
                else:
                    ob_size = (np.random.uniform(low=radius_range[0], high=radius_range[1]))
                    static_models.append(
                            f"""
                            <model name='cylinder_{i}'>
                            <static>true</static>
                            <pose>{ox} {oy} {oz+height/2.} 0 0 0</pose> <!-- X, Y, Z, Roll, Pitch, Yaw -->
                            <link name='link'>
                                <visual name='visual'>
                                <geometry>
                                    <cylinder>
                                        <radius>{ob_size}</radius>
                                        <length>{height}</length>
                                    </cylinder>
                                </geometry>
                                </visual>
                            </link>
                            </model> 
                            """
                    )
        return static_models 


    def create_world_file(self, models):
        # print(models)
        # models = "\n".join(models)
        world_model = f"""
            <sdf version='1.7'>
            <world name='default'>
                <light name='sun' type='directional'>
                <cast_shadows>1</cast_shadows>
                <pose>0 0 10 0 -0 0</pose>
                <diffuse>0.8 0.8 0.8 1</diffuse>
                <specular>0.2 0.2 0.2 1</specular>
                <attenuation>
                    <range>1000</range>
                    <constant>0.9</constant>
                    <linear>0.01</linear>
                    <quadratic>0.001</quadratic>
                </attenuation>
                <direction>-0.5 0.1 -0.9</direction>
                <spot>
                    <inner_angle>0</inner_angle>
                    <outer_angle>0</outer_angle>
                    <falloff>0</falloff>
                </spot>
                </light>
                <model name='ground_plane'>
                <static>1</static>
                <link name='link'>
                    <collision name='collision'>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                        </plane>
                    </geometry>
                    <surface>
                        <contact>
                        <collide_bitmask>65535</collide_bitmask>
                        <ode/>
                        </contact>
                        <friction>
                        <ode>
                            <mu>100</mu>
                            <mu2>50</mu2>
                        </ode>
                        <torsional>
                            <ode/>
                        </torsional>
                        </friction>
                        <bounce/>
                    </surface>
                    <max_contacts>10</max_contacts>
                    </collision>
                    <visual name='visual'>
                    <cast_shadows>0</cast_shadows>
                    <geometry>
                        <plane>
                        <normal>0 0 1</normal>
                        <size>100 100</size>
                        </plane>
                    </geometry>
                    <material>
                        <script>
                        <uri>file://media/materials/scripts/gazebo.material</uri>
                        <name>Gazebo/Grey</name>
                        </script>
                    </material>
                    </visual>
                    <self_collide>0</self_collide>
                    <enable_wind>0</enable_wind>
                    <kinematic>0</kinematic>
                </link>
                </model>
                <gravity>0 0 -9.8</gravity>
                <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
                <atmosphere type='adiabatic'/>
                <physics type='ode'>
                <max_step_size>0.001</max_step_size>
                <real_time_factor>1</real_time_factor>
                <real_time_update_rate>1000</real_time_update_rate>
                </physics>
                <scene>
                <ambient>0.4 0.4 0.4 1</ambient>
                <background>0.7 0.7 0.7 1</background>
                <shadows>1</shadows>
                </scene>
                <wind/>
                <spherical_coordinates>
                <surface_model>EARTH_WGS84</surface_model>
                <latitude_deg>0</latitude_deg>
                <longitude_deg>0</longitude_deg>
                <elevation>0</elevation>
                <heading_deg>0</heading_deg>
                </spherical_coordinates>
                {''.join(models)}
            </world>
            </sdf>
            """
        return world_model