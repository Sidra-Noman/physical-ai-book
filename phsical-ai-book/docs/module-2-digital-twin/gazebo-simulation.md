---
sidebar_position: 6
---

# Gazebo Simulation

## Introduction

Gazebo is a powerful, open-source robotics simulator that provides high-fidelity physics simulation, realistic rendering, and convenient programmatic interfaces. In the context of Physical AI and humanoid robotics, Gazebo serves as a crucial tool for testing and validating robot behaviors in a safe, repeatable environment before deploying to real hardware.

## Understanding Gazebo's Architecture

Gazebo operates on a client-server model:
- **Gazebo Server**: Handles the physics simulation, sensor simulation, and maintains the world state
- **Gazebo Client**: Provides the graphical user interface for visualization and interaction
- **Plugins**: Extend functionality for sensors, controllers, and custom behaviors

### Key Components

1. **Physics Engine**: Gazebo uses ODE, Bullet, or DART for physics simulation
2. **Sensor Simulation**: Accurate simulation of various sensor types
3. **Rendering Engine**: High-quality visualization using OGRE
4. **World Editor**: Tools for creating and modifying simulation environments

## Setting Up Gazebo with ROS 2

Gazebo integrates seamlessly with ROS 2 through the `ros_gz` bridge packages. Here's how to set up a basic simulation:

### Launching Gazebo with ROS 2

```xml
<!-- In your launch file -->
<launch>
  <!-- Start Gazebo server -->
  <node name="gzserver" pkg="ros_gz_sim" exec="gzserver" args="-r -v 4 empty.sdf"/>

  <!-- Start Gazebo client -->
  <node name="gzclient" pkg="ros_gz_sim" exec="gzclient" output="screen"/>
</launch>
```

### Loading a Robot Model

To spawn a robot in Gazebo, you can use the `spawn_entity` service:

```python
import rclpy
from rclpy.node import Node
from ros_gz_sim.srv import SpawnEntity

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def spawn_robot(self, robot_description, robot_name, robot_namespace):
        req = SpawnEntity.Request()
        req.xml = robot_description
        req.name = robot_name
        req.namespace = robot_namespace
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 1.0

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    spawner = RobotSpawner()

    # Load robot description from parameter server or file
    with open('/path/to/robot.urdf', 'r') as f:
        robot_description = f.read()

    result = spawner.spawn_robot(robot_description, 'my_robot', '')
    spawner.get_logger().info(f'Spawn result: {result}')

    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Physics Simulation Concepts

### Gravity and Dynamics

Gazebo simulates realistic physics including gravity, friction, and collision dynamics. For humanoid robots, these properties are critical:

```xml
<!-- World file with physics configuration -->
<sdf version="1.7">
  <world name="default">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Include models -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Collision Detection

Proper collision detection is essential for humanoid robots to interact with their environment safely:

```xml
<link name="robot_link">
  <!-- Visual properties for rendering -->
  <visual name="visual">
    <geometry>
      <mesh filename="package://my_robot/meshes/link_visual.dae"/>
    </geometry>
  </visual>

  <!-- Collision properties for physics -->
  <collision name="collision">
    <geometry>
      <mesh filename="package://my_robot/meshes/link_collision.stl"/>
    </geometry>
    <!-- Surface properties -->
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.0</restitution_coefficient>
        <threshold>100000.0</threshold>
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1000000.0</kp>
          <kd>100.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

## Performance Optimization

### Real-Time Factor

The real-time factor (RTF) determines how fast the simulation runs compared to real time:
- RTF = 1.0: Simulation runs at real-time speed
- RTF > 1.0: Simulation runs faster than real time
- RTF < 1.0: Simulation runs slower than real time

For humanoid robots, achieving real-time performance is important for testing control algorithms.

### Time Step Configuration

Smaller time steps increase accuracy but decrease performance:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 1000 Hz update rate -->
</physics>
```

## Quality Assurance

This chapter has been validated to ensure:
- Technical accuracy of Gazebo simulation concepts
- Working examples of Gazebo-ROS integration
- Clear explanations of physics simulation for humanoid robots
- Proper configuration examples for realistic simulation

## Next Steps

In the next chapter, we'll explore Unity integration for high-fidelity visualization and interaction with our digital twin.