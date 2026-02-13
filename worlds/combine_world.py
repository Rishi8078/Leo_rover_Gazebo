#!/usr/bin/env python3
"""Combine world file with friction zones."""

# Read friction zones
with open('friction_zones.sdf', 'r') as f:
    zones = f.read()

# Create combined world file
world = '''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="simple_world">
    
    <!-- Scene settings - disable default grid -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.85 1 1</background>
      <grid>false</grid>
      <origin_visual>false</origin_visual>
    </scene>

    <!-- Physics settings with cone friction model -->
    <physics type="ode">
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <friction_model>cone_model</friction_model>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>

    <!-- Simple Flat Ground (Base) -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <!-- Collision disabled to force contact with friction zones -->
        <!--
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
        </collision>
        -->
        <visual name="visual">
          <pose>0 0 -0.01 0 0 0</pose>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.55 0.4 1</ambient>
            <diffuse>0.76 0.70 0.50 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Friction zones with varying properties -->
'''

# Add friction zones (skip the comment lines at start)
zones_content = '\n'.join(zones.split('\n')[2:])
world += zones_content

world += '''

    <!-- Sun Light -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

  </world>
</sdf>
'''

with open('simple_world.sdf', 'w') as f:
    f.write(world)

print("Created simple_world.sdf with friction zones")
print("Surface colors: Light yellow = Loose Sand, Sandy brown = Packed Sand, Gray = Concrete")
