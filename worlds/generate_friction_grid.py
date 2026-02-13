#!/usr/bin/env python3
"""
Generate a grid of collision boxes with variable friction properties.
Three surface types: Concrete, Loose Sand, Packed Sand
"""
import random
import numpy as np

# Surface type definitions
SURFACE_TYPES = {
    'concrete': {
        'mu_range': (0.999, 0.999),
        'kp': 1e12,
        'kd': 1.0,
        'color': (0.5, 0.5, 0.5),  # Gray
    },
    'loose_sand': {
        'mu_range': (0.2, 0.3),
        'kp': 1e5,
        'kd_range': (1.0, 10.0),
        'color': (0.9, 0.85, 0.6),  # Light yellow
    },
    'packed_sand': {
        'mu_range': (0.5, 0.6),
        'kp': 1e7,
        'kd': 1.0,
        'color': (0.76, 0.70, 0.50),  # Sandy brown
    },
    'deep_sand': {
        'mu_range': (0.01, 0.05),
        'kp': 1e3,
        'kd': 20.0,
        'color': (0.8, 0.2, 0.2),  # Red-ish
    }
}

# Distribution weights (adjust to control surface type frequency)
WEIGHTS = {
    'concrete': 0,      # 10% concrete
    'loose_sand': 0,   # 35% loose sand
    'packed_sand': 0,  # 45% packed sand
    'deep_sand': 1,     # 10% deep sand (stuck zones!)
}

def generate_friction_zones(grid_cols=9, grid_rows=18, terrain_width=5.4, terrain_height=10.8, cell_height=0.1):
    """Generate SDF models for friction zones (supports rectangular grids)."""
    
    cell_width = terrain_width / grid_cols
    cell_height_dim = terrain_height / grid_rows
    offset_x = terrain_width / 2 - cell_width / 2
    offset_y = terrain_height / 2 - cell_height_dim / 2
    
    models = []
    zone_id = 0
    
    # Choose surface types based on weights
    surface_types = list(WEIGHTS.keys())
    weights = list(WEIGHTS.values())
    
    for row in range(grid_rows):
        for col in range(grid_cols):
            # Random surface type
            stype = random.choices(surface_types, weights=weights)[0]
            props = SURFACE_TYPES[stype]
            
            # Random friction within range
            mu = random.uniform(*props['mu_range'])
            
            # Damping (may have range for loose sand)
            if 'kd_range' in props:
                kd = random.uniform(*props['kd_range'])
            else:
                kd = props['kd']
            
            kp = props['kp']
            color = props['color']
            
            # Position (centered on terrain)
            x = col * cell_width - offset_x
            y = row * cell_height_dim - offset_y
            # Position so top surface is at z=0
            z = -cell_height / 2
            
            model = f"""
    <model name="friction_zone_{zone_id}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{cell_width:.2f} {cell_height_dim:.2f} {cell_height}</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>{mu:.3f}</mu>
                <mu2>{mu:.3f}</mu2>
              </ode>
            </friction>
            <contact>
              <ode>
                <kp>{kp:.0f}</kp>
                <kd>{kd:.1f}</kd>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{cell_width:.2f} {cell_height_dim:.2f} {cell_height}</size>
            </box>
          </geometry>
          <material>
            <ambient>{color[0]:.2f} {color[1]:.2f} {color[2]:.2f} 1</ambient>
            <diffuse>{color[0]:.2f} {color[1]:.2f} {color[2]:.2f} 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""
            
            models.append(model)
            zone_id += 1
    
    return '\n'.join(models)

def main():
    print("Generating friction zones...")
    zones = generate_friction_zones(grid_cols=9, grid_rows=18, terrain_width=5.4, terrain_height=10.8)
    
    with open('friction_zones.sdf', 'w') as f:
        f.write("<!-- Friction zones - include in world file -->\n")
        f.write("<!-- Generated automatically -->\n")
        f.write(zones)
    
    print(f"Saved to friction_zones.sdf")
    print("Surface distribution: 10% Concrete, 35% Loose Sand, 45% Packed Sand, 10% Deep Sand")

if __name__ == "__main__":
    main()
