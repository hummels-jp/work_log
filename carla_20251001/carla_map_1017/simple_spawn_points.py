#!/usr/bin/env python3
"""
Simple script to get and visualize CARLA spawn points
"""

import carla
import matplotlib.pyplot as plt
import math

def get_and_plot_spawn_points():
    """Get spawn points and create a simple plot"""
    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        
        # Get world and spawn points
        world = client.get_world()
        spawn_points = world.get_map().get_spawn_points()
        map_name = world.get_map().name
        
        print(f"Map: {map_name}")
        print(f"Found {len(spawn_points)} spawn points")
        
        # Extract coordinates
        x_coords = [sp.location.x for sp in spawn_points]
        y_coords = [sp.location.y for sp in spawn_points]
        
        # Create plot
        plt.figure(figsize=(10, 8))
        plt.scatter(x_coords, y_coords, c='red', s=30, alpha=0.7)
        
        # Add arrows for direction
        for sp in spawn_points:
            yaw_rad = math.radians(sp.rotation.yaw)
            dx = math.cos(yaw_rad) * 3
            dy = math.sin(yaw_rad) * 3
            plt.arrow(sp.location.x, sp.location.y, dx, dy, 
                     head_width=1, head_length=0.5, fc='blue', ec='blue', alpha=0.5)
        
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title(f'Spawn Points - {map_name}')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Save and show
        plt.savefig('spawn_points_simple.png', dpi=200, bbox_inches='tight')
        plt.show()
        
        # Print some info
        print(f"\nFirst 5 spawn points:")
        for i, sp in enumerate(spawn_points[:5]):
            loc = sp.location
            print(f"{i}: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
        
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    get_and_plot_spawn_points()
