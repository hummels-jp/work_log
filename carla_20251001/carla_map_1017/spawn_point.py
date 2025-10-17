import carla
import random
import time
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
import math

def connect_to_carla():
    """Connect to CARLA server"""
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        print("Connected to CARLA server successfully!")
        return client
    except Exception as e:
        print(f"Failed to connect to CARLA server: {e}")
        return None

def get_spawn_points(world):
    """Get all spawn points from the current world"""
    try:
        spawn_points = world.get_map().get_spawn_points()
        print(f"Found {len(spawn_points)} spawn points")
        return spawn_points
    except Exception as e:
        print(f"Error getting spawn points: {e}")
        return []

def draw_spawn_points_debug(world, spawn_points, duration=60.0):
    """Draw spawn points in CARLA world using debug shapes"""
    debug = world.debug
    
    print(f"Drawing {len(spawn_points)} spawn points in CARLA world for {duration} seconds...")
    
    for i, spawn_point in enumerate(spawn_points):
        location = spawn_point.location
        rotation = spawn_point.rotation
        
        # Draw a sphere at spawn point location
        debug.draw_point(
            location, 
            size=0.5, 
            color=carla.Color(255, 0, 0),  # Red color
            life_time=duration
        )
        
        # Draw an arrow showing spawn direction
        forward_vector = rotation.get_forward_vector()
        end_location = carla.Location(
            location.x + forward_vector.x * 2,
            location.y + forward_vector.y * 2,
            location.z + forward_vector.z * 2
        )
        
        debug.draw_arrow(
            location,
            end_location,
            thickness=0.1,
            arrow_size=0.3,
            color=carla.Color(0, 255, 0),  # Green color
            life_time=duration
        )
        
        # Draw spawn point number
        debug.draw_string(
            carla.Location(location.x, location.y, location.z + 1),
            str(i),
            draw_shadow=False,
            color=carla.Color(255, 255, 255),  # White color
            life_time=duration
        )

def plot_spawn_points_2d(spawn_points, map_name="Unknown"):
    """Create a 2D matplotlib plot of spawn points"""
    if not spawn_points:
        print("No spawn points to plot!")
        return
    
    # Extract x, y coordinates
    x_coords = [sp.location.x for sp in spawn_points]
    y_coords = [sp.location.y for sp in spawn_points]
    
    # Create the plot
    plt.figure(figsize=(12, 10))
    
    # Plot spawn points
    plt.scatter(x_coords, y_coords, c='red', s=50, alpha=0.7, label='Spawn Points')
    
    # Add direction arrows
    for i, sp in enumerate(spawn_points):
        rotation = sp.rotation
        # Convert rotation to direction vector
        yaw_rad = math.radians(rotation.yaw)
        dx = math.cos(yaw_rad) * 5  # Arrow length
        dy = math.sin(yaw_rad) * 5
        
        plt.arrow(sp.location.x, sp.location.y, dx, dy, 
                 head_width=2, head_length=1, fc='green', ec='green', alpha=0.6)
        
        # Add spawn point numbers
        if len(spawn_points) <= 50:  # Only show numbers if not too many points
            plt.annotate(str(i), (sp.location.x, sp.location.y), 
                        xytext=(5, 5), textcoords='offset points', fontsize=8)
    
    plt.xlabel('X Coordinate (meters)')
    plt.ylabel('Y Coordinate (meters)')
    plt.title(f'CARLA Spawn Points - {map_name}')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.axis('equal')
    plt.tight_layout()
    
    # Save the plot
    plt.savefig(f'spawn_points_{map_name.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return plt.gcf()

def print_spawn_point_info(spawn_points):
    """Print detailed information about spawn points"""
    print(f"\n{'='*50}")
    print(f"SPAWN POINTS INFORMATION")
    print(f"{'='*50}")
    print(f"Total spawn points: {len(spawn_points)}")
    
    if spawn_points:
        print(f"\nFirst 10 spawn points:")
        print(f"{'Index':<6} {'X':<10} {'Y':<10} {'Z':<10} {'Yaw':<10}")
        print(f"{'-'*50}")
        
        for i, sp in enumerate(spawn_points[:10]):
            loc = sp.location
            rot = sp.rotation
            print(f"{i:<6} {loc.x:<10.2f} {loc.y:<10.2f} {loc.z:<10.2f} {rot.yaw:<10.2f}")
        
        if len(spawn_points) > 10:
            print(f"... and {len(spawn_points) - 10} more spawn points")

def spawn_vehicles_at_points(world, spawn_points, num_vehicles=5):
    """Spawn some vehicles at random spawn points for visualization"""
    if not spawn_points:
        print("No spawn points available for spawning vehicles!")
        return []
    
    blueprint_library = world.get_blueprint_library()
    vehicle_blueprints = blueprint_library.filter('vehicle.*')
    
    spawned_vehicles = []
    
    for i in range(min(num_vehicles, len(spawn_points))):
        # Select random spawn point and vehicle blueprint
        spawn_point = random.choice(spawn_points)
        vehicle_bp = random.choice(vehicle_blueprints)
        
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point)
            spawned_vehicles.append(vehicle)
            print(f"Spawned vehicle {i+1}: {vehicle_bp.id} at ({spawn_point.location.x:.2f}, {spawn_point.location.y:.2f})")
        except Exception as e:
            print(f"Failed to spawn vehicle {i+1}: {e}")
    
    return spawned_vehicles

def main():
    """Main function to demonstrate spawn points"""
    # Connect to CARLA
    client = connect_to_carla()
    if not client:
        return
    
    # Get current world and map
    world = client.get_world()
    current_map = world.get_map()
    map_name = current_map.name
    
    print(f"Current map: {map_name}")
    
    # Get spawn points
    spawn_points = get_spawn_points(world)
    if not spawn_points:
        print("No spawn points found!")
        return
    
    # Print spawn point information
    print_spawn_point_info(spawn_points)
    
    # Draw spawn points in CARLA world
    print("\nDrawing spawn points in CARLA world...")
    draw_spawn_points_debug(world, spawn_points, duration=120.0)
    
    # Create 2D plot
    print("Creating 2D plot of spawn points...")
    plot_spawn_points_2d(spawn_points, map_name)
    
    # Optionally spawn some vehicles
    user_input = input("\nDo you want to spawn some vehicles at spawn points? (y/n): ")
    if user_input.lower() == 'y':
        vehicles = spawn_vehicles_at_points(world, spawn_points, num_vehicles=5)
        
        if vehicles:
            print(f"Spawned {len(vehicles)} vehicles. Press Enter to destroy them...")
            input()
            
            # Clean up vehicles
            for vehicle in vehicles:
                try:
                    vehicle.destroy()
                except:
                    pass
            print("Vehicles destroyed.")
    
    print("Script completed!")

if __name__ == "__main__":
    main()

