#!/usr/bin/env python3
"""
Utility script to clear the Gazebo world and remove all entities.
This can help resolve naming conflicts when elements are spawned multiple times.
"""

import subprocess
import time
import sys

def clear_gazebo_world():
    """Clear all entities from the Gazebo world."""
    try:
        # List all entities in the world
        print("Listing current entities in the world...")
        result = subprocess.run(['ros2', 'service', 'call', '/world/hexagon/entity_list', 'gz.msgs.StringMsg'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            print("Current entities found in world")
        else:
            print("Could not list entities, world might not be running")
            return False
        
        # Try to remove common entities that might cause conflicts
        entities_to_remove = [
            'ground_plane',
            'sun',
            'ros_symbol'
        ]
        
        for entity in entities_to_remove:
            try:
                print(f"Attempting to remove entity: {entity}")
                subprocess.run(['ros2', 'service', 'call', f'/world/hexagon/remove', 'gz.msgs.Entity', 
                              f'name: "{entity}"'], 
                             capture_output=True, text=True, timeout=5)
                print(f"Removed entity: {entity}")
            except subprocess.TimeoutExpired:
                print(f"Timeout removing entity: {entity}")
            except Exception as e:
                print(f"Error removing entity {entity}: {e}")
        
        # Remove any entities with names that might be duplicates
        duplicate_patterns = ['box', 'gate', 'cylinder', 'shelfino', 'obstacle']
        for pattern in duplicate_patterns:
            try:
                print(f"Attempting to remove entities matching pattern: {pattern}")
                # This is a simplified approach - in practice you'd need to list and filter
                subprocess.run(['ros2', 'service', 'call', f'/world/hexagon/remove', 'gz.msgs.Entity', 
                              f'name: "{pattern}"'], 
                             capture_output=True, text=True, timeout=5)
            except Exception as e:
                print(f"Error removing pattern {pattern}: {e}")
        
        print("World clearing completed")
        return True
        
    except Exception as e:
        print(f"Error clearing world: {e}")
        return False

def reset_world():
    """Reset the entire world."""
    try:
        print("Resetting world...")
        subprocess.run(['ros2', 'service', 'call', '/world/hexagon/reset', 'gz.msgs.WorldReset'], 
                      capture_output=True, text=True, timeout=10)
        print("World reset completed")
        return True
    except Exception as e:
        print(f"Error resetting world: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "reset":
        reset_world()
    else:
        clear_gazebo_world() 