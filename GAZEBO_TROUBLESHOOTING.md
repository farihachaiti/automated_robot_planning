# Ignition Gazebo Segmentation Fault Troubleshooting Guide

## Problem Description
You encountered a segmentation fault when running Ignition Gazebo with the error:
```
Segmentation fault (Address not mapped to object [(nil)])
```

The crash occurred in the rendering system at:
```
ignition::rendering::v6::BaseScene::HasVisualName()
```

## Root Causes Identified

1. **Material Script References**: The original world files referenced Gazebo Classic material scripts that are incompatible with Ignition Gazebo 6
2. **Mesh Loading Issues**: Complex mesh files may cause rendering problems
3. **SDF Version Compatibility**: Some SDF elements may not be fully compatible
4. **Multiple Robot Spawning**: Spawning multiple robots can cause duplicate visual element conflicts
5. **TF Tree Issues**: Improper TF transform publishing can cause navigation problems

## Solutions Implemented

### 1. Fixed World Files
- **`test_simple.world`**: Minimal world for testing
- **`hexagon_world_fixed.sdf`**: Hexagon world with basic geometry instead of meshes
- **Updated `empty.world`**: Removed problematic material scripts
- **Updated `hexagon_world/model.sdf`**: Removed all walls to prevent mesh loading issues

### 2. Material Script Fixes
Replaced all material script references:
```xml
<!-- OLD (problematic) -->
<material>
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/Grey</name>
  </script>
</material>

<!-- NEW (fixed) -->
<material>
  <ambient>0.5 0.5 0.5 1</ambient>
  <diffuse>0.5 0.5 0.5 1</diffuse>
</material>
```

### 3. Disabled Shadows
Set `cast_shadows` to `false` and `shadows` to `false` in scene settings to reduce rendering complexity.

### 4. Single Robot Test Launches
- **`single_robot_test.launch.py`**: Spawns only one robot to avoid conflicts
- **`teleop_test.launch.py`**: Includes teleop for testing robot movement

## Current Status ✅
- **Segmentation fault resolved**: The main issue is fixed
- **Walls removed**: The hexagon world now has no walls
- **Gazebo running**: The simulation is loading and running
- **Robots spawning**: Multiple robots are being created successfully

## Current Issues Being Addressed

### 1. Duplicate Visual Elements
```
[Err] Visual: [ground_plane] already exists
[Err] Visual: [cylinder0] already exists
[Err] Visual: [box] already exists
```
**Solution**: Use single robot test launches to avoid conflicts

### 2. TF Tree Issues
```
Could not find a connection between 'shelfino0/odom' and 'shelfino0/base_link'
```
**Solution**: Proper TF publishing setup in single robot launches

## Testing Steps

### 1. Test Simple World
```bash
cd /home/fariha/ros2_ws
source install/setup.bash
ros2 launch shelfino_gazebo test_simple.launch.py
```

### 2. Test Single Robot (Recommended)
```bash
cd /home/fariha/ros2_ws
source install/setup.bash
ros2 launch shelfino_gazebo single_robot_test.launch.py
```

### 3. Test with Teleop
```bash
cd /home/fariha/ros2_ws
source install/setup.bash
ros2 launch shelfino_gazebo teleop_test.launch.py
```

### 4. Test Fixed Hexagon World
```bash
cd /home/fariha/ros2_ws
source install/setup.bash
ros2 launch shelfino_gazebo shelfino.launch.py use_gui:=true use_rviz:=false
```

### 5. Direct Ignition Test
```bash
ign gazebo -v 4 shelfino_gazebo/worlds/hexagon_world/model.sdf
```

## Additional Troubleshooting Steps

### 1. Check Ignition Version
```bash
ign gazebo --version
```

### 2. Verify Dependencies
```bash
dpkg -l | grep ignition
```

### 3. Clear Gazebo Cache
```bash
rm -rf ~/.ignition/gazebo/
```

### 4. Check Graphics Drivers
```bash
glxinfo | grep "OpenGL version"
```

### 5. Environment Variables
Set these environment variables before running:
```bash
export QT_QPA_PLATFORM=xcb
export GAZEBO_MODEL_PATH=/path/to/your/models
```

## Prevention Measures

1. **Use Inline Materials**: Always use inline material definitions instead of material scripts
2. **Simple Geometry**: Prefer basic geometry (box, sphere, cylinder) over complex meshes
3. **Disable Shadows**: Set `cast_shadows="false"` for better performance
4. **Test Incrementally**: Start with simple worlds and add complexity gradually
5. **Single Robot Testing**: Test with one robot before spawning multiple robots

## Alternative Solutions

### 1. Use Different Rendering Engine
Try different rendering engines by setting environment variables:
```bash
export IGN_RENDERING_ENGINE_PATH=/usr/lib/x86_64-linux-gnu/ign-rendering-6/engine-plugins/
export IGN_RENDERING_ENGINE=ogre2
```

### 2. Headless Mode
If GUI issues persist, try headless mode:
```bash
ros2 launch shelfino_gazebo shelfino.launch.py use_gui:=false
```

### 3. Different Physics Engine
Try different physics engines in the SDF:
```xml
<physics type='bullet'>
  <!-- physics settings -->
</physics>
```

## File Locations

- **Fixed Worlds**: `shelfino_gazebo/worlds/`
- **Test Launches**: `shelfino_gazebo/launch/`
  - `test_simple.launch.py`
  - `single_robot_test.launch.py`
  - `teleop_test.launch.py`
- **Updated Launch**: `shelfino_gazebo/launch/shelfino.launch.py`

## Support

If issues persist:
1. Check Ignition Gazebo documentation
2. Verify ROS 2 and Ignition compatibility
3. Consider updating to latest versions
4. Report bugs to Ignition Gazebo team

## Version Information
- **Ignition Gazebo**: 6.17.0
- **ROS 2**: Humble
- **Ubuntu**: 22.04 (Jammy) 