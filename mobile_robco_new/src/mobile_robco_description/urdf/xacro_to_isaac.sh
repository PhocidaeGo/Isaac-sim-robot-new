#!/bin/bash

# Script to convert XACRO to URDF and load it into Isaac Sim
# Usage: ./xacro_to_isaac.sh <path_to_xacro_file> [output_urdf_path]

# Check if a XACRO file was provided
if [ $# -lt 1 ]; then
  echo "Usage: $0 <path_to_xacro_file> [output_urdf_path]"
  exit 1
fi

# Input xacro file
XACRO_FILE=$1

# If output URDF path is not provided, create one based on the XACRO filename
if [ $# -lt 2 ]; then
  OUTPUT_URDF="${XACRO_FILE%.xacro}.urdf"
else
  OUTPUT_URDF=$2
fi

# Setup ROS2 environment
source /opt/ros/humble/setup.bash

# Convert XACRO to URDF
echo "Converting XACRO to URDF..."
ros2 run xacro xacro $XACRO_FILE > $OUTPUT_URDF

# Check if conversion was successful
if [ $? -ne 0 ]; then
  echo "Error: Failed to convert XACRO to URDF."
  exit 1
fi

echo "XACRO successfully converted to URDF: $OUTPUT_URDF"

# Create a Python script to load the URDF into Isaac Sim
LOAD_SCRIPT="/tmp/load_urdf.py"

cat > $LOAD_SCRIPT << 'EOF'
import os
import carb
import omni.kit.commands
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.urdf import _urdf
import sys
import argparse

def load_urdf(urdf_path):
    # Parse arguments and get the URDF path
    print(f"Loading URDF: {urdf_path}")

    # Start the simulation
    simulation_context = SimulationContext(physics_dt=0.01, rendering_dt=0.01, stage_units_in_meters=1.0)
    
    # Import URDF
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.create_physics_scene = True
    import_config.make_default_prim = True
    import_config.self_collision = False
    
    # Load URDF file
    result, prim_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config)
    
    if result:
        print(f"Successfully loaded URDF at {prim_path}")
    else:
        print("Failed to load URDF")
        sys.exit(1)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Load URDF into Isaac Sim")
    parser.add_argument("urdf_path", help="Path to the URDF file to load")
    args = parser.parse_args()
    
    load_urdf(args.urdf_path)
EOF

# Make the Python script executable
chmod +x $LOAD_SCRIPT

# Start Isaac Sim and load the URDF
echo "Starting Isaac Sim and loading the URDF..."
cd /isaac-sim

# Start Isaac Sim with the Python script to load the URDF
./runapp.sh --script $LOAD_SCRIPT -- $OUTPUT_URDF

echo "Done!" 