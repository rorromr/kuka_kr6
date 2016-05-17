#!/bin/bash
# Create URDF and SDF files from xacro

cd ../robots
rm -rf kuka_kr6.urdf
rm -rf kuke_kr6.sdf
echo "Exporting URDF..."
rosrun xacro xacro.py kuka_kr6.urdf.xacro > kuka_kr6.urdf
echo "Exporting SDF..."
gzsdf print kuka_kr6.urdf > kuka_kr6.sdf
cd ../scripts
