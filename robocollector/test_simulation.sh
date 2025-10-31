#!/bin/bash
# RoboCollector Simulation Quick Test
# This script demonstrates the full simulation workflow

echo "🎮 RoboCollector Simulation Test"
echo "================================="
echo ""

# Source workspace
source ~/JetRover/ros2_ws/install/setup.bash 2>/dev/null
export need_compile=False

echo "✓ Workspace sourced"
echo ""

# Launch simulation in background
echo "🚀 Launching simulation..."
ros2 launch robocollector_bringup simulation.launch.py &
LAUNCH_PID=$!

# Wait for nodes to start
echo "⏳ Waiting for nodes to initialize..."
sleep 5

# Check if nodes are running
echo ""
echo "📊 Active nodes:"
ros2 node list 2>/dev/null | grep -E "(simulation_camera|vision_node|pickup_node|controller_node)"

# Start the mission
echo ""
echo "🎯 Starting mission..."
sleep 2
ros2 service call /controller_node/enter std_srvs/srv/Trigger 2>/dev/null

# Monitor for a bit
echo ""
echo "👀 Mission running... (monitoring for 30 seconds)"
echo "   Watch the terminal output for state transitions"
echo "   Press Ctrl+C to stop early"
echo ""

sleep 30

# Cleanup
echo ""
echo "🛑 Stopping simulation..."
kill $LAUNCH_PID 2>/dev/null
sleep 2

echo ""
echo "✅ Test complete!"
echo ""
echo "To run full simulation manually:"
echo "  ros2 launch robocollector_bringup simulation.launch.py"
echo ""
echo "To start mission:"
echo "  ros2 service call /controller_node/enter std_srvs/srv/Trigger"
