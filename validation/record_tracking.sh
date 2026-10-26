#!/bin/bash
# Runs the cooperative tracking scenario in simulation and records the signals needed to evaluate both controllers.
# Usage: validation/record_tracking.sh <scenario: crescent|limits|steps> [duration_s=240] [controller=nmpc]
#   crescent   - accuracy of the UAV (Section 5.1): the USV holds its position at the origin and the UAV flies the crescent
#                of validation/uav_tracking/crescent_reference.py (fast arcs at constant distance, slow U-turns), with
#                d_max 35 m, so that no limit of the vehicle or of the tether is reached
#   limits     - actuator limits of the UAV (Section 5.1): USV holding at the origin, d_max 35 m, the UAV given the reference
#                of validation/uav_tracking/limits_reference.py (a lap too fast for the tilt, a lap too fast for the speed
#                limit, a step up and a step down below z_min), played once
#   steps      - step references (waypoints held fixed over the horizon) for both vehicles, d_max 15 m: the NMPCs plan the
#                motion within the actuator, tether-length and pay-out limits (UAV jumps 10 m sideways and to a point
#                beyond the tether reach, then the USV jumps 18 m, then both return to the start)
# Controllers: nmpc    - the NMPC of the thesis
#              px4     - the PX4 position controller (offboard), unaware of the tether
#              noobs   - the NMPC without the disturbance observer
# The bag is written to <workspace>/bags/tracking/tracking_<scenario>_<controller>_<timestamp>.
SCENARIO=${1:-crescent}; DUR=${2:-240}; CTRL=${3:-nmpc}
UAV_ARGS=""
case $SCENARIO in
  crescent) RADIUS=3.0; UAV_ARGS="uav_trajectory_file:=$(cd "$(dirname "$0")/.." && pwd)/mission/test_uav_circle/config/crescent.csv \
           tether_max_length:=35.0 usv_waypoints:=0,0,0,0";;
  limits) RADIUS=3.0; UAV_ARGS="uav_trajectory_file:=$(cd "$(dirname "$0")/.." && pwd)/mission/test_uav_circle/config/limits.csv \
           uav_trajectory_loop:=false tether_max_length:=35.0 usv_waypoints:=0,0,0,0";;
  steps) RADIUS=3.0; UAV_ARGS="start_altitude:=7.0 \
           uav_waypoints:=0,2,0,8;15,12,0,8;35,12,0,20;85,2,0,8 \
           usv_waypoints:=0,0,0,0;60,-15,10,146;85,0,0,-34";;  # no spaces: word-split below
  *) echo "unknown scenario"; exit 1;; esac
case $CTRL in nmpc) CTRL_ARGS="";; px4) CTRL_ARGS="px4_baseline:=true";; noobs) CTRL_ARGS="disturbance_observer:=false";; *) echo "unknown controller"; exit 1;; esac
WS=$(cd "$(dirname "$0")/../.." && pwd)
source /opt/ros/*/setup.bash; source "$WS/install/setup.bash"
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}; export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
mkdir -p "$WS/bags/tracking"; BAG="$WS/bags/tracking/tracking_${SCENARIO}_${CTRL}_$(date +%Y%m%d_%H%M%S)"

stop_all() {  # stops every simulation/control process by PID (no pkill -f, the launch files kill px4 themselves)
  for p in $(ps aux | grep -E "install/|[p]x4_sitl|[g]z sim|[M]icroXRCE|parameter_bridge|foxglove_bridge|[r]uby /opt|ros2 launch|ros2 bag" | grep -v "grep\|claude\|record_tracking" | awk '{print $2}'); do kill -9 $p 2>/dev/null; done
}
stop_all; sleep 3
(setsid ros2 launch test_cooperative_mpc sim.launch.py use_tether:=true > /tmp/tracking_sim.log 2>&1 &)
sleep 80
TOPICS="/tf /tf_static /tether_geometry_marker /tether_force_markers /predicted_trajectory /reference_trajectory /mpc_predicted_path /usv/mpc_horizon \
 /mpc_states_and_inputs /drone/ground_truth/odometry /boat/ground_truth/odometry /usv/reference_path /uav/reference_path \
 /boat/thrusters/left/thrust /boat/thrusters/right/thrust /boat/thrusters/left/pos /boat/thrusters/right/pos \
 /moordyn_tether_node/tether_distance /moordyn_tether_node/tether_length /tether_force_drone_mag /tether_force_boat_mag \
 /px4_1/fmu/out/vehicle_attitude /px4_1/fmu/out/vehicle_attitude_setpoint /virtual_tether_distance /virtual_tether_limit /virtual_tether_separation_speed /uav_solve_time_ms /usv_solve_time_ms /disturbance_estimate /simulation/stats /clock"
# The bag starts before the controllers, so that the takeoff is recorded
(timeout $DUR ros2 bag record --use-sim-time -o "$BAG" $TOPICS > /tmp/tracking_bag.log 2>&1 &)
sleep 3
(setsid ros2 launch test_cooperative_mpc control.launch.py usv_circle_radius:=$RADIUS $UAV_ARGS $CTRL_ARGS > /tmp/tracking_ctl.log 2>&1 &)
sleep $((DUR + 2))  # the bag stops itself (timeout) before everything is killed
stop_all
echo "bag: $BAG"
