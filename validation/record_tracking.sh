#!/bin/bash
# Runs the cooperative tracking scenario in simulation and records the signals needed to evaluate both controllers.
# Usage: validation/record_tracking.sh <scenario: reachable|infeasible|aggressive> [duration_s=240]
#   reachable  - USV reference circle of 3 m radius (UAV reference is feasible)
#   infeasible - USV circle of 15 m (the UAV reference violates the tether limit; the constraint is active)
#   aggressive - USV circle of 3 m; UAV circle of 8 m radius at 0.8 rad/s (6.4 m/s, 5.1 m/s^2, ~27 deg of tilt) centred on the
#                boat, so the distance to the boat is constant and the attitude/thrust limits are approached
#   tether_edge- as aggressive but with the UAV circle centred 6 m away (0.7 rad/s): the tether range and pay-out rate bind
# The bag is written to <workspace>/bags/tracking/tracking_<scenario>_<timestamp>.
SCENARIO=${1:-reachable}; DUR=${2:-240}
UAV_ARGS=""
case $SCENARIO in reachable) RADIUS=3.0;; infeasible) RADIUS=15.0;;
  aggressive) RADIUS=3.0; UAV_ARGS="uav_circle_radius:=8.0 uav_circle_omega:=0.8 uav_circle_center_x:=0.0";;
  tether_edge) RADIUS=3.0; UAV_ARGS="uav_circle_radius:=8.0 uav_circle_omega:=0.7 uav_circle_center_x:=6.0";;
  *) echo "unknown scenario"; exit 1;; esac
WS=$(cd "$(dirname "$0")/../.." && pwd)
source /opt/ros/*/setup.bash; source "$WS/install/setup.bash"
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}; export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
mkdir -p "$WS/bags/tracking"; BAG="$WS/bags/tracking/tracking_${SCENARIO}_$(date +%Y%m%d_%H%M%S)"

stop_all() {  # stops every simulation/control process by PID (no pkill -f, the launch files kill px4 themselves)
  for p in $(ps aux | grep -E "install/|[p]x4_sitl|[g]z sim|[M]icroXRCE|parameter_bridge|foxglove_bridge|[r]uby /opt|ros2 launch|ros2 bag" | grep -v "grep\|claude\|record_tracking" | awk '{print $2}'); do kill -9 $p 2>/dev/null; done
}
stop_all; sleep 3
(setsid ros2 launch test_cooperative_mpc sim.launch.py use_tether:=true > /tmp/tracking_sim.log 2>&1 &)
sleep 80
(setsid ros2 launch test_cooperative_mpc control.launch.py usv_circle_radius:=$RADIUS $UAV_ARGS > /tmp/tracking_ctl.log 2>&1 &)
sleep 5
TOPICS="/mpc_states_and_inputs /drone/ground_truth/odometry /boat/ground_truth/odometry /usv/reference_path /uav/reference_path \
 /boat/thrusters/left/thrust /boat/thrusters/right/thrust /boat/thrusters/left/pos /boat/thrusters/right/pos \
 /moordyn_tether_node/tether_distance /moordyn_tether_node/tether_length /tether_force_drone_mag /tether_force_boat_mag \
 /px4_1/fmu/out/vehicle_attitude /px4_1/fmu/out/vehicle_attitude_setpoint /virtual_tether_distance /virtual_tether_limit /virtual_tether_separation_speed /uav_solve_time_ms /usv_solve_time_ms /simulation/stats /clock"
timeout $DUR ros2 bag record -o "$BAG" $TOPICS > /tmp/tracking_bag.log 2>&1
stop_all
echo "bag: $BAG"
