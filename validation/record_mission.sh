#!/bin/bash
# Runs a mission of mission_executor end to end in simulation and records a bag (simulation time).
# Usage: validation/record_mission.sh <mission> [name=<mission>] [launch_args=""] [max_wall_s=1500]
#   environment: WAVE_GAIN=<gain of the VRX wavefield>, WIND_SPEED=<m/s> (force on the UAV through the tether node)
# The bag is written to <workspace>/bags/missions/<name>_<timestamp>; the recording stops some seconds after the mission
# completes (or aborts), or after max_wall_s of wall time.
MISSION=$1; NAME=${2:-$1}; EXTRA=${3:-}; MAXW=${4:-1500}; AFTER=15
DIR=$(cd "$(dirname "$0")" && pwd); WS=$(cd "$DIR/../.." && pwd)
source /opt/ros/*/setup.bash; source "$WS/install/setup.bash"
export ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR:-$HOME/acados}; export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
LOG=/tmp/record_mission; mkdir -p $LOG "$WS/bags/missions"
BAG="$WS/bags/missions/${NAME}_$(date +%Y%m%d_%H%M%S)"

stop_all() {  # stops every simulation/control process by PID (no pkill -f, the launch files kill px4 themselves)
  for p in $(ps aux | grep -E "install/|[p]x4_sitl|[g]z sim|[M]icroXRCE|parameter_bridge|foxglove_bridge|[r]uby /opt|ros2 launch|ros2 bag|wind.py" | grep -v "grep\|claude\|record_mission" | awk '{print $2}'); do kill -9 $p 2>/dev/null; done
}
stop_all; sleep 3
(setsid ros2 launch bringup moordyn_marsupial.launch.py headless:=true use_tether:=true > $LOG/sim.log 2>&1 &)
sleep 80
if [ -n "$WAVE_GAIN" ]; then   # sea state: gain of the VRX wavefield
  MSG='params {key: "gain" value {type: DOUBLE double_value: '$WAVE_GAIN'}}'
  for i in 1 2 3 4 5; do gz topic -t /vrx/wavefield/parameters -m gz.msgs.Param -p "$MSG"; sleep 1; done
fi
if [ -n "$WIND_SPEED" ]; then
  (python3 "$DIR/wind.py" $WIND_SPEED > $LOG/wind.log 2>&1 &)
fi
setsid ros2 bag record --use-sim-time -o "$BAG" /tf /tf_static /clock /tether_geometry_marker /tether_force_markers \
  /tether_force_drone_mag /tether_force_boat_mag /moordyn_tether_node/tether_distance /moordyn_tether_node/tether_length \
  /moordyn_tether_node/tether_pull /moordyn_tether_node/external_force_drone /uav/yaw_from_reference /uav/low_altitude \
  /drone/ground_truth/odometry /boat/ground_truth/odometry /uav/reference_path /usv/reference_path /usv/mpc_horizon \
  /mpc_predicted_path /mpc_states_and_inputs /px4_1/fmu/out/vehicle_attitude /px4_1/fmu/out/vehicle_attitude_setpoint \
  /px4_1/fmu/out/vehicle_status_v1 /boat/thrusters/left/thrust /boat/thrusters/right/thrust /boat/thrusters/left/pos \
  /boat/thrusters/right/pos /virtual_tether_distance /virtual_tether_limit /virtual_tether_separation_speed \
  /uav_solve_time_ms /usv_solve_time_ms /mission/events /intruder/odometry /disturbance_estimate /simulation/stats \
  > $LOG/bag.log 2>&1 &
BAGPID=$!
(setsid ros2 launch mission_executor mission.launch.py mission:=$MISSION $EXTRA > $LOG/ctl.log 2>&1 &)
T0=$(date +%s)
until grep -qE "mission complete|abort sequence complete|Mission not loaded" $LOG/ctl.log; do
  sleep 2; [ $(( $(date +%s) - T0 )) -gt $MAXW ] && { echo "wall timeout" | tee -a $LOG/ctl.log; break; }
done
sleep $AFTER
kill -INT $BAGPID; for i in $(seq 30); do [ -f "$BAG/metadata.yaml" ] && break; sleep 1; done
stop_all
grep -E "mission complete|abort|wall timeout|Mission not loaded" $LOG/ctl.log | tail -2
echo "bag: $BAG"
