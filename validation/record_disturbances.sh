#!/bin/bash
# Records the disturbance runs of Section 5.3 with the landing_demo mission (takeoff, transfer away from the boat and
# landing on the moving boat): sea states (gain of the VRX wavefield 0, 1, 2) and wind on the UAV (5, 8, 12 m/s) with
# and without the disturbance observer. Usage: validation/record_disturbances.sh [waves|wind|all=all]
DIR=$(cd "$(dirname "$0")" && pwd); WHAT=${1:-all}
if [ "$WHAT" != wind ]; then
  for g in 0 1 2; do WAVE_GAIN=$g "$DIR/record_mission.sh" landing_demo landing_gain$g "" 1500; done
fi
if [ "$WHAT" != waves ]; then
  for w in 5 8 12; do
    WIND_SPEED=$w "$DIR/record_mission.sh" landing_demo landing_wind${w}_obs "" 1500
    WIND_SPEED=$w "$DIR/record_mission.sh" landing_demo landing_wind${w}_noobs "disturbance_observer:=false" 1500
  done
fi
echo "disturbances done"
