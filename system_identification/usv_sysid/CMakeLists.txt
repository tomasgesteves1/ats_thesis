cmake_minimum_required(VERSION 3.8)
project(usv_sysid)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install python scripts
install(
  PROGRAMS
    scripts/generate_excitation.py
    scripts/validate_model.py
    scripts/wamv_model.py
    scripts/wamv_model_sdf.py
    scripts/wamv_model_exact.py
    scripts/wamv_model_coriolis.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
