cmake_minimum_required(VERSION 3.8)
project(control_brazo)

# ─── Dependencias ────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ─── Generar interfaces custom (msg + srv) ───────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/CartesianWaypoint.msg"
  "msg/CartesianState.msg"
  "srv/RegisterServo.srv"
  "srv/ServoCommand.srv"
  "srv/ComputeIK.srv"
  "srv/ComputeIKPose.srv"
  "srv/ServoStatus.srv"
  "srv/CartesianGoto.srv"
  "srv/CartesianTrajectory.srv"
  DEPENDENCIES std_msgs
)

# ─── Instalar el paquete Python ───────────────────────────────
# rosidl_generate_interfaces ya crea el target egg de ${PROJECT_NAME},
# asi que se instalan los modulos directamente para evitar el conflicto.
install(
  DIRECTORY ${PROJECT_NAME}/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  PATTERN "__pycache__" EXCLUDE
)

# ─── Entry points (ejecutables ROS2) ─────────────────────────
install(PROGRAMS
  scripts/ax12a_driver
  scripts/ex106_driver
  scripts/ax12a_sim_driver
  scripts/ex106_sim_driver
  scripts/cinematica
  scripts/simulacion
  scripts/gui_control
  scripts/cartesian
  DESTINATION lib/${PROJECT_NAME}
)

# ─── Recursos compartidos ─────────────────────────────────────
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
