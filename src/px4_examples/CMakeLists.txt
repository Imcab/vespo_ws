cmake_minimum_required(VERSION 3.8)
project(px4_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Lista de nodos del paquete. Cada nombre corresponde a src/<nombre>.cpp y se
# convierte en un ejecutable: ros2 run px4_examples <nombre>.
# Para agregar un nodo nuevo basta con crear el .cpp y añadir su nombre aquí.
set(NODES
  square_mission       # misión del cuadrado (paso 3, en progreso)
  takeoff_mission      # despegue offboard y hover a 5 m (paso 2)
  telemetry_listener   # solo lee estado, posición, GPS, actitud y batería
  imu_listener         # solo lee giroscopio y acelerómetro
  orbit_mission        # offboard: círculo calculado por nosotros, mirando al centro
  figure_eight         # offboard: un 8 con feedforward de velocidad y aceleración
  velocity_control     # offboard: control por velocidad (NED y marco del cuerpo)
  orbit_command        # sin offboard: Takeoff + Orbit + Return con comandos de PX4
  goto_mission         # sin offboard: el cuadrado con DO_REPOSITION y Return
)

# foreach repite el bloque para cada nombre de la lista (como un for en Python).
foreach(node ${NODES})
  # Compila src/<node>.cpp en un ejecutable llamado <node>.
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${node} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  # Enlaza las librerías que usa cada nodo (headers + bibliotecas).
  ament_target_dependencies(${node} rclcpp px4_msgs)
  # Instala el ejecutable donde "ros2 run" lo busca.
  install(TARGETS ${node}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
