cmake_minimum_required(VERSION 3.8)
project(x500_retos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)
find_package(rosidl_default_generators REQUIRED)

# Mensajes propios: lo que ve el detector de ArUco (Nivel 3).
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Aruco.msg
  msg/ArucoArray.msg
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(msgs_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# Un nodo por misión: src/<nombre>.cpp -> ros2 run x500_retos <nombre>.
set(NODES
  estrella_show       # misión 1: estrella de N picos en offboard (plaza_show)
  inspeccion_pilar    # misión 2: hélice alrededor de un pilar mirando al centro (puente_rio)
  entrega_domicilio   # misión 3: entrega con comandos de PX4 y Return (barrio_entrega)
  guardian_geocerca   # misión 4: vigila la geocerca, Hold y Return (plaza_concierto)
  vuelo_prueba        # ayuda de la misión 4: despega y se escapa en línea recta
  cruzar_cueva        # misión 5: cruza la cueva con el lidar 2D (cueva, x500_lidar_2d)
  detector_aruco      # Nivel 3: la cámara de abajo -> ArUco con OpenCV -> /x500/aruco
  caza_objetivo       # misión 6: busca y sigue a la camioneta del ArUco 7 (zona_bodegas)
  estacion_carga      # misión 7: patrulla y aterriza con precisión a cargar (base_carga)
)

foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${node} PUBLIC cxx_std_17)
  ament_target_dependencies(${node} rclcpp px4_msgs sensor_msgs)
  target_link_libraries(${node} "${msgs_target}")
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()
ament_target_dependencies(detector_aruco cv_bridge)
target_link_libraries(detector_aruco ${OpenCV_LIBS})

# include/x500_retos/px4_link.hpp: lo comparten todas las misiones (y
# cualquier paquete que haga find_package(x500_retos)).
# Van a include/x500_retos/, junto a los headers de los mensajes (así los
# acomoda rosidl en Jazzy), y se incluyen igual: "x500_retos/px4_link.hpp".
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(rclcpp px4_msgs sensor_msgs rosidl_default_runtime)

# Mundos y modelos de Gazebo. Los mundos se generan con tools/build_worlds.py;
# los modelos de Fuel y las texturas de ambientCG están copiados en models/
# para que todo funcione sin internet (ver CREDITS.md).
install(
  DIRECTORY models worlds launch
  DESTINATION share/${PROJECT_NAME}
)

# Agrega models/ a GZ_SIM_RESOURCE_PATH al hacer source del workspace, para
# que los model:// de los mundos se encuentren sin exportar nada a mano.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
