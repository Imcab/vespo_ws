cmake_minimum_required(VERSION 3.8)
project(@PKG@)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Paquetes que usan los nodos. Cada uno va también como <depend> en package.xml.
# vespo new añade aquí los que necesite una plantilla (p. ej. px4_ros2_cpp).
set(DEPENDENCIES
  rclcpp
  px4_msgs
)
foreach(dep ${DEPENDENCIES})
  find_package(${dep} REQUIRED)
endforeach()

# Lista de nodos del paquete. Cada nombre corresponde a src/<nombre>.cpp y se
# convierte en un ejecutable: ros2 run @PKG@ <nombre>.
# Para agregar un nodo: vespo new @PKG@ --node <nombre> (o crea el .cpp y
# añade su nombre aquí).
set(NODES
  @NODE@
)

# foreach repite el bloque para cada nombre de la lista (como un for en Python).
foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${node} PUBLIC cxx_std_17)
  ament_target_dependencies(${node} ${DEPENDENCIES})
  # Instala el ejecutable donde "ros2 run" lo busca.
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

# Los launch quedan en share/@PKG@/launch: ros2 launch @PKG@ <archivo>.launch.py
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Se saltan los linters de copyright y cpplint hasta que el código los cumpla
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
