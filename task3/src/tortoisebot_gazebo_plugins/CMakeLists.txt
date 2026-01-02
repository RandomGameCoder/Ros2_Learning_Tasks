cmake_minimum_required(VERSION 3.8)
project(tortoisebot_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
# find_package(register REQUIRED)


set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

add_library(BallController SHARED src/BallController.cpp)
set_property(TARGET BallController PROPERTY CXX_STANDARD 17)
target_link_libraries(BallController
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

target_include_directories(BallController PUBLIC include)

# find_package(gz REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(TARGETS BallController LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

ament_package()