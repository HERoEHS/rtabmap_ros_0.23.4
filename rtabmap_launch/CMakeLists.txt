cmake_minimum_required(VERSION 3.5)
project(rtabmap_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
   DESTINATION share/${PROJECT_NAME}
   PATTERN "__pycache__" EXCLUDE
   PATTERN ".omc" EXCLUDE
)

install(PROGRAMS scripts/ground_truth_tf.py
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
