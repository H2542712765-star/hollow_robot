cmake_minimum_required(VERSION 3.22)
project(cr10_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# MoveIt Setup Assistant creates this metadata file, but it is not required at
# runtime.  Some exported configuration packages do not contain it.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.setup_assistant")
  install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
endif()
