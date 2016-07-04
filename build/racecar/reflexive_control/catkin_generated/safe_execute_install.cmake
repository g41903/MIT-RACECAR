execute_process(COMMAND "/home/racecar/team-ws/build/racecar/reflexive_control/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/racecar/team-ws/build/racecar/reflexive_control/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
