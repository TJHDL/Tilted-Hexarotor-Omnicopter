execute_process(COMMAND "/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/rotors_evaluation/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hdl/GraduateDesign/Catkin_workspace_assemble/RotorS_ws/build/rotors_evaluation/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
