execute_process(COMMAND "/home/crslab/catkin_main/build/dougsm_helpers/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/crslab/catkin_main/build/dougsm_helpers/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
