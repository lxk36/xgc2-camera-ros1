cmake_minimum_required(VERSION 3.10)
project(xgc_camera_calibration VERSION 0.2.0)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS
  scripts/extrinsic_calibrator_web.py
  scripts/extrinsic_tf_publisher.py
  scripts/run_intrinsic_calibrator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_solver.py)
  catkin_add_nosetests(test/test_intrinsic_solver.py)
  catkin_add_nosetests(test/test_intrinsic_wrapper.py)
  catkin_add_nosetests(test/test_web_service.py)
endif()
