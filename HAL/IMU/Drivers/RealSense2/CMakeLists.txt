find_package(RealSense2 QUIET)
option(BUILD_RealSense2 "Build RealSense2 Driver." ON)

if(RealSense2_FOUND AND BUILD_RealSense2)
	message(STATUS "HAL: building 'RealSense2' IMU driver.")

  set(SOURCES
	  RealSense2IMUDevice.h
	  RealSense2IMUDevice.cpp
	  RealSense2IMUDriver.h
	  RealSense2IMUDriver.cpp
	  RealSense2IMUFactory.cpp
  )

  add_to_hal_include_dirs(${RealSense2_INCLUDE_DIRS})
  add_to_hal_libraries(${RealSense2_LIBRARIES})
  add_to_hal_sources(${SOURCES})

endif()

