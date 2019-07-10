if(UNIX)
  add_library(bno055_test_fixture EXCLUDE_FROM_ALL
    fake_bno055.cpp
    unix_test_harness.cpp)
  target_link_libraries(bno055_test_fixture
    bno055)
  if(NOT APPLE)
    target_link_libraries(bno055_test_fixture util)
  endif()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(bno055_serial_tests
    bno055_serial_tests.cpp
    TIMEOUT 10)
  target_link_libraries(bno055_serial_tests
    bno055_test_fixture)
endif()
