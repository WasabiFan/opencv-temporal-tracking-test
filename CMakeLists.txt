cmake_minimum_required (VERSION 3.1)
project (temporal-tracking-test)

find_package(OpenCV REQUIRED)

add_executable(temporal-tracking-test Main.cpp TimeUtils.cpp FpsCounter.cpp AppParamsManager.cpp RobotComms.cpp ColorBasedTargetDetector.cpp HeadlessApp.cpp DesktopTest.cpp lib/PracticalSocket.cpp)
target_link_libraries( temporal-tracking-test ${OpenCV_LIBS} )
target_compile_features(temporal-tracking-test PRIVATE cxx_range_for)
