cmake_minimum_required(VERSION 3.10)
project(CrefleAprilTagTracker)

set(CMAKE_CXX_STANDARD 17) # C++17로 변경
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 라이브러리 찾기
find_package(OpenCV REQUIRED)

# Apriltag 라이브러리 찾기
find_package(apriltag REQUIRED)

# 카메라 캘리브레이션 실행 파일
add_executable(calibrate
    src/camera_calibration.cpp
)
# OpenCV 라이브러리 링크
target_link_libraries(calibrate ${OpenCV_LIBS})

# 플랫폼 추적 실행 파일
add_executable(track
    src/platform_logic.cpp
    src/run_platform_system.cpp
)
# OpenCV와 AprilTag 라이브러리를 링크합니다.
target_link_libraries(track
    ${OpenCV_LIBS}
    apriltag::apriltag
)

# 마커 생성 실행 파일
add_executable(generate_markers
    src/generate_markers.cpp
)
# OpenCV와 AprilTag 라이브러리를 링크합니다.
target_link_libraries(generate_markers
    ${OpenCV_LIBS}
    apriltag::apriltag
)

# 메시지 출력
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "apriltag library status:")
message(STATUS "    version: ${APRILTAG_VERSION_STRING}")
message(STATUS "    libraries: ${APRILTAG_LIBRARIES}")