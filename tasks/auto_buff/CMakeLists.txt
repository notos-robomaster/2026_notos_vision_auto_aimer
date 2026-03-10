cmake_minimum_required(VERSION 3.16)

# 根据平台选择不同的依赖
if(USE_OPENVINO)
    set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake")
    find_package(OpenVINO REQUIRED)
    set(BUFF_LINK_LIBS auto_aim openvino::runtime ${CERES_LIBRARIES})
else()
    message(STATUS "[auto_buff] Using OpenCV DNN backend (YOLO11 buff detection may not be available)")
    set(BUFF_LINK_LIBS auto_aim ${CERES_LIBRARIES})
endif()

find_package(Ceres REQUIRED)

add_library(auto_buff STATIC 
    buff_type.cpp
    buff_detector.cpp
    buff_solver.cpp
    buff_aimer.cpp
    buff_target.cpp
    yolo11_buff.cpp
)

target_link_libraries(auto_buff PUBLIC ${BUFF_LINK_LIBS})