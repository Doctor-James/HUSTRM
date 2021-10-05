find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        inc
        ../../tool/inc
		${PROJECT_SOURCE_DIR}/driver/camera/lib/dahen_include
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
aux_source_directory(src CAMERA_CPP)
add_library(
        camera
        ${CAMERA_CPP})
target_link_libraries(
        camera
        ${OpenCV_LIBS}
		/usr/lib/libgxiapi.so
        -lpthread
)
