find_package( OpenCV REQUIRED)

include_directories(
        inc
        ../tool/inc
        ../armor/inc
        ../solvePNP/inc
        ../driver/camera/inc
        ../driver/serialPort/inc
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
)
aux_source_directory(src THREAD_CPP)
add_library(
        thread
        ${THREAD_CPP}
)
target_link_libraries(
        thread
        solvePNP
        serialPort
        armor
        camera
        ${OpenCV_LIBS}
)