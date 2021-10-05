find_package(OpenCV REQUIRED)

include_directories(
        inc
        ${OpenCV_INCLUDE_DIRS}
        ../tool/inc
        /usr/include/eigen3
)
aux_source_directory(src ARMOR_CPP)
add_library(
        armor
        ${ARMOR_CPP}
)
target_link_libraries(
        armor
        tools
        ${OpenCV_LIBS}
)
