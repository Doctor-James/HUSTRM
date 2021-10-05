find_package( OpenCV REQUIRED)
find_package( Sophus REQUIRED )
find_package( Pangolin )

include_directories(
        inc
        ../tool/inc
        ../armor/inc
        /usr/include/eigen3
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
	${Pangolin_INCLUDE_DIRS}
)
aux_source_directory(src SOLVEPNP_CPP)
add_library(
        solvePNP
        ${SOLVEPNP_CPP}
)
target_link_libraries(
        solvePNP
        ${OpenCV_LIBS}
	${Pangolin_LIBRARIES}
        ${Sophus_LIBRARIES}
)
