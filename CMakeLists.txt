cmake_minimum_required(VERSION 3.5)
project(armorDetector)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_STANDARD 11)

add_subdirectory(./armor)
add_subdirectory(./tool)
add_subdirectory(./driver)
add_subdirectory(./solvePNP)
add_subdirectory(./thread)

include_directories(
        tool/inc
        armor/inc
        driver/camera/inc
        driver/serialPort/inc
	driver/camera/lib/dahen_include
        solvePNP/inc
        thread/inc
)
aux_source_directory(. MAIN_CPP)
add_executable(armorDetector
            ${MAIN_CPP}
        )

target_link_libraries(
        armorDetector
        tools
        camera
        armor
        solvePNP
        thread
        )

