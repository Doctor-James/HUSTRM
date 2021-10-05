find_package(Threads)
find_package(Boost)

include_directories(
        inc
        ${Boost_INCLUDE_DIRS}
)
aux_source_directory(src SERIALPORT_CPP)
add_library(
        serialPort
        ${SERIALPORT_CPP}
)
target_link_libraries(
        serialPort
        -lboost_system
        -lpthread
)
