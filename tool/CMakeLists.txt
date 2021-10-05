include_directories(
        inc
        /usr/local/include/glog
        /lib/config/include/json
)
aux_source_directory(src TOOL_CPP)
add_library(
        tools
        ${TOOL_CPP}
)
target_link_libraries(
        tools
        /usr/local/lib/libglog.a
        /usr/lib/x86_64-linux-gnu/libjson-c.a
        jsoncpp
)
