include_directories(
        inc
        /usr/local/include/glog
        /lib/config/include/json
)

add_library(
        tools
        src/tool_config.cpp
        src/tool_log.cpp
        src/tool_time.cpp
		src/tool_Drawcurve.cpp
)
target_link_libraries(
        tools
        /usr/local/lib/libglog.so
        /usr/local/lib/libjson-c.a
)
