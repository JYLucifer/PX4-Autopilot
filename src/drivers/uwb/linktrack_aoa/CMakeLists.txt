# 在 linktrack/CMakeLists.txt 中
px4_add_module(
    MODULE drivers__linktrack_aoa
    MAIN linktrack_aoa
    SRCS
        src/linktrack_aoa_main.cpp
        src/linktrack_aoa.cpp
        src/linktrack_aoa_parser/nlink_utils.c
		src/linktrack_aoa_parser/nlink_linktrack_aoa_nodeframe0.c

    INCLUDES
        include

    DEPENDS
		px4_work_queue
        drivers__device              # 依赖：PX4 设备驱动基础库
        drivers_rangefinder
)


