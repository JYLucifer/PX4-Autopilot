# 在 linktrack/CMakeLists.txt 中
px4_add_module(
    MODULE drivers__linktrack
    MAIN linktrack
    SRCS
        src/linktrack_main.cpp
        src/linktrack.cpp
        src/linktrack_parser/nlink_utils.c
		src/linktrack_parser/nlink_linktrack_anchorframe0.c
		src/linktrack_parser/nlink_linktrack_nodeframe0.c
		src/linktrack_parser/nlink_linktrack_nodeframe1.c
		src/linktrack_parser/nlink_linktrack_nodeframe2.c
		src/linktrack_parser/nlink_linktrack_nodeframe3.c
		src/linktrack_parser/nlink_linktrack_nodeframe4.c
		src/linktrack_parser/nlink_linktrack_nodeframe5.c
		src/linktrack_parser/nlink_linktrack_nodeframe6.c
		src/linktrack_parser/nlink_linktrack_nodeframe7.c
		src/linktrack_parser/nlink_linktrack_tagframe0.c

    INCLUDES
        include

    DEPENDS
		px4_work_queue
        drivers__device              # 依赖：PX4 设备驱动基础库
        drivers_rangefinder
)


