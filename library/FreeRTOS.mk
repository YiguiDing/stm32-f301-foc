# ##################################################################################################################################
# 必须至少包含
# FreeRTOS/Source/tasks.c
# FreeRTOS/Source/queue.c
# FreeRTOS/Source/list.c
# FreeRTOS/Source/portable/[compiler]/[architecture]/port.c
# FreeRTOS/Source/portable/MemMang/heap_x.c 其中 "x" 可以是 1、2、3、4 或 5。
# heap_1—— 最简单，不允许释放内存。
# heap_2—— 允许释放内存，但不会合并相邻的空闲块。
# heap_3—— 简单包装了标准 malloc() 和 free()，以保证线程安全。
# heap_4—— 合并相邻的空闲块以避免碎片化。包含绝对地址放置选项。
# heap_5—— 如同 heap_4，能够跨越多个不相邻内存区域的堆。
# ##################################################################################################################################
# 可选源文件
# 如果需要软件定时器功能，请在项目中添加 FreeRTOS/Source/timers.c。
# 如果需要事件组功能，请在项目中添加 FreeRTOS/Source/event_groups.c。
# 如果需要流缓冲区或消息缓冲区功能，请在项目中添加 FreeRTOS/Source/stream_buffer.c。
# 如果需要协程功能，请在项目中添加 FreeRTOS/Source/croutine.c（请注意，协程已弃用， 不推荐用于新设计）。
# ##################################################################################################################################
# 头文件
# 以下目录必须位于编译器的 include 路径中（必须告知编译器在这些目录中搜索 头文件）：
# FreeRTOS/Source/include
# FreeRTOS/Source/portable/[compiler]/[architecture]。
# 无论哪个目录包含要使用的 FreeRTOSConfig.h 文件，请参阅下文“配置文件”段落。
# 根据移植的不同，也可能需要将相同的目录放在汇编器的 include 路径中。
# ##################################################################################################################################
# 配置文件
# 每个项目还需要一个名为 FreeRTOSConfig.h 的文件。 
# FreeRTOSConfig.h 它为正在构建的应用程序量身定制 RTOS 内核。
# 因此，它是取决于 应用程序的，而不是 RTOS，并且应位于应用程序目录中， 而不是 RTOS 内核源代码目录中。
# 如果您的项目包含 heap_1、heap_2、heap_4 或 heap_5，则 FreeRTOSConfig.h 的 configTOTAL_HEAP_SIZE 定义将决定 FreeRTOS 堆的大小。
# 如果 configTOTAL_HEAP_SIZE 设置得太高，则您的应用程序将无法建立连接。
# FreeRTOSConfig.h 中的 configMINIMAL_STACK_SIZE 定义 设定了闲置任务使用的堆栈大小。
# 如果 configMINIMAL_STACK_SIZE 设置得太低， 则空闲任务将造成栈溢出。
# 建议您找到 使用相同微控制架构的 FreeRTOS 官方演示， 复制其中的 configMINIMAL_STACK_SIZE 设置。
# FreeRTOS 演示 项目存储在 FreeRTOS/Demo 目录的子目录中。 请注意，一些演示项目的时间距离现在比较久，因此不包含所有可用的 配置选项。
# ##################################################################################################################################

# ################################################################# ASM sources ################################################################# 
ASM_SOURCES += 
# ################################################################# C sources ################################################################# 
# FreeRTOS
C_SOURCES += $(wildcard library/FreeRTOSv202406.01-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/*.c)
# heap
C_SOURCES += library/FreeRTOSv202406.01-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/MemMang/heap_4.c
# MPU
C_SOURCES += $(wildcard library/FreeRTOSv202406.01-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/*.c)
# ################################################################# CPP sources ################################################################# 
CPP_SOURCES += 
# ################################################################# C/CPP header ################################################################# 
# FreeRTOS
C_CPP_INCLUDES += -I library/FreeRTOSv202406.01-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/include/
# MPU
C_CPP_INCLUDES += -I library/FreeRTOSv202406.01-LTS/FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel/portable/GCC/ARM_CM4F/
# ################################################################# LD_SCRIPT ################################################################# 
LD_SCRIPT += 