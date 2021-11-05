# SystemView V3.30 和 RTT 二合一库
因为SystemView会引用到RTT，所以将两个库合并在一起较好
在nRF52中不再使用SDK中的RTT库，需要在Makefile中修改下面内容

## Makefile中配置
SRC_FILES += \
  $(DRIVERS_DIR)/SystemView_V330/SEGGER/SEGGER_SYSVIEW.c \
  $(DRIVERS_DIR)/SystemView_V330/Sample/FreeRTOSV10/SEGGER_SYSVIEW_FreeRTOS.c \
  $(DRIVERS_DIR)/SystemView_V330/Sample/FreeRTOSV10/Config/Cortex-M/SEGGER_SYSVIEW_Config_FreeRTOS.c \
  $(DRIVERS_DIR)/SystemView_V330/SEGGER/SEGGER_RTT.c \
  $(DRIVERS_DIR)/SystemView_V330/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
  $(DRIVERS_DIR)/SystemView_V330/SEGGER/SEGGER_RTT_printf.c \

INC_FOLDERS += \
  $(DRIVERS_DIR)/SystemView_V330/SEGGER \

## 问题修复
此代码修复了一个gcc下的Link问题：undefined references to SEGGER_RTT_ASM_WriteSkipNoLock

Fix SystemView V3.30 in GCC link problem: comment L918 and L979 in SEGGER_RTT.c
L918:  //#if (RTT_USE_ASM == 0)
L979:  //#endif
reference: https://www.silabs.com/community/mcu/32-bit/forum.topic.html/systemview_on_slstk3401amicriumosdynamic-l3OF



