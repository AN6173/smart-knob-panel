#pragma once

/**
 * @file console_cmd.h
 * @brief UART 控制台（knob>）：切换控制路径 / 触感模式 / 读遥测。
 */

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化并启动 ESP-IDF console REPL（默认 UART0 / USB-Serial） */
void console_cmd_start(void);

#ifdef __cplusplus
}
#endif
