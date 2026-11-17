#ifndef __LOG_H__
#define __LOG_H__

#include <stdint.h>

/* 非阻塞日志: printf 只把字节写入环形缓冲 (满则丢弃并计数),
 * 由最低优先级的 log 任务搬到 UART TX FIFO. 任何上下文调用 printf
 * 都不会忙等串口.
 * 注意: 中断里仍不建议 printf (格式化本身耗时, 且与任务共享缓冲时
 * 靠关中断保护), 只是不会再阻塞几毫秒. */

void log_uart_init(void);
void log_start(void);
uint32_t log_dropped(void);

#endif /* __LOG_H__ */
