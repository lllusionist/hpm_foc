#include <stdio.h>
#include "log.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"

/* 取代 SDK 的 hpm_debug_console.c (该文件已从 SES 工程排除):
 * 原实现逐字节忙等 UART 发送完毕, 115200 下一行约 5ms. */

#define LOG_BUF_SIZE (4096U) /* 2 的幂 */
#define LOG_TASK_PERIOD_MS (2U)

static char log_buf[LOG_BUF_SIZE];
static volatile uint32_t log_head; /* 写入位置 (生产者) */
static volatile uint32_t log_tail; /* 读出位置 (log 任务) */
static volatile uint32_t log_drop_cnt;

void log_uart_init(void)
{
	uart_config_t cfg = {0};

	/* 先配引脚再开时钟: 否则 RX 电平跳变会被当成起始位 */
	HPM_IOC->PAD[SYS_CONSOLE_TX_PAD].FUNC_CTL = SYS_CONSOLE_TX_FUNC;
	HPM_IOC->PAD[SYS_CONSOLE_RX_PAD].FUNC_CTL = SYS_CONSOLE_RX_FUNC;
	clock_add_to_group(SYS_CONSOLE_UART_CLK, 0);

	uart_default_config(SYS_CONSOLE_UART, &cfg);
	cfg.src_freq_in_hz = clock_get_frequency(SYS_CONSOLE_UART_CLK);
	cfg.baudrate = SYS_CONSOLE_BAUDRATE;
	cfg.fifo_enable = true;
	if (uart_init(SYS_CONSOLE_UART, &cfg) != status_success) {
		while (1) {
		}
	}
}

/* 写入环形缓冲; 放不下的字节丢弃. 多个任务和中断都可能调用,
 * 用关全局中断保护 (只拷贝几十字节, 时间很短). */
static unsigned int log_put(const char *data, unsigned int size)
{
	uint32_t level = disable_global_irq(CSR_MSTATUS_MIE_MASK);
	uint32_t head = log_head;
	uint32_t used = head - log_tail;

	for (unsigned int i = 0; i < size; i++) {
		uint32_t need = (data[i] == '\n') ? 2U : 1U;

		if (used + need > LOG_BUF_SIZE) {
			log_drop_cnt += size - i;
			break;
		}
		if (data[i] == '\n') {
			log_buf[head & (LOG_BUF_SIZE - 1U)] = '\r';
			head++;
		}
		log_buf[head & (LOG_BUF_SIZE - 1U)] = data[i];
		head++;
		used += need;
	}
	log_head = head;
	restore_global_irq(level);
	return size;
}

uint32_t log_dropped(void)
{
	return log_drop_cnt;
}

/* 把缓冲里的字节尽量塞进 UART TX FIFO, 不等待 */
static void log_drain(void)
{
	while (log_tail != log_head) {
		if (uart_get_data_count_in_tx_fifo(SYS_CONSOLE_UART) >=
		    UART_SOC_FIFO_SIZE)
			break;
		uart_write_byte(SYS_CONSOLE_UART,
				(uint8_t)log_buf[log_tail & (LOG_BUF_SIZE - 1U)]);
		log_tail++;
	}
}

static void Log_task(void *argument)
{
	(void)argument;
	for (;;) {
		log_drain();
		vTaskDelay(pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
	}
}

void log_start(void)
{
	if (xTaskCreate(Log_task, "log", 256, NULL, 1, NULL) != pdPASS) {
		while (1) {
		}
	}
}

/* ---- SEGGER RTL 的 stdio 底层接口 ---- */
#ifdef __SEGGER_RTL_VERSION
#include "__SEGGER_RTL_Int.h"

struct __SEGGER_RTL_FILE_impl {
	int stub;
};

static FILE log_stdin_file = {0};
static FILE log_stdout_file = {0};
static FILE log_stderr_file = {0};

__attribute__((used)) FILE *stdin = &log_stdin_file;
__attribute__((used)) FILE *stdout = &log_stdout_file;
__attribute__((used)) FILE *stderr = &log_stderr_file;

__attribute__((used)) int __SEGGER_RTL_X_file_write(__SEGGER_RTL_FILE *file,
						     const char *data,
						     unsigned int size)
{
	(void)file;
	return (int)log_put(data, size);
}

__attribute__((used)) int __SEGGER_RTL_X_file_read(__SEGGER_RTL_FILE *file,
						    char *s, unsigned int size)
{
	(void)file;
	(void)s;
	(void)size;
	return 0;
}

__attribute__((used)) int __SEGGER_RTL_X_file_stat(__SEGGER_RTL_FILE *stream)
{
	(void)stream;
	return 0;
}

__attribute__((used)) int __SEGGER_RTL_X_file_bufsize(__SEGGER_RTL_FILE *stream)
{
	(void)stream;
	return 64;
}

__attribute__((used)) int __SEGGER_RTL_X_file_unget(__SEGGER_RTL_FILE *stream,
						     int c)
{
	(void)stream;
	(void)c;
	return EOF;
}

__attribute__((used)) int __SEGGER_RTL_X_file_flush(__SEGGER_RTL_FILE *stream)
{
	(void)stream;
	return 1;
}
#endif

/* GCC/newlib 构建 (CMake) 走 _write */
int _write(int file, char *data, int size)
{
	(void)file;
	return (int)log_put(data, (unsigned int)size);
}
