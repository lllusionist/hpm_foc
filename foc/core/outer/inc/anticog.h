#ifndef __ANTICOG_H__
#define __ANTICOG_H__

#include <stdbool.h>
#include <stdint.h>

/* 齿槽转矩补偿标定接口
 * - 位置保持法 (0x28): 逐点停稳后多点平均 Iq, 正反两遍消摩擦
 * - 恒速扫描法 (0x2E): 匀速转 N 圈, 按机械角分箱平均 */

/* 0 = 位置保持(0x28), 1 = 恒速扫描(0x2E) */
void anticogging_set_method(uint8_t m);

void anticogging_start(void);
/* 返回 true 表示标定结束 (成功或超时) */
bool anticogging_loop(void);
void anticogging_end(void);

/* 当前齿槽标定阶段编号 (-1 = 未在运行) */
int anticogging_step(void);

/* 齿槽表分块 dump (0x2C), 每 1kHz 拍打 8 点 */
void cogging_dump_request(void);
void cogging_dump_loop(void);

#endif /* __ANTICOG_H__ */
