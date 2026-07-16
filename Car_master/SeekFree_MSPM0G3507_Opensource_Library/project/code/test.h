//
// VOFA+ FireWater 在线调参接口
//

#ifndef TEST_H
#define TEST_H

#include <stdint.h>

// 初始化 VOFA+ 接收状态
void test_vofa_init(void);

// 轮询串口字节并解析 FireWater 命令
// 建议在高频循环中调用（例如 main 的 while(1)）
void test_vofa_poll(void);

// 向解析器输入一个字节（适用于中断/回调场景）
void test_vofa_feed_byte(uint8_t byte);

// 直接解析一整帧 FireWater 文本
// 示例: "P1=0.40,I1=0.02,D1=0.00!"
void test_vofa_parse_frame(const char *frame);

// Nextion 文本控件打印:
// 例: prints(0, "helloworld") -> t0.txt="helloworld" + 0xFF 0xFF 0xFF
void prints(uint8_t index, const char *content);

// 带格式化参数的 Nextion 文本控件打印:
// 例: printsf(0, "speed=%.2f", speed);
void printsf(uint8_t index, const char *fmt, ...);

void printsf_clear(uint8_t index);

#endif // TEST_H
