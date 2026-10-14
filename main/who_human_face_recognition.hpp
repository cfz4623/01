#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动人脸识别任务
 *
 * 该函数创建多个FreeRTOS任务来处理摄像头、LCD显示、AI人脸检测与识别、以及按键检测
 * 任务间通过队列进行帧数据的传递和同步
 */
void app_human_face_recognition(void);

#ifdef __cplusplus
}
#endif
