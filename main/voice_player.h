/**
 * @file voice_player.h
 * @brief 语音播报模块头文件
 *
 * 功能说明：
 *   基于 ES8311 + I2S 播放内嵌的 PCM 提示音，供人脸识别流程做语音反馈：
 *     - audio_01：人脸库为空，提示用户注册人脸
 *     - audio_02：人脸注册成功
 *     - audio_03：人脸注册失败（提示继续注册）
 *     - audio_04：人脸识别失败
 *     - audio_05：人脸识别成功
 *
 * 音频格式要求（必须与 I2S/ES8311 配置一致）：
 *   - 采样率：16000 Hz
 *   - 位宽：16 bit
 *   - 声道：双声道立体声（左右交错），若为单声道需转成双声道
 *   - 格式：原始 PCM 裸数据（无文件头）
 *
 * 使用说明：
 *   1. 音频文件放置于 main/audio/ 目录，文件名固定为 audio_01.pcm ~ audio_05.pcm，
 *      并在 main/CMakeLists.txt 的 EMBED_FILES 中登记（已配置好）；
 *   2. 在 i2c_init() 之后调用 voice_player_init() 完成 I2S 与 ES8311 初始化；
 *   3. 任意任务中调用 voice_play(id) 即可异步播报（不阻塞调用者）。
 */

#ifndef __VOICE_PLAYER_H__
#define __VOICE_PLAYER_H__

#include "esp_err.h"

/**
 * @enum voice_id_t
 * @brief 语音片段编号（与 audio_01.pcm ~ audio_05.pcm 一一对应）
 */
typedef enum
{
    VOICE_ID_REGISTER_PROMPT = 0, /**< 01：库为空，提示注册人脸 */
    VOICE_ID_ENROLL_OK,           /**< 02：注册成功 */
    VOICE_ID_ENROLL_FAIL,         /**< 03：注册失败，请继续注册 */
    VOICE_ID_RECOGNIZE_FAIL,      /**< 04：识别失败 */
    VOICE_ID_RECOGNIZE_OK,        /**< 05：识别成功 */
    VOICE_ID_MAX                  /**< 片段总数（内部使用） */
} voice_id_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化语音播报模块
 *
 * 依次完成：I2S 发送通道初始化 -> ES8311 编解码芯片初始化 ->
 * 创建播报队列与播报任务。
 *
 * @return esp_err_t ESP_OK 表示成功，否则返回对应错误码
 * @note 调用前必须已执行 i2c_init()（ES8311 通过 I2C 配置）
 */
esp_err_t voice_player_init(void);

/**
 * @brief 异步播报指定语音片段（线程安全、非阻塞）
 *
 * 将播放请求投递到播报队列，由播报任务实际输出到 I2S，
 * 调用者不被阻塞。若队列已满（上一段还没播完）则丢弃本次请求，
 * 避免语音积压、滞后于当前状态。
 *
 * @param id 语音片段编号（voice_id_t）
 * @note 若模块未初始化或参数非法，函数直接返回不做任何操作
 */
void voice_play(voice_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __VOICE_PLAYER_H__ */
