# 递归搜索所有源文件（.c 与 .cpp，人脸识别模块为 C++ 实现，依赖本地组件 esp-dl）
file(GLOB_RECURSE ALL_SRCS "i2s.c" "voice_player.c" "who_human_face_recognition.cpp" "./*.c" "./*.cpp")
# 注册组件
# SRCS: 源文件列表
# INCLUDE_DIRS: 头文件目录列表（"." 为本目录，"font" 为点阵字库目录）
# EMBED_FILES: 嵌入的文件列表
#   音频文件（语音播报，16kHz/16bit/双声道 原始PCM）：
#     audio_01 库为空提示注册 / audio_02 注册成功 / audio_03 注册失败
#     audio_04 识别失败 / audio_05 识别成功
idf_component_register(SRCS ${ALL_SRCS} 
                       INCLUDE_DIRS "." "font" 
                       EMBED_FILES "canon.pcm"
                                   "audio/audio_01.pcm"
                                   "audio/audio_02.pcm"
                                   "audio/audio_03.pcm"
                                   "audio/audio_04.pcm"
                                   "audio/audio_05.pcm")
