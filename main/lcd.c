/**
 * @file    lcd.c
 * @brief   LCD 驱动封装：初始化 ST7796 显示屏（SPI 接口 + PCA9557 扩展 IO 控制片选），
 *          提供整屏填充颜色、绘制图片等功能。
 *
 * @note    使用说明：
 *          - 硬件接口：
 *            * SPI3_HOST 总线：SCLK=GPIO41、MOSI=GPIO40（MISO 未使用）
 *            * DC 引脚：GPIO39（ESP32-S3 直接控制）
 *            * CS 引脚：由 PCA9557 扩展 IO 第 0 位控制（ESP32-S3 的 IO 不直连 CS）
 *            * RST 引脚：连接复位按键，非 ESP32-S3 IO，初始化时置 GPIO_NUM_NC
 *          - 前置条件：必须先调用 i2c_init() 和 pca9557_init()（CS 引脚依赖 PCA9557）
 *          - 屏幕参数：分辨率与方向由 lcd.h 的 LCD_ORIENTATION 宏决定
 *            （横屏 480x320 / 竖屏 320x480），RGB565（16 位色），RGB 字节序为 BGR
 *          - SPI 总线：SPI3_HOST，时钟 80MHz，8bit 命令/参数，DMA 自动分配
 *          - 初始化后默认：开启显示、颜色取反（invert true）、按宏配置坐标交换与镜像
 *          - 调用流程：lcd_init() -> lcd_set_color() / lcd_draw_picture()
 *            （摄像头显示任务通过 lcd_get_panel_handle() 直接调用 draw_bitmap）
 *          - 构建：idf.py build flash monitor
 */

#include "lcd.h"               // 自定义 LCD 头文件，提供 lcd_init、lcd_cs、lcd_set_color 等函数声明
#include "driver/gpio.h"       // GPIO 驱动头文件，提供 gpio 的函数接口
#include "freertos/FreeRTOS.h" // FreeRTOS 内核头文件，提供任务、延时等接口
#include "freertos/task.h"     // FreeRTOS 任务头文件，提供 vTaskDelay 等任务函数
#include "pca9557.h"           // 自定义 PCA9557 头文件，提供 pca9557_set_output_state 函数声明（用于控制 LCD 片选）
#include <stdbool.h>           // 标准布尔类型头文件，提供 true/false 定义
#include <stdio.h>             // 标准输入输出头文件，提供 printf 等函数
#include <stdarg.h>            // 可变参数头文件，提供 va_list / vsprintf 等接口
#include <string.h>            // 字符串处理头文件，提供 strncmp / strncpy 等接口
#include "esp_heap_caps.h"     // PSRAM 内存分配接口，提供 heap_caps_malloc / heap_caps_free
#include "FontDotMatrix16.h"   // 16x16 点阵字库（lcd_display_string 使用）
#include "FontDotMatrix32.h"   // 32x32 点阵字库（lcd_display_string 使用）

// 需包含的头文件
#include "driver/spi_master.h" // SPI 主机驱动头文件，提供 spi_bus_initialize 等函数
#include "esp_lcd_panel_io.h"  // ESP LCD 面板 IO 头文件，提供 esp_lcd_new_panel_io_spi 等函数
#include "esp_lcd_panel_ops.h" // ESP LCD 面板操作头文件，提供 esp_lcd_panel_draw_bitmap 等操作接口
#include "esp_lcd_panel_vendor.h" // ESP LCD 面板厂商头文件，提供通用面板创建接口
#include "esp_lcd_st7796.h"    // ST7796 面板驱动头文件，提供 esp_lcd_new_panel_st7796 函数
#include "esp_lcd_types.h"     // ESP LCD 类型定义头文件，提供 esp_lcd_panel_handle_t 等类型
#include "pca9557.h"           // 自定义 PCA9557 头文件（重复包含，用于确保接口可用）

// 创建 ST7796 Panel 句柄示例
static esp_lcd_panel_handle_t g_panel_handle = NULL; // ST7796 面板句柄（静态全局变量），后续所有面板操作都基于该句柄

/**
 * @brief 获取 LCD 面板句柄，供上层模块（如摄像头显示任务）直接绘制整帧图像
 * @return esp_lcd_panel_handle_t 返回 ST7796 面板句柄，需在 lcd_init() 成功之后调用
 */
esp_lcd_panel_handle_t lcd_get_panel_handle(void)
{
    return g_panel_handle;
}

/**
 * @brief 控制 LCD 片选（CS）引脚电平
 * @param state 片选电平：0 表示选中 LCD（低电平有效），1 表示取消选中
 * @note  CS 引脚由 PCA9557 扩展 IO 的第 0 位控制，不占用 ESP32-S3 的 GPIO
 */
void lcd_cs(uint32_t state)                 // 片选控制函数：通过 PCA9557 扩展 IO 控制 LCD 的 CS 引脚
{
    pca9557_set_output_state(0, state);     // 调用 PCA9557 驱动，设置第 0 路输出电平为 state（0 选中 / 1 取消）
}

/**
 * @brief 初始化 LCD：配置 SPI 总线、创建面板 IO 与 ST7796 面板，完成显示参数配置
 * @return esp_err_t 返回 ESP_OK 表示初始化成功
 */
esp_err_t lcd_init(void)                    // LCD 初始化函数：返回 ESP_OK 表示成功
{
    // SPI 总线初始化示例
    spi_bus_config_t buscfg = {             // 定义 SPI 总线配置结构体
        .mosi_io_num = GPIO_NUM_40,         // MOSI 数据输出引脚为 GPIO40（主机写数据给 LCD）
        .miso_io_num = GPIO_NUM_NC,         // 不使用 MISO 引脚，NC：No Connection，该值-1
        .sclk_io_num = GPIO_NUM_41,         // SPI 时钟引脚为 GPIO41
        .quadwp_io_num = GPIO_NUM_NC,       // 该值-1，不使用 quadwp 引脚（未用 QSPI 模式）
        .quadhd_io_num = GPIO_NUM_NC,       // 该值-1，不使用 quadhd 引脚（未用 QSPI 模式）
        .max_transfer_sz = LCD_WIDTH * LCD_HEIGHT * 2, // 最大单次传输字节数：宽*高*2（一屏 RGB565 数据）
    };
    // 初始化 SPI 总线，使用 DMA 通道自动分配
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO)); // 初始化 SPI3 主机并自动分配 DMA 通道，失败则断言重启

    // 创建 SPI Panel IO 句柄示例
    esp_lcd_panel_io_handle_t io_handle = NULL; // 定义 SPI 面板 IO 句柄并初始化为空
    esp_lcd_panel_io_spi_config_t io_config = { // 定义 SPI 面板 IO 配置结构体
        .cs_gpio_num =
            GPIO_NUM_NC, // 该CS引脚是使用pca9557的IO拓展芯片控制的，不是来源于ESP32-S3的IO
        .dc_gpio_num = GPIO_NUM_39,  // 该DC引脚是使用的ESP32-S3的IO39（命令/数据选择线）
        .spi_mode = 0,               // SPI模式0（CPOL=0, CPHA=0）
        .pclk_hz = 80 * 1000 * 1000, // 时钟线的速度80MHz
        .trans_queue_depth = 10,     // 传输队列深度10，允许挂起 10 笔传输
        .lcd_cmd_bits = 8,           // 命令字节宽度8位
        .lcd_param_bits = 8,         // 参数字节宽度8位
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &io_handle)); // 在 SPI3 上创建面板 IO，失败则断言重启

    esp_lcd_panel_dev_config_t panel_config = {    // 定义 ST7796 面板设备配置结构体
        .reset_gpio_num = GPIO_NUM_NC, // 该RST引脚是连接到复位按键，不是来源于ESP32-S3的IO
        .rgb_endian = LCD_RGB_ENDIAN_BGR, // 该RGB字节序是BGR（颜色分量在存储中的排列为蓝-绿-红）
        .bits_per_pixel = 16,             // 每个像素16位（RGB565 格式）
    };
    // 初始化 ST7796 Panel
    // g_panel_handle 是 ST7796 Panel 句柄，用于后续操作 ST7796
    // Panel(复位、设置显示模式、绘制图像等)
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &g_panel_handle)); // 基于面板 IO 创建 ST7796 面板句柄，失败则断言重启

    // 选中LCD，依赖于LCD_CS引脚输出低电平,才能配置LCD的参数
    lcd_cs(0);                                 // 拉低 CS 片选（低电平有效），选中 LCD，后续才能配置显示参数

    // 打开显示
    esp_lcd_panel_disp_on_off(g_panel_handle, true); // 通过面板句柄打开显示（发送开显示命令）

    // 开启颜色倒置（ST7796 发送 0x21 命令），使 RGB565 数值与屏幕显示颜色一致。
    // 注意：ST7796 的倒置方向与常见 ILI9341 相反，若实测画面颜色反相，
    // 请改为 false（发送 0x20 关闭倒置）
    esp_lcd_panel_invert_color(g_panel_handle, true);    // 开启颜色倒置，使显示颜色与 RGB 数值对应正确

    // 按 lcd.h 的 LCD_ORIENTATION 宏配置屏幕方向：
    // 横屏时交换坐标轴（MADCTL MV 位），并按宏设置水平/垂直镜像
    esp_lcd_panel_swap_xy(g_panel_handle, LCD_SWAP_XY); // 交换坐标轴：横屏开启，竖屏关闭
    esp_lcd_panel_mirror(g_panel_handle, LCD_MIRROR_X, LCD_MIRROR_Y); // 设置面板镜像（适配屏幕实际接法）

    // 初始化液晶屏，配置LCD的内部寄存器
    ESP_ERROR_CHECK(esp_lcd_panel_init(g_panel_handle)); // 初始化面板内部寄存器，失败则断言重启

    return ESP_OK;                                      // 全部初始化完成，返回成功
}

/**
 * @brief 设置液晶屏颜色函数，将整个屏幕填充为指定的颜色
 * @param color 颜色值（RGB565 格式，16 位）
 */
void lcd_set_color(uint16_t color)          // 整屏填充颜色函数：将整个屏幕绘制为指定颜色
{
    // 在外部SPIRAM中分配内存，分配液晶屏一行数据所需的大小
    // MALLOC_CAP_8BIT 表示分配8位内存，用于存储16位颜色值
    // MALLOC_CAP_SPIRAM 表示分配SPIRAM内存，用于存储16位颜色值
    uint16_t* buffer =                      // 定义一行像素缓冲区指针
        (uint16_t*)heap_caps_malloc(LCD_WIDTH * sizeof(uint16_t), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM); // 从外部 SPIRAM 分配一行（LCD_WIDTH 像素）内存，用于缓存整行颜色数据

    // 检查内存分配是否成功
    if (NULL == buffer)                     // 判断内存分配是否失败（返回 NULL 表示失败）
    {
        // 打印错误日志，提示内存不足
        printf("Memory for bitmap is not enough\n"); // 分配失败时打印错误日志，提示内存不足
        return;                             // 内存不足则直接返回，不执行后续绘制
    }

    // 循环遍历缓冲区的每个像素，将指定颜色填入缓冲区
    for (size_t i = 0; i < LCD_WIDTH; i++)        // 遍历一行中的 LCD_WIDTH 个像素
    {
        // 将指定颜色值赋给缓冲区的当前位置
        buffer[i] = color;                  // 将目标颜色写入缓冲区对应位置，填满整行
    }
    // 循环遍历屏幕的每一行，将缓冲区的颜色数据绘制到屏幕上
    for (int y = 0; y < LCD_HEIGHT; y++)           // 遍历屏幕的全部 LCD_HEIGHT 行
    {
        // 在屏幕的指定行绘制缓冲区的颜色数据
        esp_lcd_panel_draw_bitmap(g_panel_handle, 0, y, LCD_WIDTH, y + 1, buffer); // 将一行颜色数据绘制到屏幕第 y 行（坐标：x[0,LCD_WIDTH), y 单行）
    }
    // 释放之前分配的内存，避免内存泄漏
    free(buffer);                           // 绘制完成后释放缓冲区内存，避免内存泄漏
}

/**
 * @brief 在屏幕指定区域绘制一张图片
 * @param x_s    图片起始 X 坐标（左上角）
 * @param y_s    图片起始 Y 坐标（左上角）
 * @param width  图片宽度（像素）
 * @param height 图片高度（像素）
 * @param pic    图片数据指针（RGB565 格式，大小需为 width*height 个 16 位数据）
 */
void lcd_draw_picture(uint32_t x_s,uint32_t y_s,uint32_t width,uint32_t height,uint16_t* pic) // 绘制图片函数：将指定大小的 RGB565 图片绘制到屏幕指定区域
{
    uint32_t x_e = x_s + width;             // 计算图片终止 X 坐标（右边界，开区间）
    uint32_t y_e = y_s + height;            // 计算图片终止 Y 坐标（下边界，开区间）

    // 循环遍历图片的每一行，将图片数据绘制到屏幕上
    for(uint32_t y = y_s; y < y_e; y++)     // 遍历图片的每一行（从起始行到终止行）
    {
        esp_lcd_panel_draw_bitmap(g_panel_handle, x_s, y, x_e, y+1, pic + y * width);    // 将图片第 y 行的数据（偏移 y*width）绘制到屏幕对应行
    }
}

/**
 * @brief 在屏幕指定区域绘制图像数据（直接透传 esp_lcd_panel_draw_bitmap）
 * @param x_start    起始 X 坐标（左上角）
 * @param y_start    起始 Y 坐标（左上角）
 * @param x_end      结束 X 坐标（右下角，开区间）
 * @param y_end      结束 Y 坐标（右下角，开区间）
 * @param color_data RGB565 像素数据指针
 * @note 人脸识别等 AI 模块通过此接口将摄像头整帧 RGB565 数据直接绘制到屏幕
 */
void lcd_draw_bitmap(uint32_t x_start, uint32_t y_start, uint32_t x_end, uint32_t y_end, const void *color_data)
{
    esp_lcd_panel_draw_bitmap(g_panel_handle, x_start, y_start, x_end, y_end, color_data);
}

/**
 * @brief 获取 UTF-8 字符的字节长度
 * @param target 指向字符首字节的指针
 * @return int 字符占用字节数（1~4），非法序列返回 1
 */
static int GetUtf8CharLength(char *target)
{
    if (target[0] <= 0x7F) {
        return 1;
    }
    else if ((target[0] >= 0xC2 && target[0] <= 0xDF) && (target[1] & 0xC0) == 0x80) {
        return 2;
    }
    else if ((target[0] >= 0xE0 && target[0] <= 0xEF) && (target[1] & 0xC0) == 0x80
             && (target[2] & 0xC0) == 0x80) {
        return 3;
    }
    else if ((target[0] >= 0xF0 && target[0] <= 0xF7) && (target[1] & 0xC0) == 0x80
             && (target[2] & 0xC0) == 0x80 && (target[3] & 0xC0) == 0x80) {
        return 4;
    }
    return 1;
}

/**
 * @brief 在字库中查找字符对应的索引
 * @param target    指向字符首字节的指针（UTF-8）
 * @param font_size 字号（16 或 32），决定使用哪个字库
 * @return int 命中返回字库索引，未命中返回 -1
 */
static int FindFontIndex(char *target, uint32_t font_size)
{
    int i;
    int char_len = GetUtf8CharLength(target);

    if (font_size == 16) {
        for (i = 0;
             i < (sizeof(g_font_dot_matrix_16_index) / sizeof(g_font_dot_matrix_16_index[0]));
             i++) {
            if (strncmp(g_font_dot_matrix_16_index[i], (char *)target, char_len) == 0) {
                return i;
            }
        }
    }

    if (font_size == 32) {
        for (i = 0;
             i < (sizeof(g_font_dot_matrix_32_index) / sizeof(g_font_dot_matrix_32_index[0]));
             i++) {
            if (strncmp(g_font_dot_matrix_32_index[i], (char *)target, char_len) == 0) {
                return i;
            }
        }
    }

    return -1;
}

/**
 * @brief 在 LCD 上绘制单个点阵字符
 * @param x         起始 X 坐标
 * @param y         起始 Y 坐标
 * @param fc        前景色（RGB565）
 * @param bc        背景色（RGB565）
 * @param font_size 字号（16 或 32）
 * @param index     字库索引
 */
static void lcd_display_char(uint32_t x, uint32_t y, uint32_t fc, uint32_t bc,
                             uint32_t font_size, uint32_t index)
{
    uint32_t i = 0, j = 0;
    uint8_t tmp = 0;
    uint32_t pixels_byte_size = font_size * font_size * sizeof(uint16_t);
    uint16_t *pixels = (uint16_t *)heap_caps_malloc(pixels_byte_size,
                                                     MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    uint32_t pixel_idx = 0;

    if (NULL == pixels) {
        printf("Memory for font bitmap is not enough (size=%lu)\n", (unsigned long)pixels_byte_size);
        return;
    }

    for (i = 0; i < font_size * font_size / 8; i++) {
        if (font_size == 16)
            tmp = g_font_dot_matrix_16[index][i];
        else if (font_size == 32)
            tmp = g_font_dot_matrix_32[index][i];

        for (j = 0; j < 8; j++) {
            if (tmp & (1 << j)) {
                pixels[pixel_idx] = (uint16_t)fc;
            } else {
                pixels[pixel_idx] = (uint16_t)bc;
            }
            pixel_idx++;
        }
    }

    esp_lcd_panel_draw_bitmap(g_panel_handle, x, y, x + font_size, y + font_size, pixels);
    heap_caps_free(pixels);
}

/**
 * @brief 在 LCD 上显示字符串（支持中文，使用 font 目录点阵字库）
 * @param x         起始 X 坐标
 * @param y         起始 Y 坐标
 * @param fc        前景色（RGB565）
 * @param bc        背景色（RGB565）
 * @param font_size 字号（16 或 32）
 * @param str       要显示的字符串（UTF-8 编码）
 * @note 中文字符占 font_size 像素宽，ASCII 字符占 font_size/2 像素宽
 * @note 超出屏幕宽度自动换行
 */
void lcd_display_string(uint32_t x, uint32_t y, uint32_t fc, uint32_t bc,
                        uint32_t font_size, char *str)
{
    uint32_t x_s = x;
    uint32_t i = 0;
    int32_t font_index = 0;

    while (str[i] != '\0') {
        font_index = FindFontIndex(&str[i], font_size);

        if (font_index < 0) {
            i++;
            continue;
        }

        lcd_display_char(x, y, fc, bc, font_size, font_index);

        if (str[i] > 0x7F) {
            x += font_size;
            i += 3;
        } else {
            x += font_size / 2;
            i += 1;
        }

        if ((x + font_size) >= LCD_WIDTH) {
            x = x_s;
            y += font_size;
        }
    }
}

/**
 * @brief 格式化显示字符串（printf 风格格式化后调用 lcd_display_string）
 */
void lcd_display_string_fmt(uint32_t x, uint32_t y, uint32_t fc, uint32_t bc,
                            uint32_t font_size, char *str, ...)
{
    va_list args;
    char temp_buf[256];
    va_start(args, str);
    vsprintf(temp_buf, str, args);
    va_end(args);
    lcd_display_string(x, y, fc, bc, font_size, temp_buf);
}

/**
 * @brief 在 LCD 指定区域填充纯色（用于清除信息显示区域）
 * @param x     起始 X 坐标
 * @param y     起始 Y 坐标
 * @param w     区域宽度（像素）
 * @param h     区域高度（像素）
 * @param color 填充颜色（RGB565 格式）
 * @note 逐行绘制，每行仅分配一行像素缓冲区，节省内存
 */
void lcd_fill_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t color)
{
    // 分配一行像素缓冲区
    uint16_t *line_buf = (uint16_t *)heap_caps_malloc(w * sizeof(uint16_t),
                                                      MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!line_buf) {
        printf("Memory for fill rect line buffer is not enough\n");
        return;
    }

    // 填充整行颜色数据
    for (uint32_t i = 0; i < w; i++) {
        line_buf[i] = color;
    }

    // 逐行绘制填充区域
    for (uint32_t row = 0; row < h; row++) {
        esp_lcd_panel_draw_bitmap(g_panel_handle, x, y + row, x + w, y + row + 1, line_buf);
    }

    heap_caps_free(line_buf);
}
