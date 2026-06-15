#ifndef K230_RECEIVER_H
#define K230_RECEIVER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ================================================================
 *  协议常量
 * ================================================================ */
#define K230_FRAME_HEADER    "SA,"
#define K230_FRAME_TAIL      "ED"
#define K230_BUF_SIZE        128     /* 单帧最大字节数，留有余量 */
#define K230_FIELD_COUNT     10      /* SA后10个数据字段 */

/* ================================================================
 *  状态定义（与K230端一致）
 * ================================================================ */
typedef enum {
    K230_STATE_TRACKING   = 1,   /* 巡线 */
    K230_STATE_INTERSECT  = 2,   /* 进入路口 */
    K230_STATE_IDLE       = 3,   /* 空闲 */
    K230_STATE_PARKING    = 4    /* 车库停车 */
} K230_State_t;

/* ================================================================
 *  解析后的K230数据帧
 * ================================================================ */
typedef struct {
    K230_State_t state;   /* 当前状态 */

    /* 干线数据（STATE_TRACKING / STATE_PARKING 有效） */
    int16_t main_angle;   /* 干线角度 theta */
    int16_t main_cx;      /* 干线中心 x */
    int16_t main_cy;      /* 干线中心 y */
    int16_t main_x1;      /* 干线端点1 x */
    int16_t main_y1;      /* 干线端点1 y */
    int16_t main_x2;      /* 干线端点2 x */
    int16_t main_y2;      /* 干线端点2 y */

    /* 路口检测数据 */
    int16_t distance1;           /* 左右竖直线水平像素差 */
    int16_t is_at_intersection;  /* 是否到达路口 (0/1) */

    bool    valid;        /* 本帧数据是否完整有效 */
} K230_Frame_t;

/* ================================================================
 *  串口接收缓冲区管理
 * ================================================================ */
typedef struct {
    char    buf[K230_BUF_SIZE];
    uint8_t head;         /* 写入位置 */
    bool    receiving;    /* 是否正在接收帧 */
} K230_RxBuf_t;

/* ================================================================
 *  两路K230全局数据
 * ================================================================ */
extern K230_Frame_t g_k230_cam0;   /* UART0 对应的摄像头0数据 */
extern K230_Frame_t g_k230_cam1;   /* UART3 对应的摄像头1数据 */

extern K230_RxBuf_t g_rxbuf0;
extern K230_RxBuf_t g_rxbuf1;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief 初始化串口及接收缓冲区
 *        在 main() 开头调用一次
 */
void K230_Init(void);

/**
 * @brief 将一个字节送入指定通道的接收缓冲区，尝试解析完整帧
 *        在 UART0 / UART3 的 RX 中断里分别调用
 *
 * @param rxbuf   对应通道的缓冲区指针
 * @param frame   解析结果写入的帧结构体指针
 * @param byte    刚收到的字节
 */
void K230_FeedByte(K230_RxBuf_t *rxbuf, K230_Frame_t *frame, char byte);

/**
 * @brief 向指定K230发送状态切换指令（字符 '1'~'4'）
 *
 * @param cam_id  0 = UART0(cam0), 1 = UART3(cam1)
 * @param state   目标状态
 */
void K230_SendState(uint8_t cam_id, K230_State_t state);

/**
 * @brief 调试用：将帧内容格式化打印到字符串
 */
void K230_FrameToStr(const K230_Frame_t *frame, char *out, uint16_t out_size);

#endif /* K230_RECEIVER_H */
