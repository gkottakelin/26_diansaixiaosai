#include "k230_receiver.h"

/* ================================================================
 *  全局变量
 * ================================================================ */
K230_Frame_t g_k230_cam0 = {0};
K230_Frame_t g_k230_cam1 = {0};

K230_RxBuf_t g_rxbuf0 = {0};
K230_RxBuf_t g_rxbuf1 = {0};

/* ================================================================
 *  内部工具：安全发送单字节（阻塞等待TXFIFO就绪）
 * ================================================================ */
static void uart0_send_byte(char c)
{
    while (DL_UART_isBusy(UART0)) {}
    DL_UART_transmitData(UART0, (uint8_t)c);
}

static void uart3_send_byte(char c)
{
    while (DL_UART_isBusy(UART3)) {}
    DL_UART_transmitData(UART3, (uint8_t)c);
}

/* ================================================================
 *  内部：解析一条完整的帧字符串
 *  输入示例: "SA,1,90,120,120,60,120,180,120,100,1,ED"
 * ================================================================ */
static bool parse_frame(const char *raw, K230_Frame_t *frame)
{
    /* 检查帧头/帧尾 */
    if (strncmp(raw, K230_FRAME_HEADER, 3) != 0) return false;
    if (strstr(raw, K230_FRAME_TAIL) == NULL)      return false;

    /* 跳过 "SA," 从第4个字符开始 */
    const char *p = raw + 3;
    int32_t fields[K230_FIELD_COUNT];
    int     got = 0;
    char    tmp[16];

    while (got < K230_FIELD_COUNT && *p != '\0') {
        /* 跳过逗号 */
        if (*p == ',') { p++; continue; }
        /* 遇到 'E'(ED帧尾) 停止 */
        if (*p == 'E') break;

        /* 读取一个数字（含负号） */
        uint8_t i = 0;
        if (*p == '-') tmp[i++] = *p++;
        while (*p >= '0' && *p <= '9' && i < 15) {
            tmp[i++] = *p++;
        }
        tmp[i] = '\0';
        if (i == 0) { p++; continue; }  /* 跳过非法字符 */

        fields[got++] = (int32_t)atoi(tmp);
    }

    if (got < K230_FIELD_COUNT) return false;

    frame->state             = (K230_State_t)fields[0];
    frame->main_angle        = (int16_t)fields[1];
    frame->main_cx           = (int16_t)fields[2];
    frame->main_cy           = (int16_t)fields[3];
    frame->main_x1           = (int16_t)fields[4];
    frame->main_y1           = (int16_t)fields[5];
    frame->main_x2           = (int16_t)fields[6];
    frame->main_y2           = (int16_t)fields[7];
    frame->distance1         = (int16_t)fields[8];
    frame->is_at_intersection = (int16_t)fields[9];

    frame->valid = true;
    return true;
}

/* ================================================================
 *  公开 API 实现
 * ================================================================ */

void K230_Init(void)
{
    memset(&g_k230_cam0, 0, sizeof(g_k230_cam0));
    memset(&g_k230_cam1, 0, sizeof(g_k230_cam1));
    memset(&g_rxbuf0,    0, sizeof(g_rxbuf0));
    memset(&g_rxbuf1,    0, sizeof(g_rxbuf1));
    /* UART 硬件初始化由 SysConfig 生成的 SYSCFG_DL_init() 完成 */
}

/* ----------------------------------------------------------------
 *  字节流解析核心
 *  策略：
 *   1. 收到 'S' 时重置缓冲区，开始记录
 *   2. 收到 '\n' 时认为一帧结束，尝试解析
 *   3. 缓冲区溢出时强制重置
 * ---------------------------------------------------------------- */
void K230_FeedByte(K230_RxBuf_t *rxbuf, K230_Frame_t *frame, char byte)
{
    /* 帧头检测：遇到 'S' 重新开始 */
    if (byte == 'S') {
        rxbuf->head      = 0;
        rxbuf->receiving = true;
    }

    if (!rxbuf->receiving) return;

    /* 防溢出 */
    if (rxbuf->head >= K230_BUF_SIZE - 1) {
        rxbuf->head      = 0;
        rxbuf->receiving = false;
        return;
    }

    rxbuf->buf[rxbuf->head++] = byte;
    rxbuf->buf[rxbuf->head]   = '\0';

    /* 收到换行符 → 尝试解析 */
    if (byte == '\n') {
        frame->valid = false;
        parse_frame(rxbuf->buf, frame);
        rxbuf->head      = 0;
        rxbuf->receiving = false;
    }
}

void K230_SendState(uint8_t cam_id, K230_State_t state)
{
    char c = '0' + (uint8_t)state;   /* '1'~'4' */
    if (cam_id == 0) {
        uart0_send_byte(c);
    } else {
        uart3_send_byte(c);
    }
}

void K230_FrameToStr(const K230_Frame_t *frame, char *out, uint16_t out_size)
{
    if (!frame->valid) {
        snprintf(out, out_size, "[INVALID FRAME]");
        return;
    }
    snprintf(out, out_size,
             "state=%d angle=%d cx=%d cy=%d "
             "x1=%d y1=%d x2=%d y2=%d "
             "dist=%d is_int=%d",
             frame->state,
             frame->main_angle, frame->main_cx, frame->main_cy,
             frame->main_x1, frame->main_y1, frame->main_x2, frame->main_y2,
             frame->distance1, frame->is_at_intersection);
}
