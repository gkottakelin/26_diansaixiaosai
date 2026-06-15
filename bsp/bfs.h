#ifndef BFS_H
#define BFS_H

#include "ti_msp_dl_config.h"

// 地图尺寸扩大为 9x9
#define ROWS    9
#define COLS    9

// ================= 数据结构定义 =================

// 坐标结构体
typedef struct {
    uint8_t x;
    uint8_t y;
}Point;

// 绝对方向枚举 (以地图系为准)
typedef enum {
    DIR_UP = 0,    // 向着 Y 增加的方向 //向上
    DIR_RIGHT = 1, // 向着 X 增加的方向 //向右
    DIR_DOWN = 2,  // 向着 Y 减小的方向 //向下
    DIR_LEFT = 3   // 向着 X 减小的方向 //向左
}NavDirection;

// 小车运动动作枚举
typedef enum {
    ACT_FORWARD,   // 直行进入下一个
    ACT_TURN_RIGHT,// 右转90度后直行
    ACT_U_TURN,    // 掉头180度
    ACT_TURN_LEFT, // 左转90度后直行
    ACT_STOP       // 停止 (到达终点触发)
}MoveAction;

// 单步决策结构体
typedef struct {
    Point next_pos;       // 下一个目标格
    NavDirection next_dir;   // 执行动作后小车的新的绝对朝向
    MoveAction action;    // 本片刻需要执行的单步动作
}StepDecision;


// ================= 全局变量与接口 =================

extern uint8_t grid[ROWS][COLS]; // 占用栅格地图，0表示道路，1表示建筑物/障碍物
extern volatile Point path[ROWS * COLS];
extern volatile uint8_t path_len;

// 核心功能接口
void init_map(void);
void store_targets(char targets_str[]);
StepDecision search_path(uint8_t start_x, uint8_t start_y, NavDirection cur_dir, uint8_t nx, uint8_t ny);

/**
 * @brief 根据途径点序列和进出方向，计算每个路口的转向动作
 * @param waypoints 途径点字符串 (如 "ABEHIJ")
 * @param enter_dir 小车进入第一个路口时的朝向
 * @param exit_dir  小车离开最后一个路口时需要的朝向
 * @param actions   输出：每个路口要执行的转向动作
 * @return 动作数量（等于途径点长度）
 */
uint8_t compute_turn_sequence(const char waypoints[], NavDirection enter_dir,
                               NavDirection exit_dir, MoveAction actions[]);

#endif // BFS_H
