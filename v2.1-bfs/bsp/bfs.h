#ifndef BFS_H
#define BFS_H

#include "ti_msp_dl_config.h"

// 地图尺寸升级为 9x9
#define ROWS    9
#define COLS    9

// ================= 数据结构定义 =================

// 坐标结构体
typedef struct {
    uint8_t x;
    uint8_t y;
}Point;

// 绝对方向定义 (以地图系为准)
typedef enum {
    DIR_UP = 0,    // 朝向 Y 增加的方向 //向上
    DIR_RIGHT = 1, // 朝向 X 增加的方向 //向右
    DIR_DOWN = 2,  // 朝向 Y 减小的方向 //向下
    DIR_LEFT = 3   // 朝向 X 减小的方向 //向左
}NavDirection;

// 小车相对物理动作定义
typedef enum {
    ACT_FORWARD,   // 直行进入下一格
    ACT_TURN_RIGHT,// 右转90度后直行
    ACT_U_TURN,    // 掉头180度
    ACT_TURN_LEFT, // 左转90度后直行
    ACT_STOP       // 停止 (到达终点或遇障)
}MoveAction;

// 决策输出结构体
typedef struct {
    Point next_pos;       // 下一步的坐标
    NavDirection next_dir;   // 执行动作后，小车的新绝对朝向
    MoveAction action;    // 单片机需要执行的电机动作
}StepDecision;


// ================= 全局变量与接口 =================

extern uint8_t grid[ROWS][COLS]; // 占据栅格图：0代表道路，1代表建筑/障碍物
extern volatile Point path[ROWS * COLS];
extern volatile uint8_t path_len;

// 核心功能接口
void init_map(void); 
void store_targets(char targets_str[]);
StepDecision search_path(uint8_t start_x, uint8_t start_y, NavDirection cur_dir, uint8_t nx, uint8_t ny);

#endif // BFS_H