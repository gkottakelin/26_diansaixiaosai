/**
 * @file    bfs.c
 * @brief   智能驾驶小车全局路径规划与单步运动决策系统
 * @details 采用 9x9 占据栅格图(Occupancy Grid)模型，结合广度优先搜索(BFS)算法，
 * 实现小车的全局寻路、动态避障断路重算以及物理动作解算。
 */

#include <stdint.h>
#include <stdbool.h>
#include "bfs.h"

/* ======================= 全局状态与地图变量 ======================= */

// 全局占据栅格地图：0 表示该格子为可通行道路，1 表示该格子为不可进入的建筑或障碍物
uint8_t grid[ROWS][COLS];

// 存放最终计算出的从起点到终点的有效行驶路径序列（供底层控制逻辑逐格读取）
volatile Point path[ROWS * COLS]; 
// 当前有效路径的节点数量，若为 0 代表当前无有效路径或已到达终点
volatile uint8_t path_len; 

// 途经点序列数组，用于存储赛题要求依次经过的字母节点坐标 (如 O->A->B->E->...)
static Point targets[30]; 
// targets_len: 序列中存放的节点总数
// targets_i:   当前小车正在前往的途经点在 targets 数组中的索引
static uint8_t targets_len = 0, targets_i = 0;

/* ======================= BFS 算法辅助数据结构 ======================= */

// 前驱数组：记录到达当前格子的“上一个格子”的坐标。
// 作用1：用 0xFF 标记该格子是否被访问过。
// 作用2：找到终点后，可以通过该数组从终点一路逆推回起点，从而得到最短路径。
Point prev[ROWS][COLS];

// 广度优先搜索所需的辅助队列（使用数组模拟的循环队列，最大容量 81）
Point qq[ROWS * COLS];
uint8_t head, tail;

// 四方向扩展向量：定义了小车在栅格上探索相邻格子的四个物理方向
// 【注意】此处的定义顺序必须严格与 bfs.h 中的 NavDirection 枚举对齐：
// d=0: DIR_UP (y+1), d=1: DIR_RIGHT (x+1), d=2: DIR_DOWN (y-1), d=3: DIR_LEFT (x-1)
static const int8_t dx[4] = {0, 1, 0, -1};
static const int8_t dy[4] = {1, 0, -1, 0};

/* ======================= 初始化与坐标解析 ======================= */

/**
 * @brief 初始化 9x9 静态地图 (系统上电复位后必须调用一次)
 * @details 根据赛题设定的不可侵入区域（灰色建筑），铺设基础地图。
 */
void init_map(void) {
    for(uint8_t x = 0; x < ROWS; x++) {
        for(uint8_t y = 0; y < COLS; y++) {
            // 根据题意规则：11, 31, 51... 77 为不可进入区域。
            // 抽象规律：当横坐标 x 和纵坐标 y 均为奇数时，该栅格即为建筑区。
            if ((x % 2 != 0) && (y % 2 != 0)) {
                grid[x][y] = 1; // 标记为障碍物/建筑
            } else {
                grid[x][y] = 0; // 标记为道路
            }
        }
    }
}

/**
 * @brief 坐标解码器：将赛题路口字母转化为 9x9 栅格地图下的物理坐标
 * @param c 目标路口的字母 (O, A~J)
 * @return 对应的网格 Point 坐标
 */
Point solve_target(char c) {
    switch(c){
        case 'O': return (Point){0, 0}; // 起点
        case 'A': return (Point){2, 2};
        case 'B': return (Point){4, 2};
        case 'C': return (Point){6, 2};
        case 'D': return (Point){2, 4};
        case 'E': return (Point){4, 4};
        case 'F': return (Point){6, 4};
        case 'G': return (Point){2, 6};
        case 'H': return (Point){4, 6};
        case 'I': return (Point){6, 6};
        case 'J': return (Point){6, 8}; // 停车场入口前
    }
    return (Point){0, 0}; // 异常保护
}

/**
 * @brief 将接收到的赛道序列字符串解析并存入途经点任务数组
 * @param targets_str 包含任务节点的字符串，以 '\0' 结尾 (例如 "OABEHIJ")
 */
void store_targets(char targets_str[]) {
    uint8_t str_i = 0;
    
    // 防错：如果首字符不是起点 'O'，我们可以手动补一个起点 (0,0)
    if (targets_str[0] != 'O') {
        targets[targets_len++] = (Point){0, 0};
    }
    
    while(targets_str[str_i] != '\0') {
        targets[targets_len++] = solve_target(targets_str[str_i]);
        str_i++;
    }
}


/* ======================= BFS 核心算法 ======================= */

/**
 * @brief 在栅格地图中利用广度优先搜索(BFS)寻找两点间的最短路径
 * @param start  寻路的起点坐标
 * @param target 寻路的终点坐标
 * @return Point 成功找到时返回终点坐标，未找到(死胡同)则返回 (0xFF, 0xFF)
 */
Point bfs(Point start, Point target) {
    // 1. 初始化访问标记：将所有前驱坐标设为 0xFF，表示均未访问过
    for (uint8_t i = 0; i < ROWS; i++) {
        for (uint8_t j = 0; j < COLS; j++) {
            prev[i][j].x = 0xFF; 
            prev[i][j].y = 0xFF;
        }
    }

    // 2. 清空并初始化队列，将起点入队
    head = tail = 0;
    qq[tail++] = start;
    prev[start.x][start.y] = start; // 起点的前驱设为自身，标记为已访问

    // 3. 开始向外像水波纹一样层层扩散搜索
    while (head != tail) {
        // 出队当前格子
        Point cur = qq[head++];
        if (head == ROWS * COLS) head = 0; // 循环队列指针复位防越界

        // 检查是否到达目标格子
        if (cur.x == target.x && cur.y == target.y) {
            return cur; // 寻路成功，立即返回
        }

        // 尝试向绝对方向的上、右、下、左四个相邻格子进行试探
        for (uint8_t d = 0; d < 4; d++) {
            uint8_t nx = cur.x + dx[d];
            uint8_t ny = cur.y + dy[d];

            // 合法性校验：
            // nx < ROWS && ny < COLS：利用无符号数下溢变大数的特性，同时防止负数和正越界
            // grid[nx][ny] == 0：确保该邻居格子是可通行的道路
            // prev[nx][ny].x == 0xFF：确保该邻居格子之前从未走过，避免死循环
            if (nx < ROWS && ny < COLS && grid[nx][ny] == 0 && prev[nx][ny].x == 0xFF) {
                prev[nx][ny] = cur;           // 将当前格子记录为邻居格子的“引路人”
                qq[tail++] = (Point){nx, ny}; // 邻居格子入队等待下一轮扩散
                if (tail == ROWS * COLS) tail = 0;
            }
        }
    }
    // 队列清空仍未找到目标，说明被障碍物封死
    return (Point){0xFF, 0xFF};
}

/**
 * @brief 根据 prev 前驱数组，逆向回溯出一条正向的行驶路径序列
 * @param start  路径的实际起点
 * @param target BFS找到的终点坐标标志
 */
void reconstruct_path(Point start, Point target) {
    // 如果 BFS 传来的目标是无效值，说明无路可走，路径长度清零
    if (target.x == 0xFF || target.y == 0xFF){
        path_len = 0;
        return;
    }

    Point rev_path[ROWS * COLS]; // 临时栈，存放逆序倒推的路径
    uint8_t rev_len = 0;
    Point cur = target;

    // 从终点开始，顺着前驱记录一步步退回，直到退到起点为止
    while (!(cur.x == start.x && cur.y == start.y)) {
        rev_path[rev_len++] = cur;
        cur = prev[cur.x][cur.y];
        
        // 异常断链保护
        if (cur.x == 0xFF || cur.y == 0xFF){
            path_len = 0;
            return;
        }
    }
    
    // 因为 rev_path 存的是 [终点, 终点前一格, ..., 起点后一格]
    // 必须将其反转为正序：[起点后一格, ..., 终点前一格, 终点]，供小车顺序读取
    for (uint8_t i = 0; i < rev_len; i++) {
        path[i] = rev_path[rev_len - 1 - i];
    }
    path_len = rev_len; // 记录最终需要行走的有效步数
}

/* ======================= 控制系统接口 ======================= */

/**
 * @brief 触发寻路更新并直接解算下一步动作
 * @param start_x, start_y 小车当前所在的坐标
 * @param cur_dir          小车当前的绝对朝向
 * @param nx, ny           障碍物坐标；若无障碍物传 0xFF, 0xFF
 * @return StepDecision    包含小车的下一位置、新朝向及需执行的动作
 */
StepDecision search_path(uint8_t start_x, uint8_t start_y, NavDirection cur_dir, uint8_t nx, uint8_t ny) {
    static Point last_valid_pos = {0, 0}; 
    StepDecision decision;
    Point cur_pos = {start_x, start_y};
    
    // 初始化决策：默认原地停止
    decision.next_pos = cur_pos;
    decision.next_dir = cur_dir;
    decision.action = ACT_STOP;

    // 1. 任务推进逻辑
    if(start_x == targets[targets_i].x && start_y == targets[targets_i].y) {
        targets_i++;
    }
    if(targets_i >= targets_len) {
        path_len = 0;
        return decision; // 已到达最终目标，返回停车
    }
    
    // 2. 动态断路机制
    bool force_backward = false;
    if(nx != 0xff && ny != 0xff) {
        if(nx < ROWS && ny < COLS) grid[nx][ny] = 1; 
        
        // 如果踩中绝路，覆盖寻路结果，准备退回
        if(start_x == nx && start_y == ny) {
            path[0] = last_valid_pos;
            path_len = 1;
            force_backward = true;
        }
    }
    
    // 3. 执行全局 BFS 规划
    if (!force_backward) {
        last_valid_pos.x = start_x;
        last_valid_pos.y = start_y;
        
        Point target = {targets[targets_i].x, targets[targets_i].y};
        Point found = bfs(cur_pos, target);
        reconstruct_path(cur_pos, found); 
    }

    // 4. 动作解算逻辑 (原 get_next_step 整合)
    if (path_len == 0) {
        return decision; // 死胡同无路可走，返回停车
    }

    Point target_pos = path[0];
    decision.next_pos = target_pos; 

    if (cur_pos.x == target_pos.x && cur_pos.y == target_pos.y) {
        return decision; // 防错
    }

    NavDirection target_dir = cur_dir; 
    if (target_pos.y > cur_pos.y) {
        target_dir = DIR_UP;
    } else if (target_pos.y < cur_pos.y) {
        target_dir = DIR_DOWN;
    } else if (target_pos.x > cur_pos.x) {
        target_dir = DIR_RIGHT;
    } else if (target_pos.x < cur_pos.x) {
        target_dir = DIR_LEFT;
    }
    
    decision.next_dir = target_dir;

    int diff = (target_dir - cur_dir + 4) % 4;
    switch (diff) {
        case 0: decision.action = ACT_FORWARD; break;
        case 1: decision.action = ACT_TURN_RIGHT; break;
        case 2: decision.action = ACT_U_TURN; break;
        case 3: decision.action = ACT_TURN_LEFT; break;
    }

    return decision;
}
