/**
 * @file    bfs.c
 * @brief   智能驾驶小车全局路径规划与单步运动决策系统
 * @details 基于 9x9 占用栅格地图(Occupancy Grid)模型，结合广度优先搜索(BFS)算法，
 * 实现小车在全局寻路、动态避障重路由、以及单步动作生成。
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "bfs.h"

/* ======================= 全局状态与地图数据 ======================= */

// 全局占用栅格地图：0 表示该格为可通行的道路，1 表示该格为不可进入的建筑物/障碍物
uint8_t grid[ROWS][COLS];

// 存放由规划器生成的从起点到终点的有效行驶路径序列，供底层运动逻辑按序取出
volatile Point path[ROWS * COLS];
// 当前有效路径的节点数量，为 0 表示当前无有效路径或已到达终点
volatile uint8_t path_len;

// 途经点目标数组，用于存储任务需要依次经过的字母节点坐标 (如 O->A->B->E->...)
static Point targets[30];
// targets_len: 数组中存放的节点数量
// targets_i:   当前小车正前往的途经点在 targets 数组中的索引
static uint8_t targets_len = 0, targets_i = 0;

/* ======================= BFS 算法核心数据结构 ======================= */

// 前驱数组：记录到达当前格子的「上一个格子」的坐标。
// 用途1：用 0xFF 标记该格子是否被访问过。
// 用途2：找到终点后，可以通过前驱从终点一路推导回起点，从而得到最短路径。
Point prev[ROWS][COLS];

// 广度优先遍历用的循环队列，使用取模运算模拟循环队列，总容量 81。
Point qq[ROWS * COLS];
uint8_t head, tail;

// 四方向扩展偏移数组。小车将在栅格中探测邻近格子的四个方向。
// 【注意】此处的方向顺序需严格对应 bfs.h 中的 NavDirection 枚举值：
// d=0: DIR_UP (y+1), d=1: DIR_RIGHT (x+1), d=2: DIR_DOWN (y-1), d=3: DIR_LEFT (x-1)
static const int8_t dx[4] = {0, 1, 0, -1};
static const int8_t dy[4] = {1, 0, -1, 0};

/* ======================= 初始化与地图配置 ======================= */

/**
 * @brief 初始化 9x9 静态地图 (系统上电复位后调用一次)
 * @details 根据比赛设定的布局参数生成（灰色）建筑物占用地图。
 */
void init_map(void) {
    for(uint8_t x = 0; x < ROWS; x++) {
        for(uint8_t y = 0; y < COLS; y++) {
            // 建筑物坐标 11, 31, 51... 77 为不可进入区域
            // 简化规则：当 x 和 y 均为奇数时，该栅格为建筑物
            if ((x % 2 != 0) && (y % 2 != 0)) {
                grid[x][y] = 1; // 标记为障碍物/建筑
            } else {
                grid[x][y] = 0; // 标记为道路
            }
        }
    }
}

/**
 * @brief 将赛车场地图中的路口字母转换到 9x9 栅格地图下的具体坐标
 * @param c 目标路口的字母 (O, A~J)
 * @return 对应的坐标 Point 结构
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
        case 'J': return (Point){6, 8}; // 停车入库前
    }
    return (Point){0, 0}; // 异常输入
}

/**
 * @brief 根据传入的目标字母字符串，填充途经点目标数组
 * @param targets_str 所有目标节点的字符串，以 '\0' 结尾 (如 "OABEHIJ")
 */
void store_targets(char targets_str[]) {
      uint8_t str_i = 0;

      /* 若没有途经点，使用默认路径 */
      if (targets_str == NULL || targets_str[0] == '\0') {
          targets_str = "OABEHIJ";   // 默认：O到A到B到E到H到I到J
      }

      targets_len = 0;
      targets_i   = 0;

      // 确保以 O 开头，否则手动添加起点 (0,0)
      if (targets_str[0] != 'O') {
          targets[targets_len++] = (Point){0, 0};
      }

      while (targets_str[str_i] != '\0') {
          targets[targets_len++] = solve_target(targets_str[str_i]);
          str_i++;
      }
  }

/* ======================= BFS 搜索算法 ======================= */

/**
 * @brief 在栅格地图中使用广度优先搜索(BFS)寻找最短路径
 * @param start  寻路起始坐标
 * @param target 寻路目标坐标
 * @return Point 成功找到时返回终点坐标，未找到(或相同)则返回 (0xFF, 0xFF)
 */
Point bfs(Point start, Point target) {
    // 1. 初始化访问标记，将所有前驱坐标置为 0xFF，表示「尚未访问」
    for (uint8_t i = 0; i < ROWS; i++) {
        for (uint8_t j = 0; j < COLS; j++) {
            prev[i][j].x = 0xFF;
            prev[i][j].y = 0xFF;
        }
    }

    // 2. 清空并初始化队列，起点入队
    head = tail = 0;
    qq[tail++] = start;
    prev[start.x][start.y] = start; // 起点前驱设为自己，标记为已访问

    // 3. 开始灌水：一层一层向外扩散搜索
    while (head != tail) {
        // 取出当前格子
        Point cur = qq[head++];
        if (head == ROWS * COLS) head = 0; // 循环队列指针复位防越界

        // 检查是否到达目标格
        if (cur.x == target.x && cur.y == target.y) {
            return cur; // 寻路成功，返回终点坐标
        }

        // 沿当前方向按上、右、下、左四个邻近格子进行探测
        uint8_t d_limit = ((cur.x == start.x && cur.y == start.y) ? 1 : 4);
          for (uint8_t d = 0; d < d_limit; d++) {
            uint8_t nx = cur.x + dx[d];
            uint8_t ny = cur.y + dy[d];

            // 合法性校验：
            // nx < ROWS && ny < COLS：边界限制防止超出场外，同时防止坐标溢出
            // grid[nx][ny] == 0：确保邻近格子是可通行的道路
            // prev[nx][ny].x == 0xFF：确保邻近格子之前未走过，防止死循环
            if (nx < ROWS && ny < COLS && grid[nx][ny] == 0 && prev[nx][ny].x == 0xFF) {
                prev[nx][ny] = cur;           // 把当前格子记录为邻近格子的「引路人」
                qq[tail++] = (Point){nx, ny}; // 邻近格子入队等待下一轮扩散
                if (tail == ROWS * COLS) tail = 0;
            }
        }
    }
    // 队列清空仍未找到目标，说明被障碍物包围
    return (Point){0xFF, 0xFF};
}

/**
 * @brief 借助 prev 前驱数组，回溯推导出一条完整行驶路径序列
 * @param start  路径的实际起点
 * @param target BFS找到的终点坐标标志
 */
void reconstruct_path(Point start, Point target) {
    // 如果 BFS 返回的目标为无效值，说明无路可走，路径长度清零
    if (target.x == 0xFF || target.y == 0xFF){
        path_len = 0;
        return;
    }

    Point rev_path[ROWS * COLS]; // 临时栈，存放逆向推导的路径
    uint8_t rev_len = 0;
    Point cur = target;

    // 从终点开始，顺着前驱记录一步步退回，直到退到起点为止
    while (!(cur.x == start.x && cur.y == start.y)) {
        rev_path[rev_len++] = cur;
        cur = prev[cur.x][cur.y];

        // 异常情况处理
        if (cur.x == 0xFF || cur.y == 0xFF){
            path_len = 0;
            return;
        }
    }

    // 因为 rev_path 存放的是 [终点, 终点前一格, ..., 起点后一格]
    // 需将其反转为正向 [起点后一格, ..., 终点前一格, 终点]，方便小车顺序取出
    for (uint8_t i = 0; i < rev_len; i++) {
        path[i] = rev_path[rev_len - 1 - i];
    }
    path_len = rev_len; // 记录小车需要走行的有效步数
}

/* ======================= 决策系统接口 ======================= */

/**
 * @brief 全局寻路并完成下一步决策
 * @param start_x, start_y 小车当前所在坐标
 * @param cur_dir          小车当前的绝对朝向
 * @param nx, ny           障碍物坐标；若无障碍物传 0xFF, 0xFF
 * @return StepDecision    包含小车下一位置、新朝向及执行的动作
 */
StepDecision search_path(uint8_t start_x, uint8_t start_y, NavDirection cur_dir, uint8_t nx, uint8_t ny) {
    static Point last_valid_pos = {0, 0};
    StepDecision decision;
    Point cur_pos = {start_x, start_y};

    // 初始化输出，默认原地停止
    decision.next_pos = cur_pos;
    decision.next_dir = cur_dir;
    decision.action = ACT_STOP;

    // 1. 途经点推进逻辑
    if(start_x == targets[targets_i].x && start_y == targets[targets_i].y) {
        targets_i++;
    }
    if(targets_i >= targets_len) {
        path_len = 0;
        return decision; // 已到达全部目标，返回停止
    }

    // 2. 动态避障重规划
    bool force_backward = false;
    if(nx != 0xff && ny != 0xff) {
        if(nx < ROWS && ny < COLS) grid[nx][ny] = 1;

        // 如果障碍就在脚下，寻路会失败，必须退回上一步
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

    // 4. 单步动作生成逻辑 (原 get_next_step 函数)
    if (path_len == 0) {
        return decision; // 无路可走或已到达，返回停止
    }

    Point target_pos = path[0];
    decision.next_pos = target_pos;

    if (cur_pos.x == target_pos.x && cur_pos.y == target_pos.y) {
        return decision; // 已到达
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

/* ======================= 转向序列计算 ======================= */

/**
 * @brief 计算两点之间的行进方向
 */
static NavDirection direction_between(Point from, Point to)
{
    if (to.y > from.y) return DIR_UP;
    if (to.y < from.y) return DIR_DOWN;
    if (to.x < from.x) return DIR_LEFT;
    return DIR_RIGHT;
}

/**
 * @brief 根据当前方向和目标方向计算转向动作
 */
static MoveAction compute_turn_action(NavDirection from_dir, NavDirection to_dir)
{
    int diff = (int)(to_dir - from_dir + 4) % 4;
    switch (diff) {
        case 0: return ACT_FORWARD;
        case 1: return ACT_TURN_RIGHT;
        case 3: return ACT_TURN_LEFT;
        default: return ACT_U_TURN;  /* diff==2: 180度掉头 */
    }
}

/**
 * @brief 根据连续途径点序列和进出方向，计算每个路口的转向动作
 * @param waypoints  途径点字符串 (如 "ABEHIJ")，'\0'结尾
 * @param enter_dir  小车进入第一个路口时的朝向 (NavDirection)
 * @param exit_dir   小车离开最后一个路口时需要的朝向
 * @param actions    输出：每个路口要执行的转向动作 (MoveAction)
 * @return           动作数量 = strlen(waypoints)，0表示失败
 */
uint8_t compute_turn_sequence(const char waypoints[], NavDirection enter_dir,
                               NavDirection exit_dir, MoveAction actions[])
{
    uint8_t len = (uint8_t)strlen(waypoints);
    if (len == 0) return 0;

    NavDirection cur_dir = enter_dir;

    /* 前 len-1 个路口：转向指向下一个路口 */
    for (uint8_t i = 0; i < len - 1; i++) {
        Point cur  = solve_target(waypoints[i]);
        Point next = solve_target(waypoints[i + 1]);
        NavDirection target_dir = direction_between(cur, next);
        actions[i] = compute_turn_action(cur_dir, target_dir);
        cur_dir = target_dir;
    }

    /* 最后一个路口：转向指定的出口方向 */
    actions[len - 1] = compute_turn_action(cur_dir, exit_dir);

    return len;
}
