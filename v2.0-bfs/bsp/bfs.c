#include <stdint.h>
#include <stdbool.h>
#include "bfs.h"

uint8_t maze[25][25];

/* 起点和终点坐标 */
volatile Point path[ROWS * COLS];
volatile uint8_t path_len;
static Point targets[12];
static uint8_t targets_len=2,targets_i=0;

// 前驱数组：记录到达当前格子的上一个格子坐标（用于路径回溯）
Point prev[ROWS][COLS];

// 辅助队列（循环队列，最大容量为 ROWS*COLS）
Point qq[ROWS * COLS];
uint8_t head, tail;

static const int8_t dx[4] = {1, 0,  -1,  0};
static const int8_t dy[4] = { 0, 1, 0,  -1};

// 判断坐标是否有效（边界 + 非墙）
static inline bool is_valid(uint8_t fx,uint8_t fy,uint8_t x, uint8_t y) {
    return (x >= 0 && y >= 0 && x < ROWS && y < COLS && !maze[fx*ROWS+fy][x*ROWS+y]);
}

/* ======================= BFS 核心算法 ======================= */
/**
 * @brief 从起点开始 BFS，寻找最短路径
 * @param start  起点坐标
 * @param target 终点坐标
 * @return       若找到路径返回终点坐标，否则返回 (0xFF, 0xFF) 表示失败
 */
Point bfs(Point start, Point target) {
    // 1. 初始化 visited 标记（用 prev 中一个特殊值代替，此处用 x=0xFF 表示未访问）
    for (uint8_t i = 0; i < ROWS; i++) {
        for (uint8_t j = 0; j < COLS; j++) {
            prev[i][j].x = 0xFF;   // 0xFF 代表无前驱/未访问
            prev[i][j].y = 0xFF;
        }
    }

    // 2. 清空队列
    head = tail = 0;

    // 3. 起点入队
    qq[tail++] = start;
    prev[start.x][start.y] = start;   // 起点的前驱设为自身（标记已访问）

    // 4. BFS 主循环
    while (head != tail) {
        Point cur = qq[head++];
        if (head == ROWS * COLS) head = 0;   // 循环队列复位

        // 到达终点？
        if (cur.x == target.x && cur.y == target.y) {
            return cur;
        }

        // 遍历四个方向
        for (uint8_t d = 0; d < 4; d++) {
            uint8_t nx = cur.x + dx[d];
            uint8_t ny = cur.y + dy[d];

            if (is_valid(cur.x,cur.y,nx, ny) && prev[nx][ny].x == 0xFF) { // 未访问过
                prev[nx][ny] = cur;          // 记录前驱
                qq[tail++] = (Point){nx, ny};
                if (tail == ROWS * COLS) tail = 0;
            }
        }
    }

    // 无路可走
    return (Point){0xFF, 0xFF};
}

void reconstruct_path(Point start, Point target) {
    if (target.x == 0xFF || target.y == 0xFF){
		path_len=0;
		return;
	}

    // 临时反向路径栈（最大长度 25）
    Point rev_path[ROWS * COLS];
    uint8_t rev_len = 0;

    Point cur = target;
    // 从终点回溯到起点
    while (!(cur.x == start.x && cur.y == start.y)) {
        rev_path[rev_len++] = cur;
        cur = prev[cur.x][cur.y];
        if (cur.x == 0xFF || cur.y == 0xFF){
			path_len=0;
			return;
		}
    }
    //rev_path[rev_len++] = start;   // 加入起点

    // 反转为从起点到终点的顺序
    for (uint8_t i = 0; i < rev_len; i++) {
        path[i] = rev_path[rev_len - 1 - i];
    }
    path_len=rev_len;
}

Point solve_target(char c){
	switch(c){
		case 'A':return (Point){1,1};
		case 'B':return (Point){1,2};
		case 'C':return (Point){1,3};
		case 'D':return (Point){2,1};
		case 'E':return (Point){2,2};
		case 'F':return (Point){2,3};
		case 'G':return (Point){3,1};
		case 'H':return (Point){3,2};
		case 'I':return (Point){3,3};
		case 'J':return (Point){4,3};
	}
	return (Point){0,0};
}

void store_targets(char targets_str[]){
	while(targets_str[targets_len]!=0){
		targets[targets_len]=solve_target(targets_str[targets_len]);
		targets_len++;
	}
	targets[++targets_len]=(Point){4,4};
}

void search_path(uint8_t start_x,uint8_t start_y,uint8_t nx,uint8_t ny) {
	if(start_x==targets[targets_i].x && start_y==targets[targets_i].y)targets_i++;
	if(targets_i>=targets_len){
		path_len=0;
		return;
	}
	
	if(nx!=0xff&&ny!=0xff)maze[start_x*ROWS+start_y][nx*ROWS+ny]=maze[nx*ROWS+ny][start_x*ROWS+start_y]=1;
	
    Point start = {start_x, start_y};
    Point target = {targets[targets_i].x, targets[targets_i].y};

    Point found = bfs(start, target);

    reconstruct_path(start, found);

}