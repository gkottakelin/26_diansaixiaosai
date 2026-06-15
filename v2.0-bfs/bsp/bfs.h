#include "ti_msp_dl_config.h"

#define ROWS    5
#define COLS    5


typedef struct {
    uint8_t x;
    uint8_t y;
} Point;

extern volatile Point path[ROWS * COLS];
extern volatile uint8_t path_len;

void store_targets(char targets_str[]);
void search_path(uint8_t start_x,uint8_t start_y,uint8_t nx,uint8_t ny);