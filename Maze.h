#ifndef MAZE_H_
#define MAZE_H_

typedef enum
{
	D_UP = 0,
	D_RIGHT,
	D_DOWN,
	D_LEFT,
	D_NONE = 255
} eDirection;

void ClearMaze(void);
void InitMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y);
void UpdateMap(uint8_t x, uint8_t y, uint8_t Direction);
uint8_t GetMin(uint16_t Num0, uint16_t Num1, uint16_t Num2);
uint8_t GetBestDir(uint8_t x, uint8_t y, uint8_t Direction);

#endif /* MAZE_H_ */
