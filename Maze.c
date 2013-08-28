#include "include.h"


eDirection e_Direction;

extern uint8_t AvailDirection;
extern int8_t x, y;
extern int8_t BasePoint[];

uint16_t Array[3];
uint16_t Maze[16][16];

void ClearMaze(void)
{
	uint8_t i, j;
	for (i = 0; i < 16; i++)
	{
		for (j = 0; j < 16; j++)
		{
			Maze[i][j] = 0;
		}
	}
}

void InitMaze(int8_t x0, int8_t y0, int8_t goal_x, int8_t goal_y)
{
	uint8_t i, j;
	uint16_t count = 1;
	x = x0;
	y = y0;
	for (i = 0; i < 16; i++)
	{
		for (j = 0; j < 16; j++)
		{
			Maze[i][j] |= 0x00ff;
			Maze[i][j] &= ~HAS_GONE;
		}
	}
	Maze[goal_x][goal_y] = 0;
	while (count)
	{
		count = 0;
		for (i = 0; i < 16; i++)
		{
			for (j = 0; j < 16; j++)
			{
				if ((i > 0) && ((Maze[i][j] & 0xff) < (Maze[i-1][j] & 0xff)) && ((Maze[i - 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_WEST_WALL)))
					{
						Maze[i-1][j] &= 0xff00;
						Maze[i-1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((i < 15) && ((Maze[i][j] & 0xff) < (Maze[i+1][j] & 0xff)) && ((Maze[i + 1][j] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_EAST_WALL)))
					{
						Maze[i+1][j] &= 0xff00;
						Maze[i+1][j] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j > 0) && ((Maze[i][j] & 0xff) < (Maze[i][j - 1] & 0xff)) && ((Maze[i][j - 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_SOUTH_WALL)))
					{
						Maze[i][j - 1] &= 0xff00;
						Maze[i][j - 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
				if ((j < 15) && ((Maze[i][j] & 0xff) < (Maze[i][j + 1] & 0xff)) && ((Maze[i][j + 1] & 0xff) == 0x00ff))
				{
					if ((!(Maze[i][j] & IS_NORTH_WALL)))
					{
						Maze[i][j + 1] &= 0xff00;
						Maze[i][j + 1] |= ((Maze[i][j] + 1) & 0xff);
						count++;
					}
				}
			}
		}
	}
	if ((!(Maze[0][1] & IS_SOUTH_WALL)))
	{
		Maze[0][0] = Maze[0][1] + 1;
	}
	else if ((!(Maze[1][0] & IS_WEST_WALL)))
	{
		Maze[0][0] = Maze[1][0] + 1;
	}

	if ((!(Maze[0][14] & IS_NORTH_WALL)))
	{
		Maze[0][15] = Maze[0][14] + 1;
	}
	else if ((!(Maze[1][15] & IS_WEST_WALL)))
	{
		Maze[0][15] = Maze[1][15] + 1;
	}

	if ((!(Maze[14][0] & IS_EAST_WALL)))
	{
		Maze[15][0] = Maze[14][0] + 1;
	}
	else if ((!(Maze[15][1] & IS_SOUTH_WALL)))
	{
		Maze[15][0] = Maze[15][1] + 1;
	}

	if ((!(Maze[14][15] & IS_EAST_WALL)))
	{
		Maze[15][15] = Maze[14][15] + 1;
	}
	else if ((!(Maze[15][14] & IS_NORTH_WALL)))
	{
		Maze[15][15] = Maze[15][14] + 1;
	}
	Maze[goal_x][goal_y] = 0;
}

void UpdateMap(uint8_t x, uint8_t y, uint8_t Direction)
{
	Maze[x][y] |= HAS_GONE;
	switch (Direction)
	{
		case D_UP:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 15)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 15)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			break;
		case D_RIGHT:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 15)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 15)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			break;
		case D_DOWN:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_EAST_WALL;
				if (x != 15)
				{
					Maze[x+1][y] |= IS_WEST_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			break;
		case D_LEFT:
			if (!(AvailDirection & AVAIL_LEFT))
			{
				Maze[x][y] |= IS_SOUTH_WALL;
				if (y != 0)
				{
					Maze[x][y-1] |= IS_NORTH_WALL;
				}
			}
			if (!(AvailDirection & AVAIL_RIGHT))
			{
				Maze[x][y] |= IS_NORTH_WALL;
				if (y != 15)
				{
					Maze[x][y+1] |= IS_SOUTH_WALL;
				}
			}
			if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) != (AVAIL_FR | AVAIL_FL))
			{
				Maze[x][y] |= IS_WEST_WALL;
				if (x != 0)
				{
					Maze[x-1][y] |= IS_EAST_WALL;
				}
			}
			break;
	}
}

uint8_t GetMin(uint16_t Num0, uint16_t Num1, uint16_t Num2)
{
		if (Num1 > Num0)
		{
			if (Num0 > Num2)
				return (2);
			else
				return (0);
		}
		else if (Num1 > Num2)
		{
			return(2);
		}
	return (1);
}

uint8_t GetBestDir(uint8_t x, uint8_t y, uint8_t Direction)
{
	if ((AvailDirection & (AVAIL_FR | AVAIL_FL)) == (AVAIL_FR | AVAIL_FL))
	{
		switch (Direction)
		{
			case D_UP:
				Array[1] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
			case D_RIGHT:
				Array[1] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case D_DOWN:
				Array[1] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
			case D_LEFT:
				Array[1] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[1] = 0xffff;
	}
	if (AvailDirection & AVAIL_RIGHT)
	{
		switch (Direction)
		{
			case D_UP:
				Array[2] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case D_RIGHT:
				Array[2] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
			case D_DOWN:
				Array[2] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
			case D_LEFT:
				Array[2] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[2] = 0xffff;
	}
	if (AvailDirection & AVAIL_LEFT)
	{
		switch (Direction)
		{
			case D_UP:
				Array[0] = Maze[x-1][y] & (0x00ff | HAS_GONE);
				break;
			case D_RIGHT:
				Array[0] = Maze[x][y+1] & (0x00ff | HAS_GONE);
				break;
			case D_DOWN:
				Array[0] = Maze[x+1][y] & (0x00ff | HAS_GONE);
				break;
			case D_LEFT:
				Array[0] = Maze[x][y-1] & (0x00ff | HAS_GONE);
				break;
		}
	}
	else
	{
		Array[0] = 0xffff;
	}
	if (AvailDirection == 0)
		return (BACK);
	if (AvailDirection == AVAIL_LEFT)
		return (LEFT);
	if (AvailDirection == AVAIL_RIGHT)
		return (RIGHT);

	switch(GetMin(Array[0], Array[1], Array[2]))
	{
		case 0:
			return (LEFT);
		case 1:
			return (FORWARD);
		case 2:
			return (RIGHT);
	}
	return (FORWARD);
}

