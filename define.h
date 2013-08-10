/*
 * define.h
 *
 *  Created on: Aug 2, 2013
 *      Author: Admin
 */

#ifndef DEFINE_H_
#define DEFINE_H_

#define DEBUG_WALL_FOLLOW
//#define TEST_ENCODER
//#define TEST_IR
//#define SET_PID
//#define PID_SPEED
//#define PID_WALL
//#define PID_POSITION
//#define TEST_AVAIL_DIR

//#define SAVE_EEPROM

#define FOLLOW_LEFT			0x01
#define FOLLOW_RIGHT		0x02
#define FOLLOW_DISABLE		0x04
#define FOLLOW_AUTO_SELECT	0x08

#define FrameLength		sizeof(str_Network_Frame)

#define DEFAULT		20000	//H-Bridge Freq (Hz)

#define STOP		0x01
#define IDLE		0
#define TURNRIGHT	0x02
#define STRAIGHT	0x08
//#define BACK		0x04
#define TURNLEFT	0x04

#define AVAIL_LEFT		0x01
#define AVAIL_FL		0x02
#define AVAIL_FR		0x04
#define AVAIL_RIGHT		0x08
#define NOT_AVAIL		0xFE

#define LEFT		0x10
#define RIGHT		0x20
//#define LEFT_BACK	0x40
//#define RIGHT_BACK	0x50
#define BACK		0x40
#define FORWARD		0x80

#define ENAPORT_PERIPHERAL	SYSCTL_PERIPH_GPIOB
#define ENABLE_PORT			GPIO_PORTB_BASE
#define ENA_LEFT_PIN		GPIO_PIN_5
#define ENA_RIGHT_PIN		GPIO_PIN_7
#define BOOST_EN_PIN		GPIO_PIN_2

#define IS_WALL		0x80
#define HAS_GONE	0x40

#define INVALID_ADDRESS			0x00000200
#define INVALID_NUM_OF_WORDS	0x00000400

#endif /* DEFINE_H_ */
