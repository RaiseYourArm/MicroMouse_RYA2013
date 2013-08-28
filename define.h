//*****************************************************************************
//
// Raise Your Arm 2013_ Micro Mouse robot.
//
// define.h - Prototypes for the library.
//
// This is part of revision 1.2 of the RYA Micro Mouse Library.
//      Happy coding.
//           Support Team RYA!
//*****************************************************************************

#ifndef DEFINE_H_
#define DEFINE_H_


//*****************************************************************************
//
// Type of wall follow.
//
//*****************************************************************************
#define FOLLOW_LEFT			0x01
#define FOLLOW_RIGHT		0x02
#define FOLLOW_DISABLE		0x04
#define FOLLOW_AUTO_SELECT	0x08

//*****************************************************************************
//
// H-Bridge default Frequency: 20 000  (Hz).
// Recommend you NOT TO CHANGE this value.
//
//*****************************************************************************
#define DEFAULT		20000

//*****************************************************************************
//
// Type of Stage of the robot.
//
//*****************************************************************************

#define IDLE		0x00
#define STOP		0x01
#define TURNRIGHT	0x02
#define TURNLEFT	0x04
#define STRAIGHT	0x08

//*****************************************************************************
//
// Type of Available direction
//
//*****************************************************************************
#define AVAIL_LEFT		0x01
#define AVAIL_FL		0x02
#define AVAIL_FR		0x04
#define AVAIL_RIGHT		0x08
#define NOT_AVAIL		0xFE

#define LEFT		0x10
#define RIGHT		0x20
#define BACK		0x40
#define FORWARD		0x80

//*****************************************************************************
//
// Enable port base, enable pin
//
//*****************************************************************************
#define ENAPORT_PERIPHERAL	SYSCTL_PERIPH_GPIOB
#define ENABLE_PORT			GPIO_PORTB_BASE
#define ENA_LEFT_PIN		GPIO_PIN_5
#define ENA_RIGHT_PIN		GPIO_PIN_7
#define BOOST_EN_PIN		GPIO_PIN_2

#define INVALID_ADDRESS			0x00000200
#define INVALID_NUM_OF_WORDS	0x00000400

//*****************************************************************************
//
// Type of Update Map
//
//*****************************************************************************
#define IS_NORTH_WALL		0x8000
#define IS_SOUTH_WALL		0x4000
#define IS_WEST_WALL		0x2000
#define IS_EAST_WALL		0x1000
#define HAS_GONE			0x0800

#endif /* DEFINE_H_ */
