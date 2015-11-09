#ifndef _CONFIGURE_H_
#define _CONFIGURE_H_

//**************  motor  *****************
#define MOTOR_DEADZONE		10
#define POSITION_LIMIT		500
#define PWM_LIMIT					255
#define SAFETY_ANGLE			1
//****************************************

//**********  remote contorl  ************
#define	CLOCKWISE					0x01
#define	ANTICLOCKWISE			0x03
#define FORWARD						0x02
#define	BACKWARD					0X08
#define	LEFT							0X04
#define	RIGHT							0X06
//****************************************

//*************  connect  ****************
#define LED								13	// pin_13	->	L-LED(on board)

// L298 motor driver connected to the Arduino 2560
#define MOTOR_1						6		// pin_6 	->	1-L298_ENA	(timer4)
#define MOTOR_2						7		// pin_7 	->	1-L298_ENB	(timer4)
#define MOTOR_3						8		// pin_8 	->	2-L298_ENB	(timer4)
#define DIR_M1_A					22	// pin_22	->	1-L298_INA
#define DIR_M1_B					23	// pin_23	->	1-L298_INB
#define DIR_M2_A					24	// pin_24	->	1-L298_INC
#define DIR_M2_B					25	// pin_25	->	1-L298_IND
#define DIR_M3_A					26	// pin_26	->	2-L298_IND
#define DIR_M3_B					27	// pin_27	->	2-L298_INC

// Motor encoder connected to the Arduino 2560
// *The encoder signal lines are connected through A shaping with MCU
#define SPD_INT_1					1		// pin_3  ->	Motor_1_encoder_Yellow	(external interruption 1)
#define SPD_INT_2					4		// pin_19 ->	Motor_2_encoder_Yellow	(external interruption 4)
#define SPD_INT_3					5		// pin_18 ->	Motor_3_encoder_Yellow	(external interruption 5)
#define SPD_PUL_1					4		// pin_4	->	Motor_1_encoder_Orange
#define SPD_PUL_2					5		// pin_5	->	Motor_2_encoder_Orange
#define SPD_PUL_3					9		// pin_9	->	Motor_3_encoder_Orange

// MPU6050 connected to the Arduino 2560
#define SENSOR_INT				0		// pin_2  ->	MPU6050_INT	(external interruption 0)
															// pin_20	->	MPU6050_SDA		(SDA)
															// pin_21	->	MPU6050_SCL		(SCL)
// Bluetooth connected to the Arduino 2560
															// pin_16	->	Bluetooth_RX	(TX2)
															// pin_17	->	Bluetooth_TX	(RX2)
															
// By AD input K value, used for debugging
#define K_AGL_AD					A0	// pin_A0	->	(debug)
#define K_AGL_DOT_AD			A1	// pin_A1	->	(debug)
#define K_POS_AD					A2	// pin_A2	->	(debug)
#define K_POS_DOT_AD			A3	// pin_A3	->	(debug)

// L298 motor driver connected to the motor
															// 1-L298_OUTA	->	Motor_1_Black
															// 1-L298_OUTB	->	Motor_1_Red
															// 1-L298_OUTC	->	Motor_2_Grey
															// 1-L298_OUTD	->	Motor_2_Red
															// 2-L298_OUTC	->	Motor_3_Red
															// 2-L298_OUTD	->	Motor_3_Black
//****************************************
#endif