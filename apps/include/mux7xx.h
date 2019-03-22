
#ifndef	__MUX_7XX_H__
#define	__MUX_7XX_H__


/* LED */
#define VIDEO_LED_MAGIC 'V'
#define VIDEO_LED_ON  	_IO(VIDEO_LED_MAGIC,0)  
#define VIDEO_LED_OFF 	_IO(VIDEO_LED_MAGIC,1)

#define POWER_LED_MAGIC 'P'
#define POWER_LED_ON  	_IO(POWER_LED_MAGIC, 0)  
#define POWER_LED_OFF 	_IO(POWER_LED_MAGIC, 1)

#define ACT_LED_MAGIC 'A'
#define ACT_LED_ON  	_IO(ACT_LED_MAGIC, 0)  
#define ACT_LED_OFF 	_IO(ACT_LED_MAGIC, 1)


/* button */
#define BUTTON_MAGIC	'B'
#define READ_BUTTON  	_IOR(BUTTON_MAGIC, 1, unsigned char *)


#define		DEV_HOME				"/dev"

#define		MUX_DEV_BUTTON		DEV_HOME"/muxbutton"
#define		MUX_DEV_SWITCH		DEV_HOME"/muxswitch"
#define		MUX_DEV_LED			DEV_HOME"/powerled"

#define		MUX_DEV_RS232			DEV_HOME"/ttyS1"
#define		MUX_DEV_WATCHDOG	DEV_HOME"/watchdog"
#define		MUX_DEV_EXP_RESET	DEV_HOME"/exprst"
#define		MUX_DEV_HDMI_RESET	DEV_HOME"/hdmirst"	/* no in 774 */

#define		MUX_DEV_I2C_0			DEV_HOME"/i2c-0"
#define		MUX_DEV_I2C_1			DEV_HOME"/i2c-1"

#define		SET_LED_BLINK(muxMain)		((muxMain)->ledCtrl.ledMode = LED_MODE_BLINK)



typedef struct
{
	unsigned char Cmd_Code;
	unsigned char Src_Addr;
	unsigned char Dest_Addr;
} CEC_Cmd_Data;

unsigned int ASCII_2_HEX(unsigned char * ASCIIBuffer);

#define TX_COM_MSG_MAX_DATA_SIZE 508
#define TX_COM_PACKET_SIZE  TX_COM_MSG_MAX_DATA_SIZE + 4

#define INFO_FRAME_PORT 6000

int multicast_ip_change_flag;

#endif

