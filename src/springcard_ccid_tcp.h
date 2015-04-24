#ifndef __SPRINGCARD_CCID_TCP_H__
#define __SPRINGCARD_CCID_TCP_H__

/* driver informations */
#define LIB_NAME											"Springcard CCID TCP"
#define LIB_VERSION										"0.0.1"

/* frame state machine */
#define WAIT_ENDPOINT 								0
#define WAIT_OPCODE   								1
#define WAIT_LENGHT   								2
#define WAIT_FLAGS    								3
#define WAIT_PAYLOAD  								4
	
/* time limit */	
#define DELAY_WAIT_ATR_MS							50
#define DELAY_WAIT_MS									50
	
/* network default port */
#define READER_SOCKET_PORT						3999

/* how many TOTAL slots to manage */
#define CCID_DRIVER_MAX_READER_SLOT		16

/* endpoints */
#define EP_Control_To_RDR							0x00
#define EP_Control_To_PC							0x80
#define EP_Bulk_RDR_To_PC							0x81		
#define EP_Bulk_PC_To_RDR							0x02		
#define EP_Interrupt									0x83

/* opcode */
#define PC_To_RDR_IccPowerOn					0x62
#define PC_To_RDR_IccPowerOff					0x63
#define PC_To_RDR_GetSlotStatus				0x65
#define PC_To_RDR_XfrBlock						0x6F
#define PC_To_RDR_Escape							0x6B

#define GET_STATUS										0x00
#define SET_CONFIG										0x09
#define GET_DESCRIPTOR								0x06

#define RDR_To_PC_DataBlock						0x80
#define RDR_To_PC_SlotStatus					0x81
#define RDR_To_PC_Escape							0x83
#define RDR_to_PC_NotifySlotChange		0x50

/* wait response type */
#define WAIT_NONE											0
#define WAIT_ATR											1
#define WAIT_APDU											2

/* time limit */
#define WAIT_APDU_LIMIT								5
#define WAIT_ATR_LIMIT								5
#define WAIT_NO_LIMIT									9999

/* "usb" descriptors required length */
#define DESC_DEVICE_LEN								29	
#define DESC_CONFIG_LEN								104
#define DESC_VENDOR_LEN								33
#define DESC_PRODUCT_LEN							21
#define DESC_SN_LEN										37
#define DESC_SET_CFG_LEN							11
#define DESC_PING_LEN									11
		
/* start or not the reader (SET CONFIGURATION) */
#define READER_OPTION_STOP						0
#define READER_OPTION_START						1

/*  for serial */
#define READER_OPTION_FULL_DUPLEX			0x00
#define READER_OPTION_HALF_DUPLEX			0x01

/* for network */
#define READER_OPTION_UNSAFE					0x00
#define READER_OPTION_PLAIN						0x10
#define READER_OPTION_SAFE						0x30


typedef struct
{
	int type;
	int counter;
} _waitState;
	
typedef struct
{
	/*
	 * device used ("IP" or "IP:PORT")
	 */
	/*@null@*/ char 									*device;

	/*
	 * CCID infos common to USB,serial and network
	 */
	_ccid_descriptor 									ccid;
	
	/*
	 * Network part
	 */
	int            										reader_socket;          
	char           										reader_ip[16];
	unsigned int   										reader_port;
	char           										transaction_buffer[256];
	unsigned char  										transaction_length;
	_waitState     										wait_state;
	
	/* descriptors and common informations */
	int            										vendorID;
	int																productID;
	int																bcdDevice;	
	char															vendor_name[50];
	char															product_name[50];
	char															serial_number[50];
	unsigned char											options;
	char															key[16];
	
	/* thread part */
	unsigned char  										kill_thread; /*--> REPLACE with SIGNAL !!! */
	
	/* ICC presence part */
	unsigned char											there_is_a_card;
	unsigned char											new_card;	
	
} _networkDevice;	
	
#endif
/*EOF*/