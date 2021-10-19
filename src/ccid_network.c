/*
	ccid_network.c: Add PCSC Networked device capabilities
    Copyright (C) Springcard   Matthieu Barreteau Sylvain Albert 

  This work is based on Ludovic Rousseau's CCID driver.
  Thanks to his work, I was able to build this library.

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

/*
 * $Id: ccid_usb.c 6975 2014-09-04 11:33:05Z rousseau $
 */

#define __CCID_NETWORK__

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
# ifdef S_SPLINT_S
# include <sys/types.h>
# endif
#include <libusb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h>
#include <ifdhandler.h>


#include <config.h>
#include "misc.h"
#include "ccid.h"
#include "debug.h"
#include "defs.h"
#include "utils.h"
#include "parser.h"
#include "ccid_ifdhandler.h"

/* driver informations */
#define LIB_NAME										"SpringcardNetworkCCID"
#define LIB_VERSION										"1.4.20"

#define NOT_CONNECTED	0
#define CONNECTED		1


/* "usb" descriptors required length */
#define DESC_DEVICE_LEN								29	
#define DESC_CONFIG_LEN								104
#define DESC_VENDOR_LEN								33
#define DESC_PRODUCT_LEN							21
#define DESC_SN_LEN									37
#define DESC_SET_CFG_LEN							11
#define DESC_PING_LEN								11

/* network default port */
#define READER_SOCKET_PORT						3999

/* start or not the reader (SET CONFIGURATION) */
#define READER_OPTION_STOP						0
#define READER_OPTION_START						1


/* for network */
#define READER_OPTION_UNSAFE					0x00
#define READER_OPTION_PLAIN						0x10
#define READER_OPTION_SAFE						0x30


/* endpoints */
#define EP_Control_To_RDR							0x00
#define EP_Control_To_PC							0x80
#define EP_Bulk_RDR_To_PC							0x81		
#define EP_Bulk_PC_To_RDR							0x02		
#define EP_Interrupt								0x83

/* opcode */
#define PC_To_RDR_IccPowerOn					0x62
#define PC_To_RDR_IccPowerOff					0x63
#define PC_To_RDR_GetSlotStatus					0x65
#define PC_To_RDR_XfrBlock						0x6F
#define PC_To_RDR_Escape						0x6B


#define PC_To_RDR_GetParameters						0x6C
#define PC_To_RDR_ResetParameters					0x6D
#define PC_To_RDR_SetParameters						0x61

#define PC_To_RDR_IccClock						0x6E
#define PC_To_RDR_T0APDU						0x6A
#define PC_To_RDR_Secure						0x69
#define PC_To_RDR_Mechanical					0x71
#define PC_To_RDR_Abort							0x72
#define PC_To_RDR_SetDataRateAndClockFrequency	0x73


#define GET_STATUS														0x00
#define SET_CONFIG														0x09
#define GET_DESCRIPTOR												0x06

#define RDR_To_PC_DataBlock										0x80
#define RDR_To_PC_SlotStatus									0x81
#define RDR_To_PC_Parameters									0x82
#define RDR_To_PC_Escape											0x83
#define RDR_To_PC_DataRateAndClockFrequency		0x84
#define RDR_to_PC_NotifySlotChange						0x50
#define RDR_to_PC_AnswerToUnsupportedMessage	0x00


const unsigned char GET_DEVICE[]          = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 1, 0, 0, 0, 0};
const unsigned char GET_CONFIG[]          = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 2, 0, 0, 0, 0};
const unsigned char GET_VENDOR_NAME[]     = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 1, 0, 0, 0};
const unsigned char GET_PRODUCT_NAME[]    = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 2, 0, 0, 0};
const unsigned char GET_SERIAL_NUMBER[]   = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 3, 0, 0, 0};
const unsigned char GET_READER_STATUS[]   = {EP_Control_To_RDR, GET_STATUS,     0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char       SET_CONFIGURATION[]   = {EP_Control_To_RDR, SET_CONFIG,     0, 0, 0, 0, 0, 0, 0, 0, 0};

#define CCID_INTERRUPT_SIZE 8
#define CCID_BUFFER_SIZE	1024
#define SPRINGCARD_MAX_SLOT 8

struct networkDevice_MultiSlot_Extension
{
	int reader_index;

	// The multi-threaded polling part 
	int terminated;
	int status;
	
	unsigned char buffer[CCID_BUFFER_SIZE];
	unsigned int buffer_size;
	

	pthread_t thread_proc;
	pthread_mutex_t mutex;
	pthread_cond_t condition;
	
	
	/* ICC presence part */
	unsigned char								card_status[SPRINGCARD_MAX_SLOT];
	unsigned char								card_error[SPRINGCARD_MAX_SLOT];
	
	int power_on_in_progress[SPRINGCARD_MAX_SLOT];
};


typedef struct
{
	/*
	 * device used ("IP" or "IP:PORT")
	 */
	/*@null@*/ //char 									*device;


	/* Number of slots using the same device */
	int real_nb_opened_slots;
	int *nb_opened_slots;

	/*
	 * CCID infos common to USB and serial
	 */
	_ccid_descriptor ccid;
	
	/*
	 * Network part
	 */
	int													dev_handle;
	int 												status;
	char           							reader_ip[16];
	unsigned int   							reader_port;
	
	/* descriptors and common informations */
	int            										vendorID;
	int													productID;
	int													bcdDevice;	
	char												vendor_name[50];
	char												product_name[50];
	char												serial_number[50];
	unsigned char										options;
	char												key[16];
	


	/* pointer to the multislot extension (if any) */
	struct networkDevice_MultiSlot_Extension *multislot_extension;
} _networkDevice;


/* store ccid descriptor for each slot */
static _networkDevice networkDevice[CCID_DRIVER_MAX_READERS];
static unsigned char driver_setup_is_done = 0;

/* The _networkDevice structure must be defined before including ccid_network.h */
#include "ccid_network.h"

/* Specific hooks for multislot readers */
static struct networkDevice_MultiSlot_Extension *Multi_CreateFirstSlot(int reader_index);
static void Multi_PollingTerminate(struct networkDevice_MultiSlot_Extension *msExt);


#define PCSCLITE_MANUKEY_NAME "ifdVendorID"
#define PCSCLITE_PRODKEY_NAME "ifdProductID"
#define PCSCLITE_NAMEKEY_NAME "ifdFriendlyName"



/****************************************************************************/
/***        clean_endpoint_header                                         ***/
/****************************************************************************/
void clean_endpoint_header(char * astring, int preambule, int len)
{
  int cpt=0;
  
    /* remove endpoint header */  
  for(cpt=0;cpt< (len-preambule);cpt++)
  {    
    astring[cpt] = astring[cpt+11+preambule];
  }
}

/****************************************************************************/
/***        get_ccid_descriptor                                           ***/
/****************************************************************************/
_ccid_descriptor *get_ccid_descriptor(unsigned int reader_index)
{
	return &networkDevice[reader_index].ccid;
} /* get_ccid_descriptor */

/****************************************************************************/
/***        get_string_descriptor_string                                  ***/
/****************************************************************************/
void get_string_descriptor_string(char * astring)
{
  int cpt=0,cpt2=0;
  
  /* remove endpoint header */      
  cpt2=0;
  for(cpt=2;cpt<(astring[0]-2);cpt+=2)
  { 
    astring[cpt2++] = astring[cpt];
  }
  astring[cpt2] = '\0';
  
}
/****************************************************************************/
/***        init_driver                                                   ***/
/****************************************************************************/
void init_driver(void)
{
  unsigned char cpt = 0;
 
  
  if(driver_setup_is_done==0)
  {
   
    for(cpt=0;cpt<CCID_DRIVER_MAX_READERS;cpt++)
    {  
      memset(&networkDevice[cpt], 0, sizeof(_networkDevice));
      networkDevice[cpt].dev_handle = -1;
    }
    driver_setup_is_done = 1;
    DEBUG_INFO1("init " LIB_NAME "Driver version: " LIB_VERSION);
    
  } 
  else 
  {
    DEBUG_INFO1(LIB_NAME "Driver version: " LIB_VERSION);
  }
}


/****************************************************************************/
/***        connectToReader                                               ***/
/****************************************************************************/
static RESPONSECODE connectToReader(unsigned int reader_index)
{
  struct timeval tv;
  struct sockaddr_in reader_addr;
  struct hostent *reader;
    
  DEBUG_INFO2("Reader index: %X", reader_index);
  
  networkDevice[reader_index].dev_handle = socket(AF_INET, SOCK_STREAM, 0);
  if (networkDevice[reader_index].dev_handle < 0)
  {
    DEBUG_INFO1("Unable to create socket");    
    return IFD_COMMUNICATION_ERROR;
  }

  /* add a timeout (750ms) on the socket */
  tv.tv_sec = 2;          /* 0s     */
  tv.tv_usec = 750000;    /* 750 ms */    
  setsockopt(networkDevice[reader_index].dev_handle, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
  setsockopt(networkDevice[reader_index].dev_handle, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,sizeof(struct timeval));

  DEBUG_INFO2("gethostbyname: %s", networkDevice[reader_index].reader_ip);
  reader = gethostbyname(networkDevice[reader_index].reader_ip);
  if (reader == NULL)
  {
    DEBUG_INFO1("Unable to resolv host");     
    close(networkDevice[reader_index].dev_handle);
    return IFD_COMMUNICATION_ERROR;
  }
  
  bzero((char *) &reader_addr, sizeof(reader_addr));
  reader_addr.sin_family = AF_INET;
  bcopy((char *)reader->h_addr,(char *)&reader_addr.sin_addr.s_addr, reader->h_length);
  reader_addr.sin_port = htons(networkDevice[reader_index].reader_port);
  
  DEBUG_INFO2("gethostbyname: %d", networkDevice[reader_index].reader_port);
  
  if (connect(networkDevice[reader_index].dev_handle,(struct sockaddr *) &reader_addr,sizeof(reader_addr)) < 0) 
  { 
    DEBUG_INFO1("Unable to connect to reader, will try later");        
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;    
    return IFD_COMMUNICATION_ERROR;
  }
  
  networkDevice[reader_index].status = CONNECTED;
  
  DEBUG_INFO2("CONNECTED %d", networkDevice[reader_index].reader_port);
  return IFD_SUCCESS;
}

/*****************************************************************************
 *
 *					tryConnection
 *
 ****************************************************************************/
static RESPONSECODE tryConnection(unsigned int reader_index)
{
	char working_buffer[256]; 
	
	if (networkDevice[reader_index].dev_handle > 0)
	{
		close(networkDevice[reader_index].dev_handle);
	}
	
	if( connectToReader(reader_index ) == IFD_SUCCESS )
	{

		// flush socket ?! 
		int r = read(networkDevice[reader_index].dev_handle,working_buffer, 256);
		
		DEBUG_INFO1("Start Configuration");
		// request set configuration and start reader 
		SET_CONFIGURATION[6] = READER_OPTION_START;
		SET_CONFIGURATION[10] = networkDevice[reader_index].options;  
		if(write(networkDevice[reader_index].dev_handle,SET_CONFIGURATION,sizeof(SET_CONFIGURATION)) <0)
		{
			DEBUG_INFO1("Unable to query set configuration and start the reader");        
			close(networkDevice[reader_index].dev_handle);
			networkDevice[reader_index].dev_handle = -1;
			return STATUS_UNSUCCESSFUL;
		} 

		if(read(networkDevice[reader_index].dev_handle,working_buffer,DESC_SET_CFG_LEN) < 0)
		{
			DEBUG_INFO1("Unable to start the reader");              
			close(networkDevice[reader_index].dev_handle);
			networkDevice[reader_index].dev_handle = -1;
			return STATUS_UNSUCCESSFUL;    
		} 
		
		if(working_buffer[10] == READER_OPTION_STOP)
		{
			DEBUG_INFO1("Unable to start the reader");              
			close(networkDevice[reader_index].dev_handle);
			networkDevice[reader_index].dev_handle = -1;
			return STATUS_UNSUCCESSFUL;    
		}  
		
		return IFD_SUCCESS;
	} 
	return STATUS_UNSUCCESSFUL;
}


/*****************************************************************************
 *
 *					OpenNetwork
 *
 ****************************************************************************/
status_t OpenNetwork(unsigned int reader_index, /*@unused@*/ int Channel)
{
	(void)Channel;
	
	DEBUG_INFO2("Reader index: %X", reader_index);
	
	return OpenNetworkByName(reader_index, NULL);
} /* OpenUSB */


/*****************************************************************************
 *
 *					OpenNetworkByName
 *
 ****************************************************************************/
status_t OpenNetworkByName(unsigned int reader_index, /*@null@*/ char *device)
{

  int return_value = STATUS_SUCCESS;
  char device_descriptor[64];
  char configuration_descriptor[256]; 
  char working_buffer[256];  
  int cpt=0,cpt2=0;
  int rv = 0;
  char *dashPosition;
  char *startfind = device;
  unsigned char option_counter = 0;
  int offset = 0; 
  int try = 0;
  
  DEBUG_INFO3("Reader index: %X, Device: %s", reader_index, device);
  
  
  /* setup driver once when first call*/
  init_driver();
  
  
  networkDevice[reader_index].reader_port = READER_SOCKET_PORT;
  networkDevice[reader_index].options = 0x00;
  networkDevice[reader_index].key[0] = '\0';
  
  
  /* look for port in device name */  
  dashPosition = strchr(startfind,':');
  while(dashPosition!=NULL && option_counter<4)
  {
    *dashPosition = '\0';
    switch(option_counter)
    {
      case 0:
      {        
        /* save device IP */
        snprintf(networkDevice[reader_index].reader_ip, 16, "%s", startfind);
        DEBUG_INFO2("Reader IP is %s",networkDevice[reader_index].reader_ip);            
      } break;
      
      case 1:
      {        
        /* save device IP */
        networkDevice[reader_index].reader_port = atoi(startfind);
        DEBUG_INFO2("TCP port %d",networkDevice[reader_index].reader_port);             
      } break;      
      
      case 2:
      {        
        /* save device option */
        networkDevice[reader_index].options = atoi(startfind);
        DEBUG_INFO2("Reader option 0x%02x",networkDevice[reader_index].options);             
      } break;       
      
      case 3:
      {        
        /* save device key */        
        snprintf(networkDevice[reader_index].key, 16, "%s", startfind);
        DEBUG_INFO2("Reader key %s",networkDevice[reader_index].key);             
      } break;       
    }
    option_counter++;
    startfind = dashPosition+1;
    dashPosition = strchr(startfind,':');
  }
 
  networkDevice[reader_index].status =  NOT_CONNECTED;
  /* if socket already here (from a previous Lun) */
  if(networkDevice[reader_index].dev_handle == -1)
  {
    if(connectToReader(reader_index) != IFD_SUCCESS)
    {     
			DEBUG_INFO1("Unable to get reader informations over network");
			return STATUS_UNSUCCESSFUL; 
	  	/* claim success, will retry later but keep reader in list */
    }
  }
  /* get information about reader */
   

  // flush socket ?! 
  rv = read(networkDevice[reader_index].dev_handle, working_buffer, 256);
  if( rv > 0 )
  {
		DEBUG_INFO2("flush socket %d", rv);
	}
    
  // query for reader informations 
  rv = write(networkDevice[reader_index].dev_handle,GET_DEVICE,sizeof(GET_DEVICE));
  if ( rv != sizeof(GET_DEVICE))
  {    
    DEBUG_COMM("Unable to query device descriptor");  
    DEBUG_INFO3("Unable to query device descriptor %d %ld", rv, sizeof(GET_DEVICE) );     
    
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }  
  offset = 0; 
  try = 0;  
full_dd: 
  rv = read(networkDevice[reader_index].dev_handle,&device_descriptor[offset],DESC_DEVICE_LEN-offset);
  if ( rv != DESC_DEVICE_LEN )
  {
		offset += rv;
    try++;
    if( rv > 0 && try < 10)
    {
			goto full_dd;
		}
    DEBUG_COMM("Unable to get proper device descriptor");  
    DEBUG_INFO3("Unable to get proper device descriptor %d %d", rv, DESC_DEVICE_LEN ); 
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  }
  clean_endpoint_header(device_descriptor,0,DESC_DEVICE_LEN);
      
  // query for reader configuration 
  rv = write(networkDevice[reader_index].dev_handle,GET_CONFIG,sizeof(GET_CONFIG));
  if ( rv != sizeof(GET_CONFIG) )
  {
    DEBUG_COMM("Unable to query configuration descriptor");    
    DEBUG_INFO3("Unable to query configuration descriptor %d %ld", rv, sizeof(GET_CONFIG) );     
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }   
  
full_cd: 
  offset = 0; 
  try = 0;   
  rv = read(networkDevice[reader_index].dev_handle,&configuration_descriptor[offset],DESC_CONFIG_LEN-offset);
  if ( rv != DESC_CONFIG_LEN)
  {
		offset += rv;
		try++;
    if( rv > 0 && try < 10)
    {
			goto full_cd;
		}
    DEBUG_COMM("Unable to get proper configuration descriptor");
    DEBUG_INFO3("Unable to get proper configuration descriptor %d %d", rv, DESC_CONFIG_LEN ); 
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  } 
  clean_endpoint_header(configuration_descriptor,18,DESC_CONFIG_LEN);
  
  // query for vendor name 
  rv = write(networkDevice[reader_index].dev_handle,GET_VENDOR_NAME,sizeof(GET_VENDOR_NAME));
  if ( rv != sizeof(GET_VENDOR_NAME))
  {
    DEBUG_COMM("Unable to query vendor name");      
    DEBUG_INFO3("Unable to query vendor name %d %ld", rv, sizeof(GET_VENDOR_NAME) );   
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }    
  offset = 0; 
  try = 0; 
full_vn: 
  rv = read(networkDevice[reader_index].dev_handle,&networkDevice[reader_index].vendor_name[offset],DESC_VENDOR_LEN-offset);
  if( rv != DESC_VENDOR_LEN)
  {
		offset += rv;
		try++;
    if( rv > 0 && try < 10)
    {
			offset += rv;
			goto full_vn;
		}
    DEBUG_COMM("Unable to get vendor name");
    DEBUG_INFO3("Unable to get vendor name %d %d", rv, DESC_VENDOR_LEN );
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return IFD_COMMUNICATION_ERROR;    
  } 
  clean_endpoint_header(networkDevice[reader_index].vendor_name,0,DESC_VENDOR_LEN);
  get_string_descriptor_string(networkDevice[reader_index].vendor_name);

  // query for product name 
  rv = write(networkDevice[reader_index].dev_handle,GET_PRODUCT_NAME,sizeof(GET_PRODUCT_NAME));
  if ( rv != sizeof(GET_PRODUCT_NAME) )
  {
    DEBUG_COMM("Unable to query product name");     
    DEBUG_INFO3("Unable to query product name %d %ld", rv, sizeof(GET_PRODUCT_NAME) );  
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }    
  
  offset = 0; 
  try = 0;  
full_pn :
  rv = read(networkDevice[reader_index].dev_handle,&networkDevice[reader_index].product_name[offset],DESC_PRODUCT_LEN-offset);
  if ( rv !=  DESC_PRODUCT_LEN)
  {
		offset += rv;
		try++;
    if( rv > 0 && try < 10)
    {
			goto full_pn;
		}
    DEBUG_COMM("Unable to get product name");       
    DEBUG_INFO3("Unable to get product name %d %d", rv, DESC_PRODUCT_LEN ); 
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  } 
  clean_endpoint_header(networkDevice[reader_index].product_name,0,DESC_PRODUCT_LEN);
  get_string_descriptor_string(networkDevice[reader_index].product_name);

  // query for serial number 
  rv = write(networkDevice[reader_index].dev_handle,GET_SERIAL_NUMBER,sizeof(GET_SERIAL_NUMBER));
  if ( rv != sizeof(GET_SERIAL_NUMBER))
  {
    DEBUG_COMM("Unable to query serial number");   
    DEBUG_INFO3("Unable to query serial number %d %ld", rv, sizeof(GET_SERIAL_NUMBER) );     
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }    
  
  offset = 0; 
  try = 0; 
full_sn: 
  rv =  read(networkDevice[reader_index].dev_handle,&networkDevice[reader_index].serial_number[offset],DESC_SN_LEN-offset) ;
  if( rv != DESC_SN_LEN)
  {
		offset += rv;
		try++;
    if( rv > 0 && try < 10)
    {
			goto full_sn;
		}
    DEBUG_COMM("Unable to get serial number");        
    DEBUG_INFO3("Unable to get serial number %d %d", rv, DESC_SN_LEN );        
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  } 
  clean_endpoint_header(networkDevice[reader_index].serial_number,0,DESC_SN_LEN);
  get_string_descriptor_string(networkDevice[reader_index].serial_number);
  
  
  DEBUG_INFO2("Vendor Name  : %s",networkDevice[reader_index].vendor_name);  
  DEBUG_INFO2("Product Name : %s",networkDevice[reader_index].product_name);  
  DEBUG_INFO2("Serial Number: %s",networkDevice[reader_index].serial_number);   
  
  //CCID_CLASS_SHORT_APDU
  //configuration_descriptor[42] = 0x02;
  
  DEBUG_INFO2("dwFeatures : 0x%04x",dw2i(configuration_descriptor, 40));  
  DEBUG_INFO5("dwFeatures : %02X %02X %02X %02X ",configuration_descriptor[40], configuration_descriptor[41], configuration_descriptor[42], configuration_descriptor[43]);
    
  DEBUG_INFO1("Start Configuration");
  // request set configuration and start reader 
  SET_CONFIGURATION[6] = READER_OPTION_START;
  SET_CONFIGURATION[10] = networkDevice[reader_index].options;  
  if(write(networkDevice[reader_index].dev_handle,SET_CONFIGURATION,sizeof(SET_CONFIGURATION)) <0)
  {
    DEBUG_INFO1("Unable to query set configuration and start the reader");        
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;
  }      

  if(read(networkDevice[reader_index].dev_handle,working_buffer,DESC_SET_CFG_LEN) < 0)
  {
    DEBUG_INFO1("Unable to start the reader");              
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  } 
  if(working_buffer[10] == READER_OPTION_STOP)
  {
    DEBUG_INFO1("Unable to start the reader");              
    close(networkDevice[reader_index].dev_handle);
    networkDevice[reader_index].dev_handle = -1;
    return STATUS_UNSUCCESSFUL;    
  }    
    
  // create descriptors based on network informations 
  networkDevice[reader_index].vendorID                        = device_descriptor[8]<<8|device_descriptor[9];
  networkDevice[reader_index].productID                       = device_descriptor[10]<<8|device_descriptor[11];
  networkDevice[reader_index].bcdDevice                       = (device_descriptor[12]<<8|device_descriptor[13])<<16;
  
  networkDevice[reader_index].ccid.real_bSeq                  = 0;
  networkDevice[reader_index].ccid.pbSeq                      = &networkDevice[reader_index].ccid.real_bSeq;  
  networkDevice[reader_index].ccid.readerID                   = (networkDevice[reader_index].vendorID << 16) + networkDevice[reader_index].productID;
 
  networkDevice[reader_index].ccid.dwFeatures     						= dw2i(configuration_descriptor,40);
  networkDevice[reader_index].ccid.wLcdLayout                 =	(configuration_descriptor[51] << 8) 
                                                                      + configuration_descriptor[50];
  networkDevice[reader_index].ccid.bPINSupport                = configuration_descriptor[52];
  networkDevice[reader_index].ccid.dwMaxCCIDMessageLength     = dw2i(configuration_descriptor, 44);
  networkDevice[reader_index].ccid.dwMaxIFSD                  = dw2i(configuration_descriptor, 28);
  networkDevice[reader_index].ccid.dwDefaultClock             = dw2i(configuration_descriptor, 10);
  networkDevice[reader_index].ccid.dwMaxDataRate              = dw2i(configuration_descriptor, 23);
  networkDevice[reader_index].ccid.bMaxSlotIndex              = configuration_descriptor[4];
  networkDevice[reader_index].ccid.bCurrentSlotIndex          = 0;
  networkDevice[reader_index].ccid.readTimeout                = DEFAULT_COM_READ_TIMEOUT;
  
  networkDevice[reader_index].ccid.arrayOfSupportedDataRates  = NULL;
  networkDevice[reader_index].ccid.bInterfaceProtocol         = 0;
  networkDevice[reader_index].ccid.bNumEndpoints              = 0; // 3 to force to use own pooling thread 0;
  networkDevice[reader_index].ccid.dwSlotStatus               = IFD_ICC_PRESENT;
  networkDevice[reader_index].ccid.bVoltageSupport            = configuration_descriptor[5];
  networkDevice[reader_index].ccid.sIFD_serial_number         = networkDevice[reader_index].serial_number; //NULL;
  networkDevice[reader_index].ccid.gemalto_firmware_features  = NULL;
  networkDevice[reader_index].ccid.sIFD_iManufacturer					= networkDevice[reader_index].vendor_name;

	/* Vendor-supplied interface device version (DWORD in the form
				 * 0xMMmmbbbb where MM = major version, mm = minor version, and
				 * bbbb = build number). */
	networkDevice[reader_index].ccid.IFD_bcdDevice							= device_descriptor[13]<<24 | device_descriptor[12] << 16;
  
	networkDevice[reader_index].real_nb_opened_slots = (int) (networkDevice[reader_index].ccid.bMaxSlotIndex) + 1 ;
  networkDevice[reader_index].nb_opened_slots = &networkDevice[reader_index].real_nb_opened_slots;

   DEBUG_INFO2("IFD_bcdDevice " DWORD_X "", networkDevice[reader_index].ccid.IFD_bcdDevice); 
	 DEBUG_INFO2("sIFD_serial_number " DWORD_X "", networkDevice[reader_index].ccid.sIFD_serial_number); 
   
  networkDevice[reader_index].status =  CONNECTED;
  
  /* Init thread that read continuously  */
  networkDevice[reader_index].multislot_extension = Multi_CreateFirstSlot(reader_index);

  
  return STATUS_SUCCESS;
} /* OpenNETWORKByName */


/*****************************************************************************
 *
 *					WriteNetwork
 *
 ****************************************************************************/
status_t WriteNetwork(unsigned int reader_index, unsigned int length, unsigned char *buffer)
{
	int rv;
	unsigned char cmd_to_send[CCID_BUFFER_SIZE];
	char debug_header[256];
	int show_debug = 1;
	
	if (networkDevice[reader_index].status != CONNECTED )
	{		
		//if ( tryConnection( reader_index ) != IFD_SUCCESS )
		{
			DEBUG_INFO1( "STATUS_NO_SUCH_DEVICE");
			return STATUS_NO_SUCH_DEVICE;
		}
	}
	
	/* make ccid command like usb */
	if ( buffer[0] == PC_To_RDR_IccPowerOn)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;	
		snprintf( debug_header, 256, "PC_To_RDR_IccPowerOn");
		if( networkDevice[reader_index].multislot_extension != NULL )
		{
			networkDevice[reader_index].multislot_extension->power_on_in_progress[(int)buffer[5]] = 1;
		}
	}
	else if ( buffer[0] == PC_To_RDR_IccPowerOff)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_IccPowerOff");
	}
	else if ( buffer[0] == PC_To_RDR_GetSlotStatus)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_GetSlotStatus");
		show_debug = 1;
	}
	else if ( buffer[0] == PC_To_RDR_XfrBlock)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_XfrBlock");
		
		DEBUG_INFO5("%X %X %X %X", buffer[11], buffer[12], buffer[13], buffer[14]);
	}
	else if ( buffer[0] == PC_To_RDR_Escape)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_Escape");
	}
	else if ( buffer[0] == PC_To_RDR_GetParameters)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_GetParameters");
	}
	else if ( buffer[0] == PC_To_RDR_ResetParameters)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_ResetParameters");
	}
	else if ( buffer[0] == PC_To_RDR_SetParameters)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_Escape");
	}
	else if ( buffer[0] == PC_To_RDR_IccClock)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PCmsExt_To_RDR_IccClock");
	}
	else if ( buffer[0] == PC_To_RDR_T0APDU)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_T0APDU");
	}
	else if ( buffer[0] == PC_To_RDR_Secure	)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_Secure");
	}
	else if ( buffer[0] == PC_To_RDR_Mechanical)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_Mechanical");
	}
	else if ( buffer[0] == PC_To_RDR_Abort)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_Abort");
	}
	else if ( buffer[0] == PC_To_RDR_SetDataRateAndClockFrequency)
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "PC_To_RDR_SetDataRateAndClockFrequency");
	}
	else
	{
		cmd_to_send[0] = EP_Bulk_PC_To_RDR;
		snprintf( debug_header, 256, "Unknow code %X", buffer[0]);
	}
	
	if ( length > CCID_BUFFER_SIZE )
	{
		length = CCID_BUFFER_SIZE;
		DEBUG_INFO1( "CCID Command is over buffer limits");
	}
	memcpy( &cmd_to_send[1], buffer, length );	
	length++;
	
	rv = write(networkDevice[reader_index].dev_handle, cmd_to_send, length);

	if (rv < 0)
	{
		DEBUG_CRITICAL4("write failed (%s): %d %s",
			networkDevice[reader_index].reader_ip,
			rv, strerror(errno));

		DEBUG_INFO1( "STATUS_NO_SUCH_DEVICE");
		networkDevice[reader_index].status = NOT_CONNECTED;
		return STATUS_NO_SUCH_DEVICE;
	}
	
	if (show_debug == 1)
	{
		DEBUG_INFO3("%s -> OK Write %d bytes",debug_header, rv);
	}

	return STATUS_SUCCESS;
} /* WriteNetwork */


/*****************************************************************************
 *
 *					isBufferComplete
 *
 ****************************************************************************/
status_t isBufferComplete( unsigned char *buffer, unsigned int size, int *deal )
{
	unsigned int datalenght = 0;
	
	*deal = 0;
	if( (buffer == NULL) || (size < 11) )
	{
		return STATUS_UNSUCCESSFUL;
	}
	
	datalenght = buffer[5]; datalenght *= 0x0000100;
	datalenght += buffer[4]; datalenght *= 0x0000100;
	datalenght += buffer[3]; datalenght *= 0x0000100;
	datalenght += buffer[2];
	
	if( datalenght > 0x0010000)
	{
		DEBUG_INFO2("Error %d", datalenght);
		*deal = 0;
		return STATUS_SUCCESS;
	}
	if( size == (11+datalenght) )
	{
		*deal = -1;
		return STATUS_SUCCESS;
	}
	if( size > (11+datalenght) )
	{
		DEBUG_INFO3("OK %d but over %d", size, (11+datalenght));
		*deal = (11+datalenght);
		return STATUS_SUCCESS;
	}
	
	return STATUS_UNSUCCESSFUL;
}
	
/*****************************************************************************
 *
 *					ReadNetwork
 *
 ****************************************************************************/
 status_t ReadNetwork(unsigned int reader_index, unsigned int * length, unsigned char *buffer)
{
	struct networkDevice_MultiSlot_Extension *msExt;
	int rv = 0;
	struct timeval local_time;
	struct timespec cond_wait_until;
	int timeout	= 1000; //ms
	status_t status = STATUS_UNSUCCESSFUL;
	int check_read = 0;
	
	msExt = networkDevice[reader_index].multislot_extension;
	if( msExt == NULL )
	{
		DEBUG_INFO1( "Interrupt thread is OFF !!!");
		return STATUS_NO_SUCH_DEVICE;
	}
	
	if (networkDevice[reader_index].status != CONNECTED )
	{
		return STATUS_NO_SUCH_DEVICE;
	}	
  

wait_again:

/* Wait until the condition is signaled or a timeout occurs */
	pthread_mutex_lock(&msExt->mutex);
	
	gettimeofday(&local_time, NULL);
	cond_wait_until.tv_sec = local_time.tv_sec;
	cond_wait_until.tv_nsec = local_time.tv_usec * 1000;

	cond_wait_until.tv_sec += timeout / 1000;
	cond_wait_until.tv_nsec += 1000000 * (timeout % 1000);

	rv = pthread_cond_timedwait(&msExt->condition, &msExt->mutex, &cond_wait_until);

	if (0 == rv)
	{		
		memcpy(buffer, msExt->buffer, msExt->buffer_size );
		*length = msExt->buffer_size;
		status = msExt->status;
	}
	else
	{
		if (rv == ETIMEDOUT)
		{
			check_read++;
			if( check_read > 100 )
			{
				//status = STATUS_UNSUCCESSFUL;
				DEBUG_INFO1( "Timeout");
				status = LIBUSB_TRANSFER_TIMED_OUT;
			}
			else
			{
				pthread_mutex_unlock(&msExt->mutex);
				usleep(100);
				goto wait_again;
			}
		}
		else if (rv == EINVAL)
		{
			DEBUG_CRITICAL2( "pthread_cond_timedwait ETINVAL %s", strerror(errno));	
		}
		else
		{
			DEBUG_CRITICAL3( "pthread_cond_timedwait %d %s", rv, strerror(errno));			
		}
	}
	/* Don't forget to unlock the mutex */
	pthread_mutex_unlock(&msExt->mutex);
	
	
	/*DEBUG_INFO5("%X %X %X %X", buffer[0], buffer[1], buffer[2], buffer[3]);
	DEBUG_INFO5("%X %X %X %X", buffer[4], buffer[5], buffer[6], buffer[7]);
	DEBUG_INFO5("%X %X %X %X", buffer[8], buffer[9], buffer[10], buffer[11]);
	
	DEBUG_INFO3( "Status %d Read %d", status,  *length);*/

  return status;
	
}

/*****************************************************************************
 *
 *					CloseNetwork
 *
 ****************************************************************************/
status_t CloseNetwork(unsigned int reader_index)
{
	char working_buffer[256];
	DEBUG_INFO1("");
	/* device not opened */
	if (networkDevice[reader_index].dev_handle < 0)
		return STATUS_UNSUCCESSFUL;

	DEBUG_COMM2("Closing Network device: %s",networkDevice[reader_index].reader_ip);

	/* one slot closed */
	(*networkDevice[reader_index].nb_opened_slots)--;

	/* release the allocated ressources for the last sDEBUG_INFO3lot only */
	if (0 == *networkDevice[reader_index].nb_opened_slots)
	{
		struct networkDevice_MultiSlot_Extension *msExt = NULL;

		DEBUG_COMM("Last slot closed. Release resources");

		msExt = networkDevice[reader_index].multislot_extension;
		// If this is a multislot reader, close using the multislot stuff 
		if (msExt)
		{
			// terminate the interrupt waiter thread 
			Multi_PollingTerminate(msExt);

			// wait for the thread to actually terminate 
			pthread_join(msExt->thread_proc, NULL);

			// release the shared objects 
			pthread_cond_destroy(&msExt->condition);
			pthread_mutex_destroy(&msExt->mutex);

			// Deallocate the extension itself 
			free(msExt);

			// Stop the slot 
			networkDevice[reader_index].multislot_extension = NULL;
		}
			
		if(networkDevice[reader_index].dev_handle != -1)
		{
			
			DEBUG_INFO1("Stop Configuration");
			// request set configuration and start reader 
			SET_CONFIGURATION[6] = READER_OPTION_STOP;
			SET_CONFIGURATION[10] = networkDevice[reader_index].options;  
			if(write(networkDevice[reader_index].dev_handle,SET_CONFIGURATION,sizeof(SET_CONFIGURATION)) > 0)
			{
				if(read(networkDevice[reader_index].dev_handle,working_buffer,DESC_SET_CFG_LEN) > 0)
				{
					if(working_buffer[10] != READER_OPTION_STOP)
					{
						DEBUG_INFO1("Unable to stop the reader (code)");              
					}
				}
				else
				{
					DEBUG_INFO1("Unable to stop the reader (read)");              
				} 			 
				
			}
			else
			{
				DEBUG_INFO1("Unable to stop the reader (write)");        
			} 
			
			shutdown(networkDevice[reader_index].dev_handle, 2 );
			DEBUG_INFO2("close socket %d", reader_index);
			close(networkDevice[reader_index].dev_handle); 
		}

		/* mark the resource unused */
		networkDevice[reader_index].dev_handle = -1;
	}
	return STATUS_SUCCESS;
} /* CloseNetwork */



/*****************************************************************************
 *
 *					InterruptRead (reserved for possible improvement)
 *
 ****************************************************************************/
int InterruptRead(int reader_index, int timeout /* in ms */)
{
	
	DEBUG_INFO3("################%d %d ms", reader_index, timeout);
	

	/* Multislot reader: redirect to Multi_InterrupRead */
	/*if (networkDevice[reader_index].multislot_extension != NULL)
		return Multi_InterruptRead(reader_index, timeout);
		
	DEBUG_INFO2("%d IFD_COMMUNICATION_ERROR", reader_index);*/
	return STATUS_SUCCESS;

}

/*****************************************************************************
 *
 *					Stop the async loop (reserved for possible improvement)
 *
 ****************************************************************************/
void InterruptStop(int reader_index)
{
	DEBUG_INFO2("################%d ", reader_index);
	

	/* Multislot reader: redirect to Multi_InterrupStop */
	/*if (networkDevice[reader_index].multislot_extension != NULL)
	{
		Multi_InterruptStop(reader_index);
		return;
	}*/
}


/*****************************************************************************
 *
 *					Multi_PollingProc
 *
 ****************************************************************************/
static void *Multi_PollingProc(void *p_ext)
{
	struct networkDevice_MultiSlot_Extension *msExt = p_ext;
	
	int rv;/* ICC presence part */
	unsigned char											there_is_a_card;
	unsigned char											new_card;
	unsigned int offset = 0;
	unsigned int trame_size = 0;
	int deal = 0;

	unsigned char receive_buffer[CCID_BUFFER_SIZE];
	unsigned char extra_buffer[CCID_BUFFER_SIZE];
	int concatenated_answer = 0;
	char debug_header[256];
	int show_debug = 1;
	
	fd_set fds;
	struct timeval read_wait_timeout;
	char ping_buffer[11];
	
	
	DEBUG_INFO3("%d : (%s): thread starting", msExt->reader_index, networkDevice[msExt->reader_index].reader_ip);

	
	//DEBUG_INFO2("ID (%d) ",	t);
	//rv = 0;
	while (!msExt->terminated)
	{
		
select_again:
		if ( (networkDevice[msExt->reader_index].status == CONNECTED) && 
			(networkDevice[msExt->reader_index].dev_handle != -1) )
		{
			if ( msExt->terminated )
			{
				break;
			}
			FD_ZERO( &fds);
			FD_SET( networkDevice[msExt->reader_index].dev_handle, &fds );
	
			read_wait_timeout.tv_sec = 10;
			read_wait_timeout.tv_usec = 0;
			
			rv = select( networkDevice[msExt->reader_index].dev_handle+1, &fds, NULL, NULL, &read_wait_timeout);
			if( rv == -1 )
			{
				DEBUG_INFO1("select error");
				goto read_error;
			}
			else if( rv  && FD_ISSET(networkDevice[msExt->reader_index].dev_handle, &fds) )
			{
				//DEBUG_INFO1("select ok");
			}
			else
			{
				//DEBUG_INFO1("select no data but try read to check connection");
				goto select_again;
			}
			
read_again:			
			rv = read( networkDevice[msExt->reader_index].dev_handle, &receive_buffer[offset], (1024-offset) );
			if (rv < 0)
			{
				DEBUG_INFO4( "%s read error %d (%s)", networkDevice[msExt->reader_index].reader_ip, errno, strerror(errno));
				
				/* request set status (Keep Alive) */          
				if(write(networkDevice[msExt->reader_index].dev_handle, GET_READER_STATUS, sizeof(GET_READER_STATUS)) <0)
				{            
					DEBUG_CRITICAL5("read failed (%s): %d %s %d",
						networkDevice[msExt->reader_index].reader_ip,
						rv, strerror(errno), errno);
					
					DEBUG_INFO1( "STATUS_NO_SUCH_DEVICE");
					networkDevice[msExt->reader_index].status = NOT_CONNECTED;
					
					/* Lock the mutex */
					pthread_mutex_lock(&msExt->mutex);
					
					 /* Set the status and fill-in the interrupt buffer */
					memset(msExt->buffer, 0x00, CCID_BUFFER_SIZE);
					msExt->buffer_size = 0;
					msExt->status = STATUS_NO_SUCH_DEVICE;
					/* Broadcast the condition */
					pthread_cond_broadcast(&msExt->condition);

					/* readUnlock */
					pthread_mutex_unlock(&msExt->mutex);	
					goto read_error;
				}    
				
				/* wait the pong */
				if(read(networkDevice[msExt->reader_index].dev_handle, ping_buffer, DESC_PING_LEN) < 0)
				{           
					DEBUG_CRITICAL5("read failed (%s): %d %s %d",
						networkDevice[msExt->reader_index].reader_ip,
						rv, strerror(errno), errno);
					
					DEBUG_INFO1( "STATUS_NO_SUCH_DEVICE");
					networkDevice[msExt->reader_index].status = NOT_CONNECTED;
					
					/* Lock the mutex */
					pthread_mutex_lock(&msExt->mutex);
					
					 /* Set the status and fill-in the interrupt buffer */
					memset(msExt->buffer, 0x00, CCID_BUFFER_SIZE);
					msExt->buffer_size = 0;
					msExt->status = STATUS_NO_SUCH_DEVICE;
					/* Broadcast the condition */
					pthread_cond_broadcast(&msExt->condition);

					/* readUnlock */
					pthread_mutex_unlock(&msExt->mutex);	
					goto read_error;
				}
			  goto select_again;

			}
			else if( rv == 0 )
			{
				//DEBUG_INFO1( "STATUS_NO_DATA_TO_READ");
				goto select_again;
			}
	
			/* check and slip data if necessary*/
			trame_size =  rv+offset;			
			if( isBufferComplete( receive_buffer, trame_size, &deal ) != STATUS_SUCCESS )
			{
				DEBUG_INFO1( "STATUS_ANSWER_UNCOMPLETE");
				offset += rv;
				goto read_again;
			}
filter_interrupt:
	
			// means two reading concatenated 
			if (deal != -1 )
			{
				DEBUG_INFO4("Detect over buffer trame %d deal %d %d", trame_size, deal, trame_size-deal);				
				memcpy( extra_buffer, &receive_buffer[deal], trame_size-deal );				
				concatenated_answer = trame_size-deal;
			}
			
			if ( receive_buffer[0] == EP_Bulk_RDR_To_PC)
			{
				snprintf( debug_header, 256, "EP_Bulk_RDR_To_PC");
			}
			else if ( receive_buffer[0] == EP_Control_To_PC)
			{
				snprintf( debug_header, 256, "EP_Control_To_PC");
			}
			else if ( receive_buffer[0] == EP_Interrupt)
			{
				if( deal == -1 )
				{
					deal = trame_size;
				}
				snprintf( debug_header, 256, "EP_Interrupt %d", deal);
				DEBUG_INFO3("################### %s Command %02X", debug_header, receive_buffer[11]);
				
				int b = 0;
				int s;
				int slot = 0;
				int slot_status;
				const char *present, *change;
				// log the RDR_to_PC_NotifySlotChange data 
				slot = 0;
				for (b=0; b<(deal-11); b++)
				{
					// 4 slots per byte 
					for (s=0; s<4; s++)
					{
						// 2 bits per slot 
						slot_status = ((receive_buffer[b+11] >> (s*2)) & 3);			

						present = (slot_status & 1) ? "present" : "absent";
						change = (slot_status & 2) ? "status changed" : "no change";
						if (slot_status & 1)
						{
							msExt->card_status[(b*4) + s] &= 0xFD;
						}
						else
						{
							msExt->card_status[(b*4) +s]  = 0x02;
						}
						msExt->card_error[(b*4) +s] = 0x00;						
						
						//DEBUG_INFO3("slot %d status: %d", s + b*4, slot_status);
						//DEBUG_INFO2("CardStatus %02X", msExt->card_status);
					}
					slot += 4;
				}	
			
				if ( concatenated_answer != 0)
				{			
					memcpy( &receive_buffer[0], extra_buffer, concatenated_answer );
					
					DEBUG_INFO5("Copy over buffer %X %X %X %X ", receive_buffer[0], receive_buffer[1], receive_buffer[2], receive_buffer[3]);
					trame_size = concatenated_answer;
					concatenated_answer = 0;
					if( isBufferComplete( receive_buffer, trame_size, &deal ) == STATUS_SUCCESS )
					{
						goto filter_interrupt;
					}
				}
				offset = 0;
				
				goto select_again;
			}
			else
			{
				DEBUG_INFO2("Unknow code %X", receive_buffer[0]);
			}
			trame_size--;
		
			if ( receive_buffer[1] == GET_STATUS)
			{
				snprintf( debug_header, 256, "GET_STATUS");
			}
			else if ( receive_buffer[1] == GET_DESCRIPTOR)
			{
				snprintf( debug_header, 256, "GET_DESCRIPTOR");
			}
			else if ( receive_buffer[1] == SET_CONFIG)
			{
				snprintf( debug_header, 256, "SET_CONFIG");
			}
			if ( receive_buffer[1] == RDR_To_PC_DataBlock)
			{
				snprintf( debug_header, 256, "RDR_To_PC_DataBlock");
				if( msExt->power_on_in_progress[(int)receive_buffer[6]] == 1 )
				{
					msExt->power_on_in_progress[(int)receive_buffer[6]] = 0;
					msExt->card_status[(int)receive_buffer[6]] = receive_buffer[8];
				  msExt->card_error[(int)receive_buffer[6]] = receive_buffer[9];
				  DEBUG_INFO3("power_on_in_progress %d CardStatus %02X", (int)receive_buffer[6], msExt->card_status[(int)receive_buffer[6]]);
				}
			}
			else if ( receive_buffer[1] == RDR_To_PC_SlotStatus)
			{
				snprintf( debug_header, 256, "RDR_To_PC_SlotStatus");	
				msExt->card_status[(int)receive_buffer[6]] = receive_buffer[8];
				msExt->card_error[(int)receive_buffer[6]] = receive_buffer[9];
				
				//DEBUG_INFO2("CardStatus %02X", msExt->card_status[(int)receive_buffer[6]]);
				//DEBUG_INFO4("CardStatus %02X %02X %02X", receive_buffer[6], receive_buffer[8], receive_buffer[9]);
				//show_debug = 0;
			}
			else if ( receive_buffer[1] == RDR_To_PC_Parameters)
			{
				snprintf( debug_header, 256, "RDR_To_PC_Parameters");	
			}	
			else if ( receive_buffer[1] == RDR_To_PC_Escape)
			{
				snprintf( debug_header, 256, "RDR_To_PC_Escape");	
			}
			else if ( receive_buffer[1] == RDR_To_PC_DataRateAndClockFrequency)
			{
				snprintf( debug_header, 256, "RDR_To_PC_DataRateAndClockFrequency");	
			}	
			else if ( receive_buffer[1] == RDR_to_PC_NotifySlotChange)
			{
				snprintf( debug_header, 256, "RDR_to_PC_NotifySlotChange");	
			}
			else if ( receive_buffer[1] == RDR_to_PC_AnswerToUnsupportedMessage)
			{
				snprintf( debug_header, 256, "RDR_to_PC_AnswerToUnsupportedMessage");
			}
			else 
			{
				snprintf( debug_header, 256, "Unknow command code %X", receive_buffer[1]);
			}
			
			/* Lock the mutex */
			pthread_mutex_lock(&msExt->mutex);
			
			 /* Set the status and fill-in the interrupt buffer */
			memcpy(msExt->buffer, &receive_buffer[1], trame_size); 
			msExt->buffer_size = trame_size;
			msExt->status = STATUS_SUCCESS;
			
			/*DEBUG_INFO2("trame_size %d", trame_size);
			DEBUG_INFO5("%X %X %X %X", receive_buffer[1], receive_buffer[2], receive_buffer[3], receive_buffer[4]);
			DEBUG_INFO5("%X %X %X %X", receive_buffer[5], receive_buffer[6], receive_buffer[7], receive_buffer[8]);
			DEBUG_INFO5("%X %X %X %X", receive_buffer[9], receive_buffer[10], receive_buffer[11], receive_buffer[12]);*/
			
			/* Broadcast the condition */
			pthread_cond_broadcast(&msExt->condition);

			/* readUnlock */
			pthread_mutex_unlock(&msExt->mutex);	
			offset = 0;
			
			DEBUG_INFO3("%s read %d", debug_header, trame_size);
		} 
		else 
		{
read_error:	  
			DEBUG_INFO1( "Check Connection -> STATUS_NO_SUCH_DEVICE");

		  if ( tryConnection( msExt->reader_index ) != IFD_SUCCESS )
			{
				DEBUG_INFO1( "Check Connection -> STATUS_NO_SUCH_DEVICE");
				sleep(10);
			}
			else
			{
				DEBUG_INFO1( "Check Connection -> STATUS_DEVICE_IS_PRESENT");
			}
		}
	}

	msExt->terminated = TRUE;


	// Wake up the slot threads so they will exit as well 

	// Lock the mutex 
	pthread_mutex_lock(&msExt->mutex);
	// Set the status and fill-in the read buffer
	msExt->status = 0;
	memset(msExt->buffer, 0xFF, CCID_BUFFER_SIZE);
	msExt->buffer_size = 0;
	// Broadcast the condition 
	pthread_cond_broadcast(&msExt->condition);
	// Unlock 
	pthread_mutex_unlock(&msExt->mutex);

	// Now exit
	DEBUG_INFO2("(%s): Thread terminated", networkDevice[msExt->reader_index].reader_ip);

	pthread_exit(NULL);
	return NULL;
} /* Multi_PollingProc */


/*****************************************************************************
 *
 *					CmdNetworkGetSlotStatus
 *
 ****************************************************************************/
RESPONSECODE CmdNetworkGetSlotStatus(unsigned int reader_index, unsigned char buffer[])
{
	RESPONSECODE return_value = IFD_SUCCESS;

	if( networkDevice[reader_index].multislot_extension == NULL )
	{
		return IFD_COMMUNICATION_ERROR;
	}
	buffer[7] = networkDevice[reader_index].multislot_extension->card_status[reader_index];
	
	/*DEBUG_INFO5("Card Status %02d -> %X %X %X", reader_index, buffer[5], buffer[6], buffer[7]);
	DEBUG_INFO5("%X %X %X %X", buffer[0], buffer[1], buffer[2], buffer[3]);*/

	return return_value;
	
	
}

/*****************************************************************************
 *
 *					Multi_PollingTerminate
 *
 ****************************************************************************/
static void Multi_PollingTerminate(struct networkDevice_MultiSlot_Extension *msExt)
{
	if (msExt && !msExt->terminated)
	{
		msExt->terminated = TRUE;
		DEBUG_INFO2("(%s)", networkDevice[msExt->reader_index].reader_ip);

	}
} /* Multi_PollingTerminate */


/*****************************************************************************
 *
 *					Multi_CreateFirstSlot
 *
 ****************************************************************************/
static struct networkDevice_MultiSlot_Extension *Multi_CreateFirstSlot(int reader_index)
{
	struct networkDevice_MultiSlot_Extension *msExt;
	int i = 0;
	
	// Allocate a new extension buffer 
	msExt = malloc(sizeof(struct networkDevice_MultiSlot_Extension));
	if (NULL == msExt)
		return NULL;

	// Remember the index 
	msExt->reader_index = reader_index;
	msExt->terminated = FALSE;
	msExt->status = 0;
	memset( msExt->buffer, 0x00, CCID_BUFFER_SIZE  );
	msExt->buffer_size = 0;

	for( i = 0; i < SPRINGCARD_MAX_SLOT; i++ )
	{
		msExt->card_status[i] = 0x02;
		msExt->card_error[i] = 0x00;
		msExt->power_on_in_progress[i] = 0;
	}
	
	// Create mutex and condition object for the reading polling 
	pthread_mutex_init(&msExt->mutex, NULL);
	pthread_cond_init(&msExt->condition, NULL);
	
	// create the thread in charge of the interrupt polling 
	pthread_create(&msExt->thread_proc, NULL, Multi_PollingProc, msExt);

	return msExt;
} /* Multi_CreateFirstSlot */




