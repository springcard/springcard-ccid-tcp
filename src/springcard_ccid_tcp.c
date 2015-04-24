/*
    springcard_ccid_tcp.c: Add PCSC Networked device capabilities
    Copyright (C) Springcard   Matthieu Barreteau
    
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

#include <stdio.h>
#include <errno.h>
#include <syslog.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>	
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits.h>  
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <pthread.h>
#include <libusb.h>

#include "misc.h"
#include "defs.h"
#include "ifdhandler.h"
#include "reader.h"
#include "ccid_ifdhandler.h"
#include "ccid.h"
#include "springcard_ccid_tcp.h"
#include "commands.h"
#include "debug.h"

int LogLevel = DEBUG_LEVEL_CRITICAL | DEBUG_LEVEL_INFO;
// int LogLevel = DEBUG_LEVEL_CRITICAL | DEBUG_LEVEL_INFO | DEBUG_LEVEL_COMM;

/* Array of structures to hold the ATR and other state value of each slot */
static CcidDesc CcidSlots[CCID_DRIVER_MAX_READER_SLOT];  

/* store ccid descriptor for each slot */
static _networkDevice networkDevice[CCID_DRIVER_MAX_READER_SLOT];

const unsigned char GET_DEVICE[]          = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 1, 0, 0, 0, 0};
const unsigned char GET_CONFIG[]          = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 2, 0, 0, 0, 0};
const unsigned char GET_VENDOR_NAME[]     = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 1, 0, 0, 0};
const unsigned char GET_PRODUCT_NAME[]    = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 2, 0, 0, 0};
const unsigned char GET_SERIAL_NUMBER[]   = {EP_Control_To_RDR, GET_DESCRIPTOR, 0, 0, 0, 0, 3, 3, 0, 0, 0};
const unsigned char GET_READER_STATUS[]   = {EP_Control_To_RDR, GET_STATUS,     0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char       SET_CONFIGURATION[]   = {EP_Control_To_RDR, SET_CONFIG,     0, 0, 0, 0, 0, 0, 0, 0, 0};

/* network thread part */
static pthread_t      socket_thread[CCID_DRIVER_MAX_READER_SLOT];

static unsigned char driver_setup_is_done = 0;

/****************************************************************************/
/***        connectToReader                                               ***/
/****************************************************************************/
static RESPONSECODE connectToReader(unsigned int reader_index)
{
  struct timeval tv;
  struct sockaddr_in reader_addr;
  struct hostent *reader;
  
  networkDevice[reader_index].reader_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (networkDevice[reader_index].reader_socket < 0)
  {
    DEBUG_CRITICAL("Unable to create socket");    
    return IFD_COMMUNICATION_ERROR;
  }

  /* add a timeout (750ms) on the socket */
  tv.tv_sec = 0;          /* 0s     */
  tv.tv_usec = 750000;    /* 750 ms */    
  setsockopt(networkDevice[reader_index].reader_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
  setsockopt(networkDevice[reader_index].reader_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv,sizeof(struct timeval));

  reader = gethostbyname(networkDevice[reader_index].reader_ip);
  if (reader == NULL)
  {
    DEBUG_CRITICAL("Unable to resolv host");     
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }
  
  bzero((char *) &reader_addr, sizeof(reader_addr));
  reader_addr.sin_family = AF_INET;
  bcopy((char *)reader->h_addr,(char *)&reader_addr.sin_addr.s_addr, reader->h_length);
  reader_addr.sin_port = htons(networkDevice[reader_index].reader_port);
  
  if (connect(networkDevice[reader_index].reader_socket,(struct sockaddr *) &reader_addr,sizeof(reader_addr)) < 0) 
  { 
    DEBUG_CRITICAL("Unable to connect to reader, will try later");        
    close(networkDevice[reader_index].reader_socket);
    networkDevice[reader_index].reader_socket = -1;    
    return IFD_COMMUNICATION_ERROR;
  }
  return IFD_SUCCESS;
}


/****************************************************************************/
/***        get_ccid_descriptor                                           ***/
/****************************************************************************/
_ccid_descriptor *get_ccid_descriptor(unsigned int reader_index)
{
	return &networkDevice[reader_index].ccid;
} /* get_ccid_descriptor */


/****************************************************************************/
/***        CmdXfrBlockTPDU_T0                                            ***/
/****************************************************************************/
static RESPONSECODE CmdXfrBlockTPDU_T0(unsigned int reader_index,
	unsigned int tx_length, unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[])
{
	RESPONSECODE return_value = IFD_SUCCESS;
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);
  unsigned char buffer_to_use[256];
  int cpt = 0;
  int previous_state = 0;
  
	DEBUG_COMM2("T=0: %d bytes", tx_length);

  memset(buffer_to_use,0,sizeof(buffer_to_use));
  
	/* command length too big for CCID reader? */
	if (tx_length > ccid_descriptor->dwMaxCCIDMessageLength-10)
	{
		DEBUG_CRITICAL3("Command too long (%d bytes) for max: %d bytes",
			tx_length, ccid_descriptor->dwMaxCCIDMessageLength-10);
		return IFD_COMMUNICATION_ERROR;
	}

	/* command length too big for CCID driver? */
	if (tx_length > CMD_BUF_SIZE)
	{
		DEBUG_CRITICAL3("Command too long (%d bytes) for max: %d bytes",
				tx_length, CMD_BUF_SIZE);
		return IFD_COMMUNICATION_ERROR;
	}
  
  /* prepare USB like query */
  buffer_to_use[0] = EP_Bulk_PC_To_RDR;
  buffer_to_use[0+1] = PC_To_RDR_XfrBlock;
  for(cpt=0;cpt<tx_length;cpt++)
  {
    buffer_to_use[cpt+10+1] = tx_buffer[cpt];
  }
  
  buffer_to_use[1+1] = (unsigned char) (tx_length % 256);
  buffer_to_use[2+1] = (unsigned char) (tx_length / 256);
  buffer_to_use[3+1] = 0;
  buffer_to_use[4+1] = 0;
  buffer_to_use[5+1] = ccid_descriptor->bCurrentSlotIndex;	/* slot number */
  buffer_to_use[6+1] = (*ccid_descriptor->pbSeq)++;
  
  /* Clear transaction buffer */
  networkDevice[reader_index].transaction_length = 0;
  networkDevice[reader_index].transaction_buffer[0] = '\0';
    
  /* wait APDU */  
  networkDevice[reader_index].wait_state.type     = WAIT_APDU;
  networkDevice[reader_index].wait_state.counter  = WAIT_APDU_LIMIT;
  previous_state      = WAIT_NO_LIMIT;
  
  DEBUG_INFO1("WAIT_APDU"); 
  while(networkDevice[reader_index].transaction_length == 0 && networkDevice[reader_index].wait_state.counter>=0)
  {
    if(previous_state != networkDevice[reader_index].wait_state.counter)
    {
      previous_state = networkDevice[reader_index].wait_state.counter;
      if(write(networkDevice[reader_index].reader_socket,buffer_to_use,tx_length+10+1) <0)
      {
        DEBUG_CRITICAL("Unable to write APDU"); 
        close(networkDevice[reader_index].reader_socket);
        return IFD_COMMUNICATION_ERROR;    
      }       
    }
  }
  DEBUG_INFO1("AFTER WAIT_APDU"); 
  
  if(networkDevice[reader_index].transaction_length == 0 || networkDevice[reader_index].wait_state.counter<0)
  {
    DEBUG_INFO1("ERROR WAIT_APDU");
    networkDevice[reader_index].wait_state.type     = WAIT_NONE;
    networkDevice[reader_index].wait_state.counter  = WAIT_NO_LIMIT;
    networkDevice[reader_index].there_is_a_card     = 0;
    return IFD_COMMUNICATION_ERROR;
  }

  /* ok, got the thing, disable timeout */
  networkDevice[reader_index].wait_state.type     = WAIT_NONE;
  networkDevice[reader_index].wait_state.counter  = WAIT_NO_LIMIT;  
  
  *rx_length = networkDevice[reader_index].transaction_length;
  memcpy(rx_buffer, networkDevice[reader_index].transaction_buffer, networkDevice[reader_index].transaction_length);
 
  /* Clear transaction buffer */
  networkDevice[reader_index].transaction_length = 0;
  networkDevice[reader_index].transaction_buffer[0] = '\0';  
 
  return IFD_SUCCESS;
} /* CmdXfrBlockTPDU_T0 */


/****************************************************************************/
/***        CmdXfrBlock                                                   ***/
/****************************************************************************/
RESPONSECODE CmdXfrBlock(unsigned int reader_index, unsigned int tx_length,
	unsigned char tx_buffer[], unsigned int *rx_length,
	unsigned char rx_buffer[], int protocol)
{
	RESPONSECODE return_value = IFD_SUCCESS;
	_ccid_descriptor *ccid_descriptor = get_ccid_descriptor(reader_index);

  DEBUG_INFO2("ccid_descriptor->dwFeatures & CCID_CLASS_EXCHANGE_MASK = 0x%04x",(ccid_descriptor->dwFeatures & CCID_CLASS_EXCHANGE_MASK));
  DEBUG_INFO2("ccid_descriptor->dwFeatures 0x%04x",ccid_descriptor->dwFeatures);  
  DEBUG_INFO2("protocol %d",protocol);  
  
	/* APDU or TPDU? */
	switch (ccid_descriptor->dwFeatures & CCID_CLASS_EXCHANGE_MASK)
	{
		case CCID_CLASS_TPDU:
		{
      if (protocol == T_0) 
      {        
        return_value = IFD_PROTOCOL_NOT_SUPPORTED;
        DEBUG_INFO1("CCID_CLASS_TPDU==T_0 not supported");
			} else if (protocol == T_1)
      {        
        return_value = IFD_PROTOCOL_NOT_SUPPORTED;
        DEBUG_INFO1("CCID_CLASS_TPDU==T_1 not supported");
			}	else {        
					return_value = IFD_PROTOCOL_NOT_SUPPORTED;
      }
		}	break;

		case CCID_CLASS_SHORT_APDU:
    {
			return_value = CmdXfrBlockTPDU_T0(reader_index,	tx_length, tx_buffer, rx_length, rx_buffer);      
		}	break;

		case CCID_CLASS_EXTENDED_APDU:
    {
      DEBUG_INFO1("CCID_CLASS_EXTENDED_APDU");
		}	break;

		case CCID_CLASS_CHARACTER:
    {
			if (protocol == T_0)
      {
        return_value = IFD_PROTOCOL_NOT_SUPPORTED;
        DEBUG_INFO1("CCID_CLASS_CHARACTER==T_0 not supported");
			} else if (protocol == T_1)
      {
        return_value = IFD_PROTOCOL_NOT_SUPPORTED;
        DEBUG_INFO1("CCID_CLASS_CHARACTER==T_1 not supported");
			}	else {
				return_value = IFD_PROTOCOL_NOT_SUPPORTED;
      }
		}	break;

		default:
    {
      DEBUG_INFO1("dwFeature not supported");
			return_value = IFD_COMMUNICATION_ERROR;
    }
	}

	return return_value;
} /* CmdXfrBlock */


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
/***        OpenNetPortByName                                             ***/
/****************************************************************************/
status_t OpenNetPortByName(unsigned int reader_index, /*@null@*/ char *device)
{
  int return_value = STATUS_SUCCESS;  
  char device_descriptor[64];
  char configuration_descriptor[256]; 
  char working_buffer[256];  
  int cpt=0,cpt2=0;
  
  DEBUG_INFO3("Reader index: %X, Device: %s", reader_index, device);
  
  /* flush socket ?! */
  read(networkDevice[reader_index].reader_socket,working_buffer,sizeof(working_buffer));
    
  /* query for reader informations */  
  if(write(networkDevice[reader_index].reader_socket,GET_DEVICE,sizeof(GET_DEVICE)) <0)
  {    
    DEBUG_COMM("Unable to query device descriptor");       
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
             
  if(read(networkDevice[reader_index].reader_socket,device_descriptor,DESC_DEVICE_LEN) < 0)
  {
    DEBUG_COMM("Unable to get proper device descriptor");  
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  }
  clean_endpoint_header(device_descriptor,0,DESC_DEVICE_LEN);
      
  /* query for reader configuration */
  if(write(networkDevice[reader_index].reader_socket,GET_CONFIG,sizeof(GET_CONFIG)) <0)
  {
    DEBUG_COMM("Unable to query configuration descriptor");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
      
  if(read(networkDevice[reader_index].reader_socket,configuration_descriptor,DESC_CONFIG_LEN) < 0)
  {
    DEBUG_COMM("Unable to get proper configuration descriptor");
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  } 
  clean_endpoint_header(configuration_descriptor,18,DESC_CONFIG_LEN);
  
  /* query for vendor name */
  if(write(networkDevice[reader_index].reader_socket,GET_VENDOR_NAME,sizeof(GET_VENDOR_NAME)) <0)
  {
    DEBUG_COMM("Unable to query vendor name");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
      
  if(read(networkDevice[reader_index].reader_socket,networkDevice[reader_index].vendor_name,DESC_VENDOR_LEN) < 0)
  {
    DEBUG_COMM("Unable to get vendor name");
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  } 
  clean_endpoint_header(networkDevice[reader_index].vendor_name,0,DESC_VENDOR_LEN);
  get_string_descriptor_string(networkDevice[reader_index].vendor_name);

  /* query for product name */
  if(write(networkDevice[reader_index].reader_socket,GET_PRODUCT_NAME,sizeof(GET_PRODUCT_NAME)) <0)
  {
    DEBUG_COMM("Unable to query product name");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
      
  if(read(networkDevice[reader_index].reader_socket,networkDevice[reader_index].product_name,DESC_PRODUCT_LEN) < 0)
  {
    DEBUG_COMM("Unable to get product name");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  } 
  clean_endpoint_header(networkDevice[reader_index].product_name,0,DESC_PRODUCT_LEN);
  get_string_descriptor_string(networkDevice[reader_index].product_name);

  /* query for serial number */
  if(write(networkDevice[reader_index].reader_socket,GET_SERIAL_NUMBER,sizeof(GET_SERIAL_NUMBER)) <0)
  {
    DEBUG_COMM("Unable to query serial number");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
      
  if(read(networkDevice[reader_index].reader_socket,networkDevice[reader_index].serial_number,DESC_SN_LEN) < 0)
  {
    DEBUG_COMM("Unable to get serial number");              
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  } 
  clean_endpoint_header(networkDevice[reader_index].serial_number,0,DESC_SN_LEN);
  get_string_descriptor_string(networkDevice[reader_index].serial_number);
  
  
  DEBUG_INFO2("Vendor Name  : %s",networkDevice[reader_index].vendor_name);  
  DEBUG_INFO2("Product Name : %s",networkDevice[reader_index].product_name);  
  DEBUG_INFO2("Serial Number: %s",networkDevice[reader_index].serial_number);   
  DEBUG_INFO2("dwFeatures : 0x%04x",dw2i(configuration_descriptor, 40));
  
  /* request set configuration and start reader */
  SET_CONFIGURATION[6] = READER_OPTION_START;
  SET_CONFIGURATION[10] = networkDevice[reader_index].options;  
  if(write(networkDevice[reader_index].reader_socket,SET_CONFIGURATION,sizeof(SET_CONFIGURATION)) <0)
  {
    DEBUG_CRITICAL("Unable to query set configuration and start the reader");        
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;
  }    
      
  if(read(networkDevice[reader_index].reader_socket,working_buffer,DESC_SET_CFG_LEN) < 0)
  {
    DEBUG_CRITICAL("Unable to start the reader");              
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  } 
  if(working_buffer[10] == READER_OPTION_STOP)
  {
    DEBUG_CRITICAL("Unable to start the reader");              
    close(networkDevice[reader_index].reader_socket);
    return IFD_COMMUNICATION_ERROR;    
  }    
    
  /* create descriptors based on network informations */
  networkDevice[reader_index].vendorID                        = device_descriptor[8]<<8|device_descriptor[9];
  networkDevice[reader_index].productID                       = device_descriptor[10]<<8|device_descriptor[11];
  networkDevice[reader_index].bcdDevice                       = (device_descriptor[12]<<8|device_descriptor[13])<<16;
  
  networkDevice[reader_index].ccid.real_bSeq                  = 0;
	networkDevice[reader_index].ccid.pbSeq                      = &networkDevice[reader_index].ccid.real_bSeq;  
  networkDevice[reader_index].ccid.readerID                   = (networkDevice[reader_index].vendorID << 16) + networkDevice[reader_index].productID;
 
  networkDevice[reader_index].ccid.dwFeatures                 = dw2i(configuration_descriptor,40);
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
  networkDevice[reader_index].ccid.bNumEndpoints              = 0;
  networkDevice[reader_index].ccid.dwSlotStatus               = IFD_ICC_PRESENT;
  networkDevice[reader_index].ccid.bVoltageSupport            = configuration_descriptor[5];
  networkDevice[reader_index].ccid.sIFD_serial_number         = NULL;
  networkDevice[reader_index].ccid.gemalto_firmware_features  = NULL;
  
	return return_value;
} /* OpenNetPortByName */

/****************************************************************************/
/***        forcePowerUp                                                  ***/
/****************************************************************************/
void forcePowerUp(unsigned int reader_index)
{  
  unsigned char FORCE_A_POWER_UP[] = {EP_Bulk_PC_To_RDR, PC_To_RDR_IccPowerOn, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  if(networkDevice[reader_index].reader_socket != -1)
  {
    write(networkDevice[reader_index].reader_socket,FORCE_A_POWER_UP,sizeof(FORCE_A_POWER_UP));
  }
}

/****************************************************************************/
/***        networkThread                                                 ***/
/****************************************************************************/
void *networkThread(void *params)
{  
  int received = -1; 
  int cpt = 0;
  int currentStage = WAIT_ENDPOINT;
  long currentCounter = 0;
  int currentPos = 0;
  long msgLen = 0;
  unsigned char receiveBuffer;
  unsigned char working_buffer[256];
  char ping_buffer[11];
  unsigned char complete_frame = 0;  
  unsigned int reader_index;

  /* get reader_index from thread creation call */
  reader_index = (int) params;
  
  DEBUG_COMM2("networkThread started for reader %d",reader_index);  
  
  while(networkDevice[reader_index].kill_thread != 1)
  {
    if(networkDevice[reader_index].reader_socket != -1)
    {
      received = read(networkDevice[reader_index].reader_socket,&receiveBuffer,sizeof(receiveBuffer));
      if(received >0)
      {
        DEBUG_COMM2("received : 0x%02X",receiveBuffer);
        
        switch(currentStage)
        {
          case WAIT_ENDPOINT:
          {
            switch(receiveBuffer)
            {
              case EP_Control_To_RDR:
              case EP_Control_To_PC:
              case EP_Bulk_RDR_To_PC:
              case EP_Bulk_PC_To_RDR:	
              case EP_Interrupt:
              {
                currentPos = 0;  
                working_buffer[currentPos++] = receiveBuffer;
                currentStage = WAIT_OPCODE;
                DEBUG_COMM2("WAIT_ENDPOINT : 0x%02X",receiveBuffer);
                currentCounter = 0;                              
              } break; 
            }
          } break;
          
          case WAIT_OPCODE:
          {           
            DEBUG_COMM2("WAIT_OPCODE : 0x%02X",receiveBuffer);
            working_buffer[currentPos++] = receiveBuffer;
            currentCounter = 4;              
            currentStage = WAIT_LENGHT;
          } break;
          
          case WAIT_LENGHT:
          {
            DEBUG_COMM2("WAIT_LENGHT : 0x%02X",receiveBuffer);
            working_buffer[currentPos+(--currentCounter)] = receiveBuffer;
            if(currentCounter == 0)
            {
              currentCounter = 5;
              currentPos += 4;
              currentStage = WAIT_FLAGS;
            }
          } break;
          
          case WAIT_FLAGS:
          {
            DEBUG_COMM2("WAIT_FLAGS : 0x%02X",receiveBuffer);            
            working_buffer[currentPos+(--currentCounter)] = receiveBuffer;
            if(currentCounter == 0)
            {
              currentCounter = working_buffer[2]<<24 | working_buffer[3]<<16 | working_buffer[4]<<8 | working_buffer[5];
              msgLen = currentCounter;
              currentPos += 4;
              DEBUG_COMM2("networkThread build payload length %ld",msgLen);              
              if(msgLen == 0)
              {
                complete_frame = 1;
              }
              currentStage = WAIT_PAYLOAD;                              
            }            
          } break;   

          case WAIT_PAYLOAD:
          {     
            DEBUG_COMM3("WAIT_PAYLOAD : 0x%02X @ %d",receiveBuffer,currentPos);  
            working_buffer[currentPos++] = receiveBuffer;
            
            --currentCounter;            
            if(currentCounter == 0)
            {
              complete_frame = 1;              
            }                  
          }
        }

        /* if we got a full frame */
        if(complete_frame == 1)
        {
          DEBUG_COMM3("Endpoint 0x%02X Opcode 0x%02X",working_buffer[0], working_buffer[1]);
          
          complete_frame = 0;            
          currentCounter  = 0;
          currentPos      = 0;
          currentStage    = WAIT_ENDPOINT;  
          
          DEBUG_COMM2("wait_state.type = %d",networkDevice[reader_index].wait_state.type);
          
          switch(working_buffer[0])
          {
            case EP_Interrupt:
            {
              if(working_buffer[1] == RDR_to_PC_NotifySlotChange)
              {       
            
                /* this is an interrupt */                
                if(working_buffer[10] == 0x03)
                {
                  networkDevice[reader_index].there_is_a_card = 1;
                  DEBUG_COMM2("Interrupt CARD PRESENT on reader %d",reader_index);
                } else if(working_buffer[10] == 0x02)
                {
                  networkDevice[reader_index].there_is_a_card = 0;
                  DEBUG_COMM2("Interrupt NO CARD OR CARD REMOVED on reader %d",reader_index);
                  
                  /* don't change type but force time out (if type != WAIT_NONE), see next lines */
                  networkDevice[reader_index].wait_state.counter  = 0; 
                }
                
                if(networkDevice[reader_index].wait_state.type != WAIT_NONE)
                {
                  networkDevice[reader_index].wait_state.counter--;                
                }
              }
            } break;
            
            case EP_Control_To_PC:
            {
              DEBUG_COMM("Interrupt EP_Control_To_PC ");             
              for(cpt=0;cpt<msgLen;cpt++) {
                printf(" %02X",working_buffer[cpt]);
              }
              printf("\n");
            } break;     

            case EP_Bulk_RDR_To_PC:
            {
              // DEBUG_INFO2("Interrupt EP_Bulk_RDR_To_PC on slot %d",working_buffer[6]);  
              DEBUG_COMM2("Interrupt EP_Bulk_RDR_To_PC on reader %d",reader_index);  
              DEBUG_COMM2("msgLen == %ld",msgLen);
              
              /* not clean but the only working way from now */
              if(msgLen>0 && msgLen<sizeof(working_buffer))
              {
                if( networkDevice[reader_index].wait_state.type == WAIT_APDU)
                {
                  /* at this stage we got a transaction */
                  memcpy(networkDevice[reader_index].transaction_buffer,working_buffer+10,msgLen);
                  networkDevice[reader_index].transaction_length = msgLen;
                } else {
                  // DEBUG_INFO2("EP_Bulk_RDR_To_PC got ATR for ccid %d",working_buffer[6]);
                  DEBUG_COMM2("EP_Bulk_RDR_To_PC got ATR for reader %d",reader_index);
                  /* at this stage we got an ATR */
                  memcpy(CcidSlots[reader_index].pcATRBuffer,working_buffer+10,msgLen);
                  CcidSlots[reader_index].nATRLength = msgLen; 
                }
              } else {
                DEBUG_COMM2("EP_Bulk_RDR_To_PC error (wait_state.type = %d)",networkDevice[reader_index].wait_state.type);
                if(networkDevice[reader_index].wait_state.type != WAIT_NONE)
                {
                  networkDevice[reader_index].wait_state.counter--;                
                }
              }
            } break;     

            default:
            {
              DEBUG_COMM2("Interrupt OTHER 0x%02X", working_buffer[0]);            
            }
          }            
        }
        
      } else {        

        if(networkDevice[reader_index].wait_state.type != WAIT_NONE)
        {
          if(networkDevice[reader_index].wait_state.counter < 0)
          {
            DEBUG_CRITICAL("NETWORK TIMEOUT");
            close(networkDevice[reader_index].reader_socket);
            networkDevice[reader_index].reader_socket = -1;
          } else {
            DEBUG_COMM4("socket error %d (%s) ->timeout : %d",errno,strerror(errno),--networkDevice[reader_index].wait_state.counter);            
            currentCounter = 0;
            currentStage = WAIT_ENDPOINT; 
          }          
        } else { 
          /* request set status (Keep Alive) */          
          if(write(networkDevice[reader_index].reader_socket,GET_READER_STATUS,sizeof(GET_READER_STATUS)) <0)
          {            
            close(networkDevice[reader_index].reader_socket);
            networkDevice[reader_index].reader_socket = -1;
          }    
          
          /* wait the pong */
          if(read(networkDevice[reader_index].reader_socket,ping_buffer,DESC_PING_LEN) < 0)
          {           
            close(networkDevice[reader_index].reader_socket);
            networkDevice[reader_index].reader_socket = -1;
          }
        
          currentCounter = 0;
          currentStage = WAIT_ENDPOINT; 
        }
      }
    } else {
      /* here we are if a network error occured */
      /* let's try to create connection again */      
            
      while(connectToReader(reader_index) != IFD_SUCCESS)
      {        
        sleep(10);
      } 
      /* get information about reader */
      OpenNetPortByName(reader_index, networkDevice[reader_index].reader_ip);      
      DEBUG_COMM("Connected to reader");          
    }
  }
  DEBUG_INFO1("networkThread ended");
}


/****************************************************************************/
/***        init_driver                                                   ***/
/****************************************************************************/
void init_driver(void)
{
  unsigned char cpt = 0;
 
  
  if(driver_setup_is_done==0)
  {
    
    
    InitReaderIndex();
    
    for(cpt=0;cpt<CCID_DRIVER_MAX_READER_SLOT;cpt++)
    {  
      /* reset some values */      
      memset(&CcidSlots[cpt], 0, sizeof(CcidDesc));
      memset(&networkDevice[cpt], 0, sizeof(_networkDevice));
      
      networkDevice[cpt].kill_thread        = 0;  
      networkDevice[cpt].there_is_a_card    = 0;
      networkDevice[cpt].new_card           = 0;  
      networkDevice[cpt].reader_socket      = -1;
      networkDevice[cpt].wait_state.type    = WAIT_NONE;
      networkDevice[cpt].wait_state.counter = WAIT_NO_LIMIT;    
    }
    driver_setup_is_done = 1;
    DEBUG_INFO1("init " LIB_NAME "Driver version: " LIB_VERSION);
  } else {
    DEBUG_INFO1(LIB_NAME "Driver version: " LIB_VERSION);
  }
}

/****************************************************************************/
/***        IFDHCloseChannel                                              ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHCloseChannel(DWORD Lun)
{	
  /*
	 * This function should close the reader communication channel for the
	 * particular reader.  Prior to closing the communication channel the
	 * reader should make sure the card is powered down and the terminal
	 * is also powered down.
	 *
	 * returns:
	 *
	 * IFD_SUCCESS IFD_COMMUNICATION_ERROR
	 */
	int reader_index;

	if (-1 == (reader_index = LunToReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;

	DEBUG_INFO3("%s (lun: " DWORD_X ")", CcidSlots[reader_index].readerName,
		Lun);
  
  /* ask the thread to die */
  networkDevice[reader_index].kill_thread = 1;
  
  /* wait his death */
  pthread_join( socket_thread[reader_index], NULL);
  
  /* close socket if needed */
  if(networkDevice[reader_index].reader_socket != -1)
  {
    close(networkDevice[reader_index].reader_socket); 
  }
  
	return IFD_SUCCESS;
} /* IFDHCloseChannel */


/****************************************************************************/
/***        CreateChannelByNameOrChannel                                  ***/
/****************************************************************************/
static RESPONSECODE CreateChannelByNameOrChannel(DWORD Lun,	LPSTR lpcDevice, DWORD Channel)
{
  char *dashPosition;
  char *startfind = lpcDevice;
  unsigned char option_counter = 0;
  
  int reader_index;  
  int cpt = 0;
 
  /* setup driver */
  init_driver();
  
	if (lpcDevice)
	{
		DEBUG_INFO3("Lun: " DWORD_X ", device: %s", Lun, lpcDevice);
	}
	else
	{
		DEBUG_INFO3("Lun: " DWORD_X ", Channel: " DWORD_X, Lun, Channel);
	}  
  
  if (-1 == (reader_index = GetNewReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;
    
  DEBUG_INFO2("Working on reader %d", reader_index);    
    
    
  /* Reset ATR buffer */
	CcidSlots[reader_index].nATRLength = 0;
	*CcidSlots[reader_index].pcATRBuffer = '\0';

	/* Reset PowerFlags */
	CcidSlots[reader_index].bPowerFlags = POWERFLAGS_RAZ;
  
  /* reader name */
	CcidSlots[reader_index].readerName = strdup(lpcDevice);  
  
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
        strcpy(networkDevice[reader_index].reader_ip,startfind);
        DEBUG_COMM2("Reader IP is %s",networkDevice[reader_index].reader_ip);            
      } break;
      
      case 1:
      {        
        /* save device IP */
        networkDevice[reader_index].reader_port = atoi(startfind);
        DEBUG_COMM2("TCP port %d",networkDevice[reader_index].reader_port);             
      } break;      
      
      case 2:
      {        
        /* save device option */
        networkDevice[reader_index].options = atoi(startfind);
        DEBUG_COMM2("Reader option 0x%02x",networkDevice[reader_index].options);             
      } break; 
      
      
      case 3:
      {        
        /* save device key */        
        strcpy(networkDevice[reader_index].key,startfind);
        DEBUG_COMM2("Reader key %s",networkDevice[reader_index].key);             
      } break;       
    }
    option_counter++;
    startfind = dashPosition+1;
    dashPosition = strchr(startfind,':');
  } 
  
  
  /* if socket already here (from a previous Lun) */
  if(networkDevice[reader_index].reader_socket == -1)
  {
    if(connectToReader(reader_index) == IFD_SUCCESS)
    {
      /* get information about reader */
      if(OpenNetPortByName(reader_index, lpcDevice) != STATUS_SUCCESS)
      {
        DEBUG_CRITICAL("Unable to get reader informations over network");
        close(networkDevice[reader_index].reader_socket);
        networkDevice[reader_index].reader_socket = -1;
       //return IFD_COMMUNICATION_ERROR; /* claim success, will retry later but keep reader in list */
      }
    }
  }
 
 /* create thread */
  DEBUG_INFO2("Creating networkThread for reader: %d", reader_index);
  if(pthread_create( &socket_thread[reader_index], NULL, networkThread, (void*) reader_index) )
  {
    DEBUG_CRITICAL("Unable to create thread");      
    close(networkDevice[reader_index].reader_socket);
    networkDevice[reader_index].reader_socket = -1;
    //return IFD_COMMUNICATION_ERROR; /* claim success, will retry later but keep reader in list */
  }
  
  return IFD_SUCCESS;
} /* CreateChannelByNameOrChannel */


/****************************************************************************/
/***        IFDHCreateChannel                                             ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHCreateChannel(DWORD Lun, DWORD Channel)
{
	return CreateChannelByNameOrChannel(Lun, NULL, Channel);
} /* IFDHCreateChannel */


/****************************************************************************/
/***        IFDHCreateChannelByName                                       ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHCreateChannelByName(DWORD Lun, LPSTR lpcDevice)
{ 
	return CreateChannelByNameOrChannel(Lun, lpcDevice, -1);
} /* IFDHCreateChannelByName */


/****************************************************************************/
/***        IFDHGetCapabilities                                           ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHGetCapabilities(DWORD Lun, DWORD Tag, PDWORD Length, PUCHAR Value)
{
	/*
	 * This function should get the slot/card capabilities for a
	 * particular slot/card specified by Lun.  Again, if you have only 1
	 * card slot and don't mind loading a new driver for each reader then
	 * ignore Lun.
	 *
	 * Tag - the tag for the information requested example: TAG_IFD_ATR -
	 * return the Atr and it's size (required). these tags are defined in
	 * ifdhandler.h
	 *
	 * Length - the length of the returned data Value - the value of the
	 * data
	 *
	 * returns:
	 *
	 * IFD_SUCCESS IFD_ERROR_TAG
	 */  
	int reader_index;
	RESPONSECODE return_value = IFD_SUCCESS;

	if (-1 == (reader_index = LunToReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;

	DEBUG_INFO4("tag: 0x" DWORD_X ", %s (lun: " DWORD_X ")", Tag,
		CcidSlots[reader_index].readerName, Lun);
  
  switch(Tag)
  {
    case TAG_IFD_THREAD_SAFE:
    {
			if (*Length >= 1)
			{
				*Length = 1;
				*Value = 1; /* Can talk to multiple readers at the same time */
      }
    } break;
    
    case TAG_IFD_SIMULTANEOUS_ACCESS:
    {
			if (*Length >= 1)
			{
				*Length = 1;
				*Value = CCID_DRIVER_MAX_READER_SLOT;
			}	else {
				return_value = IFD_ERROR_INSUFFICIENT_BUFFER;
      }			
    } break;
    
    /**< number of slots of the reader */
    case TAG_IFD_SLOTS_NUMBER :
    {      
      if (*Length >= 1)
			{
				*Length = 1;
				*Value = 1;
      }
      DEBUG_INFO2("Reader supports %d slot(s)", *Value);
    } break;
    
    
    /**< driver uses a polling thread with a timeout parameter */
    case TAG_IFD_POLLING_THREAD_WITH_TIMEOUT :
    {
      /* MUST VERIFY */
      /* default value: not supported */
      *Length = 0;
    } break;
    
    
    /**< the polling thread can be killed */
    case TAG_IFD_POLLING_THREAD_KILLABLE :
    {
      /* MUST VERIFY */
      /* default value: not supported */
      *Length = 0;
    } break;  
    
    
    /**< method used to stop the polling thread (instead of just pthread_kill()) */
    case TAG_IFD_STOP_POLLING_THREAD :
    {
      /* MUST VERIFY */
      /* default value: not supported */
      *Length = 0;
    } break;
    
    case SCARD_ATTR_ATR_STRING :
    case TAG_IFD_ATR:
    {     
      /* If Length is not zero, powerICC has been performed.
			 * Otherwise, return NULL pointer
			 * Buffer size is stored in *Length */
			if ((int)*Length >= CcidSlots[reader_index].nATRLength)
			{
        *Length = CcidSlots[reader_index].nATRLength;
				memcpy(Value, CcidSlots[reader_index].pcATRBuffer, *Length);
			} else {
				return_value = IFD_ERROR_INSUFFICIENT_BUFFER;
      }
    } break;
        
    case SCARD_ATTR_VENDOR_IFD_VERSION:
    { 
      /* Vendor-supplied interface device version (DWORD in the form
       * 0xMMmmbbbb where MM = major version, mm = minor version, and
       * bbbb = build number). */
      *Length = 4;
      if (Value)
        *(uint32_t *)Value = networkDevice[reader_index].bcdDevice;
    } break;
        
    case SCARD_ATTR_VENDOR_NAME:
    {
      /* vendor_name should be found in string descriptor */
      // char vendor_name[] = "Springcard";      
      *Length = strlen(networkDevice[reader_index].vendor_name);
      memcpy(Value, networkDevice[reader_index].vendor_name, *Length);
    } break;
        
    case SCARD_ATTR_MAXINPUT:
    {
      /* get value from configuration descriptor */
      *Length = sizeof(uint32_t);
			if (Value)
				*(uint32_t *)Value = get_ccid_descriptor(reader_index) -> dwMaxCCIDMessageLength -10;      
      
    } break;   
    
    default :
    {      
      DEBUG_INFO2("GetCapabilities unmanaged tag: 0x%lX",Tag);     
      return_value = IFD_ERROR_TAG;
    }
  }
  
  return return_value;
} /* IFDHGetCapabilities */


/****************************************************************************/
/***        IFDHSetCapabilities                                           ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHSetCapabilities(DWORD Lun, DWORD Tag,
	/*@unused@*/ DWORD Length, /*@unused@*/ PUCHAR Value)
{
	/*
	 * This function should set the slot/card capabilities for a
	 * particular slot/card specified by Lun.  Again, if you have only 1
	 * card slot and don't mind loading a new driver for each reader then
	 * ignore Lun.
	 *
	 * Tag - the tag for the information needing set
	 *
	 * Length - the length of the returned data Value - the value of the
	 * data
	 *
	 * returns:
	 *
	 * IFD_SUCCESS IFD_ERROR_TAG IFD_ERROR_SET_FAILURE
	 * IFD_ERROR_VALUE_READ_ONLY
	 */

	(void)Length;
	(void)Value;

	int reader_index;

	if (-1 == (reader_index = LunToReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;

	DEBUG_INFO4("tag: 0x" DWORD_X ", %s (lun: " DWORD_X ")", Tag,
		CcidSlots[reader_index].readerName, Lun);

	return IFD_NOT_SUPPORTED;
} /* IFDHSetCapabilities */


/****************************************************************************/
/***        IFDHPowerICC                                                  ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHPowerICC(DWORD Lun, DWORD Action,	PUCHAR Atr, PDWORD AtrLength)
{
  RESPONSECODE return_value = IFD_SUCCESS;
  int reader_index;
  int previous_state = 0;
  
  unsigned char GET_ATR_UP[] =   {EP_Bulk_PC_To_RDR, PC_To_RDR_IccPowerOn, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  unsigned char GET_ATR_DOWN[] = {EP_Bulk_PC_To_RDR, PC_To_RDR_IccPowerOff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  

	if (-1 == (reader_index = LunToReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;  
  
  /* specify which slot to use */
  /* Need multi-slot reader to check */
  /*
  GET_ATR_UP[6] = reader_index;
  GET_ATR_DOWN[6] = reader_index;
  */
  
  /* in the worst case, say 0 */
  *AtrLength = 0;
  
  DEBUG_COMM2("IFDHPowerICC, Action = %ld",Action);    
  
  switch(Action)
  {
    case IFD_POWER_DOWN:
    {
      DEBUG_COMM("IFD_POWER_DOWN");
      /* Clear ATR buffer */
			CcidSlots[reader_index].nATRLength = 0;
			*CcidSlots[reader_index].pcATRBuffer = '\0';

			/* Memorise the request */
			CcidSlots[reader_index].bPowerFlags |= MASK_POWERFLAGS_PDWN;
      
      /* send a power down request */
      write(networkDevice[reader_index].reader_socket,GET_ATR_DOWN,sizeof(GET_ATR_DOWN));
      
    } break;    
    
    case IFD_POWER_UP:
		case IFD_RESET:
    {
      DEBUG_COMM("IFD_POWER_UP or IFD_RESET");
      if(networkDevice[reader_index].reader_socket != -1)
      {
        /* wait ATR */             
        networkDevice[reader_index].wait_state.type     = WAIT_ATR;
        networkDevice[reader_index].wait_state.counter  = WAIT_ATR_LIMIT;
        previous_state      = WAIT_NO_LIMIT;
        DEBUG_COMM("WAIT_ATR"); 
        while(CcidSlots[reader_index].nATRLength == 0 && networkDevice[reader_index].wait_state.counter>=0)
        {
          if(previous_state != networkDevice[reader_index].wait_state.counter)
          {
            previous_state = networkDevice[reader_index].wait_state.counter;
            if(write(networkDevice[reader_index].reader_socket,GET_ATR_UP,sizeof(GET_ATR_UP)) < 0)
            {
              DEBUG_COMM("Unable to power up");    
              return IFD_ERROR_POWER_ACTION;
            }       
          }
        }
        DEBUG_COMM("AFTER WAIT_ATR"); 
        
        if(CcidSlots[reader_index].nATRLength ==0 || networkDevice[reader_index].wait_state.counter<=0)
        {
          DEBUG_COMM("ERROR WAIT_ATR"); 
          networkDevice[reader_index].wait_state.type     = WAIT_NONE;
          networkDevice[reader_index].wait_state.counter  = WAIT_NO_LIMIT;          
          networkDevice[reader_index].there_is_a_card     = 0;
          return IFD_COMMUNICATION_ERROR;
        } 
        
        /* ok, got the thing, disable timeout */
        networkDevice[reader_index].wait_state.type     = WAIT_NONE;
        networkDevice[reader_index].wait_state.counter  = WAIT_NO_LIMIT;
        
        /* Power up successful, set state variable to memorise it */
        CcidSlots[reader_index].bPowerFlags |= MASK_POWERFLAGS_PUP;
        CcidSlots[reader_index].bPowerFlags &= ~MASK_POWERFLAGS_PDWN;
        
        /* give ATR and is length (received in the network thread) */
        *AtrLength = CcidSlots[reader_index].nATRLength;
        memcpy(Atr, CcidSlots[reader_index].pcATRBuffer, *AtrLength);
        
        
      }
    } break;
    
    case SCARD_ATTR_VENDOR_IFD_VERSION:
    {
    }
    break;
    
    default:
    {
      return_value = IFD_NOT_SUPPORTED;
    }
  }
  
  
  return return_value;  
} /* IFDHPowerICC */


/****************************************************************************/
/***        IFDHTransmitToICC                                             ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHTransmitToICC(DWORD Lun, SCARD_IO_HEADER SendPci,
	PUCHAR TxBuffer, DWORD TxLength,
	PUCHAR RxBuffer, PDWORD RxLength, /*@unused@*/ PSCARD_IO_HEADER RecvPci)
{
	/*
	 * This function performs an APDU exchange with the card/slot
	 * specified by Lun.  The driver is responsible for performing any
	 * protocol specific exchanges such as T=0/1 ... differences.  Calling
	 * this function will abstract all protocol differences.
	 *
	 * SendPci Protocol - 0, 1, .... 14 Length - Not used.
	 *
	 * TxBuffer - Transmit APDU example (0x00 0xA4 0x00 0x00 0x02 0x3F
	 * 0x00) TxLength - Length of this buffer. RxBuffer - Receive APDU
	 * example (0x61 0x14) RxLength - Length of the received APDU.  This
	 * function will be passed the size of the buffer of RxBuffer and this
	 * function is responsible for setting this to the length of the
	 * received APDU.  This should be ZERO on all errors.  The resource
	 * manager will take responsibility of zeroing out any temporary APDU
	 * buffers for security reasons.
	 *
	 * RecvPci Protocol - 0, 1, .... 14 Length - Not used.
	 *
	 * Notes: The driver is responsible for knowing what type of card it
	 * has.  If the current slot/card contains a memory card then this
	 * command should ignore the Protocol and use the MCT style commands
	 * for support for these style cards and transmit them appropriately.
	 * If your reader does not support memory cards or you don't want to
	 * then ignore this.
	 *
	 * RxLength should be set to zero on error.
	 *
	 * returns:
	 *
	 * IFD_SUCCESS IFD_COMMUNICATION_ERROR IFD_RESPONSE_TIMEOUT
	 * IFD_ICC_NOT_PRESENT IFD_PROTOCOL_NOT_SUPPORTED
	 */

	RESPONSECODE return_value;
	unsigned int rx_length;
	int reader_index;

	(void)RecvPci;

	if (-1 == (reader_index = LunToReaderIndex(Lun)))
		return IFD_COMMUNICATION_ERROR;

	DEBUG_INFO3("%s (lun: " DWORD_X ")", CcidSlots[reader_index].readerName,
		Lun);

	rx_length = *RxLength;  
	return_value = CmdXfrBlock(reader_index, TxLength, TxBuffer, &rx_length,
		RxBuffer, SendPci.Protocol);
	if (IFD_SUCCESS == return_value)
		*RxLength = rx_length;
	else
		*RxLength = 0;
  
	return return_value;
} /* IFDHTransmitToICC */


/****************************************************************************/
/***        IFDHICCPresence                                               ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHICCPresence(DWORD Lun)
{
  int cpt = 0;
  char buffer[256];
  int reader_index;
  
  if (-1 == (reader_index = LunToReaderIndex(Lun)))
  return IFD_COMMUNICATION_ERROR;
  
  if(networkDevice[reader_index].there_is_a_card  == 1)
  {     
    return IFD_ICC_PRESENT;
  } else {    
    return IFD_ICC_NOT_PRESENT;
  }
} /* IFDHICCPresence */


/****************************************************************************/
/***        IFDHSetProtocolParameters                                     ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHSetProtocolParameters(DWORD Lun, DWORD Protocol,
	UCHAR Flags, UCHAR PTS1, UCHAR PTS2, UCHAR PTS3)
{
  DEBUG_INFO1("IFDHSetProtocolParameters");    

  return IFD_SUCCESS;
} /* IFDHSetProtocolParameters */

/****************************************************************************/
/***        IFDHControl                                                   ***/
/****************************************************************************/
EXTERNAL RESPONSECODE IFDHControl(DWORD Lun, DWORD dwControlCode,
	PUCHAR TxBuffer, DWORD TxLength, PUCHAR RxBuffer, DWORD RxLength,
	PDWORD pdwBytesReturned)
{
  /*
	 * This function performs a data exchange with the reader (not the
	 * card) specified by Lun.  Here XXXX will only be used. It is
	 * responsible for abstracting functionality such as PIN pads,
	 * biometrics, LCD panels, etc.  You should follow the MCT, CTBCS
	 * specifications for a list of accepted commands to implement.
	 *
	 * TxBuffer - Transmit data TxLength - Length of this buffer. RxBuffer
	 * - Receive data RxLength - Length of the received data.  This
	 * function will be passed the length of the buffer RxBuffer and it
	 * must set this to the length of the received data.
	 *
	 * Notes: RxLength should be zero on error.
	 */
	RESPONSECODE return_value = IFD_ERROR_NOT_SUPPORTED;  
	int reader_index;
	_ccid_descriptor *ccid_descriptor;

	reader_index = LunToReaderIndex(Lun);
	if ((-1 == reader_index) || (NULL == pdwBytesReturned))
		return IFD_COMMUNICATION_ERROR;

	ccid_descriptor = get_ccid_descriptor(reader_index);

	DEBUG_INFO4("ControlCode: 0x" DWORD_X ", %s (lun: " DWORD_X ")",
		dwControlCode, CcidSlots[reader_index].readerName, Lun);
	DEBUG_INFO_XXD("Control TxBuffer: ", TxBuffer, TxLength);

	/* Set the return length to 0 to avoid problems */
	*pdwBytesReturned = 0;
  
  if (IOCTL_SMARTCARD_VENDOR_IFD_EXCHANGE == dwControlCode)
	{   
      DEBUG_INFO1("ifd exchange (Escape command) not allowed");
			return_value = IFD_COMMUNICATION_ERROR;		
  }

	if (IFD_SUCCESS != return_value)
		*pdwBytesReturned = 0;

	DEBUG_INFO_XXD("Control RxBuffer: ", RxBuffer, *pdwBytesReturned);
	return return_value;
} /* IFDHControl */

/*EOF*/
