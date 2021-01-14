/*
    ccid_network.h: Add PCSC Networked device capabilities
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
 * $Id: ccid_usb.h 5473 2011-01-04 09:52:26Z rousseau $
 */

#ifndef __CCID_NETWORK_H__
#define __CCID_NETWORK_H__


status_t OpenNetwork(unsigned int reader_index, int channel);

status_t OpenNetworkByName(unsigned int reader_index, /*@null@*/ char *device);

status_t WriteNetwork(unsigned int reader_index, unsigned int length,
	unsigned char *Buffer);

status_t ReadNetwork(unsigned int reader_index, unsigned int *length,
	/*@out@*/ unsigned char *Buffer);

status_t CloseNetwork(unsigned int reader_index);

int InterruptRead(int reader_index, int timeout);
void InterruptStop(int reader_index);


#endif
