LUSB=$(shell pkg-config --libs libusb)
IUSB=$(shell pkg-config --cflags libusb-1.0)

CC=gcc
LDFLAGS=-shared -fPIC $(LUSB)

CCID_DIR=src/CCID/CCID
CCID_SRC_DIR=$(CCID_DIR)/src

SRC_DIR=src
SRC_LIST=src/springcard_ccid_tcp.c $(CCID_SRC_DIR)/utils.c

CFLAGS=-I$(CCID_SRC_DIR) -I/opt/pcsc/include/PCSC/ -I$(SRC_DIR) $(IUSB)

$(SRC_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

springcard_ccid_tcp: $(SRC_LIST)
	gcc -o $@ $^ $(CFLAGS) $(LDFLAGS)
	mv springcard_ccid_tcp /opt/pcsc/lib/pcsc/drivers/springcard_ccid_tcp.so

.PHONY: clean springcard_ccid_tcp

clean:
	rm -f $(SRC_DIR)/*.o  

