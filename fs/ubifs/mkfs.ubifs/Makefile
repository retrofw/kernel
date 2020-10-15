ifeq ($(origin CC),default)
CC = gcc
endif

ALL_SOURCES=*.[ch] hashtable/*.[ch]

CFLAGS := $(CFLAGS) -Wall -O2 -I./lzo/include -I/usr/include
#CFLAGS := $(CFLAGS) -Wall -O2 
LDFLAGS := $(LDFLAGS) -lz -llzo2 -lm -L./lzo/lib
#LDFLAGS := $(LDFLAGS) -lz -llzo2 -lm 
TARGETS = mkfs.ubifs

all: $(TARGETS)

OBJ_UBI=crc16.o crc32.o lpt.o compr.o hashtable/hashtable.o hashtable/hashtable_itr.o devtable.o mkfs.ubifs.o

#mkfs.ubifs: crc16.o crc32.o lpt.o compr.o hashtable/hashtable.o \
            hashtable/hashtable_itr.o devtable.o

mkfs.ubifs:$(OBJ_UBI)
	$(CC) -o $@ $(OBJ_UBI) $(LDFLAGS)

clean:
	rm -f *.o $(TARGETS) cscope.*

cscope:
	@echo $(ALL_SOURCES) > cscope.files
	@cscope -bR
	@rm cscope.files
