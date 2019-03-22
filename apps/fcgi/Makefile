#
# $Id: Makefile,v 1.1.1.1 2006/11/29 18:13:58 lizhijie Exp $
#


include $(RULE_DIR)/Makefile.pre


CODE_SUBDIRS += \
			.
		

CFLAGS += $(SHARED_CFLAGS) -I$(RULE_DIR)/libfcgi -I$(RULE_DIR)/libfcgi/include 
################################################
####   source file 
################################################


ifeq ($(ARCH),arm)
	SRC += \
	
else
	SRC += \

endif

SRC += \
		first.c		\
		authorizer.c		\
		echo.c		\
		echo-x.c		\
		log-dump.c		\
		size.c		\
		threaded.c		\


LOCAL_ALL=muxApis authorizer echo echo-x log-dump size threaded

include $(ROOT_DIR)/Makefile.post

LDFLAGS := -L$(BIN_DIR) -L$(BIN_DIR)/lib \
		$(LDFLAGS)

          

ifeq ($(ARCH),arm)
else
endif

LIBS += 
#-lm -lpthread -lrt -ldl 

DEST_DIR=$(BIN_DIR)/var/www/cgi-bin

# -Wl,-verbose -Wl,-static  -s $(SHARED_LDFLAGS) 
muxApis:$(OBJ_DIR)/first.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

authorizer:$(OBJ_DIR)/authorizer.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

echo:$(OBJ_DIR)/echo.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

echo-x:$(OBJ_DIR)/echo-x.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

log-dump:$(OBJ_DIR)/log-dump.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

size:$(OBJ_DIR)/size.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)

threaded:$(OBJ_DIR)/threaded.o
		$(CC) -s -o $@ $< -L$(ROOT_DIR)/Linux.bin.$(ARCH)/lib -lfcgi -lpthread $(LIB_LDFLAGS) $(LIBS) 
		$(MOVE) $@ $(DEST_DIR)
				