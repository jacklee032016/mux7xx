#
# $Id$
#

include $(RULE_DIR)/Makefile.pre

CODE_SUBDIRS = \
		src

C_FLAGS += $(SHARED_CFLAGS) -std=gnu99 -pthread -pipe -g -O2 \
	-DHAVE_CONFIG_H -Iinclude 


################################################
####   source file 
################################################


SRC += \
		src/fcgiapp.c \
		src/fcgi_stdio.c	\
		src/os_unix.c
		
 
TARGET_LIB:=libfcgi

TARGET_LIB_SHARED:=$(TARGET_LIB).so
TARGET_LIB_STATIC:=$(TARGET_LIB).a
		 
LOCAL_ALL:=

include $(ROOT_DIR)/Makefile.post

