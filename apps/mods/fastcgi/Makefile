#
# $Id$
#

include $(RULE_DIR)/Makefile.pre

CODE_SUBDIRS = \
		.
#	-fstack-protector \

C_FLAGS += $(SHARED_CFLAGS) -std=gnu99 -pthread -pipe -g -O2 \
	--param=ssp-buffer-size=4 \
	-Wformat -Werror=format-security \
	-DLINUX -D_REENTRANT -D_GNU_SOURCE -D_FORTIFY_SOURCE=2 -I/home/lzj/yocto/774/buildroot/output/host/arm-buildroot-linux-uclibcgnueabihf/sysroot/usr/include/apr-1 \
	-D STANDARD20_MODULE_STUFF 


################################################
####   source file 
################################################


SRC += \
		$(PORTABL_SRC) \
		mod_fastcgi.c \
		fcgi_pm.c	\
		fcgi_util.c	\
		fcgi_protocol.c	\
		fcgi_buf.c \
		fcgi_config.c
		
 
TARGET_LIB:=mod_fastcgi

TARGET_LIB_SHARED:=$(TARGET_LIB).so
TARGET_LIB_STATIC:=$(TARGET_LIB).a
		 
LOCAL_ALL:=

include $(ROOT_DIR)/Makefile.post

