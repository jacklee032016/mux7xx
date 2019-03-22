#
# $Id$
#

include $(RULE_DIR)/Makefile.pre

CODE_SUBDIRS = \
		.
#	-fstack-protector \
#	--param=ssp-buffer-size=4 \
#	-D STANDARD20_MODULE_STUFF 

C_FLAGS += $(SHARED_CFLAGS) -std=gnu99 -pthread -pipe -g -O2 \
	-Wformat -Werror=format-security \
	-DFCGID_APXS_BUILD -DLINUX -D_REENTRANT -D_GNU_SOURCE -D_FORTIFY_SOURCE=2 -I. -I/home/lzj/yocto/774/buildroot/output/host/arm-buildroot-linux-uclibcgnueabihf/sysroot/usr/include/apr-1 \
	

################################################
####   source file 
################################################


SRC += \
		$(PORTABL_SRC) \
		mod_fcgid.c \
		fcgid_bridge.c \
		fcgid_conf.c \
		fcgid_pm_main.c \
		fcgid_protocol.c \
		fcgid_spawn_ctl.c \
		fcgid_proctbl_unix.c \
		fcgid_pm_unix.c \
		fcgid_proc_unix.c \
		fcgid_bucket.c \
		fcgid_filter.c \
		fcgid_mutex_unix.c \
		
 
TARGET_LIB:=mod_fcgid

TARGET_LIB_SHARED:=$(TARGET_LIB).so
TARGET_LIB_STATIC:=$(TARGET_LIB).a
		 
LOCAL_ALL:=

include $(ROOT_DIR)/Makefile.post

