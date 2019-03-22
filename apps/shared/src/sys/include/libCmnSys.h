#ifndef  __LIB_CMN_SYS_H__
#define	__LIB_CMN_SYS_H__


int cmnSysI2cWrite(int deviceID, unsigned char offset, unsigned char *buffer, int length);
int cmnSysI2cRead(int deviceID, unsigned char offset, unsigned char *buffer, int length);
int getLocalIp(char * hw_name, char *ip);
int getNetmask(char *hw_name, char *netmask);


#endif

