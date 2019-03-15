#include <linux/sockios.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <net/if_arp.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "system_info.h"

void GetGateWay(char * gateway)
{
	FILE *fp;
	char buf[512];
	char cmd[128];
	char *tmp;

	if (gateway == NULL)
		return;

	strcpy(cmd, "ip route");
	fp = popen(cmd, "r");
	if (NULL == fp)
	{
		printf("GetGateWay: popen error\n");
		return;
	}
	while (fgets(buf, sizeof(buf), fp) != NULL)
	{
		tmp = buf;
		while (*tmp && isspace(*tmp))
			++tmp;
		if (strncmp(tmp, "default", strlen("default")) == 0)
			break;
	}
	sscanf(buf, "%*s%*s%s", gateway);
	printf("default gateway:%s\n", gateway);
	pclose(fp);

}

int getLocalIp(char * hw_name, char *ip)
{
	char ipAddr[20];
	struct ifaddrs *ifaddr, *ifa;
	int family;

	if (getifaddrs(&ifaddr) == -1) {
		printf("getLocalIp: getifaddrs\n");
		return -1;
	}

	/* Walk through linked list, maintaining head pointer so we
	 can free list later */

	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
		if (ifa->ifa_addr == NULL)
			continue;

		family = ifa->ifa_addr->sa_family;

		/* Display interface name and family (including symbolic
		 form of the latter for the common families) */

		/* For an AF_INET* interface address, display the address */
		if (strcmp(ifa->ifa_name, hw_name) == 0) {
			if (family == AF_INET || family == AF_INET6) {
				void *tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
				inet_ntop(AF_INET, tmpAddrPtr, ipAddr, INET_ADDRSTRLEN);
				strncpy(ip, ipAddr, 15);
				//printf("\n\n%s   %s\n\n", ip, ipAddr);
				freeifaddrs(ifaddr);
				return 0;
			}
		}
	}
	freeifaddrs(ifaddr);
	return -1;
}

void getNetmask(char *hw_name, char *netmask)
{
	struct sockaddr_in *pAddr;
	struct ifreq ifr;
	int sockfd;

	if (hw_name == NULL || netmask == NULL)
		return;

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&ifr, 0, sizeof(ifr));
	strcpy(ifr.ifr_name, hw_name);

	if (ioctl(sockfd, SIOCGIFNETMASK, &ifr) < 0) {
		printf("SIOCGIFADDR socket failed \n");
		close(sockfd);
		return;
	}

	pAddr = (struct sockaddr_in *) &(ifr.ifr_addr);
	strcpy(netmask, (char *)(inet_ntoa(pAddr->sin_addr)));
	close(sockfd);
}


int readParamFromDrive(char * partitionName, char * param, int bufLens) 
{
	int fd;
	if (!param) 	
		return -1;	
	fd = open(partitionName, O_RDONLY);
	if (fd < 0) 
	{
		printf("failed to open block\n");
		return -1;
	}
	int len = read(fd, param, bufLens);
	if (len == -1) 
	{
		printf("failed to read data from block\n");
		close(fd);
		return -1;
	}	
	close(fd);
	return 0;
}

int saveParamToDrive(char * partitionName, char * param, int strlength) 
{
	int fd;
	if (!param) 
		return -1;	

	char tmp[BLOCK_LENGTH];
	memset(tmp, 0, sizeof(tmp));
	if(strlength < BLOCK_LENGTH)
		memcpy(tmp, param, strlength);
	else
		memcpy(tmp, param, BLOCK_LENGTH);

	fd = open(partitionName, O_WRONLY | O_TRUNC);
	if (fd < 0) 
	{
		printf("failed to open block\n");
		return -1;
	}

	int len = write(fd, tmp, sizeof(tmp));
	if (len == -1) 
	{
		printf("failed to write data to block\n");
		close(fd);
		return -1;
	}

	close(fd);
	return 0;
}

int eraseParamDrive(char * partitionName) 
{
	int fd;
	char tmp[BLOCK_LENGTH];
	memset(tmp, 0, sizeof(tmp));	
	
	fd = open(partitionName, O_WRONLY | O_TRUNC);
	if (fd < 0) 
	{
		printf("failed to open block\n");
		return -1;
	}

	int len = write(fd, tmp, sizeof(tmp));
	if (len == -1) 
	{
		printf("failed to write data to block\n");
		close(fd);
		return -2;
	}

	close(fd);
	return 0;
}

