
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


#include "libCmn.h"
#include "mux7xx.h"


int getLocalIp(char * hw_name, char *ip)
{
	char ipAddr[20];
	struct ifaddrs *ifaddr, *ifa;
	int family;

TRACE();
	if (getifaddrs(&ifaddr) == -1)
	{
		MUX_ERROR("getLocalIp: getifaddrs");
		return -1;
	}

	/* Walk through linked list, maintaining head pointer so we
	 can free list later */

	for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
	{
		if (ifa->ifa_addr == NULL)
		{
			continue;
		}

		family = ifa->ifa_addr->sa_family;

		/* Display interface name and family (including symbolic
		 form of the latter for the common families) */

		/* For an AF_INET* interface address, display the address */
		if (strcmp(ifa->ifa_name, hw_name) == 0)
		{
			if (family == AF_INET || family == AF_INET6)
			{
				void *tmpAddrPtr = &((struct sockaddr_in *) ifa->ifa_addr)->sin_addr;
				
				inet_ntop(AF_INET, tmpAddrPtr, ipAddr, INET_ADDRSTRLEN);
				strncpy(ip, ipAddr, 15);
				MUX_DEBUG("\n\n%s   %s\n\n", ip, ipAddr);
				freeifaddrs(ifaddr);
				return 0;
			}
		}
	}
	
	freeifaddrs(ifaddr);
	return -1;
}

int getNetmask(char *hw_name, char *netmask)
{
	struct sockaddr_in *pAddr;
	struct ifreq ifr;
	int sockfd;

TRACE();
	if (hw_name == NULL || netmask == NULL)
	{
		return EXIT_FAILURE;
	}

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&ifr, 0, sizeof(ifr));
	strcpy(ifr.ifr_name, hw_name);

	if (ioctl(sockfd, SIOCGIFNETMASK, &ifr) < 0)
	{
		MUX_ERROR("SIOCGIFADDR socket failed");
		close(sockfd);
		return EXIT_FAILURE;
	}

	pAddr = (struct sockaddr_in *) &(ifr.ifr_addr);
	strcpy(netmask, (char *)(inet_ntoa(pAddr->sin_addr)));
	MUX_DEBUG("mask : %s", netmask)
	close(sockfd);

	return EXIT_SUCCESS;
}



