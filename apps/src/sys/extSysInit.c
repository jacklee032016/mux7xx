#include "mux.h"

int extSystemJsonInit(const cJSON *array)
{
	cJSON *c = array->child;
	size_t i = 0;


	while(c)
	{
		cJSON *type = cJSON_GetObjectItem(c, "type");
		if(type)
		{
			MUX_DEBUG("JSON %s, %s", type->string, type->valuestring);
		}
		
		i++;
		c = c->next;
	}


	return (int)i;
}


int32_t	extSystemInit(MuxMain		*muxMain)
{
	uint32_t ip, mask;
	char *ifName ="eth0";
	EXT_RUNTIME_CFG		*runCfg = &muxMain->runCfg;

	extCfgFromFactory(runCfg);
	
	{
TRACE();
		runCfg->local.ip = cmnSysNetGetIp(ifName);
		runCfg->ipMask = cmnSysNetGetMask(ifName);
		runCfg->ipGateway = cmnSysNetGetDefaultGw(ifName);

		if(cmnSysNetGetMacAddress(ifName, &muxMain->runCfg.local.mac) == EXIT_FAILURE)
		{
			return EXIT_FAILURE;
		}
		
		MUX_DEBUG("IP is %s, 0x%x", cmnSysNetAddress(runCfg->local.ip), runCfg->local.ip);
		MUX_DEBUG("MASK is %s, 0x%x", cmnSysNetAddress(runCfg->ipMask), runCfg->ipMask);
		MUX_DEBUG("Gateway is %s, 0x%x", cmnSysNetAddress(runCfg->ipGateway), runCfg->ipGateway);

		MUX_DEBUG("Mac : %.2X:%.2X:%.2X:%.2X:%.2X:%.2X\n" , runCfg->local.mac.address[0], runCfg->local.mac.address[1], runCfg->local.mac.address[2], 
			runCfg->local.mac.address[3], runCfg->local.mac.address[4], runCfg->local.mac.address[5]);

	}

	{
		cJSON				*muxSystemJson = NULL;
		int count = 0;

		muxSystemJson = cmnMuxJsonLoadConfiguration(MUX_SYSTEM_CONFIG_FILE);
		if (muxSystemJson== NULL)
		{
			MUX_ERROR("IP Command configuration file '%s' Parsing failed", MUX_SYSTEM_CONFIG_FILE);
			return NULL;
		}


		count = extSystemJsonInit(muxSystemJson);
		MUX_DEBUG("Total %d sub functions", count);
	}
	

	return EXIT_SUCCESS;
}

