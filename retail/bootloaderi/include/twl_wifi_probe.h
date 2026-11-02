#ifndef TWL_WIFI_PROBE_H
#define TWL_WIFI_PROBE_H

#include <nds/ndstypes.h>

typedef struct {
	u8 sdioRevision;
	u32 manufacturer;
	u32 chipId;
	u32 firmwareUploaded;
	u32 hostInterest0;
	u32 boardData;
	u32 firmwareRam;
	u32 resetCause;
	u32 bmiVersion;
	u8 irqEnable;
	u8 ioEnable;
	u8 ioReady;
	u8 busControl;
} twlWifiProbeResult;

void twlWifiEarlyCheck(void);
int twlWifiProbe(twlWifiProbeResult* result);

#endif // TWL_WIFI_PROBE_H
