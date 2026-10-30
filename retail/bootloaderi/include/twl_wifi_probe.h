#ifndef TWL_WIFI_PROBE_H
#define TWL_WIFI_PROBE_H

#include <nds/ndstypes.h>

typedef struct {
	u8 sdioRevision;
	u32 manufacturer;
	u32 chipId;
	u32 firmwareUploaded;
} twlWifiProbeResult;

int twlWifiProbe(twlWifiProbeResult* result);

#endif // TWL_WIFI_PROBE_H
