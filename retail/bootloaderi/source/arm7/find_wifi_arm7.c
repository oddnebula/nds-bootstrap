#include <stddef.h> // NULL
#include "find.h"
#include "debug_file.h"

extern u32 newArm7binarySize;

//
// WM (wireless manager) request dispatcher, ARM7
//
// WMSP_RequestThread receives WM API requests from the ARM9 over PXI and calls
// the handler at WmspRequestFuncTable[apiId]. The table layout is the same in
// every SDK build checked so far (pokediamond and pokeheartgold arm7):
//   0x0C WMSP_StartConnectEx, 0x0D WMSP_Disconnect, 0x11 WMSP_StartDCF,
//   0x12 WMSP_SetDCFData, 0x13 WMSP_EndDCF, 0x14 WMSP_SetWEPKey,
//   0x26 WMSP_StartScanEx, 0x27 WMSP_SetWEPKeyEx
// Everything below this table is the NTR-mode WL driver, so this is the seam a
// WPA implementation would hook.
//
// The dispatch is (registers differ between SDK builds):
//   cmp   rId, #count
//   ...
//   ldr   r1, [rTable, rId, lsl #2]
//   mov   lr, pc
//   bx    r1
//

#define WM_DISPATCH_CALL_LR  0xE1A0E00F // mov lr, pc
#define WM_DISPATCH_CALL_BX  0xE12FFF11 // bx r1
#define WM_DISPATCH_LDR      0xE7901100 // ldr r1, [rN, rM, lsl #2]
#define WM_DISPATCH_LDR_MASK 0xFFF0FFF0
#define WM_DISPATCH_CMP      0xE3500000 // cmp rM, #imm
#define WM_DISPATCH_LDR_PC   0xE59F0000 // ldr rN, [pc, #imm]

// table is the candidate in the loaded image, at runtime address tableAddr
static bool isWmRequestFuncTable(const u32* start, u32 t, u32 count, u32 tableAddr) {
	const u32 base = tableAddr - t*sizeof(u32); // runtime address of start[0]
	for (u32 i = 0; i < count; i++) {
		const u32 func = start[t+i];
		// Handlers are ARM code in the same module, placed before the table
		if ((func & 3) || func < base || func >= tableAddr) {
			return false;
		}
		// and start with an unconditional instruction
		if ((start[(func - base)/sizeof(u32)] & 0xF0000000) != 0xE0000000) {
			return false;
		}
	}
	return true;
}

u32* findWmRequestFuncTable(const u32* start, u32 size, u32* count) {
	const u32 len = size/sizeof(u32);

	for (u32 i = 1; i+1 < len; i++) {
		if (start[i] != WM_DISPATCH_CALL_LR || start[i+1] != WM_DISPATCH_CALL_BX
		 || (start[i-1] & WM_DISPATCH_LDR_MASK) != WM_DISPATCH_LDR) {
			continue;
		}
		const u32 rTable = (start[i-1] >> 16) & 0xF;
		const u32 rId = start[i-1] & 0xF;

		// cmp rId, #count
		u32 apiCount = 0;
		for (u32 j = 2; j <= 16 && j <= i; j++) {
			if ((start[i-j] & 0xFFFFFF00) == (WM_DISPATCH_CMP | (rId << 16))) {
				apiCount = start[i-j] & 0xFF;
				break;
			}
		}
		if (apiCount == 0) {
			continue;
		}

		// ldr rTable, =WmspRequestFuncTable
		u32 tableAddr = 0;
		u32 ldrPos = 0;
		for (u32 j = 2; j <= 32 && j <= i; j++) {
			if ((start[i-j] & 0xFFFFF000) == (WM_DISPATCH_LDR_PC | (rTable << 12))) {
				ldrPos = i-j;
				const u32 lit = ldrPos + 2 + (start[ldrPos] & 0xFFF)/sizeof(u32);
				if (lit < len) {
					tableAddr = start[lit];
				}
				break;
			}
		}
		if (tableAddr == 0) {
			continue;
		}

		// The WM code runs from an autoload section, so the table's load
		// address is not known yet. Find it by content.
		for (u32 t = i+2; t + apiCount <= len && t - i < 0x40000/sizeof(u32); t++) {
			if (isWmRequestFuncTable(start, t, apiCount, tableAddr)) {
				*count = apiCount;
				return (u32*)(start + t);
			}
		}
	}

	return NULL;
}

u32* findWmRequestFuncTableOffset(const tNDSHeader* ndsHeader, u32* count) {
	dbg_printf("findWmRequestFuncTableOffset:\n");

	u32* offset = findWmRequestFuncTable(ndsHeader->arm7destination, newArm7binarySize, count);

	if (offset) {
		dbg_printf("WM request func table found\n");
	} else {
		dbg_printf("WM request func table not found\n");
	}

	dbg_printf("\n");
	return offset;
}
