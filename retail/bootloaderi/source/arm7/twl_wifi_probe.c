/*
 * DSi-mode (Atheros) WiFi probe, ARM7
 *
 * Brings up the Atheros WiFi chip over the second TMIO SDIO controller and logs
 * what it reports, so DSi-mode WiFi can be checked on real hardware before the
 * game starts. Only CMD52 (single register) accesses are used. The chip is reset
 * into its bootloader (BMI) to read its version; no firmware is uploaded.
 *
 * The SDIO controller code and the bring-up sequence are based on DSiWifi:
 *   Copyright (c) 2021 Max Thomas
 *   Copyright (c) 2015-2016, Daz Jones
 *   Distributed under the MIT license, see dsiwifi_license.txt.
 * Changes from DSiWifi: every wait has a timeout, and only the parts needed to
 * identify the chip are kept.
 */

#include <nds/ndstypes.h>
#include <nds/system.h>
#include <nds/bios.h>
#include <nds/arm7/i2c.h>
#include "debug_file.h"
#include "twl_wifi_probe.h"

#define REG_GPIO_WIFI	*(vu16*)0x4004C04
#define REG_SCFG_WL	*(vu16*)0x4004020

#define SDIO_BASE	0x04004A00

// TMIO register offsets
#define SDIO_CMD		0x000
#define SDIO_PORT_SEL		0x002
#define SDIO_CMD_PARAM0		0x004
#define SDIO_CMD_PARAM1		0x006
#define SDIO_STOP		0x008
#define SDIO_DATA16_BLK_CNT	0x00A
#define SDIO_RESP0		0x00C
#define SDIO_RESP1		0x00E
#define SDIO_IRQ_STAT0		0x01C
#define SDIO_IRQ_STAT1		0x01E
#define SDIO_IRQ_MASK		0x020
#define SDIO_CLK_CNT		0x024
#define SDIO_DATA16_BLK_LEN	0x026
#define SDIO_CARD_OPT		0x028
#define SDIO_DATA16_CNT		0x0D8
#define SDIO_RESET		0x0E0
#define SDIO_IRQ32		0x100
#define SDIO_DATA32_BLK_LEN	0x104
#define SDIO_DATA32_BLK_CNT	0x108

#define STAT0_CMDRESPEND	0x0001
#define STAT1_CMD_BUSY		0x4000
#define STAT1_ERR		0x807F // ILL_ACCESS, CMDTIMEOUT, TXUNDERRUN, RXOVERFLOW, DATATIMEOUT, STOPBIT_ERR, CRCFAIL, CMD_IDX_ERR

// Command register values (index | response type << 8)
#define CMD0	(0  | (3 << 8)) // GO_IDLE_STATE, no response
#define CMD3	(3  | (4 << 8)) // SEND_RELATIVE_ADDR, 48-bit response
#define CMD5	(5  | (7 << 8)) // IO_SEND_OP_COND, 48-bit OCR response, no CRC7
#define CMD7	(7  | (5 << 8)) // SELECT_CARD, 48-bit response with busy
#define CMD52	(52 | (4 << 8)) // IO_RW_DIRECT, 48-bit response

#define TIMEOUT	0x100000

// Atheros internal addresses
#define AR6002_HOST_INTEREST_ADDRESS	0x00500400
#define AR601x_HOST_INTEREST_ADDRESS	0x00520000
#define CHIP_ID_AR6002	0x02000001

// Function 1 registers
#define F1_MBOX			0x0FF
#define F1_RX_LOOKAHEAD_VALID	0x405
#define F1_WINDOW_DATA		0x474
#define F1_WINDOW_WRITE_ADDR	0x478
#define F1_WINDOW_READ_ADDR	0x47C

#define BMI_GET_TARGET_ID	0x8

static u32 sdioResp = 0;
static u16 sdioRca = 0;
static bool sdioError = false;
static bool readOnly = false;
static u16 sdioFirstErrCmd = 0;
static u32 sdioFirstErrArg = 0;
static u16 sdioFirstErrStat = 0;
static bool cardInitSkipped = false;
static bool functionToggled = false;

static inline u16 sdioRead16(u32 reg) {
	return *(vu16*)(SDIO_BASE + reg);
}

static inline void sdioWrite16(u32 reg, u16 val) {
	*(vu16*)(SDIO_BASE + reg) = val;
}

static void sdioMask16(u32 reg, u16 clear, u16 set) {
	sdioWrite16(reg, (sdioRead16(reg) & ~clear) | set);
}

static void sdioSetBusWidth(int width) {
	sdioMask16(SDIO_PORT_SEL, 3, 0);
	sdioMask16(SDIO_CLK_CNT, 0x100, 0);
	sdioMask16(SDIO_CLK_CNT, 0x2FF, 0); // HCLK/2
	sdioMask16(SDIO_CLK_CNT, 0, 0x100);
	if (width == 4) {
		sdioMask16(SDIO_CARD_OPT, BIT(15), 0);
	} else {
		sdioMask16(SDIO_CARD_OPT, 0, BIT(15));
	}
}

static void sdioControllerInit(void) {
	sdioMask16(SDIO_IRQ32, 0x1800, 0x0402);
	sdioMask16(SDIO_DATA16_CNT, 0x0022, 0x0002);
	sdioMask16(SDIO_DATA16_CNT, 0x0020, 0x0000);
	sdioWrite16(SDIO_DATA32_BLK_LEN, 128);
	sdioWrite16(SDIO_DATA32_BLK_CNT, 1);
	sdioMask16(SDIO_RESET, 3, 0);
	sdioMask16(SDIO_RESET, 0, 3);
	*(vu32*)(SDIO_BASE + SDIO_IRQ_MASK) = 0xFFFFFFFF; // Disable all interrupts
	sdioMask16(0x0FC, 0, 0x00DB);
	sdioMask16(0x0FE, 0, 0x00DB);
	sdioMask16(SDIO_PORT_SEL, 3, 0);
	sdioWrite16(SDIO_CLK_CNT, 0x0020);
	sdioWrite16(SDIO_CARD_OPT, 0x40EE);
	sdioMask16(SDIO_IRQ32, 0x8000, 0);
	sdioMask16(SDIO_IRQ32, 0, 0x0100);
	sdioMask16(SDIO_IRQ32, 0x0100, 0);
	sdioWrite16(SDIO_DATA16_BLK_LEN, 128);
	sdioWrite16(SDIO_STOP, 0x0100);
}

// Sends a command without data. Returns false on error or timeout.
static bool sdioCommand(u16 cmd, u32 arg) {
	int i;
	for (i = 0; i < TIMEOUT && (sdioRead16(SDIO_IRQ_STAT1) & STAT1_CMD_BUSY); i++);
	if (i == TIMEOUT) {
		sdioError = true;
		return false;
	}

	sdioWrite16(SDIO_IRQ_STAT0, 0);
	sdioWrite16(SDIO_IRQ_STAT1, 0);
	sdioWrite16(SDIO_CMD_PARAM0, arg & 0xFFFF);
	sdioWrite16(SDIO_CMD_PARAM1, arg >> 16);
	sdioWrite16(SDIO_DATA16_BLK_LEN, 128);
	sdioWrite16(SDIO_DATA16_BLK_CNT, 0);
	sdioWrite16(SDIO_CMD, cmd);

	bool ok = false;
	u16 stat1Last = 0;
	for (i = 0; i < TIMEOUT; i++) {
		const u16 stat1 = sdioRead16(SDIO_IRQ_STAT1);
		stat1Last = stat1;
		if (stat1 & STAT1_ERR) {
			break;
		}
		if (!(stat1 & STAT1_CMD_BUSY) && (sdioRead16(SDIO_IRQ_STAT0) & STAT0_CMDRESPEND)) {
			ok = true;
			break;
		}
	}

	sdioResp = sdioRead16(SDIO_RESP0) | ((u32)sdioRead16(SDIO_RESP1) << 16);
	sdioWrite16(SDIO_IRQ_STAT0, 0);
	sdioWrite16(SDIO_IRQ_STAT1, 0);
	if (!ok) {
		if (!sdioFirstErrCmd) {
			sdioFirstErrCmd = cmd;
			sdioFirstErrArg = arg;
			sdioFirstErrStat = stat1Last;
		}
		sdioError = true;
	}
	return ok;
}

static u8 cmd52Read(u8 func, u32 addr) {
	if (!sdioCommand(CMD52, (func << 28) | ((addr & 0x1FFFF) << 9))) {
		return 0xFF;
	}
	return sdioResp & 0xFF;
}

static bool cmd52Write(u8 func, u32 addr, u8 val) {
	return sdioCommand(CMD52, BIT(31) | (func << 28) | ((addr & 0x1FFFF) << 9) | val);
}

static u32 read32(u8 func, u32 addr) {
	u32 val = 0;
	for (int i = 0; i < 4; i++) {
		val |= (u32)cmd52Read(func, addr+i) << (i*8);
	}
	return val;
}

static void write32Msb(u8 func, u32 addr, u32 val) {
	// Write from MSB to LSB: the LSB write triggers the access
	for (int i = 3; i >= 0; i--) {
		cmd52Write(func, addr+i, (val >> (i*8)) & 0xFF);
	}
}

static u32 readInternWord(u32 addr) {
	write32Msb(1, F1_WINDOW_READ_ADDR, addr);
	return read32(1, F1_WINDOW_DATA);
}

static void writeInternWord(u32 addr, u32 val) {
	write32Msb(1, F1_WINDOW_DATA, val);
	write32Msb(1, F1_WINDOW_WRITE_ADDR, addr);
}

static void mboxWrite32(u32 val) {
	for (int i = 0; i < 4; i++) {
		cmd52Write(1, F1_MBOX, val & 0xFF);
		val >>= 8;
	}
}

static bool mboxRead32(u32* val) {
	int i;
	for (i = 0; i < 0x4000 && !sdioError && !(cmd52Read(1, F1_RX_LOOKAHEAD_VALID) & 1); i++);
	if (i == 0x4000 || sdioError) {
		return false;
	}

	*val = 0;
	for (i = 0; i < 4; i++) {
		*val |= (u32)cmd52Read(1, F1_MBOX) << (i*8);
	}
	return !sdioError;
}

static bool bmiGetVersion(u32* version) {
	mboxWrite32(BMI_GET_TARGET_ID);
	if (!mboxRead32(version)) {
		return false;
	}
	if (*version != 0xFFFFFFFF) {
		return true;
	}

	// Extended reply: length, version, then the rest
	u32 len = 0;
	if (!mboxRead32(&len) || len < 8 || len > 0x40 || !mboxRead32(version)) {
		return false;
	}
	for (u32 i = 0; i < (len/4)-2; i++) {
		u32 unused;
		if (!mboxRead32(&unused)) {
			return false;
		}
	}
	return true;
}

static void logValue(const char* name, u32 val) {
	dbg_printf(name);
	dbg_hexa(val);
	dbg_printf("\n");
}

// Controller registers changed by the probe, in restore order
static const u16 sdioSavedRegs[] = {
	SDIO_IRQ32, SDIO_DATA16_CNT, 0x0FC, 0x0FE, SDIO_CARD_OPT, SDIO_PORT_SEL,
	SDIO_DATA16_BLK_LEN, SDIO_DATA32_BLK_LEN, SDIO_DATA32_BLK_CNT, SDIO_STOP,
	SDIO_CLK_CNT, SDIO_RESET
};
#define SDIO_SAVED_COUNT (sizeof(sdioSavedRegs)/sizeof(sdioSavedRegs[0]))

static void sdioSave(u16* saved, u32* savedIrqMask) {
	for (u32 i = 0; i < SDIO_SAVED_COUNT; i++) {
		saved[i] = sdioRead16(sdioSavedRegs[i]);
	}
	*savedIrqMask = *(vu32*)(SDIO_BASE + SDIO_IRQ_MASK);
}

static void sdioRestore(const u16* saved, u32 savedIrqMask) {
	sdioMask16(SDIO_CLK_CNT, 0x100, 0); // Stop the clock
	sdioWrite16(SDIO_IRQ_STAT0, 0);
	sdioWrite16(SDIO_IRQ_STAT1, 0);
	*(vu32*)(SDIO_BASE + SDIO_IRQ_MASK) = savedIrqMask;
	for (u32 i = 0; i < SDIO_SAVED_COUNT; i++) {
		sdioWrite16(sdioSavedRegs[i], saved[i]);
	}
}

// Puts the chip's SDIO side back to its power-on state, then the controller.
// Replies aren't checked: the card stops answering once it's reset.
static void sdioShutdown(const u16* saved, u32 savedIrqMask) {
	cmd52Write(0, 0x04, 0x00); // Interrupts off
	cmd52Write(0, 0x02, 0x00); // Function 1 off
	cmd52Write(0, 0x07, 0x00); // 1-bit bus
	sdioSetBusWidth(1);
	cmd52Write(0, 0x06, 0x08); // I/O reset
	swiDelay(0xF000);

	sdioRestore(saved, savedIrqMask);
}

static int probe(twlWifiProbeResult* result) {
	sdioControllerInit();
	sdioSetBusWidth(4);
	swiDelay(0xF000);

	// Fall back to 1-bit if the card doesn't answer in 4-bit mode
	cmd52Read(0, 0x00);
	if (sdioError) {
		sdioError = false;
		sdioSetBusWidth(1);
	}

	sdioWrite16(SDIO_STOP, 0x100);

	// The card ignores CMD5 after a warm start unless this comes first
	if (readOnly) {
		sdioCommand(CMD0, 0);
		sdioError = false;
		swiDelay(0xF000);
	}

	// CMD5 until the card reports it's ready
	u32 ocr = 0;
	bool skipInit = false;
	int i;
	for (i = 0; i < 0x1000; i++) {
		if (!sdioCommand(CMD5, ocr)) {
			// Already initialized (e.g. by the system menu)
			sdioError = false;
			skipInit = true;
			cardInitSkipped = true;
			break;
		}
		ocr = sdioResp & 0xFFFFFF;
		if (sdioResp & 0x80000000) {
			break;
		}
	}
	if (i == 0x1000) {
		return -1;
	}

	if (!skipInit) {
		if (!sdioCommand(CMD3, 0)) return -2;
		sdioRca = sdioResp >> 16;
		if (!sdioCommand(CMD7, (u32)sdioRca << 16)) return -3;
	}

	if (!readOnly) {
		cmd52Write(0, 0x02, 0x00);
		cmd52Write(0, 0x02, 0x02);
		swiDelay(0xF000);
	}

	if (!readOnly) {
		// Bus Interface Control: 4-bit mode and card detect
		const u8 busControl = cmd52Read(0, 0x07);
		if (sdioError) return -4;
		if (!cmd52Write(0, 0x07, busControl | 0x82)) return -5;
		sdioSetBusWidth(4);

		cmd52Write(0, 0x12, 0x02); // Power Control
		cmd52Write(0, 0x110, 128); // Function 1 block size
		cmd52Write(0, 0x111, 0);
		cmd52Write(0, 0x10, 128); // Function 0 block size
		cmd52Write(0, 0x11, 0);
		if (sdioError) return -6;
		swiDelay(0xF000);
	}

	result->sdioRevision = cmd52Read(0, 0x00);

	if (readOnly) {
		result->ioEnable = cmd52Read(0, 0x02);
		result->ioReady = cmd52Read(0, 0x03);
		result->busControl = cmd52Read(0, 0x07);
	}

	// Enable function 1 and wait until it's ready
	cmd52Write(0, 0x02, 0x02);
	for (i = 0; i < 0x10000 && !sdioError && cmd52Read(0, 0x03) != 0x02; i++);
	if (i == 0x10000 || sdioError) return -7;

	// A chip left running by the system may not answer on function 1 until
	// the function is toggled
	if (readOnly) {
		cmd52Read(1, F1_RX_LOOKAHEAD_VALID);
		if (sdioError) {
			sdioError = false;
			functionToggled = true;
			cmd52Write(0, 0x02, 0x00);
			cmd52Write(0, 0x02, 0x02);
			for (i = 0; i < 0x10000 && !sdioError && cmd52Read(0, 0x03) != 0x02; i++);
			if (i == 0x10000 || sdioError) return -11;
		}
	}

	result->manufacturer = read32(0, 0x1007);
	result->chipId = readInternWord(0x000040EC);
	const u32 hostInterest = (result->chipId == CHIP_ID_AR6002) ? AR6002_HOST_INTEREST_ADDRESS : AR601x_HOST_INTEREST_ADDRESS;
	result->firmwareUploaded = readInternWord(hostInterest + 0x58);
	result->hostInterest0 = readInternWord(hostInterest);
	result->boardData = readInternWord(hostInterest + 0x54);
	result->firmwareRam = readInternWord(0x00524C00);
	if (sdioError) return -8;

	if (readOnly) {
		return 0;
	}

	// Reset into the bootloader and ask for its version
	writeInternWord(0x4000, 0x00000100);
	swiDelay(0x10000);
	result->resetCause = readInternWord(0x40C0);
	if (sdioError) return -9;
	if (!bmiGetVersion(&result->bmiVersion)) return -10;

	return 0;
}

static void resetResult(twlWifiProbeResult* result) {
	result->sdioRevision = 0;
	result->manufacturer = 0;
	result->chipId = 0;
	result->firmwareUploaded = 0;
	result->hostInterest0 = 0;
	result->boardData = 0;
	result->firmwareRam = 0;
	result->resetCause = 0;
	result->bmiVersion = 0;
	result->ioEnable = 0;
	result->ioReady = 0;
	result->busControl = 0;
	sdioError = false;
	sdioFirstErrCmd = 0;
	sdioFirstErrArg = 0;
	sdioFirstErrStat = 0;
	cardInitSkipped = false;
	functionToggled = false;
}

// Looks at the chip as it was left by whatever ran before us. Doesn't touch the
// LED/reset register or the chip's CPU, and leaves the card initialized. Function
// 1 is only toggled if it doesn't answer.
void twlWifiEarlyCheck(void) {
	dbg_printf("twlWifiEarlyCheck:\n");

	twlWifiProbeResult result;
	resetResult(&result);
	readOnly = true;

	const u16 oldGpioWifi = REG_GPIO_WIFI;
	logValue("GPIO_WIFI : ", oldGpioWifi);
	logValue("SCFG_WL : ", REG_SCFG_WL);
	logValue("LED : ", i2cReadRegister(0x4A, 0x30));
	logValue("WL firm info 0 : ", *(vu32*)0x020005E0);
	logValue("WL firm info 1 : ", *(vu32*)0x020005E4);

	u16 sdioSaved[SDIO_SAVED_COUNT];
	u32 sdioSavedIrqMask;
	sdioSave(sdioSaved, &sdioSavedIrqMask);

	REG_GPIO_WIFI &= ~BIT(8); // Select Atheros
	const int ret = probe(&result);
	sdioRestore(sdioSaved, sdioSavedIrqMask);
	REG_GPIO_WIFI = oldGpioWifi;

	readOnly = false;

	logValue("Result : ", (u32)ret);
	logValue("Card init skipped : ", cardInitSkipped);
	logValue("Function toggled : ", functionToggled);
	logValue("SDIO revision : ", result.sdioRevision);
	logValue("Function enable : ", result.ioEnable);
	logValue("Function ready : ", result.ioReady);
	logValue("Bus control : ", result.busControl);
	logValue("Manufacturer : ", result.manufacturer);
	logValue("First failed cmd : ", sdioFirstErrCmd);
	logValue("First failed arg : ", sdioFirstErrArg);
	logValue("First failed stat : ", sdioFirstErrStat);
	logValue("Chip ID : ", result.chipId);
	logValue("Firmware uploaded : ", result.firmwareUploaded);
	logValue("Host interest 0 : ", result.hostInterest0);
	logValue("Board data : ", result.boardData);
	logValue("Firmware RAM : ", result.firmwareRam);
	dbg_printf("\n");
}

int twlWifiProbe(twlWifiProbeResult* result) {
	dbg_printf("twlWifiProbe:\n");

	resetResult(result);

	const u16 oldGpioWifi = REG_GPIO_WIFI;
	const u16 oldScfgWl = REG_SCFG_WL;
	const u8 oldLed = i2cReadRegister(0x4A, 0x30);

	REG_SCFG_WL = 1;
	REG_GPIO_WIFI &= ~BIT(8); // Select Atheros
	i2cWriteRegister(0x4A, 0x30, 0x13);

	u16 sdioSaved[SDIO_SAVED_COUNT];
	u32 sdioSavedIrqMask;
	sdioSave(sdioSaved, &sdioSavedIrqMask);

	const int ret = probe(result);

	sdioShutdown(sdioSaved, sdioSavedIrqMask);

	i2cWriteRegister(0x4A, 0x30, oldLed);
	REG_GPIO_WIFI = oldGpioWifi;
	REG_SCFG_WL = oldScfgWl;

	logValue("Old GPIO_WIFI : ", oldGpioWifi);
	logValue("Old SCFG_WL : ", oldScfgWl);
	logValue("Old LED : ", oldLed);
	logValue("Old SDIO_RESET : ", sdioSaved[SDIO_SAVED_COUNT-1]);
	logValue("Old SDIO_CLK_CNT : ", sdioSaved[SDIO_SAVED_COUNT-2]);
	logValue("Result : ", (u32)ret);
	logValue("SDIO revision : ", result->sdioRevision);
	logValue("Manufacturer : ", result->manufacturer);
	logValue("Chip ID : ", result->chipId);
	logValue("Firmware uploaded : ", result->firmwareUploaded);
	logValue("Reset cause : ", result->resetCause);
	logValue("BMI version : ", result->bmiVersion);
	dbg_printf("\n");

	return ret;
}
