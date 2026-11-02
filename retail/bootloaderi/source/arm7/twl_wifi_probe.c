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
 * The BMI, HTC and WMI sequences and message layouts follow Calico:
 *   Copyright fincs, devkitPro
 *   Distributed under the Zope Public License 2.1.
 * Changes from DSiWifi: every wait has a timeout, and only the parts needed to
 * identify the chip are kept.
 */

#include <stddef.h> // NULL
#include <nds/ndstypes.h>
#include <nds/system.h>
#include <nds/bios.h>
#include <nds/arm7/i2c.h>
#include <nds/arm7/serial.h>
#include "debug_file.h"
#include "twl_wifi_probe.h"
#include "twl_wifi_wpa.h"

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
#define SDIO_DATA32_FIFO	0x10C

#define STAT0_CMDRESPEND	0x0001
#define STAT0_DATAEND		0x0004
#define STAT1_RXRDY		0x0100
#define STAT1_TXRQ		0x0200
#define STAT1_CMD_BUSY		0x4000
#define STAT1_ERR		0x807F // ILL_ACCESS, CMDTIMEOUT, TXUNDERRUN, RXOVERFLOW, DATATIMEOUT, STOPBIT_ERR, CRCFAIL, CMD_IDX_ERR

// Command register values (index | response type << 8)
#define CMD3	(3  | (4 << 8)) // SEND_RELATIVE_ADDR, 48-bit response
#define CMD5	(5  | (7 << 8)) // IO_SEND_OP_COND, 48-bit OCR response, no CRC7
#define CMD7	(7  | (5 << 8)) // SELECT_CARD, 48-bit response with busy
#define CMD52	(52 | (4 << 8)) // IO_RW_DIRECT, 48-bit response
#define CMD53_WRITE	(53 | (4 << 8) | BIT(11) | BIT(14)) // IO_RW_EXTENDED, single block
#define CMD53_READ	(CMD53_WRITE | BIT(12))

#define TIMEOUT	0x100000

// Atheros internal addresses
#define AR6002_HOST_INTEREST_ADDRESS	0x00500400
#define AR601x_HOST_INTEREST_ADDRESS	0x00520000
#define CHIP_ID_AR6002	0x02000001

// Function 1 registers
#define F1_MBOX			0x0FF
#define F1_RX_LOOKAHEAD_VALID	0x405
#define F1_COUNT4		0x450
#define F1_WINDOW_DATA		0x474
#define F1_WINDOW_WRITE_ADDR	0x478
#define F1_WINDOW_READ_ADDR	0x47C

#define BMI_DONE		0x1
#define BMI_WRITE_MEMORY	0x3
#define BMI_WRITE_SOC_REGISTER	0x7
#define BMI_GET_TARGET_ID	0x8

static u32 sdioResp = 0;
static u16 sdioRca = 0;
static bool sdioError = false;
static u16 sdioFirstErrCmd = 0;
static u32 sdioFirstErrArg = 0;
static u16 sdioFirstErrStat = 0;

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

// Byte mode function 1 transfer, polled through the 32-bit FIFO
static bool cmd53(bool write, u32 addr, u32* data, u32 len) {
	int i;
	for (i = 0; i < TIMEOUT && (sdioRead16(SDIO_IRQ_STAT1) & STAT1_CMD_BUSY); i++);
	if (i == TIMEOUT) {
		sdioError = true;
		return false;
	}

	const u32 arg = (write ? BIT(31) : 0) | (1 << 28) | BIT(26) | ((addr & 0x1FFFF) << 9) | len;
	const u16 cmd = write ? CMD53_WRITE : CMD53_READ;
	sdioWrite16(SDIO_IRQ_STAT0, 0);
	sdioWrite16(SDIO_IRQ_STAT1, 0);
	sdioWrite16(SDIO_CMD_PARAM0, arg & 0xFFFF);
	sdioWrite16(SDIO_CMD_PARAM1, arg >> 16);
	sdioWrite16(SDIO_DATA16_BLK_LEN, len);
	sdioWrite16(SDIO_DATA16_BLK_CNT, 1);
	sdioWrite16(SDIO_DATA32_BLK_LEN, len);
	sdioWrite16(SDIO_DATA32_BLK_CNT, 1);
	sdioWrite16(SDIO_CMD, cmd);

	bool transferred = false;
	bool ok = false;
	u16 stat1 = 0;
	for (i = 0; i < TIMEOUT; i++) {
		stat1 = sdioRead16(SDIO_IRQ_STAT1);
		if (stat1 & STAT1_ERR) {
			break;
		}
		if (!write && !transferred && (sdioRead16(SDIO_IRQ32) & 0x100)) {
			sdioMask16(SDIO_IRQ_STAT1, STAT1_RXRDY, 0);
			for (u32 w = 0; w < (len+3)/4; w++) {
				data[w] = *(vu32*)(SDIO_BASE + SDIO_DATA32_FIFO);
			}
			transferred = true;
		}
		if (write && !transferred && (stat1 & STAT1_TXRQ)) {
			sdioMask16(SDIO_IRQ_STAT1, STAT1_TXRQ, 0);
			for (u32 w = 0; w < (len+3)/4; w++) {
				*(vu32*)(SDIO_BASE + SDIO_DATA32_FIFO) = data[w];
			}
			transferred = true;
		}
		if (transferred && (sdioRead16(SDIO_IRQ_STAT0) & STAT0_DATAEND)) {
			ok = true;
			break;
		}
	}

	sdioWrite16(SDIO_IRQ_STAT0, 0);
	sdioWrite16(SDIO_IRQ_STAT1, 0);
	if (!ok) {
		if (!sdioFirstErrCmd) {
			sdioFirstErrCmd = cmd;
			sdioFirstErrArg = arg;
			sdioFirstErrStat = stat1;
		}
		sdioError = true;
	}
	return ok;
}

// The way official code reads chip memory: three high address bytes, then the
// low byte, which starts the read
static u32 readInternWord53(u32 addr) {
	u32 val = 0xFFFFFFFF;
	for (int i = 3; i >= 0 && !sdioError; i--) {
		u32 byte = (addr >> (i*8)) & 0xFF;
		cmd53(true, F1_WINDOW_READ_ADDR + i, &byte, 1);
	}
	if (!sdioError) {
		cmd53(false, F1_WINDOW_DATA, &val, 4);
	}
	return val;
}

static void writeInternWord53(u32 addr, u32 val) {
	cmd53(true, F1_WINDOW_DATA, &val, 4);
	for (int i = 3; i >= 0 && !sdioError; i--) {
		u32 byte = (addr >> (i*8)) & 0xFF;
		cmd53(true, F1_WINDOW_WRITE_ADDR + i, &byte, 1);
	}
}

// BMI messages end at the top of mailbox 0
static bool bmiSend(u32* msg, u32 len) {
	u32 credits = 0;
	for (int i = 0; i < 0x1000 && !credits; i++) {
		if (!cmd53(false, F1_COUNT4, &credits, 4)) {
			return false;
		}
		credits &= 0xFF;
	}
	return credits && cmd53(true, 0x1000 - len, msg, len);
}

static bool bmiRecv(u32* out) {
	return cmd53(false, 0x1000 - sizeof(u32), out, sizeof(u32));
}

static bool bmiWriteMemoryWord(u32 addr, u32 val) {
	u32 msg[4] = {BMI_WRITE_MEMORY, addr, sizeof(u32), val};
	return bmiSend(msg, sizeof(msg));
}

static bool bmiWriteSocReg(u32 addr, u32 val) {
	u32 msg[3] = {BMI_WRITE_SOC_REGISTER, addr, val};
	return bmiSend(msg, sizeof(msg));
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

	// CMD5 until the card reports it's ready
	u32 ocr = 0;
	bool skipInit = false;
	int i;
	for (i = 0; i < 0x1000; i++) {
		if (!sdioCommand(CMD5, ocr)) {
			// Already initialized (e.g. by the system menu)
			sdioError = false;
			skipInit = true;
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

	cmd52Write(0, 0x02, 0x00);
	cmd52Write(0, 0x02, 0x02);
	swiDelay(0xF000);

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

	result->sdioRevision = cmd52Read(0, 0x00);

	// Enable function 1 and wait until it's ready
	cmd52Write(0, 0x02, 0x02);
	for (i = 0; i < 0x10000 && !sdioError && cmd52Read(0, 0x03) != 0x02; i++);
	if (i == 0x10000 || sdioError) return -7;

	result->manufacturer = read32(0, 0x1007);
	result->chipId = readInternWord(0x000040EC);
	const u32 hostInterest = (result->chipId == CHIP_ID_AR6002) ? AR6002_HOST_INTEREST_ADDRESS : AR601x_HOST_INTEREST_ADDRESS;
	result->firmwareUploaded = readInternWord(hostInterest + 0x58);
	result->hostInterest0 = readInternWord(hostInterest);
	result->boardData = readInternWord(hostInterest + 0x54);
	result->firmwareRam = readInternWord(0x00524C00);
	if (sdioError) return -8;

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
	result->firmwareRam2 = 0;
	result->count4 = 0;
	result->resetCause = 0;
	result->bmiVersion = 0;
	result->irqEnable = 0;
	result->ioEnable = 0;
	result->ioReady = 0;
	result->busControl = 0;
	sdioError = false;
	sdioFirstErrCmd = 0;
	sdioFirstErrArg = 0;
	sdioFirstErrStat = 0;
}

// Reads the state of a chip the system already set up, in the order official
// code does it: a register read at the slow identification clock decides if the
// card needs bootup at all. Nothing here reinitializes or reconfigures the card.
static void warmControllerInit(void) {
	sdioControllerInit();
	sdioMask16(SDIO_PORT_SEL, 3, 0);
	sdioMask16(SDIO_CLK_CNT, 0x100, 0);
	sdioMask16(SDIO_CLK_CNT, 0x2FF, 0x40); // HCLK/256
	sdioMask16(SDIO_CLK_CNT, 0, 0x100);
	sdioMask16(SDIO_CARD_OPT, 0, BIT(15)); // 1-bit
	swiDelay(0xF000);
}

static int probeWarm(twlWifiProbeResult* result) {
	warmControllerInit();

	result->irqEnable = cmd52Read(0, 0x04);
	if (sdioError) return -12; // Needs bootup, so it isn't warm

	result->sdioRevision = cmd52Read(0, 0x00);
	result->ioEnable = cmd52Read(0, 0x02);
	result->ioReady = cmd52Read(0, 0x03);
	result->busControl = cmd52Read(0, 0x07);
	result->manufacturer = read32(0, 0x1007);
	if (sdioError) return -13;

	// CMD53 data follows the card's bus width, which the system left at 4-bit
	if (result->busControl & 2) {
		sdioMask16(SDIO_CARD_OPT, BIT(15), 0);
	}

	// Reads only: is function 1 there for CMD53?
	if (!cmd53(false, F1_COUNT4, &result->count4, 4)) return -16;

	result->chipId = readInternWord53(0x000040EC);
	if (sdioError) return -14;
	const u32 hostInterest = (result->chipId == CHIP_ID_AR6002) ? AR6002_HOST_INTEREST_ADDRESS : AR601x_HOST_INTEREST_ADDRESS;
	result->firmwareUploaded = readInternWord53(hostInterest + 0x58);
	result->hostInterest0 = readInternWord53(hostInterest);
	result->boardData = readInternWord53(hostInterest + 0x54);
	result->firmwareRam = readInternWord53(0x00524C00);
	result->firmwareRam2 = readInternWord53(0x00527000);
	if (sdioError) return -15;

	return 0;
}

void twlWifiEarlyCheck(void) {
	dbg_printf("twlWifiEarlyCheck:\n");

	twlWifiProbeResult result;
	resetResult(&result);

	const u16 oldGpioWifi = REG_GPIO_WIFI;
	logValue("GPIO_WIFI : ", oldGpioWifi);
	logValue("SCFG_WL : ", REG_SCFG_WL);
	logValue("LED : ", i2cReadRegister(0x4A, 0x30));
	logValue("WL firm info 0 : ", *(vu32*)0x020005E0);
	logValue("WL firm info 1 : ", *(vu32*)0x020005E4);
	logValue("WL firm info 2 : ", *(vu32*)0x020005E8);
	logValue("WL firm info 3 : ", *(vu32*)0x020005EC);

	u16 sdioSaved[SDIO_SAVED_COUNT];
	u32 sdioSavedIrqMask;
	sdioSave(sdioSaved, &sdioSavedIrqMask);

	REG_GPIO_WIFI &= ~BIT(8); // Select Atheros
	const int ret = probeWarm(&result);
	sdioRestore(sdioSaved, sdioSavedIrqMask);
	REG_GPIO_WIFI = oldGpioWifi;

	logValue("Result : ", (u32)ret);
	logValue("IRQ enable : ", result.irqEnable);
	logValue("SDIO revision : ", result.sdioRevision);
	logValue("Function enable : ", result.ioEnable);
	logValue("Function ready : ", result.ioReady);
	logValue("Bus control : ", result.busControl);
	logValue("Manufacturer : ", result.manufacturer);
	logValue("First failed cmd : ", sdioFirstErrCmd);
	logValue("First failed arg : ", sdioFirstErrArg);
	logValue("First failed stat : ", sdioFirstErrStat);
	logValue("F1 count4 : ", result.count4);
	logValue("Chip ID : ", result.chipId);
	logValue("Firmware uploaded : ", result.firmwareUploaded);
	logValue("Host interest 0 : ", result.hostInterest0);
	logValue("Board data : ", result.boardData);
	logValue("Firmware RAM : ", result.firmwareRam);
	logValue("Firmware RAM 2 : ", result.firmwareRam2);
	dbg_printf("\n");
}

// HTC: every message starts with endpoint, flags, payload length, trailer length
// and a pad byte. Packets are rounded up to the mailbox block size.
#define HTC_BLOCK		128
#define HTC_HDR			6
#define HTC_MAX_PACKET		2048
#define HTC_MSG_READY		1
#define HTC_MSG_CONNECT		2
#define HTC_MSG_CONNECT_REPLY	3
#define HTC_MSG_SETUP_COMPLETE	4
#define HTC_SRV_WMI_CONTROL	0x0100
#define HTC_SRV_WMI_DATA_BE	0x0101
#define HTC_SRV_WMI_DATA_VO	0x0104
#define HTC_FLAGS_WMI_DATA	5 // Reduce credit dribble, threshold 0.5
#define HTC_FLAG_NEED_CREDIT_UPDATE 1
#define HTC_FLAG_RECV_TRAILER	2
#define HTC_RECORD_CREDIT	1

#define WMI_CMD_CONNECT		0x0001
#define WMI_CMD_DISCONNECT	0x0003
#define WMI_CMD_SYNCHRONIZE	0x0004
#define WMI_CMD_CREATE_PSTREAM	0x0005
#define WMI_CMD_START_SCAN	0x0007
#define WMI_CMD_SET_SCAN_PARAMS	0x0008
#define WMI_CMD_SET_BSS_FILTER	0x0009
#define WMI_CMD_SET_PROBED_SSID	0x000A
#define WMI_CMD_SET_DISC_TIMEOUT 0x000D
#define WMI_CMD_GET_CHANNEL_LIST 0x000E
#define WMI_CMD_SET_CHANNEL_PARAMS 0x0011
#define WMI_CMD_SET_POWER_MODE	0x0012
#define WMI_CMD_ADD_CIPHER_KEY	0x0016
#define WMI_CMD_ERROR_REPORT	0x0022
#define WMI_CMD_SET_KEEP_ALIVE	0x003D
#define WMI_CMD_SET_WSC_STATUS	0x0041
#define WMI_CMD_SET_HB_TIMEOUT	0x0047
#define WMI_CMD_SET_FRAME_RATE	0x0048
#define WMI_CMD_SET_BIT_RATE	0xF000
#define WMI_EVENT_CHANNEL_LIST	0x000E
#define WMI_EVENT_READY		0x1001
#define WMI_EVENT_CONNECTED	0x1002
#define WMI_EVENT_DISCONNECTED	0x1003
#define WMI_EVENT_BSS_INFO	0x1004
#define WMI_EVENT_REG_DOMAIN	0x1006
#define WMI_EVENT_SCAN_COMPLETE	0x100A
#define WMI_EVENT_EXTENSION	0x1010
#define WMI_PROTOCOL_VERSION	2
#define WMI_PHY_11G		2
#define WMI_BSS_FILTER_NONE	0
#define WMI_BSS_FILTER_ALL	1
#define WMI_SCAN_CONNECT	BIT(0)
#define WMI_SCAN_ACTIVE		BIT(2)
#define WMI_SCAN_ROAM		BIT(3)
#define WMI_POWER_MAX_PERFORMANCE 2
#define WMI_CIPHER_TKIP		3
#define WMI_CIPHER_AES		4
#define WMI_AUTH_WPA2_PSK	5
#define WMI_KEY_USAGE_GROUP	1
#define WMI_KEY_OP_INIT_TSC_RSC	3

#define CMD53_MAX_LEN		256
#define WMI_BODY		(HTC_HDR + 2)

// The last received message
static u32 htcWords[HTC_MAX_PACKET/sizeof(u32)];
#define htcBuf ((u8*)htcWords)

// The message being built. Separate, since waiting for a credit receives messages
static u32 txWords[CMD53_MAX_LEN/sizeof(u32)];
#define txBuf ((u8*)txWords)

static u32 htcPayloadLen = 0; // Without the trailer
static u32 htcCredits = 0;
static u8 wmiEndpoint = 0;
static u8 wmiDataEndpoint = 0;
static u32 hostInterestAddr = 0;
static u8 selfMac[6];

// The saved connections, slots 4 to 6 in the system settings
#define WIFI_CFG_ADDR		0x1F400
#define WIFI_CFG_SIZE		0x200
#define WIFI_CFG_SSID		0x40
#define WIFI_CFG_WPA_MODE	0xE7
#define WIFI_CFG_SSID_LEN	0xE8
#define WIFI_CFG_SLOT		0xEF
#define WIFI_CFG_PMK		0x100
#define WIFI_CFG_WPA_TYPE	0x181

typedef struct {
	u8 ssid[32];
	u8 ssidLen;
	u8 pmk[32];
} savedNetwork;

static savedNetwork savedNetworks[3];
static u32 savedCount = 0;

// The saved network the scan found
static const savedNetwork* targetNetwork = NULL;
static u8 targetBssid[6];
static u16 targetMhz = 0;
static u8 targetSnr = 0;
static u8 targetGroupCipher = 0;

static bool scanDone = false;
static u32 scanResults = 0;
static u16 channels[14];
static u32 channelCount = 0;
static bool channelsReady = false;

static bool wifiAssociated = false;
static bool wifiDisconnected = false;
static u32 dataFrames = 0;

// WPA2 handshake state
#define EAPOL_ETHERTYPE		0x888E
#define EAPOL_HDR		4
#define EAPOL_KEY_HDR		95
#define EAPOL_KEY_INFO		1
#define EAPOL_KEY_REPLAY	5
#define EAPOL_KEY_NONCE		13
#define EAPOL_KEY_RSC		61
#define EAPOL_KEY_MIC		77
#define EAPOL_KEY_DATA_LEN	93
#define EAPOL_INFO_VERSION	0x0007
#define EAPOL_INFO_PAIRWISE	0x0008
#define EAPOL_INFO_INDEX	0x0030
#define EAPOL_INFO_ACK		0x0080
#define EAPOL_INFO_MIC		0x0100
#define EAPOL_INFO_SECURE	0x0200
#define EAPOL_INFO_ERROR	0x0400
#define EAPOL_INFO_REQUEST	0x0800
#define EAPOL_INFO_ENCRYPTED	0x1000
#define EAPOL_MIC_LEN		16
#define EAPOL_NONCE_LEN		32
#define EAPOL_REPLAY_LEN	8
#define WPA_KCK_LEN		16
#define WPA_KEK_OFFSET		16
#define WPA_TK_OFFSET		32
#define WPA_TK_LEN		16
#define WPA_PTK_LEN		48

static u8 apMac[6];
static u8 assocIe[64];
static u32 assocIeLen = 0;
static u8 eapolVersion = 0;
static u8 eapolReplay[EAPOL_REPLAY_LEN];
static bool eapolReplaySet = false;
static u8 anonce[EAPOL_NONCE_LEN];
static u8 ptk[WPA_PTK_LEN];
static bool ptkReady = false;
static u8 gtk[32];
static u32 gtkLen = 0;
static u32 gtkIndex = 0;
static u8 groupRsc[8];
static u8 keyData[256];
static bool wpaBusy = false;
static bool wpaComplete = false;

static inline u16 htcGet16(u32 offset) {
	return htcBuf[offset] | (htcBuf[offset+1] << 8);
}

static inline u16 htcGetBe16(u32 offset) {
	return (htcBuf[offset] << 8) | htcBuf[offset+1];
}

static inline void txPut16(u32 offset, u16 val) {
	txBuf[offset] = val & 0xFF;
	txBuf[offset+1] = val >> 8;
}

static inline void txPutBe16(u32 offset, u16 val) {
	txBuf[offset] = val >> 8;
	txBuf[offset+1] = val & 0xFF;
}

static inline void txPut32(u32 offset, u32 val) {
	txPut16(offset, val & 0xFFFF);
	txPut16(offset+2, val >> 16);
}

static void txCopy(u32 offset, const u8* data, u32 len) {
	for (u32 i = 0; i < len; i++) {
		txBuf[offset+i] = data[i];
	}
}

static void txClear(void) {
	for (u32 i = 0; i < sizeof(txWords)/sizeof(u32); i++) {
		txWords[i] = 0;
	}
}

static bool bytesLess(const u8* a, const u8* b, u32 len) {
	for (u32 i = 0; i < len; i++) {
		if (a[i] != b[i]) {
			return a[i] < b[i];
		}
	}
	return false;
}

static void clearBytes(u8* data, u32 len) {
	while (len--) {
		*data++ = 0;
	}
}

static void copyBytes(u8* dst, const u8* src, u32 len) {
	while (len--) {
		*dst++ = *src++;
	}
}

static bool sameBytes(const u8* a, const u8* b, u32 len) {
	while (len--) {
		if (*a++ != *b++) {
			return false;
		}
	}
	return true;
}

static void logText(const char* name, const u8* text, u32 len) {
	char line[33];
	for (u32 i = 0; i < len; i++) {
		line[i] = (text[i] >= 0x20 && text[i] < 0x7F) ? text[i] : '?';
	}
	line[len] = 0;
	dbg_printf(name);
	dbg_printf(line);
	dbg_printf("\n");
}

static bool htcSend(u32 payloadLen) {
	txPut16(2, payloadLen);
	const u32 len = (HTC_HDR + payloadLen + HTC_BLOCK - 1) & ~(HTC_BLOCK - 1);
	return cmd53(true, 0x1000 - len, txWords, len);
}

// Takes the payload length and any credits out of a received message
static void htcProcessTrailer(void) {
	htcPayloadLen = htcGet16(2);
	if (!(htcBuf[1] & HTC_FLAG_RECV_TRAILER) || htcBuf[4] > htcPayloadLen) {
		return;
	}

	htcPayloadLen -= htcBuf[4];
	u32 pos = HTC_HDR + htcPayloadLen;
	const u32 end = pos + htcBuf[4];
	while (pos + 2 <= end) {
		const u8 id = htcBuf[pos];
		const u8 len = htcBuf[pos+1];
		pos += 2;
		if (pos + len > end) {
			break;
		}
		if (id == HTC_RECORD_CREDIT) {
			for (u32 i = 1; i < len; i += 2) {
				htcCredits += htcBuf[pos+i];
			}
		}
		pos += len;
	}
}

static bool htcRecv(void) {
	u32 regs[3];
	u32 attempts;
	for (attempts = 0; attempts < 200; attempts++) {
		if (!cmd53(false, 0x400, regs, sizeof(regs))) {
			return false;
		}
		if ((regs[0] & 1) && (regs[1] & 0x100)) {
			break;
		}
		swiDelay(0x4000);
	}
	if (attempts == 200) {
		return false;
	}

	const u32 lookahead = regs[2];
	const u32 len = (HTC_HDR + (lookahead >> 16) + HTC_BLOCK - 1) & ~(HTC_BLOCK - 1);
	if (len > sizeof(htcWords)) {
		logValue("Message too long : ", lookahead);
		return false;
	}
	for (u32 done = 0; done < len; done += CMD53_MAX_LEN) {
		const u32 chunk = (len - done > CMD53_MAX_LEN) ? CMD53_MAX_LEN : (len - done);
		if (!cmd53(false, 0x1000 - len + done, htcWords + done/sizeof(u32), chunk)) {
			return false;
		}
	}
	if (htcWords[0] != lookahead) {
		logValue("Lookahead mismatch : ", lookahead);
		logValue("Frame start : ", htcWords[0]);
		return false;
	}

	htcProcessTrailer();
	return true;
}

// Returns the endpoint, or 0 on failure
static u8 htcConnect(u16 service, u16 flags) {
	txClear();
	txPut16(HTC_HDR, HTC_MSG_CONNECT);
	txPut16(HTC_HDR+2, service);
	txPut16(HTC_HDR+4, flags);
	if (!htcSend(HTC_HDR + 8) || !htcRecv()) {
		return 0;
	}

	const u8 endpoint = htcBuf[HTC_HDR+5];
	if (htcBuf[0] != 0 || htcGet16(HTC_HDR) != HTC_MSG_CONNECT_REPLY || htcGet16(HTC_HDR+2) != service
	 || htcBuf[HTC_HDR+4] != 0 || endpoint == 0 || endpoint > 8) {
		return 0;
	}
	return endpoint;
}

static bool wmiPump(void);

// Waits for a send credit, then starts a new message
static bool wmiBegin(void) {
	int silent = 0;
	for (int i = 0; htcCredits == 0; i++) {
		if (i > 500 || silent > 8) {
			logValue("Out of credits : ", i);
			return false;
		}
		if (wmiPump()) {
			silent = 0;
		} else if (sdioError) {
			return false;
		} else {
			silent++;
		}
	}
	txClear();
	return true;
}

static bool wmiSend(u16 id, u32 bodyLen) {
	txBuf[0] = wmiEndpoint;
	txBuf[1] = HTC_FLAG_NEED_CREDIT_UPDATE;
	txPut16(HTC_HDR, id);
	htcCredits--;
	return htcSend(2 + bodyLen);
}

static bool wmiCmd(u16 id) {
	return wmiBegin() && wmiSend(id, 0);
}

static bool wmiCmd8(u16 id, u8 val) {
	if (!wmiBegin()) {
		return false;
	}
	txBuf[WMI_BODY] = val;
	return wmiSend(id, 1);
}

static bool wmiCmd32(u16 id, u32 val) {
	if (!wmiBegin()) {
		return false;
	}
	txPut32(WMI_BODY, val);
	return wmiSend(id, sizeof(u32));
}

static void readSavedNetworks(void) {
	savedCount = 0;
	for (u32 i = 0; i < 3; i++) {
		const u32 base = WIFI_CFG_ADDR + i*WIFI_CFG_SIZE;
		savedNetwork* net = &savedNetworks[savedCount];
		u8 wpaMode = 0xFF;
		u8 wpaType = 0;
		u8 slot = 0;
		readFirmware(base + WIFI_CFG_WPA_MODE, &wpaMode, 1);
		readFirmware(base + WIFI_CFG_WPA_TYPE, &wpaType, 1);
		readFirmware(base + WIFI_CFG_SLOT, &slot, 1);
		readFirmware(base + WIFI_CFG_SSID_LEN, &net->ssidLen, 1);
		if (net->ssidLen == 0 || net->ssidLen > 32 || wpaMode == 0xFF || slot == 0) {
			continue;
		}

		readFirmware(base + WIFI_CFG_SSID, net->ssid, net->ssidLen);
		readFirmware(base + WIFI_CFG_PMK, net->pmk, sizeof(net->pmk));
		logText("Saved network : ", net->ssid, net->ssidLen);
		logValue(" Slot : ", i + 4);
		logValue(" WPA mode : ", wpaMode);
		logValue(" WPA type : ", wpaType);
		savedCount++;
	}
	logValue("Saved networks : ", savedCount);
}

// Scan result: BSS header, then a beacon body with its information elements after
// the timestamp, interval and capability fields. Only saved networks are kept.
static void handleBss(void) {
	const u32 hdr = WMI_BODY;
	const u32 end = HTC_HDR + htcPayloadLen;
	const u32 body = hdr + 16;
	scanResults++;
	if (body + 12 > end) {
		return;
	}

	u32 ssid = 0;
	u32 ssidLen = 0;
	bool ccmp = false;
	bool psk = false;
	u8 groupCipher = 0;
	for (u32 ie = body + 12; ie + 2 <= end; ie += 2 + htcBuf[ie+1]) {
		const u8 id = htcBuf[ie];
		const u8 len = htcBuf[ie+1];
		if (ie + 2 + len > end) {
			break;
		}
		if (id == 0 && len <= 32) {
			ssid = ie + 2;
			ssidLen = len;
		} else if (id == 48 && len >= 8) {
			// RSN: version, group suite, pairwise suites, key management suites
			groupCipher = htcBuf[ie+2+5];
			const u32 pairwiseCount = htcGet16(ie+2+6);
			u32 pos = ie + 2 + 8;
			for (u32 i = 0; i < pairwiseCount && pos + 4 <= ie + 2 + len; i++, pos += 4) {
				ccmp |= (htcBuf[pos+3] == 4);
			}
			if (pos + 2 <= ie + 2 + len) {
				const u32 akmCount = htcGet16(pos);
				pos += 2;
				for (u32 i = 0; i < akmCount && pos + 4 <= ie + 2 + len; i++, pos += 4) {
					psk |= (htcBuf[pos+3] == 2);
				}
			}
		}
	}

	for (u32 i = 0; i < savedCount; i++) {
		const savedNetwork* net = &savedNetworks[i];
		if (ssidLen != net->ssidLen || !sameBytes(htcBuf + ssid, net->ssid, ssidLen)) {
			continue;
		}

		const u8 snr = htcBuf[hdr+3];
		logText("Found saved network : ", net->ssid, net->ssidLen);
		logValue(" MHz : ", htcGet16(hdr));
		logValue(" SNR : ", snr);
		logValue(" CCMP and PSK : ", ccmp && psk);
		if (ccmp && psk && (targetNetwork == NULL || snr > targetSnr)) {
			targetNetwork = net;
			targetSnr = snr;
			targetMhz = htcGet16(hdr);
			targetGroupCipher = groupCipher;
			copyBytes(targetBssid, htcBuf + hdr + 6, sizeof(targetBssid));
		}
	}
}

static void handleChannelList(void) {
	channelCount = 0;
	for (u32 i = 0; i < htcBuf[WMI_BODY+1] && channelCount < 14; i++) {
		const u16 mhz = htcGet16(WMI_BODY + 2 + i*2);
		if (mhz < 3000) {
			channels[channelCount++] = mhz;
		}
	}
	channelsReady = true;
}

// The firmware's association request holds the RSN element the handshake repeats
static void handleConnected(void) {
	const u32 beaconLen = htcBuf[WMI_BODY+16];
	const u32 requestLen = htcBuf[WMI_BODY+17];
	const u32 requestPos = WMI_BODY + 19 + beaconLen;
	const u32 end = requestPos + requestLen;

	assocIeLen = 0;
	for (u32 ie = requestPos + 4; requestLen >= 4 && ie + 2 <= end; ie += 2 + htcBuf[ie+1]) {
		const u32 len = 2 + htcBuf[ie+1];
		if (ie + len > end) {
			break;
		}
		if (htcBuf[ie] == 48 && len <= sizeof(assocIe)) {
			copyBytes(assocIe, htcBuf + ie, len);
			assocIeLen = len;
			break;
		}
	}

	logValue("Associated, MHz : ", htcGet16(WMI_BODY));
	logValue("RSN element length : ", assocIeLen);
	wifiAssociated = true;
}

static bool wpaSendKey(u16 rxInfo, const u8* nonce, const u8* data, u32 dataLen) {
	const u32 mac = HTC_HDR + 2;
	const u32 llc = mac + 14;
	const u32 eapol = llc + 8;
	const u32 key = eapol + EAPOL_HDR;
	const u32 bodyLen = EAPOL_KEY_HDR + dataLen;

	if (key + bodyLen > sizeof(txWords) || !wmiBegin()) {
		return false;
	}

	txCopy(mac, apMac, sizeof(apMac));
	txCopy(mac + 6, selfMac, sizeof(selfMac));
	txPutBe16(mac + 12, 8 + EAPOL_HDR + bodyLen);

	const u8 snap[8] = {0xAA, 0xAA, 0x03, 0, 0, 0, EAPOL_ETHERTYPE >> 8, EAPOL_ETHERTYPE & 0xFF};
	txCopy(llc, snap, sizeof(snap));

	txBuf[eapol] = eapolVersion;
	txBuf[eapol+1] = 3; // EAPOL-Key
	txPutBe16(eapol + 2, bodyLen);

	txBuf[key] = 2; // RSN
	txPutBe16(key + EAPOL_KEY_INFO, (rxInfo & EAPOL_INFO_VERSION) | EAPOL_INFO_MIC | (rxInfo &
		(EAPOL_INFO_PAIRWISE | EAPOL_INFO_INDEX | EAPOL_INFO_SECURE | EAPOL_INFO_ERROR | EAPOL_INFO_REQUEST)));
	txCopy(key + EAPOL_KEY_REPLAY, eapolReplay, sizeof(eapolReplay));
	if (nonce) {
		txCopy(key + EAPOL_KEY_NONCE, nonce, EAPOL_NONCE_LEN);
	}
	txPutBe16(key + EAPOL_KEY_DATA_LEN, dataLen);
	if (data) {
		txCopy(key + EAPOL_KEY_HDR, data, dataLen);
	}

	u8 digest[WPA_SHA1_LEN];
	wpaHmacSha1(digest, ptk, WPA_KCK_LEN, txBuf + eapol, EAPOL_HDR + bodyLen);
	txCopy(key + EAPOL_KEY_MIC, digest, EAPOL_MIC_LEN);

	txBuf[0] = wmiDataEndpoint;
	txBuf[1] = HTC_FLAG_NEED_CREDIT_UPDATE;
	htcCredits--;
	return htcSend(2 + 14 + 8 + EAPOL_HDR + bodyLen);
}

// Message 1 of 4: the router's nonce gives us everything to derive the keys
static void wpaHandleMessage1(u32 key, u16 info) {
	if (targetNetwork == NULL || assocIeLen == 0) {
		logValue("Handshake not ready : ", assocIeLen);
		return;
	}

	copyBytes(anonce, htcBuf + key + EAPOL_KEY_NONCE, sizeof(anonce));

	// Our nonce: the saved key, the router's nonce and what timing we have
	u8 snonce[EAPOL_NONCE_LEN];
	u8 seed[13 + sizeof(selfMac) + sizeof(anonce) + 4 + 1];
	clearBytes(seed, sizeof(seed));
	copyBytes(seed, (const u8*)"Init Counter", 12);
	copyBytes(seed + 13, selfMac, sizeof(selfMac));
	copyBytes(seed + 19, anonce, sizeof(anonce));
	seed[51] = REG_VCOUNT & 0xFF;
	seed[52] = dataFrames & 0xFF;
	seed[53] = htcCredits & 0xFF;
	wpaPrf(snonce, sizeof(snonce), targetNetwork->pmk, sizeof(targetNetwork->pmk), seed, sizeof(seed));

	// Pairwise key expansion: both MACs and both nonces, each pair lowest first
	u8 pad[23 + 12 + 2*EAPOL_NONCE_LEN + 1];
	clearBytes(pad, sizeof(pad));
	copyBytes(pad, (const u8*)"Pairwise key expansion", 22);

	const bool selfFirst = bytesLess(selfMac, apMac, sizeof(selfMac));
	copyBytes(pad + 23, selfFirst ? selfMac : apMac, 6);
	copyBytes(pad + 29, selfFirst ? apMac : selfMac, 6);

	const bool snonceFirst = bytesLess(snonce, anonce, sizeof(anonce));
	copyBytes(pad + 35, snonceFirst ? snonce : anonce, EAPOL_NONCE_LEN);
	copyBytes(pad + 67, snonceFirst ? anonce : snonce, EAPOL_NONCE_LEN);

	wpaPrf(ptk, sizeof(ptk), targetNetwork->pmk, sizeof(targetNetwork->pmk), pad, sizeof(pad));
	ptkReady = true;

	logValue("Handshake 1 of 4, send : ", wpaSendKey(info, snonce, assocIe, assocIeLen));
}

static bool wpaInstallKey(bool group) {
	if (!wmiCmd8(WMI_CMD_SYNCHRONIZE, 0) || !wmiBegin()) {
		return false;
	}

	const u32 keyLen = group ? gtkLen : WPA_TK_LEN;
	txBuf[WMI_BODY+0] = group ? gtkIndex : 0;
	txBuf[WMI_BODY+1] = (keyLen > WPA_TK_LEN) ? WMI_CIPHER_TKIP : WMI_CIPHER_AES;
	txBuf[WMI_BODY+2] = group ? WMI_KEY_USAGE_GROUP : 0;
	txBuf[WMI_BODY+3] = keyLen;
	if (group) {
		txCopy(WMI_BODY + 4, groupRsc, sizeof(groupRsc));
		txCopy(WMI_BODY + 12, gtk, 16);
		if (keyLen > 16) {
			// TKIP: the firmware wants the receive and transmit parts swapped
			txCopy(WMI_BODY + 28, gtk + 24, 8);
			txCopy(WMI_BODY + 36, gtk + 16, 8);
		}
	} else {
		txCopy(WMI_BODY + 12, ptk + WPA_TK_OFFSET, WPA_TK_LEN);
	}
	txBuf[WMI_BODY+44] = WMI_KEY_OP_INIT_TSC_RSC;
	if (!wmiSend(WMI_CMD_ADD_CIPHER_KEY, 45)) {
		return false;
	}

	return wmiCmd8(WMI_CMD_SYNCHRONIZE, 0);
}

// Message 3 of 4: holds the group key, wrapped with our key encryption key
static void wpaHandleMessage3(u32 key, u16 info, u32 dataLen) {
	if (!sameBytes(htcBuf + key + EAPOL_KEY_NONCE, anonce, sizeof(anonce))) {
		logValue("Handshake nonce changed : ", 1);
		return;
	}

	gtkLen = 0;
	for (u32 pos = 0; pos + 2 <= dataLen; pos += 2 + keyData[pos+1]) {
		const u32 len = keyData[pos+1];
		if (pos + 2 + len > dataLen) {
			break;
		}
		if (keyData[pos] == 0xDD && len >= 6 + 16 && len - 6 <= sizeof(gtk) && keyData[pos+2] == 0x00
		 && keyData[pos+3] == 0x0F && keyData[pos+4] == 0xAC && keyData[pos+5] == 0x01) {
			gtkIndex = keyData[pos+6] & 3;
			gtkLen = len - 6;
			copyBytes(gtk, keyData + pos + 8, gtkLen);
			break;
		}
	}
	if (gtkLen == 0) {
		logValue("Handshake group key missing : ", dataLen);
		return;
	}
	copyBytes(groupRsc, htcBuf + key + EAPOL_KEY_RSC, sizeof(groupRsc));

	logValue("Handshake 3 of 4, send : ", wpaSendKey(info, NULL, NULL, 0));
	logValue("Pairwise key : ", wpaInstallKey(false));
	logValue("Group key : ", wpaInstallKey(true));
	wpaComplete = true;
}

static void wpaHandleData(void) {
	const u32 end = HTC_HDR + htcPayloadLen;
	u32 pos = HTC_HDR + 2; // Past the WMI data header
	if (pos + 14 > end) {
		return;
	}

	const u32 source = pos + 6;
	u32 type = htcGetBe16(pos + 12);
	pos += 14;
	if (type < 0x600) {
		if (pos + 8 > end) {
			return;
		}
		type = htcGetBe16(pos + 6);
		pos += 8; // LLC-SNAP
	}
	if (type != EAPOL_ETHERTYPE) {
		dataFrames++;
		return;
	}

	const u32 eapol = pos;
	const u32 key = eapol + EAPOL_HDR;
	if (wpaBusy || key + EAPOL_KEY_HDR > end) {
		return;
	}

	const u16 info = htcGetBe16(key + EAPOL_KEY_INFO);
	u32 dataLen = htcGetBe16(key + EAPOL_KEY_DATA_LEN);
	if (htcBuf[key] != 2 || (info & EAPOL_INFO_VERSION) != 2 || !(info & EAPOL_INFO_ACK)
	 || key + EAPOL_KEY_HDR + dataLen > end) {
		logValue("EAPOL ignored : ", info);
		return;
	}

	if (info & EAPOL_INFO_MIC) {
		u8 mic[EAPOL_MIC_LEN];
		u8 digest[WPA_SHA1_LEN];
		if (!ptkReady) {
			return;
		}
		copyBytes(mic, htcBuf + key + EAPOL_KEY_MIC, sizeof(mic));
		clearBytes(htcBuf + key + EAPOL_KEY_MIC, sizeof(mic));
		wpaHmacSha1(digest, ptk, WPA_KCK_LEN, htcBuf + eapol, EAPOL_HDR + EAPOL_KEY_HDR + dataLen);
		if (!sameBytes(mic, digest, sizeof(mic))) {
			logValue("EAPOL MIC wrong : ", info);
			return;
		}
	}

	const u8* replay = htcBuf + key + EAPOL_KEY_REPLAY;
	if (eapolReplaySet && !bytesLess(eapolReplay, replay, sizeof(eapolReplay))) {
		logValue("EAPOL replay : ", info);
		return;
	}
	copyBytes(eapolReplay, replay, sizeof(eapolReplay));
	eapolReplaySet = true;
	eapolVersion = htcBuf[eapol];
	copyBytes(apMac, htcBuf + source, sizeof(apMac));

	if (dataLen && (info & EAPOL_INFO_ENCRYPTED)) {
		if (dataLen < 16 || (dataLen & 7) || dataLen - 8 > sizeof(keyData)) {
			logValue("EAPOL key data length : ", dataLen);
			return;
		}
		dataLen -= 8;
		if (!wpaAesUnwrap(ptk + WPA_KEK_OFFSET, htcBuf + key + EAPOL_KEY_HDR, keyData, dataLen/8)) {
			logValue("EAPOL key data corrupt : ", dataLen);
			return;
		}
	}

	wpaBusy = true;
	switch (info & (EAPOL_INFO_PAIRWISE | EAPOL_INFO_MIC)) {
		case EAPOL_INFO_PAIRWISE:
			wpaHandleMessage1(key, info);
			break;
		case EAPOL_INFO_PAIRWISE | EAPOL_INFO_MIC:
			wpaHandleMessage3(key, info, dataLen);
			break;
		default:
			logValue("EAPOL other : ", info);
			break;
	}
	wpaBusy = false;
}

static void wmiHandleEvent(void) {
	const u16 id = htcGet16(HTC_HDR);
	switch (id) {
		case WMI_EVENT_BSS_INFO:
			handleBss();
			break;
		case WMI_EVENT_SCAN_COMPLETE:
			logValue("Scan status : ", htcGet16(WMI_BODY));
			scanDone = true;
			break;
		case WMI_EVENT_CHANNEL_LIST:
			handleChannelList();
			break;
		case WMI_EVENT_CONNECTED:
			handleConnected();
			break;
		case WMI_EVENT_DISCONNECTED:
			logValue("Disconnected, reason : ", htcBuf[WMI_BODY+8]);
			logValue("Disconnected, 802.11 reason : ", htcGet16(WMI_BODY));
			wifiDisconnected = true;
			break;
		case WMI_EVENT_REG_DOMAIN:
		case WMI_EVENT_EXTENSION:
			break;
		default:
			logValue("Other WMI event : ", id);
			break;
	}
}

// Handles one message if there is one
static bool wmiPump(void) {
	if (!htcRecv()) {
		return false;
	}

	if (htcBuf[0] == wmiEndpoint) {
		wmiHandleEvent();
	} else if (htcBuf[0] != 0 && htcPayloadLen != 0) {
		wpaHandleData();
	}
	return true;
}

static bool wmiWaitFor(const bool* flag, int silentLimit) {
	int silent = 0;
	for (int i = 0; i < 2000 && !*flag && !wifiDisconnected && silent < silentLimit; i++) {
		if (wmiPump()) {
			silent = 0;
		} else if (sdioError) {
			return false;
		} else {
			silent++;
		}
	}
	return *flag;
}

static int wmiScan(void) {
	if (!wmiCmd32(WMI_CMD_ERROR_REPORT, 0x7F)) return -48;
	if (!wmiCmd32(WMI_CMD_SET_HB_TIMEOUT, 0)) return -49;

	// Only the channels the firmware allows
	if (!wmiCmd(WMI_CMD_GET_CHANNEL_LIST)) return -57;
	if (!wmiWaitFor(&channelsReady, 8)) return -58;
	logValue("Channels : ", channelCount);
	if (channelCount == 0) return -59;

	// Passive scan, everything else off
	if (!wmiBegin() || !wmiSend(WMI_CMD_SET_PROBED_SSID, 35)) return -50;

	if (!wmiBegin()) return -51;
	txBuf[WMI_BODY+2] = WMI_PHY_11G;
	txBuf[WMI_BODY+3] = channelCount;
	for (u32 i = 0; i < channelCount; i++) {
		txPut16(WMI_BODY + 4 + i*2, channels[i]);
	}
	if (!wmiSend(WMI_CMD_SET_CHANNEL_PARAMS, 4 + channelCount*2)) return -51;

	if (!wmiBegin()) return -52;
	txPut16(WMI_BODY+0, 0xFFFF); // Foreground start period
	txPut16(WMI_BODY+2, 0xFFFF); // Foreground end period
	txPut16(WMI_BODY+4, 0xFFFF); // Background period
	txPut16(WMI_BODY+6, 105); // Max active dwell time
	txPut16(WMI_BODY+8, 105); // Passive dwell time
	txBuf[WMI_BODY+11] = WMI_SCAN_CONNECT;
	txPut16(WMI_BODY+12, 105); // Min active dwell time
	if (!wmiSend(WMI_CMD_SET_SCAN_PARAMS, 20)) return -52;

	if (!wmiBegin()) return -53;
	txBuf[WMI_BODY] = WMI_BSS_FILTER_ALL;
	if (!wmiSend(WMI_CMD_SET_BSS_FILTER, 8)) return -53;

	if (!wmiBegin()) return -54;
	txPut32(WMI_BODY+8, 20); // Home dwell time
	if (!wmiSend(WMI_CMD_START_SCAN, 18)) return -54;

	const bool finished = wmiWaitFor(&scanDone, 30);
	logValue("Scan results : ", scanResults);
	if (sdioError) return -55;
	return finished ? 0 : -56;
}

// Stops scanning and asks the firmware to join the network, as Calico does
static int wmiConnect(void) {
	if (!wmiBegin() || !wmiSend(WMI_CMD_SET_BSS_FILTER, 8)) return -62;
	if (!wmiBegin() || !wmiSend(WMI_CMD_SET_PROBED_SSID, 35)) return -63;

	if (!wmiBegin()) return -64;
	txPut16(WMI_BODY+0, 0xFFFF);
	txPut16(WMI_BODY+2, 0xFFFF);
	txPut16(WMI_BODY+4, 0xFFFF);
	txPut16(WMI_BODY+6, 200);
	txPut16(WMI_BODY+8, 200);
	txBuf[WMI_BODY+11] = WMI_SCAN_CONNECT | WMI_SCAN_ACTIVE | WMI_SCAN_ROAM;
	txPut16(WMI_BODY+12, 200);
	if (!wmiSend(WMI_CMD_SET_SCAN_PARAMS, 20)) return -64;

	if (!wmiBegin()) return -65;
	txBuf[WMI_BODY+2] = WMI_PHY_11G;
	if (!wmiSend(WMI_CMD_SET_CHANNEL_PARAMS, 4)) return -65;

	if (!wmiBegin()) return -66;
	txBuf[WMI_BODY+0] = 0xFF; // Automatic data rate
	if (!wmiSend(WMI_CMD_SET_BIT_RATE, 3)) return -66;

	if (!wmiBegin()) return -67;
	txBuf[WMI_BODY+0] = 1;
	txBuf[WMI_BODY+1] = 4 | (10 << 4); // PS-Poll
	txPut16(WMI_BODY+2, 0xFFF7); // Everything but 11 Mbps
	if (!wmiSend(WMI_CMD_SET_FRAME_RATE, 4)) return -67;

	wmiCmd8(WMI_CMD_SYNCHRONIZE, 0);
	if (!wmiCmd8(WMI_CMD_SET_POWER_MODE, WMI_POWER_MAX_PERFORMANCE)) return -68;
	wmiCmd8(WMI_CMD_SYNCHRONIZE, 0);

	if (!wmiBegin()) return -69;
	txPut32(WMI_BODY+0, 20); // Min and max service interval
	txPut32(WMI_BODY+4, 20);
	txPut32(WMI_BODY+8, 9999999); // Inactivity
	txPut32(WMI_BODY+12, 0xFFFFFFFF); // Suspension
	txPut32(WMI_BODY+20, 83200); // Min, mean and peak data rate
	txPut32(WMI_BODY+24, 83200);
	txPut32(WMI_BODY+28, 83200);
	txPut32(WMI_BODY+40, 6000000); // Min PHY rate
	txPut32(WMI_BODY+44, 0x2000); // Surplus bandwidth allowance
	txPut16(WMI_BODY+52, 0x80D0); // Nominal MSDU
	txPut16(WMI_BODY+54, 0x00D0); // Max MSDU
	txBuf[WMI_BODY+57] = 2; // Both directions
	txBuf[WMI_BODY+58] = 0xFF; // RX queue
	txBuf[WMI_BODY+59] = 1; // Periodic
	txBuf[WMI_BODY+61] = 5; // TSID
	if (!wmiSend(WMI_CMD_CREATE_PSTREAM, 63)) return -69;

	if (!wmiCmd8(WMI_CMD_SET_WSC_STATUS, 0)) return -70;
	if (!wmiCmd8(WMI_CMD_SET_DISC_TIMEOUT, 2)) return -71;
	if (!wmiCmd8(WMI_CMD_SET_KEEP_ALIVE, 0)) return -72;

	if (!wmiBegin()) return -73;
	txBuf[WMI_BODY+0] = 1; // Infrastructure
	txBuf[WMI_BODY+1] = 1; // Open system authentication
	txBuf[WMI_BODY+2] = WMI_AUTH_WPA2_PSK;
	txBuf[WMI_BODY+3] = WMI_CIPHER_AES;
	txBuf[WMI_BODY+5] = (targetGroupCipher == 2) ? WMI_CIPHER_TKIP : WMI_CIPHER_AES;
	txBuf[WMI_BODY+7] = targetNetwork->ssidLen;
	txCopy(WMI_BODY + 8, targetNetwork->ssid, targetNetwork->ssidLen);
	txPut16(WMI_BODY+40, targetMhz);
	txCopy(WMI_BODY + 42, targetBssid, sizeof(targetBssid));
	if (!wmiSend(WMI_CMD_CONNECT, 52)) return -73;

	if (!wmiWaitFor(&wifiAssociated, 40)) return wifiDisconnected ? -74 : -75;
	if (!wmiWaitFor(&wpaComplete, 40)) return wifiDisconnected ? -76 : -77;
	logValue("WPA2 handshake complete : ", 1);

	// Encrypted traffic from the router shows the keys work
	for (int silent = 0, i = 0; i < 300 && silent < 8 && !wifiDisconnected; i++) {
		if (wmiPump()) {
			silent = 0;
		} else if (sdioError) {
			break;
		} else {
			silent++;
		}
	}
	logValue("Data frames : ", dataFrames);
	logValue("Still connected : ", !wifiDisconnected);

	wmiCmd(WMI_CMD_DISCONNECT);
	return wifiDisconnected ? -78 : 0;
}

static int htcHandshake(void) {
	if (!htcRecv()) return -41;
	if (htcBuf[0] != 0 || htcGet16(HTC_HDR) != HTC_MSG_READY) return -42;
	htcCredits = htcGet16(HTC_HDR+2);
	logValue("HTC credits : ", htcCredits);
	logValue("HTC credit size : ", htcGet16(HTC_HDR+4));

	wmiEndpoint = htcConnect(HTC_SRV_WMI_CONTROL, 0);
	if (wmiEndpoint == 0) return -43;
	logValue("WMI endpoint : ", wmiEndpoint);
	for (u16 service = HTC_SRV_WMI_DATA_BE; service <= HTC_SRV_WMI_DATA_VO; service++) {
		const u8 endpoint = htcConnect(service, HTC_FLAGS_WMI_DATA);
		if (endpoint == 0) return -44;
		if (service == HTC_SRV_WMI_DATA_BE) {
			wmiDataEndpoint = endpoint;
		}
	}
	logValue("WMI data endpoint : ", wmiDataEndpoint);

	txClear();
	txPut16(HTC_HDR, HTC_MSG_SETUP_COMPLETE);
	if (!htcSend(HTC_HDR + 2)) return -45;

	for (int i = 0; ; i++) {
		if (i == 8) return -47;
		if (!htcRecv()) return -46;
		if (htcBuf[0] == wmiEndpoint && htcGet16(HTC_HDR) == WMI_EVENT_READY) {
			break;
		}
		logValue("Other message : ", htcWords[0]);
	}
	copyBytes(selfMac, htcBuf + WMI_BODY, sizeof(selfMac));
	logValue("MAC high : ", (selfMac[0] << 16) | (selfMac[1] << 8) | selfMac[2]);
	logValue("MAC low : ", (selfMac[3] << 16) | (selfMac[4] << 8) | selfMac[5]);
	logValue("PHY capability : ", htcBuf[WMI_BODY+6]);

	// The firmware ignores WMI commands until the host reports its protocol
	// version, at the address held in the first host interest word
	const u32 appHostInterest = readInternWord53(hostInterestAddr);
	if (sdioError) return -60;
	logValue("App host interest : ", appHostInterest);
	writeInternWord53(appHostInterest, WMI_PROTOCOL_VERSION);
	if (sdioError) return -61;

	readSavedNetworks();

	const int scan = wmiScan();
	if (scan != 0) return scan;
	if (targetNetwork == NULL) return -79;

	const int connect = wmiConnect();
	logValue("Credits left : ", htcCredits);
	return connect;
}

// Nothing secret stays in memory once the test is over
static void wifiClearSecrets(void) {
	clearBytes((u8*)savedNetworks, sizeof(savedNetworks));
	clearBytes(ptk, sizeof(ptk));
	clearBytes(gtk, sizeof(gtk));
	clearBytes(keyData, sizeof(keyData));
	clearBytes(anonce, sizeof(anonce));
	clearBytes(htcBuf, sizeof(htcWords));
	clearBytes(txBuf, sizeof(txWords));
	targetNetwork = NULL;
	ptkReady = false;
}

// Soft resets the chip, keeping the firmware the system loaded, and restarts it,
// in the same order official code uses.
static int restartFirmware(u32 hostInterest) {
	writeInternWord53(0x00004000, 0x00000100);
	if (sdioError) return -25;
	swiDelay(0x80000);
	const u32 resetCause = readInternWord53(0x000040C0);
	if (sdioError) return -26;
	logValue("Reset cause : ", resetCause);
	if ((resetCause & 7) != 2) return -27;

	u32 cmd = BMI_GET_TARGET_ID;
	u32 version = 0;
	if (!bmiSend(&cmd, sizeof(cmd)) || !bmiRecv(&version)) return -28;
	if (version == 0xFFFFFFFF) {
		u32 size = 0;
		u32 type = 0;
		if (!bmiRecv(&size) || !bmiRecv(&version) || !bmiRecv(&type)) return -29;
		logValue("BMI target type : ", type);
	}
	logValue("BMI version : ", version);

	if (!bmiWriteMemoryWord(hostInterest + 0x00, 2)) return -30; // HTC protocol version
	if (!bmiWriteSocReg(0x4028, 5)) return -31; // WLAN_CLOCK_CONTROL
	if (!bmiWriteSocReg(0x4020, 0)) return -32; // WLAN_CPU_CLOCK
	if (!bmiWriteMemoryWord(hostInterest + 0x6C, 128)) return -33; // Mailbox block size
	if (!bmiWriteMemoryWord(hostInterest + 0x74, 0x63)) return -34;

	cmd = BMI_DONE;
	if (!bmiSend(&cmd, sizeof(cmd))) return -35;

	u32 attempts;
	u32 ready = 0;
	for (attempts = 0; attempts < 2000 && !ready; attempts++) {
		ready = readInternWord53(hostInterest + 0x58);
		if (sdioError) return -36;
		swiDelay(0x2000);
	}
	logValue("Firmware start attempts : ", attempts);
	logValue("Board init after start : ", ready);
	if (!ready) return -37;

	return 0;
}

// Starts the firmware, then waits for WMI to be ready and scans.
static int startFirmware(void) {
	warmControllerInit();

	cmd52Read(0, 0x04);
	if (sdioError) return -20;
	if (!(cmd52Read(0, 0x07) & 2)) return -21;
	sdioMask16(SDIO_CARD_OPT, BIT(15), 0); // 4-bit

	const u32 chipId = readInternWord53(0x000040EC);
	if (sdioError) return -22;
	logValue("Chip ID : ", chipId);
	const u32 hostInterest = (chipId == CHIP_ID_AR6002) ? AR6002_HOST_INTEREST_ADDRESS : AR601x_HOST_INTEREST_ADDRESS;

	const u32 boardInit = readInternWord53(hostInterest + 0x58);
	if (sdioError) return -23;
	logValue("Board init before reset : ", boardInit);
	if (boardInit == 0) return -24;

	const int restart = restartFirmware(hostInterest);
	if (restart != 0) return restart;

	const u32 boardData = readInternWord53(hostInterest + 0x54);
	logValue("Board data : ", boardData);
	logValue("EEPROM version : ", readInternWord53(boardData + 0x10));
	if (sdioError) return -38;

	hostInterestAddr = hostInterest;
	const int ret = htcHandshake();
	wifiClearSecrets();

	// Release the radio for the game with a fresh start. The game's WiFi search
	// locks up if the chip is left in its bootloader, so firmware has to be running
	sdioError = false;
	logValue("Final restart : ", (u32)restartFirmware(hostInterest));
	return ret;
}

void twlWifiStartFirmware(void) {
	dbg_printf("twlWifiStartFirmware:\n");

	twlWifiProbeResult unused;
	resetResult(&unused);

	const u16 oldGpioWifi = REG_GPIO_WIFI;

	u16 sdioSaved[SDIO_SAVED_COUNT];
	u32 sdioSavedIrqMask;
	sdioSave(sdioSaved, &sdioSavedIrqMask);

	REG_GPIO_WIFI &= ~BIT(8); // Select Atheros
	const int ret = startFirmware();
	sdioRestore(sdioSaved, sdioSavedIrqMask);
	REG_GPIO_WIFI = oldGpioWifi;

	logValue("Result : ", (u32)ret);
	logValue("First failed cmd : ", sdioFirstErrCmd);
	logValue("First failed arg : ", sdioFirstErrArg);
	logValue("First failed stat : ", sdioFirstErrStat);
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
