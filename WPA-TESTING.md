# WPA research build: WM request table finder

Log-only change for issue #628. Nothing in the game is patched. On boot, nds-bootstrap finds the ARM7 table that
dispatches WM (wireless) API calls and writes its location to the log.

Changed files:
- `retail/bootloaderi/source/arm7/find_wifi_arm7.c` (new): `findWmRequestFuncTable`
- `retail/bootloaderi/include/find.h`: declarations
- `retail/bootloaderi/source/arm7/patch_arm7.c`: `logWmRequestFuncTable`, called at the end of `patchCardNdsArm7`

## Expected result for Animal Crossing: Wild World (ADME, USA Rev 1)

Checked offline against the ARM7 binary of a dump with SHA-1 `77fde3e30e1e6068395d1f96ea63be569b61c351`:

| | Value |
|---|---|
| Table offset in the ARM7 binary | `0x282EC` (log shows `0x02380000 + 0x282EC` = `0x023A82EC` if loaded unmoved) |
| WM API count | `0x2C` (44) |
| `WMSP_StartConnectEx` (0x0C) | `0x060043B8` |
| `WMSP_SetDCFData` (0x12) | `0x06005690` |
| `WMSP_SetWEPKeyEx` (0x27) | `0x060058B0` |

Also checked against the wireless code assembled from the pokediamond and pokeheartgold decomps (46 APIs each).

## Building

The build has to run from a path without spaces. CI uses the `devkitpro/devkitarm:20241104` image (libnds 1.8.x);
current libnds 2.x has not been tried.

With Docker:

```bash
docker run --rm -v "$PWD":/src -w /src devkitpro/devkitarm:20241104 make -C retail nightly
```

Output: `retail/bin/nds-bootstrap-nightly.nds`.

## Testing on a DSi or 3DS

1. Back up `sd:/_nds/nds-bootstrap-nightly.nds`, then copy the new build over it (TWiLight Menu++ nightly setting),
   or over `nds-bootstrap-release.nds` if you use release.
2. In `sd:/_nds/nds-bootstrap.ini`, under `[NDS-BOOTSTRAP]`, set `LOGGING = 1`.
3. Boot Wild World in DS mode. It doesn't need to go online.
4. Open `sd:/NDSBTSRP.LOG` and look for `findWmRequestFuncTableOffset:`. It should say `WM request func table found`,
   followed by the values above.
5. Set `LOGGING = 0` again and restore the backup when done.

## DSi-mode WiFi probe (driver step 1)

`retail/bootloaderi/source/arm7/twl_wifi_probe.c`, called from `main.arm7.c` just before the switch to old NDS-WiFi
mode, only when `LOGGING = 1`. It selects the Atheros chip, brings up SDIO with CMD5/CMD3/CMD7, enables function 1
and logs the SDIO revision, manufacturer ID, chip ID and the host-interest "firmware uploaded" word. Then it restores
the GPIO, SCFG_WL and LED state. Based on DSiWifi (MIT, `dsiwifi_license.txt`), with timeouts on every wait.

## Roadmap

Each step ends with something checkable in `NDSBTSRP.LOG` on hardware.

1. **Probe** (this build): chip answers over SDIO in DSi mode on DSi and 3DS.
2. **BMI**: reset into the Atheros bootloader and read the BMI target version over the mailbox (CMD53 block
   transfers). Then restore the chip, and check that the game's own WiFi still works afterwards.
3. **Firmware and WMI**: upload/start firmware (step 1's "Firmware uploaded" value shows whether the system
   left firmware running), wait for WMI ready, and read the MAC address.
4. **Scan**: run a WMI scan and log the SSIDs and security types. That shows WPA2 networks are visible.
5. **Connect, WPA2**: load the network from DSi connection slots 4–6. Then PMK from passphrase (PBKDF2-SHA1),
   EAPOL four-way handshake (PRF, MIC, AES key unwrap of the GTK) and installing the keys with WMI. The chip does
   CCMP itself. The crypto can be checked against the IEEE 802.11i test vectors on a computer.
6. **Run during the game**: move the driver from the bootloader into the in-game ARM7 engine (memory budget),
   keep the chip in DSi mode, and replace the WM request table entries (0x0C connect, 0x0D disconnect, 0x26 scan,
   0x11–0x13 data) with handlers that talk WMI and answer the game the way the original handlers did.
7. **Wild World online**: DWC gets an IP and reaches the server (e.g. Wiimmfi), using the game's own TCP/IP stack
   on top of the replaced data path.
