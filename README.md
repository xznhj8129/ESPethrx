# ESPethrx
Use WT32-ETH01 ESP32 as IP-enabled serial receiver for drones, UDP UART Bridge (ETH primary, AP fallback)
Tunnels raw bytes between a flight controller UART and the network over UDP.  
- **MSP**: plain request/response over UDP (default port **5760**)  
- **MAVLink**: sends to broadcast until a peer talks (default port **14550**)  

Read https://github.com/egnor/wt32-eth01 in depth for how to deal with this thing.

---
## TODO:
- Fix AP rapid cycling when connected router is booting
- Convert to platformio
- Test Mavlink
- Add TCP

---

## Hardware
### Flashing:
- Connect 5V/3.3V, GND, TX to RX, and RX to TX on the USB to TTL converter.
- Short GND to BOOT pin (IO0, under rx0)
- pulse EN to GND to enter upload mode
- After the upload, power off, disconnect IO0 from GND (back to IDLE), and power the device again. It will run the uploaded code automatically.

### Use:
- ETH01 to FC:
  - **GPIO33 (CFG)** ← FC TX
  - **GPIO32 (485EN)** → FC RX
  - **5V** → FC 4V5 (or 3.3V to 3V3)
  - **GND** ↔ **GND**

---

## Boot behavior

- On power up, waits **10 s** for Ethernet link/IP.
- If link/IP is ready: runs the UDP bridge.
- If not: brings up a WiFi AP for config:
  - SSID: **FC-Configurator**
  - IP: **192.168.0.1**
  - Open network (no password)

---

## Configuration API (HTTP GET)

All endpoints are simple GETs you can paste in a browser.

- Show current settings and MAC:
  - `http://192.168.0.1/info`
  - `http://192.168.0.1/mac`
- Set static IP (disables DHCP) and network params:
  - `http://192.168.0.1/set/assign_ip=192.168.1.101`
  - `http://192.168.0.1/set/assign_gw=192.168.1.1`
  - `http://192.168.0.1/set/assign_mask=255.255.255.0`
  - Response will say: `ok (reboot to apply IP settings)`
  - Then: `http://192.168.0.1/reboot`
- Optional tweaks:
  - Set ID tag: `http://192.168.0.1/set/assign_id=1`
  - Set hostname: `http://192.168.0.1/set/hostname=fc4`
  - Select mode: `http://192.168.0.1/set/mode=msp` or `.../mode=mavlink`
  - Override port: `http://192.168.0.1/set/port=5760`

You can also use the short path form `/set/key=value` directly in the URL bar.

---

## Using it

### MSP (INAV)
1. On the FC, enable **MSP** on the UART wired to the WT32; **115200 baud**, not inverted.
2. In **INAV Configurator**:
   - Connection type: **UDP**
   - Host: your static IP (example `192.168.1.101`)
   - Port: **5760**
2. In **MWP**:
   - Host: udp://your_ip:5760
3. Click Connect. The bridge unicasts replies to the last peer that sent a packet.

### MAVLink (QGC, Mission Planner over UDP)
- **Not thouroughly tested yet**
- Ground station listens on **14550**.
- The bridge broadcasts MAVLink until a peer speaks, then unicast to that peer.

---

## Status output

On serial (for bench), you will see:
- `[ETH] MAC xx:xx:xx:xx:xx:xx`
- `[ETH] static IP a.b.c.d applied`
- `[UDP] listening on PORT (MSP|MAVLink)`
- `[UDP] peer a.b.c.d:port` when a client talks

When buried in a frame and no serial is available, use the AP `/info` page for MAC and settings.

---

## Notes and limits

- Experimental, be careful.
- One active peer at a time (last sender wins). Idle peer times out after **60 s**.
- AP appears only if Ethernet did not come up within 10 s after boot.
- If UDP connects are flaky, check for duplicate IPs and host firewalls.
- The bridge does not alter bytes. It is transparent at the packet level.

