# Palm Bridge

ESP32 (WROOM-32) firmware that gives a Palm OS 5 device (tested target: Zire 72) network access over Bluetooth.

The Palm connects to the ESP32 as a Bluetooth "PC" (LAN Access, like Linux `dund --msdun`). The ESP32 runs a
PPP server and NATs the traffic onto your LAN over Wi-Fi. A Bluetooth "Modem" (DUN) connection is also answered,
but the Zire 72 never got past "Dialing" with it. The Palm gets `192.168.44.2` and can reach anything on
the LAN or internet. LAN hosts can't start connections to the Palm, because it sits behind NAT.

## Build & flash

Needs Docker only (uses the `espressif/idf:v5.4` image).

```sh
cp main/secrets.h.example main/secrets.h   # set WIFI_SSID / WIFI_PASS (optional WIFI_MAC)
./build.sh                                 # build
./flash.sh                                 # build + flash + serial monitor (Ctrl+] exits)
PORT=/dev/ttyACM0 ./flash.sh               # other serial port
./build.sh menuconfig                      # any other idf.py command
```

## Palm setup

1. **Prefs → Bluetooth**: Bluetooth On → Setup Devices → Trusted Devices → Add Device → pick `PalmBridge`,
   passkey `0000`.
2. **Prefs → Connection**: New → Name `PalmBridge`, Connect to **PC**, Via **Bluetooth**, Device `PalmBridge`.
   Details: 115,200 bps, flow control Automatic.
3. **Prefs → Network**: Menu → Service → New. Connection: `PalmBridge`. Username/password: anything (ignored).
   Details: PPP, Query DNS on, IP address Automatic.
4. Tap **Connect**. The serial monitor shows `Palm connected`, then `PPP up, Palm is 192.168.44.2`.

## Changing Wi-Fi from the Palm

While connected, open `http://192.168.44.1/` in the Palm browser (proxy off). It shows the current network
and a scan, so you can pick a network, enter its password and tap Connect. This works even when the ESP32 has
no Wi-Fi, because the page is served over the Bluetooth link. Reconnect the Palm afterwards so it gets the
new network's DNS server. Only the Palm can use the page; requests from the LAN get 403.

The chosen network is saved to the ESP32's flash. `secrets.h` only seeds the first boot, so editing it later
has no effect until you run `./build.sh -p /dev/ttyACM0 erase-flash` (with `DOCKER_ARGS` set as in `flash.sh`)
and reflash.

## Troubleshooting

- **"DNS works, nothing loads"**: the serial log prints `stats`/`ip`/`tcp` counters every 5 s while the Palm is
  connected. `fw` rising with `out` flat means requests leave over Wi-Fi but no replies come back, so look
  upstream of the ESP32. On the test network the router stopped answering TCP for the ESP32's factory MAC
  (cause not pinned down, no firewall rule matched); setting `WIFI_MAC` in `main/secrets.h` to a fresh,
  locally administered address fixed it.
- Palm browsers may default to dead home pages or proxies (`www.palmone.com`); disable the proxy and type a URL.
- Speed tops out around 3-5 KB/s (Bluetooth 1.1 on the Palm side).

## Tuning

- Name, PIN, Wi-Fi country, IP addresses: `#define`s and `ppp_start()` in `main/main.c`.
- PPP hexdumps: uncomment the `esp_log_level_set` lines in `app_main()`. They slow the link noticeably.
