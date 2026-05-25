# BerryRocket - WiFi AP + minimal WebSocket server pushing Nectar frames
# Licence CC-BY-NC-SA

import struct
from lib.nectar import build_frame
import time

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_PAYLOAD_FMT = "<I9fB"  # time_ms, p, t, ax,ay,az, gx,gy,gz, t_imu, flags


class TelemetryWS:
    def __init__(self, ssid_prefix, open_network, password,
                 channel, port, ssid_type, apid, ssid_num=None, debug=False):
        self.ssid_prefix = ssid_prefix
        self.open_network = open_network
        self.password = password
        self.channel = channel
        self.port = port
        self.ssid_type = ssid_type
        # ssid_num=None -> dérivé du dernier octet de machine.unique_id() dans start()
        self.ssid_num = ssid_num
        self.apid = apid
        self._ok = False
        self._srv = None
        self._client = None
        self._wlan = None
        self.ssid_ap = None  # SSID final calcule dans start()
        self.debug = debug

    def _build_ssid(self):
        try:
            import machine, binascii
            uid = machine.unique_id()           # 8 octets sur RP2040
            suffix = binascii.hexlify(uid[-2:]).decode().upper()  # 2 derniers octets -> 4 hex
        except Exception:
            suffix = "0000"
        full = "{}-{}".format(self.ssid_prefix, suffix)
        return full[:32]  # limite IEEE 802.11

    def start(self):
        try:
            try:
                import os
                machine_str = os.uname().machine
            except Exception:
                machine_str = ""
            if "Pico W" not in machine_str:
                print("[telem] carte detectee:", machine_str or "inconnue",
                      "- pas de WiFi (Pico non-W), telemetrie desactivee")
                self._ok = False
                return False

            import network
            import socket
            self._socket_mod = socket

            # Code pays : évite la phase prudente du driver CYW43 au démarrage
            # (sinon scan régulatoire + délais d'association inutiles).
            try:
                network.country("FR")
            except (AttributeError, OSError):
                pass

            ssid = self._build_ssid()
            self.ssid_ap = ssid

            # Si ssid_num n'a pas été imposé, on le dérive du dernier octet
            # de machine.unique_id() -> mission Nectar unique par carte,
            # visuellement corrélée au suffixe WiFi (le dernier byte hex).
            if self.ssid_num is None:
                try:
                    import machine
                    self.ssid_num = machine.unique_id()[-1] & 0xFF
                except Exception:
                    self.ssid_num = 0

            wlan = network.WLAN(network.AP_IF)
            wlan.active(False)

            if self.open_network:
                # security=0 -> reseau ouvert (constante MicroPython OPEN)
                try:
                    wlan.config(essid=ssid, security=0, channel=self.channel)
                except (OSError, ValueError, TypeError):
                    # Firmware ancien : config separee
                    try:
                        wlan.config(essid=ssid, security=0)
                    except (OSError, ValueError, TypeError):
                        wlan.config(essid=ssid)
                        try:
                            wlan.config(security=0)
                        except Exception:
                            pass
            else:
                try:
                    wlan.config(essid=ssid, password=self.password,
                                channel=self.channel)
                except (OSError, ValueError, TypeError):
                    wlan.config(essid=ssid, password=self.password)

            # Désactive le powersave WiFi
            try:
                wlan.config(pm = network.WLAN.PM_NONE, txpower = 18)
            except (OSError, ValueError, TypeError):
                pass

            wlan.active(True)
            # Wait for the AP interface to be up
            for _ in range(50):
                if wlan.active():
                    break
                time.sleep_ms(100)
            self._wlan = wlan

            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
            srv.setblocking(False)
            self._srv = srv

            self._ok = True
            try:
                ip = wlan.ifconfig()[0]
            except Exception:
                ip = "?"
            mode = "OPEN" if self.open_network else "WPA2"
            ssid_type_str = ("FX", "MF", "BALLOON", "OTHER")[self.ssid_type & 0x03]
            print("[telem] AP up SSID='{}' ({}) IP={} WS port={} mission={}{} APID={}".format(
                ssid, mode, ip, self.port,
                ssid_type_str, self.ssid_num, self.apid))
            return True
        except Exception as e:
            print("[telem] start failed:", e)
            self._ok = False
            return False

    # --- internals -----------------------------------------------------

    def _handshake(self, cli):
        # Read HTTP request until \r\n\r\n (or short timeout)
        cli.settimeout(0.1)
        buf = b""
        try:
            while b"\r\n\r\n" not in buf and len(buf) < 2048:
                chunk = cli.recv(512)
                if not chunk:
                    return False
                buf += chunk
        except OSError:
            return False

        # Extract Sec-WebSocket-Key (case-insensitive header lookup)
        key = None
        for line in buf.split(b"\r\n"):
            if b":" in line:
                name, _, value = line.partition(b":")
                if name.strip().lower() == b"sec-websocket-key":
                    key = value.strip()
                    break
        if key is None:
            # # Pas un upgrade WebSocket : très probablement une sonde captive
            # # portal de l'OS (Windows: msftconnecttest, Android: gstatic 204,
            # # macOS/iOS: captive.apple.com). On renvoie 204 No Content -> l'OS
            # # considère "internet OK" immédiatement et arrête de poller.
            # try:
            #     cli.send(b"HTTP/1.1 204 No Content\r\n"
            #              b"Content-Length: 0\r\n"
            #              b"Connection: close\r\n\r\n")
            # except OSError:
            #     pass
            return False

        try:
            import hashlib, binascii
            sha = hashlib.sha1(key + _WS_GUID.encode())
            accept = binascii.b2a_base64(sha.digest()).strip()
        except Exception as e:
            print("[telem] handshake hash error:", e)
            return False

        response = (
            b"HTTP/1.1 101 Switching Protocols\r\n"
            b"Upgrade: websocket\r\n"
            b"Connection: Upgrade\r\n"
            b"Sec-WebSocket-Accept: " + accept + b"\r\n\r\n"
        )
        try:
            cli.send(response)
        except OSError:
            return False

        cli.setblocking(False)
        return True

    def _accept_if_pending(self):
        if self._srv is None:
            return
        try:
            cli, addr = self._srv.accept()
        except OSError:
            return
        if self.debug is True:
            print("[telem] incoming client:", addr)
        # Replace any existing client
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None
        if self._handshake(cli):
            self._client = cli
            if self.debug is True:
                print("[telem] WS client connected")
        else:
            try:
                cli.close()
            except OSError:
                pass

    def _drop_client(self):
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None

    # --- public emission ----------------------------------------------

    def send_telemetry(self, time_ms, pressure_bar, temp_c,
                       ax, ay, az, gx, gy, gz, temp_imu_c, flags):
        if not self._ok:
            return
        self._accept_if_pending()
        if self._client is None:
            return

        try:
            payload = struct.pack(_PAYLOAD_FMT,
                                  int(time_ms) & 0xFFFFFFFF,
                                  (pressure_bar), (temp_c),
                                  (ax), (ay), (az),
                                  (gx), (gy), (gz),
                                  (temp_imu_c),
                                  int(flags) & 0xFF)
            frame = build_frame(self.ssid_type, self.ssid_num, self.apid, payload)
        except Exception as e:
            print("[telem] encode error:", e)
            return

        # WS server->client binary frame: FIN=1, opcode=2, no mask, len<126
        ws_msg = bytes((0x82, len(frame))) + frame
        try:
            self._client.send(ws_msg)
        except OSError:
            print("[telem] client disconnected")
            self._drop_client()

    def stop(self):
        self._drop_client()
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
            self._srv = None
        if self._wlan is not None:
            try:
                self._wlan.active(False)
            except OSError:
                pass
            self._wlan = None
        self._ok = False
