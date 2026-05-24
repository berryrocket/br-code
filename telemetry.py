# BerryRocket - WiFi AP + minimal WebSocket server pushing Nectar frames
# Licence CC-BY-NC-SA

import struct
from lib.nectar import build_frame

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
_PAYLOAD_FMT = "<I8fB"  # time_ms, p, t, ax,ay,az, gx,gy,gz, t_imu, flags


class TelemetryWS:
    def __init__(self, ssid_ap, password, channel, port,
                 ssid_type, ssid_num, apid):
        self.ssid_ap = ssid_ap
        self.password = password
        self.channel = channel
        self.port = port
        self.ssid_type = ssid_type
        self.ssid_num = ssid_num
        self.apid = apid
        self._ok = False
        self._srv = None
        self._client = None
        self._wlan = None

    def start(self):
        try:
            import network
            import socket
            self._socket_mod = socket

            wlan = network.WLAN(network.AP_IF)
            wlan.active(False)
            try:
                wlan.config(essid=self.ssid_ap, password=self.password,
                            channel=self.channel)
            except (OSError, ValueError):
                # Older MicroPython may not accept channel kw; retry without it
                wlan.config(essid=self.ssid_ap, password=self.password)
            wlan.active(True)
            # Wait for the AP interface to be up
            for _ in range(50):
                if wlan.active():
                    break
                import time
                time.sleep_ms(100)
            self._wlan = wlan

            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", self.port))
            srv.listen(1)
            srv.setblocking(False)
            self._srv = srv

            self._ok = True
            print("[telem] AP up, WS server on port", self.port)
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

    def send_telemetry(self, time_ms, pressure_hpa, temp_c,
                       ax, ay, az, gx, gy, gz, temp_imu_c, flags):
        if not self._ok:
            return
        self._accept_if_pending()
        if self._client is None:
            return

        try:
            payload = struct.pack(_PAYLOAD_FMT,
                                  int(time_ms) & 0xFFFFFFFF,
                                  float(pressure_hpa), float(temp_c),
                                  float(ax), float(ay), float(az),
                                  float(gx), float(gy), float(gz),
                                  float(temp_imu_c),
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
