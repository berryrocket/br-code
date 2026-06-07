########################################
#### BerryRocket ####
# WiFi AP + minimal WebSocket server pushing Nectar frames
# Louis Barbier
# Licence CC-BY-NC-SA
########################################

import struct
import time
try:
    import errno
    _EAGAIN = errno.EAGAIN
except (ImportError, AttributeError):
    _EAGAIN = 11  # MicroPython embarque parfois errno sans EAGAIN

from lib.nectar import build_frame

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
# Payload binaire emis dans chaque trame Nectar :
#   <   : little-endian, packed (pas d'alignement)
#   I   : time_ms                              (uint32, 4 B)
#   13f : pressure, temp, ax, ay, az,
#         gx, gy, gz, mx, my, mz
#         temp_imu, temp_mag                   (13 float32, 52 B)
#   B   : flags                                (uint8, 1 B)
# Total = 57 octets.
_PAYLOAD_FMT = "<I13fB"


class TelemetryWS:
    def __init__(self, ssid_prefix, open_network, password,
                 channel, port, ssid_type, apid, ssid_num=None,
                 rate_hz=0, debug=False):
        if not (0 <= ssid_type <= 3):
            raise ValueError("[TELEM] ssid_type out of range 0..3")
        if not (0 <= apid <= 63):
            raise ValueError("[TELEM] apid out of range 0..63")
        if ssid_num is not None and not (0 <= ssid_num <= 255):
            raise ValueError("[TELEM] ssid_num out of range 0..255")
        if rate_hz < 0:
            raise ValueError("[TELEM] rate_hz must be >= 0")

        self.ssid_prefix = ssid_prefix
        self.open_network = open_network
        self.password = password
        self.channel = channel
        self.port = port
        self.ssid_type = ssid_type
        # ssid_num=None -> dérivé du dernier octet de machine.unique_id() dans start()
        self.ssid_num = ssid_num
        self.apid = apid
        # rate_hz=0 -> pas de limitation, émet à chaque appel de send_telemetry.
        # rate_hz>0 -> au plus 1 frame toutes les (1000/rate_hz) ms ; les appels
        # trop rapprochés sont ignorés silencieusement (pour ralentir le débit
        # radio sans toucher à la fréquence d'acquisition locale).
        self.period_ms = int(1000 / rate_hz) if rate_hz > 0 else 0
        self._last_tx_ms = None
        self._ok = False
        self._srv = None
        self._client = None
        self._wlan = None
        self.ssid_ap = None  # SSID final calcule dans start()
        self.debug = debug
        # Handshake non bloquant : on lit les headers HTTP en plusieurs passes
        # à travers send_telemetry() pour ne jamais bloquer la boucle d'acquisition.
        # _pending = (socket, buf, deadline_ticks) ou None.
        self._pending = None
        # Budget temps max par appel pour drainer le handshake (en ms).
        # À 20 Hz le cycle fait 50 ms, on reste très en dessous.
        self._handshake_slice_ms = 3
        # Deadline globale d'un handshake (en ms) : au-delà on abandonne.
        self._handshake_timeout_ms = 3000
        # Buffer de réception client (pour parser les frames WS entrantes :
        # ping/pong/close). Réinitialisé à chaque nouveau client.
        self._rx_buf = b""

    # --- helpers -------------------------------------------------------

    def _log(self, *args):
        if self.debug:
            print(*args)

    def _build_ssid(self):
        try:
            import machine, binascii
            uid = machine.unique_id()           # 8 octets sur RP2040
            suffix = binascii.hexlify(uid[-2:]).decode().upper()  # 2 derniers octets -> 4 hex
        except Exception:
            suffix = "0000"
        full = "{}-{}".format(self.ssid_prefix, suffix)
        return full[:32]  # limite IEEE 802.11

    @staticmethod
    def _has_wifi(machine_str):
        # La Pico W renvoie "Raspberry Pi Pico W with RP2040",
        # la Pico 2 W renvoie "Raspberry Pi Pico 2 W with RP2350".
        # On valide la présence du suffixe " W " (entouré d'espaces) ou en fin.
        return (" W " in machine_str) or machine_str.endswith(" W")

    @staticmethod
    def _set_country(code):
        # Sur RP2 c'est rp2.country(); certains forks exposent network.country().
        try:
            import rp2
            rp2.country(code)
            return True
        except (ImportError, AttributeError, OSError, ValueError):
            pass
        try:
            import network
            network.country(code)
            return True
        except (ImportError, AttributeError, OSError, ValueError):
            pass
        return False

    # --- lifecycle -----------------------------------------------------

    def start(self):
        """Démarre le point d'accès WiFi et le serveur WebSocket.
        À appeler une seule fois au boot, après __init__. Renvoie True si
        le WiFi/AP s'est bien levé (False sur Pico non-W ou erreur réseau).
        Aucune trame n'est émise tant que start() n'a pas réussi."""
        try:
            try:
                import os
                machine_str = os.uname().machine
            except Exception:
                machine_str = ""
            if not self._has_wifi(machine_str):
                if self.debug:
                    print("[TELEM] carte detectee:", machine_str or "inconnue",
                          "- pas de WiFi (Pico non-W), telemetrie desactivee")
                self._ok = False
                return False

            import network
            import socket

            # Code pays : régulatoire + déverrouille les puissances ETSI.
            # Doit être positionné AVANT wlan.active(True).
            self._set_country("FR")

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

            # Désactive le powersave WiFi + puissance d'émission max ETSI.
            try:
                wlan.config(pm=network.WLAN.PM_NONE, txpower=18)
            except (OSError, ValueError, TypeError, AttributeError):
                try:
                    wlan.config(pm=network.WLAN.PM_NONE)
                except (OSError, ValueError, TypeError, AttributeError):
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
            # backlog=4 : le navigateur ouvre plusieurs connexions en
            # parallele au chargement de la page (GET /, /api/config,
            # /api/cmd/status, upgrade WS). Sans marge, certaines se
            # font dropper au niveau kernel -> "Failed to fetch" cote JS.
            srv.listen(4)
            srv.setblocking(False)
            self._srv = srv

            self._ok = True
            try:
                ip = wlan.ifconfig()[0]
            except Exception:
                ip = "?"
            mode = "OPEN" if self.open_network else "WPA2"
            ssid_type_str = ("FX", "MF", "BALLOON", "OTHER")[self.ssid_type]
            rate_str = "{}Hz".format(int(1000 / self.period_ms)) if self.period_ms > 0 else "unlimited"
            print("[TELEM] AP up SSID='{}' ({}) IP={} WS port={} mission={}{} APID={} rate={}".format(
                ssid, mode, ip, self.port,
                ssid_type_str, self.ssid_num, self.apid, rate_str))
            return True
        except Exception as e:
            print("[TELEM] start failed:", e)
            self._ok = False
            return False

    # --- internals -----------------------------------------------------

    def _finalize_handshake(self, cli, buf):
        # Cherche Sec-WebSocket-Key (case-insensitive).
        key = None
        for line in buf.split(b"\r\n"):
            if b":" in line:
                name, _, value = line.partition(b":")
                if name.strip().lower() == b"sec-websocket-key":
                    key = value.strip()
                    break
        if key is None:
            # Pas un upgrade WebSocket. Deux cas :
            #  - vraie requête HTTP (GET/POST) -> page de config / API JSON
            #    servie par web.py sur la même socket (même port que le WS).
            #  - sonde captive-portal de l'OS -> 204 pour faire taire l'OS.
            if buf.startswith(b"GET ") or buf.startswith(b"POST "):
                try:
                    from web import router as web
                    web.handle(cli, buf)
                except Exception as e:
                    self._log("[TELEM] web handler error:", e)
                    try:
                        cli.send(b"HTTP/1.1 500 Internal Server Error\r\n"
                                 b"Content-Length: 0\r\n"
                                 b"Connection: close\r\n\r\n")
                    except OSError:
                        pass
                return False
            # Verbe HTTP inconnu (HEAD, CONNECT...) -> redirect captif
            # vers la page de config pour declencher le portail OS.
            try:
                cli.send(b"HTTP/1.1 302 Found\r\n"
                         b"Location: http://192.168.4.1/\r\n"
                         b"Content-Length: 0\r\n"
                         b"Connection: close\r\n\r\n")
            except OSError:
                pass
            return False

        try:
            import hashlib, binascii
            sha = hashlib.sha1(key + _WS_GUID.encode())
            accept = binascii.b2a_base64(sha.digest()).strip()
        except Exception as e:
            self._log("[TELEM] handshake hash error:", e)
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
        return True

    def _drive_pending(self):
        # Avance le handshake en cours en mode non bloquant, avec un budget
        # temps strict pour ne jamais pénaliser la boucle d'acquisition.
        if self._pending is None:
            return
        cli, buf, deadline = self._pending
        slice_end = time.ticks_add(time.ticks_ms(), self._handshake_slice_ms)

        while b"\r\n\r\n" not in buf and len(buf) < 2048:
            # Sortie sur budget temps du slice OU deadline globale du handshake.
            if time.ticks_diff(slice_end, time.ticks_ms()) <= 0:
                self._pending = (cli, buf, deadline)
                return
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                self._log("[TELEM] handshake timeout, got:", buf[:80])
                self._abort_pending()
                return
            try:
                chunk = cli.recv(512)
            except OSError as e:
                err = getattr(e, "errno", None)
                if err in (_EAGAIN, 11, 110, 116):
                    # Rien à lire pour l'instant : on rend la main et on
                    # reprendra au prochain appel send_telemetry().
                    self._pending = (cli, buf, deadline)
                    return
                self._log("[TELEM] handshake recv error:", e)
                self._abort_pending()
                return
            if not chunk:
                # Peer a fermé.
                self._abort_pending()
                return
            buf += chunk

        # Headers complets reçus -> finalisation et bascule en client actif.
        self._pending = None
        if self._finalize_handshake(cli, buf):
            # Un nouveau client WS prend la place : on ferme l'eventuel
            # client WS precedent uniquement ICI, pas plus tot (sinon une
            # requete HTTP banale tuerait la session WS de telemetrie).
            if self._client is not None:
                try:
                    self._client.send(bytes((0x88, 0x00)))  # WS Close
                except OSError:
                    pass
                try:
                    self._client.close()
                except OSError:
                    pass
            self._client = cli
            self._rx_buf = b""
            self._log("[TELEM] WS client connected")
        else:
            try:
                cli.close()
            except OSError:
                pass

    def _abort_pending(self):
        if self._pending is None:
            return
        cli, _, _ = self._pending
        try:
            cli.close()
        except OSError:
            pass
        self._pending = None

    def _accept_if_pending(self):
        if self._srv is None:
            return
        # Si un handshake est deja en cours, on ne touche pas a accept() :
        # on fait juste avancer le handshake. Les nouvelles connexions TCP
        # restent en attente dans la backlog kernel et seront acceptees
        # une fois ce handshake termine. Sinon on aurait des "Failed to
        # fetch" cote JS quand le navigateur ouvre plusieurs connexions
        # en parallele au chargement de la page.
        if self._pending is not None:
            self._drive_pending()
            return
        try:
            cli, addr = self._srv.accept()
        except OSError:
            return

        self._log("[TELEM] incoming client:", addr)
        # On NE ferme PAS le client WS actif ici : tant qu'on ne sait pas
        # si la nouvelle requete est un upgrade WS ou une simple requete
        # HTTP (config, /api/cmd poll, telechargement /api/data), on doit
        # preserver la session WS de telemetrie en cours. Le drop du WS
        # actif est repousse au moment ou un nouveau handshake WS reussit
        # (cf. _drive_pending).

        # Bascule la socket en non-bloquant et démarre la machine à états.
        try:
            cli.setblocking(False)
        except OSError:
            try:
                cli.close()
            except OSError:
                pass
            return
        self._pending = (cli, b"",
                         time.ticks_add(time.ticks_ms(), self._handshake_timeout_ms))
        self._drive_pending()

    def _handle_incoming(self):
        # Lit et parse les frames WebSocket entrantes (toujours masquées
        # côté client d'après la RFC 6455). On répond aux ping (opcode 0x9)
        # par un pong (0xA) et on ferme proprement sur close (0x8).
        # Tout est non bloquant : si le buffer n'est pas complet on rend
        # la main et on reprend au prochain appel.
        if self._client is None:
            return
        # Drain ce qui est dispo en non-bloquant.
        try:
            while True:
                chunk = self._client.recv(256)
                if not chunk:
                    # Peer a fermé proprement (TCP FIN).
                    self._drop_client()
                    return
                self._rx_buf += chunk
                if len(self._rx_buf) > 4096:
                    # Sécurité : un client qui spamme sans qu'on parse.
                    self._log("[TELEM] rx buffer overflow, dropping client")
                    self._drop_client()
                    return
        except OSError as e:
            err = getattr(e, "errno", None)
            if err not in (_EAGAIN, 11, 110, 116):
                self._log("[TELEM] recv error:", e)
                self._drop_client()
                return
            # EAGAIN : rien de plus à lire, on continue avec ce qu'on a.

        # Parse autant de frames complètes que possible.
        while self._client is not None:
            consumed = self._parse_one_frame()
            if consumed <= 0:
                # Pas assez de données pour une frame complète, ou erreur.
                break
            self._rx_buf = self._rx_buf[consumed:]

    def _parse_one_frame(self):
        # Retourne le nombre d'octets consommés, 0 si frame incomplète,
        # -1 si frame invalide (on dropera le client).
        b = self._rx_buf
        n = len(b)
        if n < 2:
            return 0
        fin = (b[0] & 0x80) != 0
        opcode = b[0] & 0x0F
        masked = (b[1] & 0x80) != 0
        plen = b[1] & 0x7F
        if not masked:
            # Le client DOIT masquer ; sinon RFC = fermer.
            self._log("[TELEM] unmasked client frame, dropping")
            self._drop_client()
            return -1

        off = 2
        if plen == 126:
            if n < off + 2:
                return 0
            plen = (b[off] << 8) | b[off + 1]
            off += 2
        elif plen == 127:
            if n < off + 8:
                return 0
            plen = 0
            for i in range(8):
                plen = (plen << 8) | b[off + i]
            off += 8

        if n < off + 4 + plen:
            return 0  # frame incomplète, on attend la suite

        mask = b[off:off + 4]
        off += 4
        raw = b[off:off + plen]
        # Démasque le payload (XOR avec la masking key, cyclique sur 4 octets).
        if plen and mask:
            payload = bytes(raw[i] ^ mask[i & 3] for i in range(plen))
        else:
            payload = b""
        total = off + plen

        # Dispatch par opcode (cf RFC 6455 §5.5).
        if opcode == 0x9:  # PING
            self._send_control(0x8A, payload)  # PONG avec même payload
            self._log("[TELEM] ping->pong", len(payload), "B")
        elif opcode == 0xA:  # PONG (réponse à un ping qu'on aurait envoyé)
            pass
        elif opcode == 0x8:  # CLOSE
            # On renvoie un close en miroir puis on coupe.
            self._send_control(0x88, payload[:2] if len(payload) >= 2 else b"")
            self._log("[TELEM] client requested close")
            self._drop_client()
        elif opcode in (0x0, 0x1, 0x2):
            # Frame data du client : non utilisée par notre protocole, ignorée.
            # On garde la connexion (FIN ou non) pour ne pas casser un client
            # qui enverrait des messages applicatifs sans qu'on les attende.
            if not fin:
                self._log("[TELEM] ignored continuation frame opcode", opcode)
        else:
            # Opcode réservé/inconnu -> ferme la connexion (RFC).
            self._log("[TELEM] unknown opcode", opcode)
            self._drop_client()
            return -1
        return total

    def _send_control(self, opcode_byte, payload):
        # Frame de contrôle server->client (non masquée, payload ≤ 125 B).
        if self._client is None:
            return
        payload = payload[:125]  # RFC : control frames ≤ 125 B
        try:
            self._client.send(bytes((opcode_byte, len(payload))) + payload)
        except OSError:
            self._drop_client()

    def _drop_client(self):
        if self._client is not None:
            try:
                # Tente d'envoyer une frame WS Close (opcode 0x88) avant
                # de fermer le TCP -> coupure propre côté client.
                self._client.send(bytes((0x88, 0x00)))
            except OSError:
                pass
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None
            self._rx_buf = b""

    @staticmethod
    def _ws_header(n):
        # Construit l'en-tête WS server->client pour une frame binaire
        # (FIN=1, opcode=0x2, mask=0), longueur encodée selon la RFC 6455.
        if n < 126:
            return bytes((0x82, n))
        if n < 65536:
            return bytes((0x82, 126, (n >> 8) & 0xFF, n & 0xFF))
        # Très improbable pour de la télémétrie, mais on couvre.
        return bytes((0x82, 127)) + n.to_bytes(8, "big")

    # --- public emission ----------------------------------------------

    def send_telemetry(self, time_ms, pressure_bar, temp_c,
                       ax, ay, az, gx, gy, gz, mx, my, mz, temp_imu_c, temp_mag_c, flags):
        """Émet une trame Nectar (binaire, 57 octets) au client WebSocket.
        À appeler à chaque sample dans la boucle d'acquisition. Silencieux
        si aucun client n'est connecté ou si le rate-limiting saute la frame.
        Ne bloque jamais : draine aussi le handshake WS en cours par tranches."""
        if not self._ok:
            return
        self._accept_if_pending()
        # Traite les frames entrantes (ping/pong/close) avant d'émettre.
        # Non bloquant : se contente de ce qui est déjà dans le buffer TCP.
        self._handle_incoming()
        if self._client is None:
            return

        # Rate limiting : on saute les frames trop rapprochées pour soulager
        # le bilan de liaison (moins de collisions, plus de marge pour les
        # retransmissions TCP entre deux émissions).
        if self.period_ms > 0:
            now = time.ticks_ms()
            if self._last_tx_ms is not None and \
               time.ticks_diff(now, self._last_tx_ms) < self.period_ms:
                return
            self._last_tx_ms = now

        # NOTE : pas de float() explicite. Sur MicroPython, struct.pack("f", x)
        # accepte int et float ; ajouter float() peut au contraire échouer si
        # le capteur renvoie un tuple ou un type custom. On laisse struct gérer.
        try:
            payload = struct.pack(_PAYLOAD_FMT,
                                  int(time_ms) & 0xFFFFFFFF,
                                  pressure_bar, temp_c,
                                  ax, ay, az,
                                  gx, gy, gz,
                                  mx, my, mz,
                                  temp_imu_c,
                                  temp_mag_c,
                                  int(flags) & 0xFF)
            frame = build_frame(self.ssid_type, self.ssid_num, self.apid, payload)
        except (TypeError, ValueError) as e:
            # Capteur invalide (None, tuple, NaN entier...) : on saute cette
            # frame silencieusement pour ne pas spammer la console à 20 Hz,
            # mais on log en debug pour diagnostiquer.
            self._log("[TELEM] encode error:", e,
                      "values=", time_ms, pressure_bar, temp_c,
                      ax, ay, az, gx, gy, gz, temp_imu_c, flags)
            return

        ws_msg = self._ws_header(len(frame)) + frame
        try:
            self._client.send(ws_msg)
        except OSError as e:
            # EAGAIN/EWOULDBLOCK = buffer TCP plein, on garde le client.
            # Toute autre erreur = client mort -> on drop.
            if getattr(e, "errno", None) == _EAGAIN:
                return
            self._log("[TELEM] client disconnected")
            self._drop_client()

    def stop(self):
        """Arrête proprement le serveur WebSocket et désactive l'AP WiFi.
        À appeler au shutdown ou avant un reset. Idempotent."""
        self._abort_pending()
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
