########################################
#### BerryRocket ####
# Mini serveur DNS menteur : repond a TOUTES les requetes A avec l'IP
# de l'AP. Permet aux sondes captive-portal des OS d'aboutir chez nous,
# l'OS bascule en "captive portal detected" et ouvre automatiquement
# notre page web a la connexion au wifi.
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# Non-bloquant : poll() est appelee depuis main.py a 20 Hz, lit ce qui
# trainait sur le socket UDP 53 et repond. Charge negligeable.
#
# /!\ Cas du PC multi-homed (connecte simultanement a un vrai wifi ET
# a l'AP BerryRocket) : tant que la fenetre BerryRocket est ouverte,
# les resolutions DNS qui passent par notre interface seront menties.
# Windows et macOS gerent ca avec un resolveur multi-interface qui
# privilegie l'interface avec internet ; en pratique l'impact sur la
# navigation reste limite.

import socket

try:
    import errno
    _EAGAIN = errno.EAGAIN
except (ImportError, AttributeError):
    _EAGAIN = 11


class CaptiveDNS:
    def __init__(self, ip, debug=False):
        # ip = adresse a renvoyer pour les sondes captives uniquement
        # (typiquement l'IP de l'AP, 192.168.4.1).
        parts = [int(x) for x in ip.split(".")]
        if len(parts) != 4 or any(p < 0 or p > 255 for p in parts):
            raise ValueError("[CDNS] IP invalide: " + ip)
        self._ip_bytes = bytes(parts)
        self._sock = None
        self._ok = False
        self.debug = debug

    def _log(self, *args):
        if self.debug:
            print(*args)

    def start(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", 53))
            s.setblocking(False)
            self._sock = s
            self._ok = True
            if self.debug:
                print("[CDNS] DNS captif actif sur :53 ->",
                      ".".join(str(b) for b in self._ip_bytes),
                      "(sondes captives uniquement)")
            return True
        except OSError as e:
            print("[CDNS] start failed:", e)
            self._ok = False
            return False

    def poll(self):
        """Appelee depuis la boucle principale. Lit autant de requetes
        DNS qu'il y en a dans le buffer, repond, puis rend la main."""
        if not self._ok:
            return
        # Drain : si plusieurs sondes captives arrivent en meme temps,
        # on les traite toutes pour eviter une accumulation au noyau.
        for _ in range(8):  # max 8 requetes / poll
            try:
                data, addr = self._sock.recvfrom(512)
            except OSError as e:
                err = getattr(e, "errno", None)
                if err in (_EAGAIN, 11, 110, 116):
                    return  # rien a lire
                self._log("[CDNS] recvfrom error:", e)
                return
            if not data or len(data) < 12:
                continue
            try:
                resp = self._build_response(data)
            except Exception as e:
                self._log("[CDNS] build error:", e)
                continue
            if resp is None:
                continue
            try:
                self._sock.sendto(resp, addr)
            except OSError as e:
                self._log("[CDNS] sendto error:", e)

    def stop(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._ok = False

    # ----- Construction de la reponse ---------------------------------

    def _build_response(self, query):
        # Header DNS : 12 octets.
        # Layout : ID(2) FLAGS(2) QDCOUNT(2) ANCOUNT(2) NSCOUNT(2) ARCOUNT(2)
        if len(query) < 12:
            return None
        tid = query[0:2]
        flags_in = (query[2] << 8) | query[3]
        # On ne traite que les requetes standard (opcode=0, QR=0).
        if (flags_in & 0x8000) != 0:  # QR=1 -> deja une reponse, ignore
            return None
        qdcount = (query[4] << 8) | query[5]
        if qdcount < 1:
            return None

        # Parse la section Question : on a juste besoin de la fin du
        # QNAME pour situer QTYPE / QCLASS et copier la section dans la
        # reponse. Le nom precis ne nous interesse pas : on ment partout.
        off = 12
        while off < len(query):
            label_len = query[off]
            if label_len == 0:
                off += 1
                break
            if (label_len & 0xC0) != 0:
                # Compression dans une Question : tres rare, on ignore.
                return None
            off += 1 + label_len
            if off > len(query):
                return None
        if off + 4 > len(query):
            return None
        qtype  = (query[off] << 8) | query[off + 1]
        qclass = (query[off + 2] << 8) | query[off + 3]
        question_end = off + 4

        # Flags de reponse : QR=1, AA=1, RD copie de la requete, RA=1, RCODE=0
        rd = (query[2] & 0x01)
        flags_out = 0x8400 | (rd << 8) | 0x0080

        # Type non-A (AAAA, TXT, SRV...) -> NOERROR sans Answer. On ne
        # ment que sur les IPv4 ; pour les IPv6 on pousse l'OS a basculer
        # en IPv4 (et il refera une requete A qu'on satisfera).
        if qtype != 1 or qclass != 1:
            header = (
                tid +
                bytes((flags_out >> 8, flags_out & 0xFF)) +
                bytes((0, 1, 0, 0, 0, 0, 0, 0))  # QD=1, AN=0
            )
            return header + query[12:question_end]

        # Type A IN -> Answer avec notre IP. Pointer 0xC00C reference
        # le debut du QNAME dans la section Question (offset 12).
        header = (
            tid +
            bytes((flags_out >> 8, flags_out & 0xFF)) +
            bytes((0, 1, 0, 1, 0, 0, 0, 0))  # QD=1, AN=1
        )
        answer = (
            b"\xC0\x0C"                 # NAME pointer
            b"\x00\x01"                 # TYPE A
            b"\x00\x01"                 # CLASS IN
            b"\x00\x00\x00\x3C"         # TTL = 60 s
            b"\x00\x04"                 # RDLENGTH = 4
            + self._ip_bytes            # RDATA
        )
        return header + query[12:question_end] + answer
