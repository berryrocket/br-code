########################################
#### BerryRocket ####
# Mini routeur HTTP servi sur le meme port que le WebSocket telemetrie.
# Sert la page de config/visualisation (www/index.html) et l'API JSON
# qui lit/ecrit l'overlay config.json.
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# Pas d'auth : l'AP WiFi de la fusee est ephemere et l'acces physique
# (etre a portee radio au sol) est l'unique controle. Suffisant pour
# du materiel pedagogique trimballe en classe.

import json

from web import config_store
from web import ground_cmd


# ---- Validation par champ -----------------------------------------------
# Chaque entree : (type_python, validateur_optionnel).
# Le validateur recoit la valeur deja castee et retourne True si OK.

def _in_range(lo, hi):
    return lambda v: lo <= v <= hi

def _one_of(*choices):
    return lambda v: v in choices

_SCHEMA = {
    "MOTHER_BOARD":          (str,   _one_of("BR_MINI_AVIONIC", "BR_MICRO_AVIONIC")),
    "SENSOR_BOARD":          (str,   _one_of("NONE", "10DOF_V1", "10DOF_V2.1", "BR_MINI_SENSOR")),
    "EJECTION_CHARGE":       (bool,  None),
    "LIFTOFF_DET_IMU":       (bool,  None),
    "LIFTOFF_DET_CONTACT":   (bool,  None),
    "LIFTOFF_IMU_THRESHOLD": (float, _in_range(1.0, 10.0)),
    "TIMEOUT_APOGEE":        (int,   _in_range(1000, 30000)),
    "SERVO_OPEN":            (int,   _in_range(500, 2500)),
    "SERVO_CLOSE":           (int,   _in_range(500, 2500)),
    "BUZZER_ENABLE":         (bool,  None),
    "TELEMETRY_ENABLE":      (bool,  None),
    # TELEMETRY_SSID_NUM peut etre null -> traite separement.
    "TELEMETRY_SSID_NUM":    (int,   _in_range(0, 255)),
}


def _validate(d):
    """Valide et caste. Retourne (clean_dict, error_str_ou_None)."""
    clean = {}
    for k, v in d.items():
        if k not in _SCHEMA:
            continue  # ignore silencieusement les cles inconnues
        if k == "TELEMETRY_SSID_NUM" and v is None:
            clean[k] = None
            continue
        typ, check = _SCHEMA[k]
        try:
            if typ is bool:
                if not isinstance(v, bool):
                    return None, "type invalide pour {}".format(k)
                cast = v
            elif typ is float:
                cast = float(v)
            elif typ is int:
                # On refuse les bool implicitement convertis (True == 1).
                if isinstance(v, bool):
                    return None, "type invalide pour {}".format(k)
                cast = int(v)
            else:  # str
                if not isinstance(v, str):
                    return None, "type invalide pour {}".format(k)
                cast = v
        except (TypeError, ValueError):
            return None, "type invalide pour {}".format(k)
        if check is not None and not check(cast):
            return None, "valeur hors bornes pour {}".format(k)
        clean[k] = cast
    return clean, None


# ---- Snapshot de la config courante -------------------------------------

def _current_config():
    """Lit les valeurs effectives depuis parameters.py (overlay deja
    applique au boot) + quelques metas pour l'UI."""
    import parameters as P
    snap = {}
    for k in config_store.ALLOWED_KEYS:
        snap[k] = getattr(P, k, None)
    meta = {
        "soft_version": getattr(P, "SOFT_VERSION", "?"),
        "freq_acq":     getattr(P, "FREQ_ACQ", None),
        "rate_hz":      getattr(P, "TELEMETRY_RATE_HZ", None),
        "ssid_prefix":  getattr(P, "TELEMETRY_AP_SSID_PREFIX", ""),
    }
    return {"values": snap, "meta": meta}


# ---- HTTP helpers -------------------------------------------------------

def _sendall(cli, data):
    """Envoi complet de la reponse HTTP. Le service HTTP n'a lieu QU'AU SOL :
    en vol la telemetrie ferme toute requete HTTP (cf. TelemetryWS.mark_launched),
    donc on met la socket en mode bloquant et on envoie simplement. Un eventuel
    blocage ici (client lent, page ~10 Ko) n'impacte jamais la boucle de vol."""
    try:
        cli.setblocking(True)
    except (OSError, AttributeError):
        pass
    view = memoryview(data)
    n = len(view)
    off = 0
    while off < n:
        try:
            sent = cli.send(view[off:off + 1024])
        except OSError:
            return False
        if not sent:   # 0 ou None : connexion fermee/anormale
            return False
        off += sent
    return True


def _send_response(cli, code, ctype, body, extra_headers=b""):
    status = {
        200: b"200 OK",
        302: b"302 Found",
        400: b"400 Bad Request",
        404: b"404 Not Found",
        405: b"405 Method Not Allowed",
        409: b"409 Conflict",
        413: b"413 Payload Too Large",
        500: b"500 Internal Server Error",
    }.get(code, b"500 Internal Server Error")
    if isinstance(body, str):
        body = body.encode("utf-8")
    headers = (
        b"HTTP/1.1 " + status + b"\r\n"
        b"Content-Type: " + ctype + b"\r\n"
        b"Content-Length: " + str(len(body)).encode() + b"\r\n"
        b"Connection: close\r\n"
        b"Cache-Control: no-store\r\n"
        + extra_headers +
        b"\r\n"
    )
    if not _sendall(cli, headers):
        return
    _sendall(cli, body)


def _send_json(cli, code, obj):
    _send_response(cli, code, b"application/json; charset=utf-8",
                   json.dumps(obj))


def _send_file(cli, code, ctype, path):
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError:
        _send_response(cli, 404, b"text/plain; charset=utf-8", "introuvable: " + path)
        return
    _send_response(cli, code, ctype, data)


def _send_file_download(cli, path, filename):
    """Stream un fichier en piece jointe (Content-Disposition: attachment).
    Lecture par chunks pour ne pas exploser la RAM sur les gros logs."""
    import os
    try:
        size = os.stat(path)[6]
    except OSError:
        _send_response(cli, 404, b"text/plain; charset=utf-8", "fichier introuvable")
        return
    headers = (
        b"HTTP/1.1 200 OK\r\n"
        b"Content-Type: text/plain; charset=utf-8\r\n"
        b"Content-Length: " + str(size).encode() + b"\r\n"
        b"Content-Disposition: attachment; filename=\"" + filename.encode() + b"\"\r\n"
        b"Connection: close\r\n"
        b"Cache-Control: no-store\r\n\r\n"
    )
    if not _sendall(cli, headers):
        return
    try:
        with open(path, "rb") as f:
            while True:
                chunk = f.read(1024)
                if not chunk:
                    break
                if not _sendall(cli, chunk):
                    return
    except OSError:
        return


# ---- Lecture du body POST -----------------------------------------------

def _read_body(cli, already, content_length):
    """Retourne (bytes, error_or_None). Le corps complet a deja ete bufferise
    en amont par la couche telemetrie (lecture non bloquante par tranches) :
    on se contente de ce qui est la, sans JAMAIS bloquer la boucle d'acquisition.
    `already` = octets recus apres les en-tetes (= le corps complet en pratique)."""
    if content_length > 4096:
        return None, "body trop gros"
    if len(already) >= content_length:
        return already[:content_length], None
    # Ne devrait pas arriver : la telemetrie lit tout le corps avant de
    # dispatcher. Si le corps est incomplet (timeout cote telemetrie), on
    # refuse proprement plutot que de bloquer pour lire le reste.
    return None, "body incomplet"


# ---- Routage ------------------------------------------------------------

def handle(cli, raw):
    """Traite une requete HTTP complete (headers + eventuel debut de body)
    deja lue par telemetry.py jusqu'au CRLF CRLF. Repond et ferme la socket.
    Toujours retourne True (socket a fermer cote appelant)."""
    try:
        _route(cli, raw)
    finally:
        try:
            cli.close()
        except OSError:
            pass
    return True


def _route(cli, raw):
    # Routes servies :
    #   GET  /                      -> page web (www/index.html)
    #   GET  /api/config            -> snapshot config courante (JSON)
    #   POST /api/config            -> ecrit l'overlay config.json
    #   POST /api/config/reset      -> efface l'overlay (valeurs usine)
    #   GET  /api/cmd/status        -> etat armement/vol
    #   POST /api/cmd/arm           -> arme l'avionique (sol uniquement)
    #   POST /api/cmd/disarm        -> desarme
    #   POST /api/cmd/trap          -> ouvre/ferme la trappe (si arme)
    #   GET  /api/data              -> telecharge data_platform.txt
    #   POST /api/data/delete       -> supprime et reinitialise le fichier data
    #   *                           -> redirige vers / (captive portal)

    # Split headers / debut de body.
    sep = raw.find(b"\r\n\r\n")
    if sep < 0:
        _send_response(cli, 400, b"text/plain; charset=utf-8", "requete incomplete")
        return
    head = raw[:sep]
    body_start = raw[sep + 4:]

    lines = head.split(b"\r\n")
    if not lines:
        _send_response(cli, 400, b"text/plain; charset=utf-8", "requete vide")
        return
    parts = lines[0].split(b" ")
    if len(parts) < 2:
        _send_response(cli, 400, b"text/plain; charset=utf-8", "requete malformee")
        return
    method, path = parts[0], parts[1]

    # Parse Content-Length pour les POST.
    content_length = 0
    for line in lines[1:]:
        if b":" not in line:
            continue
        name, _, value = line.partition(b":")
        if name.strip().lower() == b"content-length":
            try:
                content_length = int(value.strip())
            except ValueError:
                content_length = 0
            break

    if method == b"GET" and path == b"/":
        _send_file(cli, 200, b"text/html; charset=utf-8", "web/www/index.html")
        return

    if method == b"GET" and path == b"/api/config":
        _send_json(cli, 200, _current_config())
        return

    if method == b"POST" and path == b"/api/config":
        body, err = _read_body(cli, body_start, content_length)
        if err is not None:
            _send_json(cli, 400, {"ok": False, "error": err})
            return
        try:
            payload = json.loads(body)
        except (ValueError, TypeError):
            _send_json(cli, 400, {"ok": False, "error": "JSON invalide"})
            return
        if not isinstance(payload, dict):
            _send_json(cli, 400, {"ok": False, "error": "objet JSON attendu"})
            return
        clean, verr = _validate(payload)
        if verr is not None:
            _send_json(cli, 400, {"ok": False, "error": verr})
            return
        # Merge avec l'overlay actuel : on permet une sauvegarde partielle
        # sans ecraser les cles non envoyees.
        merged = config_store.load()
        merged.update(clean)
        try:
            config_store.save(merged)
        except OSError as e:
            _send_json(cli, 500, {"ok": False, "error": "ecriture FS: {}".format(e)})
            return
        _send_json(cli, 200, {"ok": True, "saved": clean,
                              "note": "redemarrer l'avionique pour appliquer"})
        return

    if method == b"POST" and path == b"/api/config/reset":
        try:
            config_store.reset()
        except OSError as e:
            _send_json(cli, 500, {"ok": False, "error": str(e)})
            return
        _send_json(cli, 200, {"ok": True,
                              "note": "valeurs usine au prochain boot"})
        return

    # ---- Commandes sol --------------------------------------------------
    if method == b"GET" and path == b"/api/cmd/status":
        _send_json(cli, 200, {
            "armed":     ground_cmd.is_armed(),
            "in_flight": ground_cmd.is_in_flight(),
            "can_arm":   ground_cmd.can_arm(),
            "has_servo": ground_cmd.has_servo(),
        })
        return

    if method == b"POST" and path == b"/api/cmd/arm":
        ok = ground_cmd.arm()
        _send_json(cli, 200 if ok else 409, {
            "ok": ok, "armed": ground_cmd.is_armed(),
            "error": None if ok else "armement refusé (en vol ou pas de servo)",
        })
        return

    if method == b"POST" and path == b"/api/cmd/disarm":
        ground_cmd.disarm()
        _send_json(cli, 200, {"ok": True, "armed": ground_cmd.is_armed()})
        return

    if method == b"POST" and path == b"/api/cmd/trap":
        body, err = _read_body(cli, body_start, content_length)
        if err is not None:
            _send_json(cli, 400, {"ok": False, "error": err})
            return
        try:
            payload = json.loads(body)
        except (ValueError, TypeError):
            _send_json(cli, 400, {"ok": False, "error": "JSON invalide"})
            return
        action = payload.get("action") if isinstance(payload, dict) else None
        if action == "open":
            ok = ground_cmd.open_trap()
        elif action == "close":
            ok = ground_cmd.close_trap()
        else:
            _send_json(cli, 400, {"ok": False,
                                  "error": "action attendue: open|close"})
            return
        _send_json(cli, 200 if ok else 409, {
            "ok": ok, "action": action,
            "error": None if ok else "commande refusée (non armé ou en vol)",
        })
        return

    if method == b"GET" and path == b"/api/data":
        _send_file_download(cli, ground_cmd.data_path(), "data_platform.txt")
        return

    if method == b"POST" and path == b"/api/data/delete":
        ok = ground_cmd.delete_data()
        _send_json(cli, 200 if ok else 409, {
            "ok": ok,
            "error": None if ok else "suppression refusée (non armé, en vol, ou erreur FS)",
        })
        return

    # ---- Captive portal -------------------------------------------------
    # Toute requete inconnue (typiquement une sonde OS qui a ete redirigee
    # vers nous par le hijack DNS) est redirigee vers /. C'est ce qui fait
    # apparaitre automatiquement la page web au moment de la connexion wifi.
    if method == b"GET":
        _send_response(cli, 302, b"text/plain; charset=utf-8",
                       "captive portal",
                       extra_headers=b"Location: http://192.168.4.1/\r\n")
        return

    _send_response(cli, 404, b"text/plain; charset=utf-8", "not found")
