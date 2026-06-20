########################################
#### BerryRocket ####
# Overlay de configuration persiste sur le FS (config.json)
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# parameters.py reste la version "usine" jamais reecrite. Si config.json
# existe, ses cles surchargent les constantes correspondantes au boot.
# Pour revenir aux valeurs usine : supprimer config.json (bouton "Reset"
# de la page web ou suppression manuelle dans Thonny).

import json

_CONFIG_PATH = "config.json"
_TMP_PATH = "config.json.tmp"

# Cles autorisees dans l'overlay. Toute autre cle est silencieusement
# ignoree, que ce soit en lecture (config.json bidouille a la main) ou
# en ecriture (POST web).
ALLOWED_KEYS = (
    "MOTHER_BOARD",
    "SENSOR_BOARD",
    "EJECTION_CHARGE",
    "LIFTOFF_DET_IMU",
    "LIFTOFF_DET_CONTACT",
    "LIFTOFF_IMU_THRESHOLD",
    "TIMEOUT_APOGEE",
    "SERVO_OPEN",
    "SERVO_CLOSE",
    "BUZZER_ENABLE",
    "TELEMETRY_ENABLE",
    "TELEMETRY_SSID_NUM",
)

def _filter(d):
    if not isinstance(d, dict):
        return {}
    return {k: d[k] for k in d if k in ALLOWED_KEYS}

def load():
    """Retourne le dict d'overlay, {} si fichier absent ou JSON invalide.
    Resilient : un fichier corrompu ne casse pas le boot."""
    try:
        with open(_CONFIG_PATH, "r") as f:
            raw = json.load(f)
    except (OSError, ValueError):
        return {}
    return _filter(raw)

def save(d):
    """Ecriture atomique : ecrit dans .tmp puis rename. Une coupure
    d'alim pendant le write laisse config.json intact."""
    import os
    clean = _filter(d)
    with open(_TMP_PATH, "w") as f:
        json.dump(clean, f)
    # os.rename remplace atomiquement sur LittleFS.
    try:
        os.remove(_CONFIG_PATH)
    except OSError:
        pass
    os.rename(_TMP_PATH, _CONFIG_PATH)
    return clean

def reset():
    """Supprime l'overlay -> retour aux defauts au prochain boot."""
    import os
    for path in (_CONFIG_PATH, _TMP_PATH):
        try:
            os.remove(path)
        except OSError:
            pass
