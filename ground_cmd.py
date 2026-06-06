########################################
#### BerryRocket ####
# Commandes sol (ouverture/fermeture trappe, telechargement donnees).
# Verrouillage par armement explicite + signal buzzer dedie.
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# Principes de securite :
#  - les commandes (open/close trappe) ne sont acceptees QUE si l'avionique
#    est explicitement "armee" depuis la page web ;
#  - tant qu'elle est armee, le buzzer emet un signal aigu rapide
#    (2500 Hz toutes les 0.8 s) tres different du son pre-decollage,
#    pour signaler au public que la fusee est en mode sol "dangereux" ;
#  - le decollage (mark_launched) verrouille tout : impossible d'armer,
#    armement existant force a False -> aucune commande sol acceptee en vol.

from buzzer import SetBuzzer

# Tons buzzer (cohrents avec main.py : 1000 Hz/2 s = pre-decollage).
_TONE_PRELAUNCH_FREQ = 1000
_TONE_PRELAUNCH_TPS  = 2
_TONE_ARMED_FREQ     = 2500
_TONE_ARMED_TPS      = 0.8

# Etat (mute par main.py et web.py, mais main loop et web handler sont
# synchrones sur le meme thread -> pas de race).
_armed       = False
_in_flight   = False
_open_fn     = None        # callable() -> ouvre la trappe
_close_fn    = None        # callable() -> ferme la trappe
_buzzer_on   = True
_data_path   = "data/data_platform.txt"


def init(open_fn, close_fn, buzzer_enabled, data_path=None):
    """Appele par main.py au boot. open_fn / close_fn sont les fonctions
    de pilotage de la trappe (typiquement OpenParachute / CloseParachute
    de main.py) : on garde une source de verite unique pour l'actionnement
    physique, et le cas EJECTION_CHARGE est gere une seule fois cote main.
    Passer None pour les deux callbacks si la carte n'a pas de servo."""
    global _open_fn, _close_fn, _buzzer_on, _data_path
    _open_fn   = open_fn
    _close_fn  = close_fn
    _buzzer_on = buzzer_enabled
    if data_path is not None:
        _data_path = data_path


def has_servo():
    return _open_fn is not None and _close_fn is not None


def is_armed():
    return _armed


def is_in_flight():
    return _in_flight


def can_arm():
    """Armement possible UNIQUEMENT au sol, avant decollage."""
    return (not _in_flight) and has_servo()


def data_path():
    return _data_path


def mark_launched():
    """Appele par main.py des que le decollage est detecte. Verrouille
    tout : disarme si necessaire et bloque tout futur armement."""
    global _in_flight
    _in_flight = True
    if _armed:
        disarm()


def arm():
    """Active le mode sol. Retourne True si OK."""
    global _armed
    if not can_arm():
        return False
    _armed = True
    # Signal sonore "attention mode sol" (bip aigu rapide).
    SetBuzzer(_buzzer_on, freq=_TONE_ARMED_FREQ, tps=_TONE_ARMED_TPS)
    return True


def disarm():
    """Desactive le mode sol et restaure le ton pre-decollage."""
    global _armed
    _armed = False
    # Restaure le buzzer pre-decollage (ou silence si on est en vol et que
    # main.py reprendra la main avec ses propres tons launched/falling).
    if not _in_flight:
        SetBuzzer(_buzzer_on, freq=_TONE_PRELAUNCH_FREQ, tps=_TONE_PRELAUNCH_TPS)


def open_trap():
    """Ouvre la trappe parachute. Refuse si non arme ou en vol."""
    if _in_flight or not _armed or _open_fn is None:
        return False
    _open_fn()
    return True


def close_trap():
    """Ferme la trappe parachute. Refuse si non arme ou en vol."""
    if _in_flight or not _armed or _close_fn is None:
        return False
    _close_fn()
    return True
