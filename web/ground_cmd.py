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
#  - le decollage (lock_after_liftoff) verrouille tout : impossible d'armer,
#    armement existant force a False -> aucune commande sol acceptee en vol.

from buzzer import set_buzzer

# Tons buzzer (coherents avec main.py : 1000 Hz/2 s = pre-decollage).
_TONE_PRELAUNCH_FREQ = 1000
_TONE_PRELAUNCH_TPS  = 2
_TONE_ARMED_FREQ     = 2500
_TONE_ARMED_TPS      = 0.8

# Etat (mute par main.py et web.py, mais main loop et web handler sont
# synchrones sur le meme thread -> pas de race).
_armed         = False
_in_flight     = False
_open_fn       = None        # callable() -> ouvre la trappe
_close_fn      = None        # callable() -> ferme la trappe
_reset_data_fn = None        # callable() -> vide + reinitialise le fichier data
_buzzer_on     = True
_data_path     = "data/data_platform.txt"

def setup(open_fn, close_fn, buzzer_enabled, data_path=None, reset_data_fn=None):
    """Appele par main.py au boot. open_fn / close_fn sont les fonctions
    de pilotage de la trappe (typiquement open_parachute / close_parachute
    de main.py) : on garde une source de verite unique pour l'actionnement
    physique, et le cas EJECTION_CHARGE est gere une seule fois cote main.
    reset_data_fn (typiquement reset_data_file de main) vide le fichier data
    et reecrit son entete ; main reste seul proprietaire du handle fichier
    (ouvert en permanence), donc la suppression passe par ce callback plutot
    que par un os.remove ici.
    Passer None pour les callbacks si l'avionique n'a pas la fonctionnalite
    correspondante."""
    global _open_fn, _close_fn, _reset_data_fn, _buzzer_on, _data_path
    _open_fn       = open_fn
    _close_fn      = close_fn
    _reset_data_fn = reset_data_fn
    _buzzer_on     = buzzer_enabled
    if data_path is not None:
        _data_path = data_path

def has_servo():
    """True si l'avionique a un servomoteur de trappe pilotable."""
    return _open_fn is not None and _close_fn is not None

def is_armed():
    """True si l'avionique est en mode sol (commandes acceptees)."""
    return _armed

def is_in_flight():
    """True des que le decollage a ete detecte (verrouille les commandes sol)."""
    return _in_flight

def can_arm():
    """Armement possible UNIQUEMENT au sol, avant decollage."""
    return (not _in_flight) and has_servo()

def data_path():
    """Chemin du fichier de donnees a telecharger / supprimer."""
    return _data_path

def lock_after_liftoff():
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
    set_buzzer(_buzzer_on, freq=_TONE_ARMED_FREQ, tps=_TONE_ARMED_TPS)
    return True

def disarm():
    """Desactive le mode sol et restaure le ton pre-decollage."""
    global _armed
    _armed = False
    # Restaure le buzzer pre-decollage (ou silence si on est en vol et que
    # main.py reprendra la main avec ses propres tons launched/falling).
    if not _in_flight:
        set_buzzer(_buzzer_on, freq=_TONE_PRELAUNCH_FREQ, tps=_TONE_PRELAUNCH_TPS)

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

def delete_data():
    """Vide le fichier de donnees et le re-initialise (entete fraiche) via le
    callback reset_data_fn de main (qui possede le handle ouvert).
    Refuse si non arme ou en vol -> impossible d'effacer accidentellement
    en cours d'experience, et impossible de perdre les donnees du vol."""
    if _in_flight or not _armed or _reset_data_fn is None:
        return False
    try:
        _reset_data_fn()
    except OSError:
        return False
    return True
