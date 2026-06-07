########################################
#### BerryRocket ####
# Buzzer management
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# Pilote un piezo en PWM. Deux usages :
#  - `set_buzzer(...)` : bip périodique (utilisé pour signaler les phases
#    de vol depuis main.py)
#  - `play_startup_melody(...)` : petite mélodie de démarrage (optionnelle)

import time
from machine import Pin, PWM, Timer
import parameters as PARAMS


#############################
####  Hardware setup     ####
#############################
if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
    _buzzer_pwm = PWM(Pin(18))
elif PARAMS.MOTHER_BOARD == "BR_MICRO_AVIONIC":
    _buzzer_pwm = PWM(Pin(0))
else:
    raise ValueError(
        f"[BUZZER] MOTHER_BOARD inconnu: {PARAMS.MOTHER_BOARD!r}. "
        "Valeurs attendues : BR_MINI_AVIONIC ou BR_MICRO_AVIONIC."
    )

# Deux timers : un pour déclencher les bips périodiques, un pour les couper.
_beep_timer    = Timer()
_silence_timer = Timer()

# Fréquence courante du bip (lue par _beep_on).
_beep_freq_hz = 500


#############################
####  Mélodie démarrage  ####
#############################
def play_startup_melody(enable=True):
    """Joue une petite mélodie au démarrage (bloquante)."""
    notes = (146.83, 164.81, 174.61, 164.81, 130.81,
             146.83, 164.81, 174.61, 164.81, 196.00,
             155.56, 174.61, 196.00, 155.56, 146.83,
             138.59, 164.81)
    notes = tuple(x * 2 for x in notes)  # une octave plus haut
    durations = (1.5, 0.5, 0.5, 0.5, 1,
                 1.5, 0.5, 0.5, 0.5, 1,
                 1.5, 0.5, 1,   1,   1,
                 2,   2)
    bpm = 75

    if enable:
        _buzzer_pwm.duty_u16(0)
        _buzzer_pwm.duty_u16(32768)  # 50 %
        for i in range(len(notes)):
            _buzzer_pwm.freq(round(notes[i]))
            time.sleep((60.0 / bpm) * durations[i])

    _buzzer_pwm.duty_u16(0)


#############################
####  Bip périodique     ####
#############################
def _beep_on(timer):
    """Callback du _beep_timer : déclenche un bip court (0.1 s)."""
    _buzzer_pwm.freq(_beep_freq_hz)
    _buzzer_pwm.duty_u16(32768)  # 50 %
    _silence_timer.init(freq=1.0 / 0.1, mode=Timer.ONE_SHOT, callback=_beep_off)


def _beep_off(timer):
    """Callback du _silence_timer : coupe le bip."""
    _buzzer_pwm.duty_u16(0)


def set_buzzer(enable=True, freq=500, tps=5.0):
    """Active le buzzer avec un bip de fréquence `freq` répété toutes les `tps` secondes.
    Si `enable` est False, coupe le buzzer."""
    global _beep_freq_hz
    _beep_freq_hz = freq
    _beep_timer.deinit()
    if enable:
        _beep_timer.init(freq=1.0 / tps, mode=Timer.PERIODIC, callback=_beep_on)
