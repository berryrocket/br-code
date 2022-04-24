from machine import Pin,PWM,Timer
import time

# Declaration d'un PWM pour le buzzer
buzzer = PWM(Pin(18))

# Declaration timer pour le buzzer et son extinction
timerBuzzer = Timer()
timerOffBuzzer = Timer()

# Variables pour le buzzer
freqBuzzer  = 500
tempsBuzzer = 0

# Initialisation musique
def InitMusic():
    global buzzer
    notes = (146.83,164.81,174.61,164.81,130.81,146.83,164.81,174.61,164.81,196.00,155.56,174.61,196.00,155.56,146.83,138.59,164.81)
    notes = tuple(x*2 for x in notes) # Augmente d'une octave la musique
    tempsNotes = (1.5,0.5,0.5,0.5,1,1.5,0.5,0.5,0.5,1,1.5,0.5,1,1,1,2,2)
    bpm = 75
    buzzer.duty_u16(0) # Set to 0%
    buzzer.duty_u16(32768) # Set to 50%
    for iNote in range(0, len(notes)):
        buzzer.freq(round(notes[iNote]))
        time.sleep((60.0/bpm)*tempsNotes[iNote])

    buzzer.duty_u16(0) # Set to 0%

# Management buzzer
def MgtBuzzer(timer):
    global freqBuzzer
    global buzzer
    buzzer.freq(freqBuzzer)
    buzzer.duty_u16(32768) # Set to 50%
    timerOffBuzzer.init(freq=1.0/0.1, mode=Timer.ONE_SHOT, callback=SetOffBuzzer) # Ring the buzzer for this time (0.1s)

def SetOffBuzzer(timer):
    buzzer.duty_u16(0) # Set to 0%

# Mets en marche le buzzer pour la fréquence et la période indiquées
def SetBuzzer(enable=True, freq=500, tps=5):
    """Set the buzzer with a frequency and a time between ring"""
    global freqBuzzer
    global timerBuzzer
    freqBuzzer = freq
    timerBuzzer.deinit()
    if enable == True:
        timerBuzzer.init(freq=1.0/tps, mode=Timer.PERIODIC, callback=MgtBuzzer)
