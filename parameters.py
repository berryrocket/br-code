########################################
#### BerryRocket Avionic ####
# Parameter file
# Louis Barbier
# Licence CC-BY-NC-SA
########################################

#####
# User parameters
#####

### Mother board selection
# Board available:
#   - BR_MINI_AVIONIC
#   - BR_MICRO_AVIONIC
MOTHER_BOARD = "BR_MINI_AVIONIC"

### Sensor board selection (if BR_MINI_AVIONIC selected)
# Board available:
#   - NONE (no sensor board installed)
#   - 10DOF_V1
#   - 10DOF_V2.1
#   - BR_MINI_SENSOR
SENSOR_BOARD    = "NONE"

### Ejection charge
# activation de la version avec EJECTION_CHARGE (sans trappe parachute)
EJECTION_CHARGE = False 

### IMU lift-off detection
# activation de l'information d'accélération par l'IMU sinon par le contacteur mécanique
LIFTOFF_DET_IMU     = True
LIFTOFF_DET_CONTACT = True
# seuil de l'accélération pour détecter le décollage [g] (si LIFTOFF_DET_IMU=True)
LIFTOFF_IMU_THRESHOLD  = 1

### Falling timeout (after lift-off)
# temps après lequel la fusée passe en mode chute libre [ms]
TIMEOUT_APOGEE = 7200

### Réglage trappe parachute  
# (à régler en fonction de l'orientation du servomoteur si cela ne marche pas avec les valeurs par défaut)
SERVO_OPEN      = 800   # [us] (position ouverture trappe du servomoteur)
SERVO_CLOSE     = 1800  # [us] (position fermeture trappe du servomoteur)

### Buzzer activation
BUZZER_ENABLE   = True

### Télémétrie Nectar (WiFi AP + WebSocket serveur)
# Trames Nectar (https://github.com/mlavardin/NectarMC) émises en messages
# WebSocket binaires. NectarMC se connecte en client à ws://192.168.4.1:<port>/
TELEMETRY_ENABLE     = True
TELEMETRY_SSID_NUM   = None   # 0..255 (si None, ce sera l'id unique de la carte PicoW)

#####
# System parameters (be careful when you modify these parameters)
#####

SOFT_VERSION                = "2.0"
DEBUG                       = False
FREQ_ACQ                    = 20  # Frequence d'acquisition des données [Hz]
TELEMETRY_RATE_HZ           = 5   # Fréquence d'émission radio [Hz]
TELEMETRY_AP_SSID_PREFIX    = "BerryRocket"  # préfixe SSID
TELEMETRY_AP_OPEN           = True           # True = réseau ouvert (pas de mot de passe)
TELEMETRY_AP_PSK            = "berryrocket"  # ignoré si TELEMETRY_AP_OPEN=True ; sinon WPA2 (min 8 char)
TELEMETRY_AP_CHANNEL        = 6
TELEMETRY_WS_PORT           = 80             # NectarMC se connecte à ws://192.168.4.1:80/
TELEMETRY_SSID_TYPE         = 1   # 0=FX, 1=MF, 2=BALLOON, 3=OTHER
TELEMETRY_APID              = 0   # 0..63
