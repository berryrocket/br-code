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
#   - BR_AVIONIC
#   - BR_MICRO_AVIONIC
MOTHER_BOARD = "BR_INTERFACE"

### Sensor board selection (if BR_AVIONIC selected)
# Board available:
#   - NONE (no sensor board installed)
#   - 10DOF_V1
#   - 10DOF_V2.1
#   - BR_SENSOR
SENSOR_BOARD    = "10DOF_V2.1"

### Ejection charge
# activation de la version avec depotage (sans trappe parachute)
DEPOTAGE        = True 

### IMU lift-off detection
# activation de l'information d'accélération par l'IMU sinon par le contacteur mécanique
ACC_IMU         = True
# seuil de l'accélération pour détecter le décollage [g] (si ACC_IMU=True)
ACC_THESHOLD    = 1

### Falling timeout (after lift-off)
# temps après lequel la fusée passe en mode chute libre [ms]
TIMEOUT_FALLING = 7200

### Buzzer activation
BUZZER_ENABLE   = True

#####
# System parameters (do not modify)
#####

SOFT_VERSION    = "1.9.DEBUG"
DEBUG           = True
FREQ_ACQ        = 20    # Frequence d'acquisition des données [Hz]
SERVO_OPEN      = 800   # [us]
SERVO_CLOSE     = 1800  # [us]
