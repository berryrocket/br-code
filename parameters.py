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
SENSOR_BOARD    = "BR_MINI_SENSOR"

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

#####
# System parameters (be careful when you modify these parameters)
#####

SOFT_VERSION    = "1.9"
DEBUG           = True
FREQ_ACQ        = 20    # Frequence d'acquisition des données [Hz]
