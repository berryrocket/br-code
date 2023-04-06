# Berryrocket project
# On-board code
# Licence CC-BY-NC-SA
# Louis Barbier

import time
from machine import I2C,RTC,Timer,Pin,PWM
import lps22
import icm20948
from buzzer import *

####################
#### Constantes ####
####################
DEPOTAGE        = True  # activation de la version avec depotage (sans trappe parachute)
ACC_IMU         = True  # activation de l'information d'accélaration par l'IMU sinon par le contacteur mécanique
BUZZER_ENABLE   = True  # activation du buzzer
ACC_THESHOLD    = 1     # seuil de l'accélération pour détecter le décollage [g]
TIMEOUT_FALLING = 7200  # temps après lequel la fusée passe en mode chute libre [ms]
FREQ_ACQ        = 30    # Frequence d'acquisition des données [Hz]
SERVO_OPEN      = 800   # [us]
SERVO_CLOSE     = 1400  # [us]
DEBUG           = False

#####################
#### Declaration ####
#####################
# Declaration du bus de communication I2C
i2c = I2C(1,freq=400000)  # default assignment: scl=Pin(7), sda=Pin(6)

# Declaration des capteurs
lps22 = lps22.LPS22HB(i2c)
imu = icm20948.ICM20948(i2c_bus=i2c)

# Declaration du timer
timerAcq = Timer()

# Declaration de l'horloge temps réel (RTC)
rtc = RTC()

# Declaration de la pin du parachute
portePara = 0
if DEPOTAGE is False:
    portePara = PWM(Pin(10, Pin.OUT))
    portePara.freq(50) # 50 Hz
# portePara.calibration(700, 2400, 1510, 2500, 2000) # Min pulse, max pulse, middle pulse, 90 deg pulse, 100 speed

# Declaration de la pin de l'interrupteur d'accélération
accPin = Pin(28, Pin.IN)

###################
#### Variables ####
###################
# Initialisation des variables
isSampling      = False
isLaunched      = False
isFalling       = False
accContact      = False
tempsDecollage  = 0

###################
#### Fonctions ####
###################
# Initialisation de la carte
def InitBoard():
    # Initialisation de la date/heure
    rtc.datetime((2020,1,1,0,0,0,0,0))

    # Ajout de la detection pour l'accelero contact 
    accPin.irq(trigger=Pin.IRQ_RISING, handler=IrqAcc)

    # Initialisation du timer d'acquisition
    timerAcq.init(freq=FREQ_ACQ, mode=Timer.PERIODIC, callback=Sampling)

def InitPlatFile():
    filePlatform = open("data_platform.txt","a", encoding="utf-8")
    filePlatform.write(f"########\n")
    filePlatform.write(f"## Version soft : v1.0\n")
    filePlatform.write(f"## Type fusée : ")
    if (DEPOTAGE is True):
        filePlatform.write(f"Depotage\n")
    else:
        filePlatform.write(f"Trappe parachute\n")
    filePlatform.write(f"## Détection décollage : ")
    if (ACC_IMU is True):
        filePlatform.write(f"IMU mems (icm20948)\n")
    else:
        filePlatform.write(f"Accélero contact\n")
    filePlatform.write(f"## Fenetrage temporel : {TIMEOUT_FALLING:d} ms\n")
    filePlatform.write(f"## Frequence acq données: {FREQ_ACQ:d} Hz\n")
    filePlatform.write(f"# Temps [s] | Pression [mBar] | temperature [°C] | acc X [g] | acc Y [g] | acc Z [g]\n")
    filePlatform.close()


# Activation de l'acquisition des données
def Sampling(timer):
    global isSampling
    # if isSampling is False:
    isSampling = True

def FermetureParachute():
    if DEPOTAGE is False:
        global portePara
        portePara.duty_ns(SERVO_CLOSE*1000)

def OuvertureParachute():
    if DEPOTAGE is False:
        global portePara
        portePara.duty_ns(SERVO_OPEN*1000)

def IrqAcc(p):
    global accContact
    accContact = True

# Main fonction
if __name__ == '__main__':

    # Début initialisation avec son specific
    # SetBuzzer(BUZZER_ENABLE, freq=800, tps=0.2)
    # time.sleep(0.2)
    # SetOffBuzzer()

    # Ouvre la trappe parachute au démarrage si besoin
    OuvertureParachute()

    # Attente pour placer la trappe parachute si besoin
    # InitMusic(BUZZER_ENABLE)
    time.sleep(3)
    
    # Fermeture de la trappe parachute si besoin
    FermetureParachute()

    # Initialisation des fonctions d'acquisitions
    InitBoard()

    # Ouvre un fichier pour l'écriture des données
    InitPlatFile()
    dataFilePlatBuff = []
    fileCu = open("data_cu.txt","a", encoding="utf-8")

    # Initialisation du temps initial
    tempsMsDebut = time.ticks_ms()

    # Fin initialisation avec son specific
    SetBuzzer(BUZZER_ENABLE, freq=800, tps=0.2)
    time.sleep(0.6)
    # Configure le buzzer pour faire un son specifique avant décollage
    SetBuzzer(BUZZER_ENABLE, freq=1000, tps=2)

    while True:
        if isSampling is True:
            # Acquisition du temps actuel
            tempsAcq = time.ticks_diff(time.ticks_ms(), tempsMsDebut)/1000 + 1

            # Acquisitions des capteurs
            ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
            # mx, my, mz = imu.read_magnetometer_data()
            pressure = lps22.read_pressure()
            temp = lps22.read_temperature()
            accContact = True
            ACC_IMU = True
            # Si l'acceleration dépasse le seuil ou que la pin d'accélération est appuyée, et que le décollage n'est pas encore arrivé, il y a eu décollage
            if ((ay < -1-ACC_THESHOLD and ACC_IMU is True) or (accContact is True and ACC_IMU is False)) and (isLaunched is False):
                # Changement de status de l'indicateur de decollage
                isLaunched = True
                # Sauvegarde du temps de décollage
                tempsDecollage = time.ticks_ms()
                # Changement du son du buzzer
                SetBuzzer(BUZZER_ENABLE, freq=1500, tps=1)
                if DEBUG is True:
                    # Affichage sur la console
                    print('Decollage !')

            # Si le decollage est passé et que la chute libre n'est pas encore arrivé
            if (isLaunched is True) and (isFalling is False):
                # Si le timer de chute libre est dépassé
                if (time.ticks_ms()-tempsDecollage > TIMEOUT_FALLING):
                    # Ouverture de la trappe parachute si besoin 
                    OuvertureParachute()
                    # Changement de status de chute libre
                    isFalling = True
                    # Changement du son du buzzer
                    SetBuzzer(BUZZER_ENABLE, freq=2000, tps=0.5)
                    # Ecriture du temps actuel du debut de la chute libre dans le fichier
                    filePlatform = open("data_platform.txt","a", encoding="utf-8")
                    filePlatform.write(f"# Chute libre: {tempsAcq:.3f}s\n")
                    filePlatform.close()
                    if DEBUG is True:
                        # Affichage sur la console
                        print('Chute libre !')
            
            # Mise en forme des données à écrire sur le fichier (temps, pression, température, accélération x,y,z)
            dataFilePlat = f"{tempsAcq:.3f} {pressure:.1f} {temp:.1f} {ax:.2f} {ay:.2f} {az:.2f}\n"

            # Si le decollage est passé, on enregistre les données
            if isLaunched is True:
                # Ecriture sur le fichier
                filePlatform = open("data_platform.txt","a", encoding="utf-8")
                if (len(dataFilePlatBuff) > 0):
                    for dataEl in dataFilePlatBuff:
                        filePlatform.write(dataEl)
                    dataFilePlatBuff = []
                    filePlatform.write(f"# Decollage: {tempsAcq:.3f}s\n")
                filePlatform.write(dataFilePlat)
                filePlatform.close()


                # if (len(dataFilePlatBuff) > 1):
                #     filePlatform.write(dataFilePlatBuff.pop(0))
                #     filePlatform.write(dataFilePlatBuff.pop(0))
                #     dataFilePlatBuff.append(dataFilePlat)
                # elif (len(dataFilePlatBuff) > 0):
                #     filePlatform.write(dataFilePlatBuff.pop(0))
                #     filePlatform.write(f"# Decollage: {tempsAcq:.3f}s\n")
                #     filePlatform.write(dataFilePlat)
                # else:
                #     filePlatform.write(dataFilePlat)

                ############################################################
                ########## Mettre ici le code de la charge utile  ##########
                ########## qui va se dérouler APRES le décollage  ##########
                ############################################################

                # Mise en forme des données à écrire sur le fichier
                # Par exemple: le temps et la température
                # dataCu = f"{tempsAcq:.2f} {temp:.1f}\n"

                # Ecriture des données dans le fichier data_cu.txt
                # fileCu.write(dataCu)

                # Si la fusee est en chute libre
                # if isFalling is True:

                ############################################################
                ########## Fin du code de la charge utile         ##########
                ############################################################
            else:
                # Garde les données sur 1/2 seconde avant le décollage
                dataFilePlatBuff.append(dataFilePlat)
                if (len(dataFilePlatBuff) > FREQ_ACQ/2):
                    del dataFilePlatBuff[0]

                ############################################################
                ########## Mettre ici le code de la charge utile  ##########
                ########## qui va se dérouler AVANT le décollage  ##########
                ############################################################

                # ICI

                ############################################################
                ########## Fin du code de la charge utile         ##########
                ############################################################

            # Reinitialisation de l'indicateur pour le timer d'acquisition
            isSampling = False

            if DEBUG is True:
                # Affichage des resultats sur la console
                tempsRtc = rtc.datetime()
                print(f'\nTime:        {tempsRtc[4]:d}h{tempsRtc[5]:d}m{tempsRtc[6]:d}s / {tempsAcq:.2f}')
                print(f'Acceleration:  X = {ax:.2f} , Y = {ay:.2f} , Z = {az:.2f}')
                print(f'Gyroscope:     X = {gx:.2f} , Y = {gy:.2f} , Z = {gz:.2f}')
                print(f'Magnetic:      X = {mx:.2f} , Y = {my:.2f} , Z = {mz:.2f}')
                print(f'Pressure:      P = {pressure:.2f} hPa')
                print(f'Temperature:   T = {temp:.2f} °C')
                print(f'Acc contact:   {accContact:.1d}')
