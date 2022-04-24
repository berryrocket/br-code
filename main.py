from machine import I2C,RTC,Timer,Pin,PWM
import time, struct, math
import lps22, icm20948
from buzzer import *

####################
#### Constantes ####
####################
ACC_IMU = True
BUZZER_ENABLE = False
ACC_THESHOLD = 1

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
# parachute = PWM(Pin(XXX))

# Declaration de la pin de l'interrupteur d'accélération
accPin = Pin(28, Pin.IN)

###################
#### Fonctions ####
###################
# Initialisation de la carte
def InitBoard():
    # Initialisation de la date/heure
    rtc.datetime((2020,1,1,0,0,0,0,0))

    # Initialisation du timer d'acquisition
    timerAcq.init(freq=10, mode=Timer.PERIODIC, callback=Sampling)

# Activation de l'acquisition des données
def Sampling(timer):
    global isSampling
    isSampling=True

def OuvertureParachute():
    global parachute
    # parachute.duty_u16(XXX)

# Main fonction
if __name__ == '__main__':
    
    # Initialisation des variables
    isSampling = False
    isLaunched = False
    isFalling = False
    tempsMsDebut = time.ticks_ms()

    # InitMusic()
    InitBoard()

    # Ouvre un fichier pour l'écriture des données
    filePlatform = open("data_platform.txt","a")
    fileCu = open("data_cu.txt","a")
    
    # Configure le buzzer pour faire un son specifique avant décollage
    SetBuzzer(BUZZER_ENABLE, freq=1000, tps=2)

    while True:
        if isSampling==True:
            # Acquisition du temps actuel
            tempsAcq = time.ticks_diff(time.ticks_ms(), tempsMsDebut)/1000 + 1

            # Acquisitions des capteurs
            ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
            mx, my, mz = imu.read_magnetometer_data()
            pressure = lps22.read_pressure()
            temp = lps22.read_temperature()

            # Si l'acceleration dépasse le seuil ou que la pin d'accélération est appuyée, et que le décollage n'est pas encore arrivé, il y a eu décollage
            if ((ay > ACC_THESHOLD and ACC_IMU == True) or (accPin.value() == 0 and ACC_IMU == False)) and (isLaunched == False):
                # Changement de status de l'indicateur de decollage
                isLaunched = True
                # Changement du son du buzzer
                SetBuzzer(BUZZER_ENABLE, freq=1500, tps=1)
                # Acquisition du temps du composant RTC
                tempsRtc = rtc.datetime()
                # Ecriture du temps actuel du decollage dans le fichier
                filePlatform.write("Decollage: {:d}h{:d}m{:d}s{:.0f}\n".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], (tempsAcq % 1)*100))
                filePlatform.write("Temps (s) / Pression (mBar) / temperature (°C) / acc X (g/s^2) / acc Y (g/s^2) / acc Z (g/s^2)\n")
                # Affichage sur la console
                print('Decollage !')
            
            # Si le decollage est passé et que la chute libre n'est pas encore arrivé
            if (isLaunched == True) and (isFalling == False):
                # Si l'acceleration est quasi nulle ou négative alors qu'on a décollé c'est qu'on retombe
                if (ay <= 0.2 and ACC_IMU == True) or (accPin.value() == 1 and ACC_IMU == False):
                    # Ouverture du parachute
                    # OuvertureParachute()
                    # Changement de status de chute libre
                    isFalling = True
                    # Changement du son du buzzer
                    SetBuzzer(BUZZER_ENABLE, freq=2000, tps=0.5)
                    # Acquisition du temps du composant RTC
                    tempsRtc = rtc.datetime()
                    # Ecriture du temps actuel du debut de la chute libre dans le fichier
                    filePlatform.write("Chute libre: {:d}h{:d}m{:d}s{:.0f}\n".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], (tempsAcq % 1)*100))
                    # Affichage sur la console
                    print('Chute libre !')

            # Si la fusee est en chute libre
            # if isFalling == True:
                # isLaunched = False

            # Si le decollage est passé, on enregistre les données
            if isLaunched == True:
                # Mise en forme des données à écrire sur le fichier (temps, pression, température, accélération x,y,z)
                dataFilePlat = "{:.2f} {:.1f} {:.1f} {:.2f} {:.2f} {:.2f}\n".format(tempsAcq, pressure, temp, ax, ay, az)
                # Ecriture sur le fichier
                filePlatform.write(dataFilePlat)

                ############################################################
                ########## Mettre ici le code de la charge utile  ##########
                ########## qui va se dérouler après le décollage  ##########
                ############################################################

                # Mise en forme des données à écrire sur le fichier
                # Par exemple: le temps et la température
                dataCu = "{:.2f} {:.1f}\n".format(tempsAcq, temp)
            
                # Ecriture des données dans le fichier data_cu.txt
                fileCu.write(dataCu)

                ############################################################
                ########## Fin du code de la charge utile         ##########
                ############################################################

            # Reinitialisation de l'indicateur pour le timer d'acquisition
            isSampling = False  

            # Affichage des resultats sur la console 
            tempsRtc = rtc.datetime()
            print("\nTime:        {:d}h{:d}m{:d}s / {:.2f}".format(tempsRtc[4], tempsRtc[5], tempsRtc[6], tempsAcq))
            print('Acceleration:  X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(ax, ay, az))  
            print('Gyroscope:     X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(gx, gy, gz))
            print('Magnetic:      X = {:.2f} , Y = {:.2f} , Z = {:.2f}'.format(mx, my, mz))
            print('Pressure:      P = {:.2f} hPa'.format(pressure))
            print('Temperature:   T = {:.2f} °C'.format(temp))

