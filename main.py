########################################
#### BerryRocket ####
# On-board code
# Louis Barbier
# Licence CC-BY-NC-SA
########################################

import time
import os
from machine import I2C,RTC,Timer,Pin,PWM
from lib.lps22hb import LPS22HB
from lib.icm20948 import ICM20948
from lib.lsm6dsx import LSM6DSx
from lib.mpu9250 import MPU9250
from cu import *
from buzzer import *
import parameters as PARAMS

#####################
#### Declaration ####
#####################
# Declaration du bus de communication I2C
if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
    i2c = I2C(1,freq=400000)  # default assignment: scl=Pin(7), sda=Pin(6)
elif PARAMS.MOTHER_BOARD == "BR_MICRO_AVIONIC": 
    i2c = I2C(0, sda = Pin(4), scl = Pin(5), freq = 400000) #i2c detains
else:
    print("/!\\ La carte mère sélectionnée n'est pas référencée !\n")
    exit(1)

print(i2c.scan())
# Declaration des capteurs
if PARAMS.SENSOR_BOARD == "NONE" or PARAMS.SENSOR_BOARD == None:
    SENSOR_BOARD = None
    baro = None
    imu = None
else:
    baro = LPS22HB(i2c)
    if PARAMS.SENSOR_BOARD == '10DOF_V1':
        imu = ICM20948(i2c_bus=i2c)
    elif PARAMS.SENSOR_BOARD == '10DOF_V2.1':
        imu = MPU9250(i2c=i2c)
    elif PARAMS.SENSOR_BOARD == 'BR_SENSOR':
        imu = LSM6DSx(i2c_bus=i2c)
    else:
        print("Attention: la carte sensor selectionnée ne correspond pas à une carte connue") 
        print("Selection par défaut de la carte 10DOF V2.1")
        PARAMS.SENSOR_BOARD = '10DOF_V2.1'
        imu = MPU9250(i2c=i2c)

# Declaration du timer
acq_timer = Timer()

# Declaration de l'horloge temps réel (RTC)
rtc = RTC()

# Declaration de la pin du parachute
porte_para = 0
if PARAMS.EJECTION_CHARGE is False and PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
    porte_para = PWM(Pin(10, Pin.OUT))
    porte_para.freq(50) # 50 Hz
# porte_para.calibration(700, 2400, 1510, 2500, 2000) # Min pulse, max pulse, middle pulse, 90 deg pulse, 100 speed

# Declaration de la pin de l'interrupteur d'accélération
if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
    acc_contact_in = Pin(28, Pin.IN)

# Dossier des données
data_folder_name = 'data'
data_folder = data_folder_name+'/'

###################
#### Variables ####
###################
# Initialisation des variables
is_sampling         = False
is_launched         = False
is_falling          = False
acc_contact         = False
launch_time         = 0
write_data_file     = True
first_write_file    = True

###################
#### Fonctions ####
###################
# Initialisation de la carte
def InitBoard():
    # Initialisation de la date/heure
    rtc.datetime((2020,1,1,0,0,0,0,0))

    # Ajout de la detection pour l'accelero contact 
    if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
        acc_contact_in.irq(trigger=Pin.IRQ_RISING, handler=IrqAcc)

    # Initialisation du timer d'acquisition
    acq_timer.init(freq=PARAMS.FREQ_ACQ, mode=Timer.PERIODIC, callback=Sampling)

def InitPlatFile():
    if data_folder_name not in os.listdir():
        os.mkdir(data_folder_name)

    platform_file = open(data_folder+"data_platform.txt","a", encoding="utf-8")
    platform_file.write(f"########\n")
    platform_file.write(f"## Version soft : v{PARAMS.SOFT_VERSION:s}\n")
    platform_file.write(f"## Type fusée : ")
    if (PARAMS.EJECTION_CHARGE is True):
        platform_file.write(f"Ejection charge\n")
    else:
        platform_file.write(f"Trappe parachute\n")
    platform_file.write(f"## Détection décollage : ")
    if (PARAMS.LIFTOFF_DET_IMU is True):
        platform_file.write(f"IMU\n")
    if (PARAMS.LIFTOFF_DET_CONTACT is True):
        platform_file.write(f"Accélero contact\n")
    platform_file.write(f"## Fenetrage temporel : {PARAMS.TIMEOUT_APOGEE:d} ms\n")
    platform_file.write(f"## Frequence acq données: {PARAMS.FREQ_ACQ:d} Hz\n")
    platform_file.write(f"# Temps [s] | Pression [mBar] | temperature [°C] | acc X [g] | acc Y [g] | acc Z [g] | gyro X [dps] | gyro Y [dps] | gyro Z [dps]\n")
    platform_file.close()

# Activation de l'acquisition des données
def Sampling(timer):
    global is_sampling
    #if is_sampling is False:
    is_sampling = True

def FermetureParachute():
    if PARAMS.EJECTION_CHARGE is False:
        global porte_para
        porte_para.duty_ns(PARAMS.SERVO_CLOSE*1000)

def OuvertureParachute():
    if PARAMS.EJECTION_CHARGE is False:
        global porte_para
        porte_para.duty_ns(PARAMS.SERVO_OPEN*1000)

def IrqAcc(p):
    global acc_contact
    acc_contact = True

# Main fonction
if __name__ == '__main__':

    # Début initialisation avec son specific
    SetBuzzer(PARAMS.BUZZER_ENABLE, freq=800, tps=0.2)
    time.sleep(0.2)
    SetBuzzer(False)

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
    data_platform_buffer = []

    # Exécute les actions de la charge utile au démarrage de la carte
    CU_Initialisation(baro, imu)

    # Initialisation du temps initial
    tempsMsDebut = time.ticks_ms()

    # Fin initialisation avec son specific
    SetBuzzer(PARAMS.BUZZER_ENABLE, freq=800, tps=0.2)
    time.sleep(0.6)
    # Configure le buzzer pour faire un son specifique avant décollage
    SetBuzzer(PARAMS.BUZZER_ENABLE, freq=1000, tps=2)

    ax, ay, az, gx, gy, gz, mx, my, mz = 0,0,0,0,0,0,0,0,0
    pressure = 0
    temp = 0
    
    while True:
        if is_sampling is True:
            # Acquisition du temps actuel
            current_time = time.ticks_diff(time.ticks_ms(), tempsMsDebut)/1000.0

            # Acquisitions des capteurs
            # ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro()
            # mx, my, mz = imu.read_magnetometer_data()

            if PARAMS.SENSOR_BOARD != None:
                pressure = baro.read_pressure()
                temp = baro.read_temperature()
                if PARAMS.SENSOR_BOARD == '10DOF_V1':
                    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro()
                    temp_imu = imu.read_temperature()
                elif PARAMS.SENSOR_BOARD == 'BR_SENSOR':
                    ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro()
                    temp_imu = imu.read_temperature()
                else: # SENSOR_BOARD == '10DOF_V2.1'
                    ax, ay, az = imu.acceleration
                    gx, gy, gz = imu.gyro
                    temp_imu = imu.temperature

            # Si l'acceleration dépasse le seuil ou que la pin d'accélération est appuyée, et que le décollage n'est pas encore arrivé, il y a eu décollage
            if ((ay < -1-PARAMS.LIFTOFF_IMU_TH and PARAMS.LIFTOFF_DET_IMU is True) or (acc_contact is True and PARAMS.LIFTOFF_DET_CONTACT is True)) and (is_launched is False):
                # Changement de status de l'indicateur de decollage
                is_launched = True
                # Sauvegarde du temps de décollage
                launch_time = time.ticks_ms()
                # Changement du son du buzzer
                SetBuzzer(PARAMS.BUZZER_ENABLE, freq=1500, tps=1)
                if PARAMS.DEBUG is True:
                    # Affichage sur la console
                    print('Decollage !')

            # Si le decollage est passé et que la chute libre n'est pas encore arrivé
            if (is_launched is True) and (is_falling is False):
                # Si le timer de chute libre est dépassé
                if (time.ticks_ms()-launch_time > PARAMS.TIMEOUT_APOGEE):
                    # Ouverture de la trappe parachute si besoin 
                    OuvertureParachute()
                    # Changement de status de chute libre
                    is_falling = True
                    # Changement du son du buzzer
                    SetBuzzer(PARAMS.BUZZER_ENABLE, freq=2000, tps=0.5)
                    # Ecriture du temps actuel du debut de la chute libre dans le fichier
                    platform_file = open(data_folder+"data_platform.txt","a", encoding="utf-8")
                    platform_file.write(f"# Chute libre: {current_time:.3f}s\n")
                    platform_file.close()
                    if PARAMS.DEBUG is True:
                        # Affichage sur la console
                        print('Chute libre !')
            
            # Mise en forme des données à écrire sur le fichier (temps, pression, température, accélération x,y,z)
            dataFilePlat = f"{current_time:.3f} {pressure:.1f} {temp:.1f} {ax:.2f} {ay:.2f} {az:.2f} {gx:.2f} {gy:.2f} {gz:.2f}\n"

            # Si le decollage n'a pas encore eu lieu
            if is_launched is False:
                # Garde les données sur 0.5 seconde avant le décollage
                data_platform_buffer.append(dataFilePlat)
                if (len(data_platform_buffer) > PARAMS.FREQ_ACQ/2):
                    del data_platform_buffer[0]

                # Exécute les actions de la charge utile avant décollage
                CU_AvantDecollage(current_time, baro, imu)

            # Si le decollage a eu lieu
            else:
                if write_data_file is False:
                    # Sauvegarde RAM avant écriture
                    data_platform_buffer.append(dataFilePlat)
                    if (len(data_platform_buffer) >= PARAMS.FREQ_ACQ/2):
                        write_data_file = True
                elif write_data_file is True:
                    # Ecriture sur le fichier de plusieurs points de mesure
                    platform_file = open(data_folder+"data_platform.txt","a", encoding="utf-8")
                    for dataEl in data_platform_buffer:
                        platform_file.write(dataEl)
                    data_platform_buffer = []
                    if first_write_file is True:
                        platform_file.write(f"# Decollage: {current_time:.3f}s\n")
                        first_write_file = False
                    platform_file.write(dataFilePlat)
                    platform_file.close()
                    write_data_file = False

                # Exécute les actions de la charge utile après décollage
                CU_ApresDecollage(current_time, baro, imu)

            # Si la fusée est en chute libre
            if is_falling is True:
                # Exécute les actions de la charge utile prendant la redescente
                CU_Redescente(current_time, baro, imu)

            # Reinitialisation de l'indicateur pour le timer d'acquisition
            is_sampling = False

            if PARAMS.DEBUG is True:
                # Affichage des resultats sur la console
                rtc_time = rtc.datetime()
                print(f'\nTime:        {rtc_time[4]:d}h{rtc_time[5]:d}m{rtc_time[6]:d}s / {current_time:.2f}')
                if PARAMS.SENSOR_BOARD != None:
                    print(f'Acceleration:  X = {ax:.2f} , Y = {ay:.2f} , Z = {az:.2f}')
                    print(f'Gyroscope:     X = {gx:.2f} , Y = {gy:.2f} , Z = {gz:.2f}')
                    print(f'Pressure:      P = {pressure:.2f} hPa')
                    print(f'Temperature:   T = {temp:.2f} dC / IMU = {temp_imu:.2f} dC')
                print(f'Acc contact:   {acc_contact:.1d}')
                time.sleep(0.250)
