########################################
#### BerryRocket ####
# On-board code
# Louis Barbier
# Licence CC-BY-NC-SA
########################################

import time
import os
from machine import I2C, RTC, Timer, Pin, PWM
from lib.lps22hb import LPS22HB
from lib.icm20948 import ICM20948
from lib.lsm6dsx import LSM6DSx
from lib.mpu9250 import MPU9250
from lib.xis2mdx import xIS2MDx
from cu import *
from buzzer import *
from telemetry import TelemetryWS
from web.captive_dns import CaptiveDNS
from web import ground_cmd
import parameters as PARAMS


#############################
####     Constants       ####
#############################
DATA_FOLDER = "data"
DATA_FILE   = DATA_FOLDER + "/data_platform.txt"


#############################
####  Hardware setup     ####
#############################
def setup_i2c():
    """Configure le bus I2C selon la carte mère."""
    if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
        return I2C(1, freq=400000)  # default: scl=Pin(7), sda=Pin(6)
    if PARAMS.MOTHER_BOARD == "BR_MICRO_AVIONIC":
        return I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
    print("[ATTENTION] La carte mère sélectionnée n'est pas référencée !")
    print("            Changer les paramètres - Mise off de la carte par sécurité")
    exit(1)


def setup_sensors(i2c):
    """Choisit les drivers capteurs selon SENSOR_BOARD. Retourne (baro, imu, mag)."""
    if PARAMS.SENSOR_BOARD in (None, "NONE"):
        PARAMS.SENSOR_BOARD = None
        return None, None, None

    baro = LPS22HB(i2c)
    if PARAMS.SENSOR_BOARD == "10DOF_V1":
        return baro, ICM20948(i2c_bus=i2c), None
    if PARAMS.SENSOR_BOARD == "10DOF_V2.1":
        return baro, MPU9250(i2c=i2c), None
    if PARAMS.SENSOR_BOARD == "BR_MINI_SENSOR":
        imu = LSM6DSx(i2c_bus=i2c)
        mag = xIS2MDx(i2c_bus=i2c) if 0x1E in i2c.scan() else None
        return baro, imu, mag

    print("[ATTENTION] La carte sensor sélectionnée ne correspond pas à une carte connue")
    print("            Sélection par défaut de la carte BR-MINI-SENSOR")
    PARAMS.SENSOR_BOARD = "BR_MINI_SENSOR"
    return baro, MPU9250(i2c=i2c), None


def setup_parachute_servo():
    """Retourne le PWM du servomoteur de trappe, ou None si pas applicable."""
    if not PARAMS.EJECTION_CHARGE and PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
        servo = PWM(Pin(10, Pin.OUT))
        servo.freq(50)  # 50 Hz
        return servo
    return None


def setup_contact_pin():
    """Retourne la pin de l'accéléromètre contact, ou None si pas applicable."""
    if PARAMS.MOTHER_BOARD == "BR_MINI_AVIONIC":
        return Pin(28, Pin.IN)
    return None


# Objets matériels (créés une seule fois au chargement du module)
i2c             = setup_i2c()
baro, imu, mag  = setup_sensors(i2c)
parachute_servo = setup_parachute_servo()
acc_contact_in  = setup_contact_pin()
acq_timer       = Timer()
rtc             = RTC()


#############################
####     Flight state    ####
#############################
# État global du vol — modifié uniquement par les fonctions ci-dessous
# et par le bloc d'entrée tout en bas du fichier.
is_sampling     = False  # True quand le timer d'acquisition demande une mesure
contact_fired   = False  # True après le 1er front montant de l'accéléro contact
launched        = False  # True après détection du décollage
falling         = False  # True après le délai d'apogée (chute libre)
launch_time_ms  = 0      # ticks_ms() au moment du décollage
low_space_fs    = False  # True si l'espace libre est insuffisant avant le vol


#############################
####     Tiny helpers    ####
#############################
def open_parachute():
    if parachute_servo is not None:
        parachute_servo.duty_ns(PARAMS.SERVO_OPEN * 1000)


def close_parachute():
    if parachute_servo is not None:
        parachute_servo.duty_ns(PARAMS.SERVO_CLOSE * 1000)


def on_contact_rise(pin):
    """IRQ du contacteur d'accélération (front montant)."""
    global contact_fired
    contact_fired = True


def on_acq_tick(timer):
    """Callback du timer d'acquisition à FREQ_ACQ Hz."""
    global is_sampling
    is_sampling = True


def get_free_space_bytes():
    """Espace libre du FS en octets. Renvoie -1 si indisponible.
    LittleFS peut renvoyer un f_bavail négatif en overcommit (FS saturé) :
    on clamp à 0 pour que les comparaisons numériques restent correctes."""
    try:
        s = os.statvfs('/')
        free = s[0] * s[3]  # f_bsize * f_bavail
        return free if free > 0 else 0
    except Exception:
        return -1


def print_fs_info():
    """Affiche l'état du FS — diagnostic avant les écritures."""
    try:
        s = os.statvfs('/')
        bsize, frsize, blocks, bfree, bavail = s[0], s[1], s[2], s[3], s[4]
        total = blocks * bsize
        free  = bavail * bsize
        used  = total - free
        pct   = (used * 100) // total if total else 0
        print(f"[FS] bloc={bsize}B total={total}B used={used}B ({pct}%) free={free}B "
              f"blocs total={blocks} libres={bfree}")
        try:
            sz = os.stat(DATA_FILE)[6]
            print(f"[FS] data_platform.txt = {sz} B")
        except OSError:
            print("[FS] data_platform.txt absent")
    except Exception as e:
        print(f"[FS] statvfs indisponible: {e}")


def init_data_file():
    """Crée data/data_platform.txt et écrit son en-tête de mission."""
    try:
        if DATA_FOLDER not in os.listdir():
            os.mkdir(DATA_FOLDER)

        with open(DATA_FILE, "a", encoding="utf-8") as f:
            f.write("########\n")
            f.write(f"## Version logiciel : v{PARAMS.SOFT_VERSION:s}\n")
            f.write("## Type fusée : ")
            f.write("Dépotage\n" if PARAMS.EJECTION_CHARGE else "Trappe parachute\n")

            f.write("## Détection décollage : ")
            modes = []
            if PARAMS.LIFTOFF_DET_IMU:     modes.append("IMU")
            if PARAMS.LIFTOFF_DET_CONTACT: modes.append("Accélero contact")
            f.write(" + ".join(modes) + "\n")

            f.write(f"## Fenetrage temporel : {PARAMS.TIMEOUT_APOGEE:d} ms\n")
            f.write(f"## Frequence acquisition données: {PARAMS.FREQ_ACQ:d} Hz\n")
            f.write("# Temps [s] | Pression [mBar] | temperature [°C] | acc X [g] | acc Y [g] | acc Z [g] "
                    "| gyro X [dps] | gyro Y [dps] | gyro Z [dps] | mag X [Gauss] | mag Y [Gauss] | mag Z [Gauss]\n")
    except OSError as e:
        print(f"[ERREUR] Initialisation data_platform.txt impossible: {e}")
        print(get_free_space_bytes())


#############################
####   Sampling step     ####
#############################
def _safe(value):
    """Remplace None par 0.0 pour éviter de crasher si un capteur ne répond pas."""
    return 0.0 if value is None else value


def read_all_sensors():
    """Lit baro + IMU (+ mag) et renvoie un tuple de 13 valeurs :
    (pressure, temp, ax, ay, az, gx, gy, gz, mx, my, mz, temp_imu, temp_mag)."""
    if PARAMS.SENSOR_BOARD is None:
        return (0.0,) * 13

    pressure = _safe(baro.read_pressure())
    temp     = _safe(baro.read_temperature())
    mx = my = mz = 0.0
    temp_imu = temp_mag = 0.0

    if PARAMS.SENSOR_BOARD == "10DOF_V1":
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro()
        temp_imu = _safe(imu.read_temperature())
    elif PARAMS.SENSOR_BOARD == "10DOF_V2.1":
        ax, ay, az = imu.acceleration
        gx, gy, gz = imu.gyro
        temp_imu   = _safe(imu.temperature)
    else:  # BR_MINI_SENSOR
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro()
        temp_imu = _safe(imu.read_temperature())
        if mag is not None:
            mx_raw, my_raw, mz_raw = mag.read_mag()
            mx = _safe(mx_raw)
            my = -_safe(my_raw)   # alignement axe Y du magnéto avec l'IMU
            mz = _safe(mz_raw)
            temp_mag = _safe(mag.read_temperature())

    return (pressure, temp,
            _safe(ax), _safe(ay), _safe(az),
            _safe(gx), _safe(gy), _safe(gz),
            mx, my, mz,
            temp_imu, temp_mag)


def detect_liftoff(ay):
    """Renvoie True si on doit déclencher le décollage à cet instant."""
    if launched:
        return False
    imu_trigger     = PARAMS.LIFTOFF_DET_IMU     and ay < -1 - PARAMS.LIFTOFF_IMU_THRESHOLD
    contact_trigger = PARAMS.LIFTOFF_DET_CONTACT and contact_fired
    return imu_trigger or contact_trigger


def detect_apogee(now_ms):
    """Renvoie True si on doit basculer en chute libre maintenant."""
    return launched and (not falling) and (now_ms - launch_time_ms > PARAMS.TIMEOUT_APOGEE)


def format_data_line(time_s, reading):
    """Formate une ligne du fichier data_platform.txt."""
    pressure, temp, ax, ay, az, gx, gy, gz, mx, my, mz, _ti, _tm = reading
    return (f"{time_s:.3f} {pressure:.1f} {temp:.1f} "
            f"{ax:.2f} {ay:.2f} {az:.2f} "
            f"{gx:.2f} {gy:.2f} {gz:.2f} "
            f"{mx:.2f} {my:.2f} {mz:.2f}\n")


def append_lines(path, lines):
    """Ajoute des lignes au fichier. Renvoie True si OK, False sinon."""
    try:
        with open(path, "a", encoding="utf-8") as f:
            for line in lines:
                f.write(line)
        return True
    except OSError as e:
        if PARAMS.DEBUG:
            print(f"[ERREUR] Ecriture data_platform.txt impossible: {e}")
        return False


def send_telemetry_frame(telem, time_s, reading):
    """Émet une trame Nectar via le WebSocket (silencieux si désactivé)."""
    if telem is None:
        return
    pressure, temp, ax, ay, az, gx, gy, gz, mx, my, mz, ti, tm = reading
    flags = (   int(launched)
             | (int(falling)       << 1)
             | (int(contact_fired) << 2)
             | (int(low_space_fs)  << 3))
    telem.send_telemetry(int(time_s * 1000), pressure, temp,
                         ax, ay, az, gx, gy, gz, mx, my, mz,
                         ti, tm, flags)


def print_debug(time_s, reading):
    """Affiche un récap console (mode DEBUG uniquement, non bloquant)."""
    pressure, temp, ax, ay, az, gx, gy, gz, mx, my, mz, ti, tm = reading
    rtc_t = rtc.datetime()
    print(f'\nTime:        {rtc_t[4]:d}h{rtc_t[5]:d}m{rtc_t[6]:d}s / {time_s:.2f}')
    if PARAMS.SENSOR_BOARD is not None:
        print(f'Acceleration:  X = {ax:.2f} , Y = {ay:.2f} , Z = {az:.2f}')
        print(f'Gyroscope:     X = {gx:.2f} , Y = {gy:.2f} , Z = {gz:.2f}')
        print(f'Magnetometre:  X = {mx:.2f} , Y = {my:.2f} , Z = {mz:.2f}')
        print(f'Pressure:      P = {pressure:.2f} hPa')
        print(f'Temperature:   T = {temp:.2f} dC / IMU = {ti:.2f} dC / MAG = {tm:.2f} dC')
    print(f'Acc contact:   {contact_fired:.1d}')


#############################
####  Setup procedures   ####
#############################
def init_board():
    """Initialise RTC, IRQ contact, et démarre le timer d'acquisition."""
    rtc.datetime((2020, 1, 1, 0, 0, 0, 0, 0))
    if acc_contact_in is not None:
        acc_contact_in.irq(trigger=Pin.IRQ_RISING, handler=on_contact_rise)
    acq_timer.init(freq=PARAMS.FREQ_ACQ, mode=Timer.PERIODIC, callback=on_acq_tick)


def start_telemetry():
    """Lance le serveur WebSocket Nectar + le DNS captif. Retourne (telem, cdns)."""
    if not PARAMS.TELEMETRY_ENABLE:
        return None, None
    telem = TelemetryWS(
        ssid_prefix  = PARAMS.TELEMETRY_AP_SSID_PREFIX,
        open_network = PARAMS.TELEMETRY_AP_OPEN,
        password     = PARAMS.TELEMETRY_AP_PSK,
        channel      = PARAMS.TELEMETRY_AP_CHANNEL,
        port         = PARAMS.TELEMETRY_WS_PORT,
        ssid_type    = PARAMS.TELEMETRY_SSID_TYPE,
        apid         = PARAMS.TELEMETRY_APID,
        ssid_num     = PARAMS.TELEMETRY_SSID_NUM,
        rate_hz      = PARAMS.TELEMETRY_RATE_HZ,
        debug        = PARAMS.DEBUG,
    )
    telem.start()
    # DNS captif : redirige toute résolution vers nous, déclenche le popup
    # "Se connecter au réseau" des OS et ouvre la page web automatiquement
    # à la connexion au wifi.
    cdns = CaptiveDNS("192.168.4.1", debug=PARAMS.DEBUG)
    cdns.start()
    return telem, cdns


#############################
####  Main flight loop   ####
#############################
def run_flight_loop(telem, cdns):
    """Boucle principale : acquisition + détection + écriture + télémétrie."""
    global is_sampling, launched, falling, launch_time_ms

    start_ms             = time.ticks_ms()
    pending_lines        = []                              # lignes en attente (RAM)
    flush_every          = max(1, PARAMS.FREQ_ACQ // 2)    # flush toutes les ~0.5 s
    samples_since_flush  = 0
    first_flight_write   = True
    debug_print_every    = max(1, PARAMS.FREQ_ACQ // 4)    # 4 prints/s en DEBUG
    debug_tick           = 0

    while True:
        if is_sampling:
            is_sampling = False

            # --- Mesure ---
            time_s  = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
            reading = read_all_sensors()
            ay      = reading[3]

            # --- Détection décollage ---
            if detect_liftoff(ay):
                launched       = True
                launch_time_ms = time.ticks_ms()
                ground_cmd.lock_after_liftoff()
                set_buzzer(PARAMS.BUZZER_ENABLE, freq=1500, tps=1)
                if PARAMS.DEBUG:
                    print('Decollage !')

            # --- Détection apogée / chute libre ---
            if detect_apogee(time.ticks_ms()):
                open_parachute()
                falling = True
                set_buzzer(PARAMS.BUZZER_ENABLE, freq=2000, tps=0.5)
                append_lines(DATA_FILE, [f"# Free-fall: {time_s:.3f}s\n"])
                if PARAMS.DEBUG:
                    print('Free-fall !')

            # --- Télémétrie radio ---
            send_telemetry_frame(telem, time_s, reading)

            # --- Buffer + écriture fichier ---
            pending_lines.append(format_data_line(time_s, reading))

            if not launched:
                # Avant décollage : on garde seulement les 0.5 s pré-décollage.
                if len(pending_lines) > flush_every:
                    del pending_lines[0]
                payload_before_liftoff(time_s, baro, imu)
            else:
                samples_since_flush += 1
                if samples_since_flush >= flush_every:
                    if first_flight_write:
                        pending_lines.insert(0, f"# Lift-off: {time_s:.3f}s\n")
                        first_flight_write = False
                    append_lines(DATA_FILE, pending_lines)
                    pending_lines       = []
                    samples_since_flush = 0
                payload_after_liftoff(time_s, baro, imu)

            if falling:
                payload_during_descent(time_s, baro, imu)

            # --- Debug console (non bloquant : 4 prints/s) ---
            if PARAMS.DEBUG:
                debug_tick += 1
                if debug_tick >= debug_print_every:
                    debug_tick = 0
                    print_debug(time_s, reading)

        # System code — DNS captif (non bloquant, sert l'auto-popup wifi)
        if cdns is not None:
            cdns.poll()


#############################
####    Entry point      ####
#############################
if __name__ == '__main__':

    if PARAMS.DEBUG:
        print_fs_info()

    # Début initialisation avec son specific
    set_buzzer(PARAMS.BUZZER_ENABLE, freq=800, tps=0.2)
    time.sleep(0.2)
    set_buzzer(False)

    # Cycle trappe parachute au démarrage (open → 3 s d'attente → close)
    open_parachute()
    time.sleep(3)
    close_parachute()

    # Partage des fonctions de pilotage trappe avec le module ground_cmd
    # pour les commandes web (sous armement). Source de vérité unique.
    if parachute_servo is not None:
        ground_cmd.setup(open_parachute, close_parachute,
                         PARAMS.BUZZER_ENABLE,
                         data_path=DATA_FILE,
                         init_data_fn=init_data_file)
    else:
        ground_cmd.setup(None, None, PARAMS.BUZZER_ENABLE,
                         data_path=DATA_FILE,
                         init_data_fn=init_data_file)

    init_board()
    telem, cdns = start_telemetry()
    init_data_file()

    # Vérification de l'espace libre pour un vol
    free_bytes = get_free_space_bytes()
    if 0 <= free_bytes < PARAMS.MIN_FREE_SPACE_BYTES:
        low_space_fs = True
        print(f"[ERREUR] Espace libre insuffisant pour un vol : "
              f"{free_bytes} B < {PARAMS.MIN_FREE_SPACE_BYTES} B "
              f"(>= ~500 kB requis). Purger data/ avant le décollage !")
        # Son d'alerte dédié : grave et rapide, distinct des autres tonalités
        set_buzzer(PARAMS.BUZZER_ENABLE, freq=200, tps=0.25)
        time.sleep(3)

    # Exécute les actions de la charge utile au démarrage de la carte
    payload_on_boot(baro, imu)

    # Fin initialisation avec son specific
    set_buzzer(PARAMS.BUZZER_ENABLE, freq=800, tps=0.2)
    time.sleep(0.6)
    # Son pré-décollage
    set_buzzer(PARAMS.BUZZER_ENABLE, freq=1000, tps=2)

    run_flight_loop(telem, cdns)
