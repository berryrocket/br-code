########################################
#### BerryRocket ####
# On-board Payload code
# Louis Barbier
# Licence CC-BY-NC-SA
########################################
#
# Ce fichier est l'endroit où les élèves écrivent leur code "charge utile".
# Il y a 4 fonctions, appelées automatiquement à 4 moments du vol.
# Chaque fonction reçoit :
#   - `time_s` : le temps écoulé depuis le démarrage [s] (sauf on_boot)
#   - `baro`  : le capteur barométrique (pression + température)
#   - `imu`   : la centrale inertielle (accéléromètre + gyroscope)
# Mettez votre code à l'intérieur des fonctions, entre les commentaires.

def payload_on_boot(baro, imu):
    """Appelée UNE SEULE FOIS au démarrage de la carte (avant le vol)."""
    # Mettez ici le code à exécuter au démarrage
    pass

def payload_before_liftoff(time_s, baro, imu):
    """Appelée en boucle (~20 fois par seconde) AVANT le décollage."""
    # Mettez ici le code à exécuter pendant l'attente au sol
    pass

def payload_after_liftoff(time_s, baro, imu):
    """Appelée en boucle (~20 fois par seconde) APRÈS le décollage."""
    # Mettez ici le code à exécuter pendant le vol.
    #
    # Exemple : enregistrer le temps et la température dans un fichier
    #
    #   temp = baro.read_temperature()
    #   ligne = f"{time_s:.2f} {temp:.1f}\n"
    #   with open("data/data_cu.txt", "a", encoding="utf-8") as f:
    #       f.write(ligne)
    pass

def payload_during_descent(time_s, baro, imu):
    """Appelée en boucle (~20 fois par seconde) PENDANT la redescente (sous parachute)."""
    # Mettez ici le code à exécuter pendant la chute libre / descente
    pass
