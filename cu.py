# Berryrocket project
# On-board Payload code
# Licence CC-BY-NC-SA
# Louis Barbier

# Exécute les actions de la charge utile au démarrage de la carte
def CU_Initialisation(baro, imu):
    """Exécute les actions de la charge utile au démarrage de la carte"""
    ############################################################
    ########## Mettre ici le code de la charge utile  ##########
    ########## qui va se dérouler APRES le décollage  ##########
    ############################################################

    # ICI

    ############################################################
    ########## Fin du code de la charge utile         ##########
    ############################################################

# Exécute les actions de la charge utile avant décollage
def CU_AvantDecollage(temps, baro, imu):
    """Exécute les actions de la charge utile avant décollage"""
    ############################################################
    ########## Mettre ici le code de la charge utile  ##########
    ########## qui va se dérouler AVANT le décollage  ##########
    ############################################################

    # ICI

    ############################################################
    ########## Fin du code de la charge utile         ##########
    ############################################################

# Exécute les actions de la charge utile après décollage
def CU_ApresDecollage(temps, baro, imu):
    """Exécute les actions de la charge utile après décollage"""
    ############################################################
    ########## Mettre ici le code de la charge utile  ##########
    ########## qui va se dérouler APRES le décollage  ##########
    ############################################################

    # Mise en forme des données à écrire sur le fichier
    # Par exemple: le temps et la température
    # dataCu = f"{tempsAcq:.2f} {temp:.1f}\n"

    # Ecriture des données dans le fichier data_cu.txt
    # fileCu = open("data/data_cu.txt","a", encoding="utf-8")
    # fileCu.write(dataCu)
    # fileCu.close()

    ############################################################
    ########## Fin du code de la charge utile         ##########
    ############################################################

# Exécute les actions de la charge utile prendant la redescente
def CU_Redescente(temps, baro, imu):
    """Exécute les actions de la charge utile prendant la redescente"""
    ############################################################
    ########## Mettre ici le code de la charge utile  ##########
    ########## qui va se dérouler PENDANT la redescente ########
    ############################################################

    # ICI

    ############################################################
    ########## Fin du code de la charge utile         ##########
    ############################################################