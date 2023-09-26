# BerryRocket code

Berryrocket code is in charge of several functions :
- Manage the platform
    - Detect lift-off
    - Detect max altitude
    - Open parachute
- Acquire and store data from platform sensor : acceleration, angular speed, pressure, ...
- Manage the payload (if needed) : do the experience and store data (can be customized)

## File organisation
- main.py : contains rocket sequencer and payload code
- lps22.py : contains pressure sensor interface code
- icm20984.py : contains IMU sensor interface code
- buzzer.py : contains buzzer management code


## Organisation des fichiers
- main.py : contient le programme qui permet de séquencer la fusée ainsi que l'acquisition de la charge utile
- lps22.py : contient le code qui permet d'utiliser capteur de pression et de température
- icm20984.py : contient le code qui permet d'utiliser l'IMU (accéléromètre, gyroscope et magnétomètre)
- buzzer.py : contient des fonctions relatives à la gestion du buzzer

![cc-by
-nc-sa](https://user-images.githubusercontent.com/1367183/214925257-8b6ebb08-f1ee-49e5-85f3-be77d70f8bf6.png)
