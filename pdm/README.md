# ZG6: PDM Microphones project

* [Préparation](#prep)
* [Introduction aux microphones PDM](#pdm)
* [Carte d'acquisition des microphones CCA02M2](#cca02m2)
* [Projet d'exemple](#project)

<a id="prep"></a>
## Préparation

Compléter en premier lieu le projet de préparation [pdm_prep](../pdm_prep/README.md),
puis importer le code développé dans la fonction `AUDIO_IN_Init` du fichier `cca02m2/cca02m2.c`.

Dans la fonction `AUDIO_IN_Init` du fichier `cca02m2/cca02m2.c`, compléter également la configuration du `DMA End Point` correspondant au coupleur périphérique SPI2, ainsi que le **numéro de stream** utilisé dans l'initialisation via la fonction `dma_stream_init`.

<a id="pdm"></a>
## Microphones PDM

Des vidéos d'introduction à la techno des microphones PDM

* part 1 : techno existantes et vocabulaire (analog/digital, ECM/MEMS, PDM/PCM/I2S) : https://www.youtube.com/watch?v=_YQSJJQUp-g
* part 2 : techno PDM plus en détail (connexion, paramètres) : https://www.youtube.com/watch?v=a-diRANswfw
* part 3 : converion PDM -> PCM (filtrage LP FIR, décimation, filtrage HP, gain) : https://www.youtube.com/watch?v=5lH-tQw0tlU
* part 4 : bibliothèque ST PDM2PCM : https://www.youtube.com/watch?v=QgecXVmNyiI
* part 5 : périphériques utilisables (I2S,SPI,SAI,DFSDM) et horloge : https://www.youtube.com/watch?v=Ls1mXjlkJHk
* part 6 : détails I2S et SPI : https://www.youtube.com/watch?v=z25zckhHzC8

Autres interfaces présentes sur certains microcontrôleurs

* part 7 : détails SAI : https://www.youtube.com/watch?v=0bviu-1L1bI
* part 8 : détails DFSDM : https://www.youtube.com/watch?v=uMCTkd0PGRs

De la doc générale sur la techno des microphones MEMS : [AN4426.pdf](docs/AN4426.pdf)

La documentation de la bibliothèque PDM2PCM : [UM2372.pdf](docs/UM2372.pdf)

**Vocabulaire** :

SPI: Serial Peripheral Interface
I2S: Inter Integrated circuit Sound
SAI: Serial Audio Interface
PCM: Pulse coded Modulation
PDM: Pulse Density Modulation
FDSDM: Digital Filter for Sigma Delta Modulation


<a id="cca02m2"></a>
## Carte d'acquisition des microphones CCA02M2

* getting started : [UM2631.pdf](docs/UM2631.pdf)
* **A lire absolument!** : [Config matérielle et logicielle de la carte CCA02M2](docs/cca02m2-intro.pdf)


<a id="project"></a>
## Projet d'exemple

Il permet de récupérer des échantillons par tranche d'1ms (voir 
NMS_PER_INTERRUPT défini dans `cca02m2/cca02m2_audio.h`) et de réaliser
la conversion PDM vers PCM. Vous pourrez alors utiliser les échantillons du buffer PCM pour réaliser les traitements requis (détection d'activité, corrélation, estimation de l'angle).


```
 +- config/            linker script, openocd script
 +- docs/              some doc (microphones ...)
 +- include/
 |   +- cmsis/         ARM CMSIS core
 |   +- board.h        global peripheral pointers
 |   +- config.h       global project IO pin configuration
 |   +- stm32f411xe.h  peripheral register structure definition
 +- lib/               microcontroller peripheral drivers
 +- cca02m2/           CCA02M2 microphone board support
 |   +- cca02m2_audio.h                 audio function exports
 |   +- libPDMFilter_CM4_wc32_softfp.a  PDM2PCM library
 +- DSP/
 +   +- Include/
 |   |   +- arm_math.h           ARM DSP function exports
 |   +- libarm_cortexM4l_math.a  ARM DSP library
 +- src/               Application src
 +- startup/           Startup code
 +- Makefile
 +- README.md          This file
```

Comme d'habitude : `make clean / make / tdb main.elf`
