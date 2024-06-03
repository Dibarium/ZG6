# STM32F401/STM32F411 project

* [Organisation du projet](#about)
* [Projet main_timer_div2](#timer)


<a id="about"></a>
## Organisation du projet

```
 +- config/            linker script, openocd script
 +- docs/              some doc
 +- include/
 |   +- cmsis/         ARM CMSIS core
 |   +- board.h        global peripheral pointers
 |   +- config.h       global project IO pin configuration
 |   +- stm32f411xe.h  peripheral register structure definition
 +- lib/               microcontroller peripheral drivers
 +- src/               Application src
 +- startup/           Startup code ()
 +- Makefile
 +- README.md
```

Ensuite, c'est comme d'habitude : `ocd / make clean / make / tdb main.elf`


<a id="timer"></a>
## Projet main_timer_div2

C'est un projet qui se contente de configurer le Timer 3 de manière à fournir sur la broche PB5 un signal carré dont la fréquence a été divisée par 2 par rapport à celle du signal amené sur la broche PB4. Le registre ARR permet de modifier le rapport de division.

```
               +-------+
  input ch1 -->|  F/2  |--> output chan2
     PB4       +-------+       PB5
```

Il utilise le fait qu'une broche externe peut fournir l'horloge du compteur.

1. Analyser dans la documentation du périphérique Timer 2 à 5 la **partie 13.3.3 Clock selection** et notamment les explications sur la configuration **External clock source mode 1**. Analyser les différents bits des registres associés.
2. Compléter le code. S'inspirer des commentaires.
3. Tester avec un GBF et un oscilloscope. **Attention!** le signal d'entrée doit être compatible avec l'alimentation du microcontrôleur (0-3.3V).
4. Quand le projet est fonctionnel, copier le contenu de la fonction `main` (sans le démarrage du compteur) dans la partie dédiée à la configuration du Timer 3 du le projet `pdm`, dans la fonction `AUDIO_IN_Init` du fichier `cca02m2/cca02m2.c`. C'est le fichier qui fournit l'API d'acquisition des microphones PDM : voir le [README du projet pdm](../pdm/README.md) pour les détails.

