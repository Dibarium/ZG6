# STM32F401/STM32F411 project

* [Organisation du projet](#about)
* [Projet main_dma](#dma)


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


<a id="dma"></a>
## Projet main_dma

Ce projet permet de tester le DMA pour permettre le transfert entre la mémoire
et l'USART2 en transmission et en réception.

Le DMA (Direct Memory Access) permet d'établir un tuyau de communication entre deux éléments (mémoire/périphérique).

### Travail au niveau registre (MAIN1)

La première version du projet est codée au niveau registre.

1. Analyser la documentation du périphérique DMA et le code proposé ici pour comprendre sa structure.
2. Compléter le code avec la configuration du registre de contrôle `CR` du stream choisi.
3. Vérifier le fonctionement. En particulier, le fait qu'il faut taper 10 caractère avant qu'on récupère la main. Il sera intéressant de tester avec l'incrémentation automatique du pointeur vers la mémoire et sans.


### Travail avec une API (MAIN2)

La seconde version du projet utilise l'API définie dans `lib/dma.h`

```
DMA_Stream_t* dma_stream_init(DMA_t* dma, uint32_t stream, DMAEndPoint_t *src, DMAEndPoint_t *dest, uint32_t mode, OnTC cb);

int dma_start(DMA_Stream_t* s, uint16_t size);
int dma_stop (DMA_Stream_t *s);

int dma_complete(DMA_Stream_t *s);

int dma_status(DMA_t* dma, uint32_t stream);

```

`dma_stream_init(DMA_t* dma, uint32_t stream, DMAEndPoint_t *src, DMAEndPoint_t *dest, uint32_t mode, OnTC cb)` initialise le tuyau de communication à partir des structures `src` et `dest` de type `DMAEndPoint_t` passées en paramètres et qui décrivent les caractéristiques de chaque bout du tuyau (endpoint).

```
typedef struct _DMAEndPoint {
	Peripheral	type;
	void*		addr0;
	void*		addr1;
	int			channel;
	uint32_t	cfg;
} DMAEndPoint_t;
```

La structure décrit

- `type` : le **type** de périphérique : EP_UART_TX, EP_UART_RX, EP_I2C_TX, EP_I2C_RX, EP_SPI_TX, EP_SPI_RX, EP_ADC, EP_MEM. En réalité, l'essentiel est de pouvoir distinguer la mémoire (**EP_MEM**) des autres types.
- `addr0` : l'adresse du **buffer en mémoire** ou du **registre data du périphérique**.
- `addr1` : utilisé uniquement pour avec le type EP_MEM, dans le cas où on organise la mémoire en double buffer.
- `channel` : numéro de requête de canal obtenue à partir des tables 27 et 28 p 168,169 du manuel de référence du microcontrôleur STM32F411 (voir aussi `stream` plus bas).
- `cfg` : une configuration indiquant :

	* le format des données : EP_FMT_BYTE, EP_FMT_HALF, EP_FMT_WORD
	* si l'adresse passée doit être incrémentée après chaque transfert : EP_AUTOINC
	* si la mémoire doit être gérée en buffer circulaire, (voir *dma_start*) : EP_BUF_CIRC.
	* si la mémoire doit être gérée en double buffer circulaire (voir *dma_start*) : EP_BUF_DBL.

Les **paramètres** de la fonction `dma_stream_init` sont

- `dma` : le pointeur sur les registres du DMA.
- `stream` : le numéro de stream, associé au canal, il permet d'identifier un tuyau connecté à un périphérique (pour transmettre des données ou pour récupérer des données reçues) : voir les tables 27 et 28 p 168,169 du manuel de référence du microcontrôleur STM32F411.
- `src` et `dst` : description des endpoints src et dest de la transmission.
- `mode` : permet de fixer un niveau de priorité () et l'utilisation de la FIFO pour le tuyau de communication ().
- `cb` : callback qui, si elle est définie, sera appelée à la fin de la transmission (évènement **Transmission Complete**). La callback est appelée à partir de la routine d'interruption associé au tuyau.

	Le prototype de `cb` est
	
	`void cb(uint32_t stream, uint32_t bufid);`
	
	avec `stream` le numéro du stream et `bufid` le numéro du buffer (utile uniquement dans le cas où la mémoire est configurée en double buffer).

	Si **cb** est `NULL`, Aucune interruption n'est générée.

La fonction **renvoie** un pointeur sur le *stream utilisé* (les registres associés à la gestion du tuyau particulier).


`dma_start(DMA_Stream_t* s, uint16_t size)` permet de démarrer la transmission, avec

- `s` : le stream renvoyé par la fonction d'initialisation
- `size` le nombre d'octets à transférer.

Si on a spécifié la mémoire en buffer circulaire ou en double buffer, **size** permet de spécifier la taille du buffer. A l'issue du transfert de *size* octets, l'évènement **Transmission Complete** est généré et permet de générer une IRQ (si on a défini la callback), puis redémarre aussitôt un nouveau transfert. Dans le cas contraire, le tuyau est automatiquement désactivé à l'issue de la transmission.

`dma_stop(DMA_Stream_t *s)` interrompt le transfert

`dma_complete(DMA_Stream_t *s)` renvoie l'information le transfert est terminé (booléen). Cette fonction permet de faire du polling pour attendre la fin de la transmission quand on utilise pas le mécanisme de callback.

1. Analyser l'API fournie pour comprendre sa logique, ainsi que son utilisation en rapport avec le périphérique USART.
2. Compléter les **DMA_Endpoints** source et destination définis dans les fonctions `uart_puts̀ et `uart_gets` qui décrivent les extrémités du tube de communication. Il faudra aussi définir les numéros de stream et de canal utilisés pour chaque cas (se référer à la partie 9.3.3 Channel selection et les tables 27 et 28).
3. Vérifier le fonctionnement.
4. Questions ouvertes :
	
	- Quel est l'intérêt de l'utilisation du DMA par rapport aux intérruptions ?
	- L'utilisation du DMA vous semble-t-elle préférable à l'utilisation des interruptions
		
		* pour l'envoi de données sur l'UART ?
		* pour la réception de données par l'UART ?
	
	