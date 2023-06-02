Le WAOU est un lecteur de musique au format .WAV. Il utilise le codec audio wm8994 qui permet de faire sortie un son plus agréable que le Convertisseur Numérique Analogique de la carte d'apprentissage. De plus toutes les musiques sont stockées sur une carte micro SD. Les musiques n'ont pas besoin d'avoir de traitements préalable particulier car le programme est capable de lire la taille et la fréquence d'échantillonnage depuis l'en-tête du ficher .WAV. Il est donc possible de mettre n'importe quel morceau (tant que cela reste en rapport avec les animés, bien entendu). Le projet s'est basé sur le travail de Louis Lalay qui a été réalisé il y a 2 ans.

### _Caractéristiques:_

Le programme doit être capable de:

- Mettre en pause la musique
- La relancer
- Pouvoir parcourir la playlist dans les 2 sens
- Afficher le titre du fichier
- Afficher le timer de la musique


### _Communication:_ 

La communication avec le codec audio est fait par liaison SAI. Cette liaison est similaire à la liaison I2S qui est parfois utilisés par d'autres appareils audios.

### _Périphériques utilisés:_

- Un Codec audio wm8994
- Ecran LCD Tactile
- Le lecteur de SD Card

### _Bibliothèques utilisés:_ 

La librairy BSP_AUDIO a été utile dans le projet pour gérer la communication avec le codec Audio. Celle de BSP_LCD à permit d'interagir avec l'écran tactile.

_### Fonctions crées:_

- Plusieurs fonctions pour dessiner les boutons sur l'écran
- Une fonction permettant de lire l'en-tête d'un .WAV pour lire la fréquence d'échantillonnage , la taille du fichier et le nombre d'octet par seconde. 
- Une fonction pour initialiser le codec audio
- Une fonction pour charger un fichier depuis une SD Card
- Une fonction pour initialiser la SD Card 
- Une fonction pour initialiser l'écran

### _Tâches:_

- Une tâche pour initialiser la SD Card (détruite après une première itération)
- Une tâche qui gère l'affichage
- Une tâche qui charge les données de la SD Card


### _Difficultés rencontrées:_

- Difficulté de prise en main de la SD Card
- Impossibilité de reproduire le TP DAC avec le codec audio 
- Petit bug lors lorsque l'on parcourt la playlist, la lecture de l'en-tête donne de fausses valeurs

### _Améliorations possibles:_

- Rajouter un interface graphique plus accueillante
- Régler les problèmes de lecture d'en-tête
- Pourvoir mettre n'importe quel nom de musique
- Ne pas limiter le nombre de morceaux à lire
