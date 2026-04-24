#Projet Robotique 2-en-1 : Suiveur de Ligne & Solveur de Labyrinthe

Bienvenue sur le dépôt de notre projet de robot mobile autonome ! Ce robot polyvalent est conçu autour d'un microcontrôleur ESP32 et possède deux modes de navigation distincts : **Suiveur de Ligne (Line Follower)** et **Solveur de Labyrinthe (Maze Solver / Micromouse)**.

## 🛠️ Architecture Matérielle (Nomenclature)
* **Cerveau :** Microcontrôleur ESP32 (monté sur Shield)
* **Motorisation :** 2x Moteurs N20 avec motoréducteurs + Contrôleur de moteurs (Pont en H)
* **Capteurs Labyrinthe :** 3x Capteurs à ultrasons HC-SR04 (Gauche, Devant, Droite)
* **Capteur de Mouvement :** Gyroscope MPU6050 (pour l'odométrie et les virages précis)
* **Capteurs Ligne :** 6x capteurs infrarouges (IR)
* **Énergie :** Support 4x Piles Li-Ion + Module Buck Converter (abaisseur 5V pour la logique)
* **Interface :** Bouton poussoir (Start/Stop) et LED de statut.

---

## 🏎️ Mode 1 : Suiveur de Ligne (Line Follower)
Dans ce mode, le robot utilise son réseau de capteurs infrarouges orientés vers le sol pour détecter et suivre une ligne noire sur un fond blanc (ou inversement).
* **Contrôle PID :** Un algorithme PID (Proportionnel, Intégral, Dérivé) analyse l'erreur de positionnement de la ligne sous le robot et ajuste continuellement la vitesse des moteurs gauche et droit.
* **Fluidité :** Cela permet au robot de prendre des virages serrés de manière fluide sans saccades, en maximisant sa vitesse en ligne droite.

---

## 🧱 Mode 2 : Solveur de Labyrinthe (Maze Solver)
Dans ce mode, le robot est placé dans un labyrinthe inconnu et doit trouver son chemin de manière 100% autonome en utilisant la **règle de la main droite** (Right-Hand Rule).

### Fonctionnalités principales :
1. **Navigation "Wall-Hugger" (Asservissement PID) :** S'il y a deux murs, le robot calcule la différence entre les capteurs gauche et droit pour se centrer parfaitement. S'il n'y a qu'un seul mur, un PID asymétrique ("Single Wall") prend le relais pour suivre ce mur à une distance cible (ex: 10 cm).
2. **Virages Gyroscopiques :** Aux intersections, le robot utilise l'intégration numérique de la vitesse angulaire du gyroscope (`mpu.getGyroZ()`) pour effectuer des rotations strictes à 90° ou 180°.
3. **Sécurités Embarquées (Smart Escape) :** * *Détection de collision :* Si un capteur lit moins de 2 cm, le robot recule et pivote pour se dégager.
   * *Anti-Stagnation :* Si les moteurs tournent mais que les distances ne changent pas pendant 500ms (roues qui patinent), le robot lance une procédure de dégagement.
   * *Loop Trap :* Si le robot tourne en rond autour d'un seul mur pendant plus de 6 secondes, il force un demi-tour à 180°.

### ⚠️ AVERTISSEMENT : État actuel du code (Imperfections & Tuning)
**Le mode Solveur de Labyrinthe est fonctionnel, mais le code n'est pas encore parfait ("Plug & Play").** Le robot parvient à naviguer et à prendre ses décisions, cependant, il présente encore quelques imperfections mécaniques et logicielles liées à l'environnement physique. 

Pour que le robot soit parfaitement fluide, **le code nécessite un tuning (calibrage) rigoureux de votre part** selon votre matériel :
* **Paramètres PID (`P` et `D`) :** À ajuster en fonction du poids de votre robot et de l'adhérence de vos pneus pour éviter qu'il ne zigzague.
* **Délais d'intersection (`delay()`) :** Le temps d'avancement *avant* et *après* un virage à 90° doit être ajusté selon la taille de vos couloirs de labyrinthe. S'il tape le coin intérieur du mur, le délai doit être augmenté.
* **Friction et batterie :** La vitesse de base (`baseSpeed`) de l'ESP32 varie selon le niveau de charge des piles Li-Ion.
*Attendez-vous à faire plusieurs essais et à modifier ces variables en haut du script avant d'obtenir une course parfaite !*

---

## 🚀 Installation & Utilisation
1. Clonez ce dépôt sur votre ordinateur.
2. Ouvrez le fichier principal avec **Arduino IDE** (ou PlatformIO).
3. Assurez-vous d'avoir installé les bibliothèques suivantes via le gestionnaire de bibliothèques :
   * `MPU6050_light.h` (Pour le gyroscope)
   * `NewPing.h` (Pour les capteurs ultrasons rapides)
4. Sélectionnez le mode désiré dans le code, compilez, et téléversez sur l'ESP32.
5. Posez le robot, appuyez sur le bouton Start, éloignez vos mains et laissez la magie opérer !

---
*Projet réalisé dans le cadre de la conception d'un robot autonome. Les contributions et suggestions d'amélioration (notamment pour le tuning du PID !) sont les bienvenues.*
