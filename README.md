<p align="center">
  <img src="https://github.com/user-attachments/assets/7452899d-4287-4e80-9178-42f58f441bcc" alt="1" />
</p>

# ⚡ Crazy Scope

**Crazy Scope** est un projet d'oscilloscope dual channel Wi-Fi open-source, propulsé par un simple **ESP32**.
Grâce à l'utilisation de l'ADC en mode continu (I2S/DMA) et à une transmission binaire optimisée via WebSockets, il permet de visualiser des signaux en temps réel directement depuis un navigateur web, que ce soit sur PC, tablette ou smartphone.

## ✨ Fonctionnalités

* **Acquisition Dual Channel** : CH1 (GPIO34) et CH2 (GPIO35) avec calibration matérielle intégrée (eFuses).
* **Haute Performance** : Acquisition gérée par DMA pour éviter tout saut d'échantillon, avec des presets allant jusqu'à 500 kHz (single channel) ou 100 kHz (dual channel) réels.
* **Interface Web Fluide** : Rendu HTML5 Canvas, mode sombre/clair, et mesures automatiques (Vpp, Vmoy, Vrms, Fréquence).
* **Triggers Avancés** : Modes Auto, Normal, et Single, avec sélection du front (montant/descendant) et du niveau.
* **Connectivité Intelligente** : Connexion au réseau Wi-Fi local (STA) ou création automatique d'un point d'accès (AP) en cas d'échec. Accès facile via `http://crazyscope.local` (mDNS).
* **Panel d'Administration** : Interface dédiée pour la configuration Wi-Fi, la gestion des fichiers sur la mémoire flash (LittleFS), et les mises à jour du firmware (OTA).

## 🛠️ Matériel Requis

* 1x **ESP32 Dev Module** (ex: ESP32-WROOM-32)
* **Étage Analogique (recommandé)** : L'ESP32 tolère 0..3.3V en entrée. Le firmware est calibré pour un étage d'entrée utilisant un pont diviseur 90kΩ / 10kΩ associé à un offset de 1.65V grâce à une pull-up de 10kΩ vers le 3.3V.
    * *Ceci permet de mesurer des tensions alternatives jusqu'à 30V, centrées autour de 0V.*

**Brochage par défaut :**
* `GPIO34` : Entrée Canal 1 (ADC1_CH6)
* `GPIO35` : Entrée Canal 2 (ADC1_CH7)
* `GPIO2`  : LED de statut (intégrée)

## 💻 Logiciel & Dépendances

Ce projet est développé pour l'IDE Arduino. Le Core ESP32 **v3.0 ou supérieur** est requis.

**Bibliothèques tierces à installer via le Library Manager :**
* `ESP Async WebServer` (par ESP32Async)
* `Async TCP` (par ESP32Async)
* `ArduinoJson`

## 🚀 Installation

1.  Cloner le dépôt ou télécharger le code source.
2.  Ouvrir `CrazyScope.ino` dans l'IDE Arduino.
3.  Compiler et uploader le firmware sur la board ESP32.
4.  **Première connexion** : L'ESP32 va créer un réseau Wi-Fi ouvert nommé `CrazyScope-XXXXXX`. Après la connexion, uploader les pages Web en ligne de commande : 
   ```bash
   curl -F "filename=@filesystem/index.html" http://192.168.4.1/api/upload
   curl -F "filename=@filesystem/admin.html" http://192.168.4.1/api/upload
   ```
5.  Aller sur `http://192.168.4.1/admin.html` pour configurer un Wi-Fi domestique.

## 🕹️ Utilisation

* **Interface Principale** : Accéder à `http://crazyscope.local` (ou directement l'IP locale). S'y trouvent l'écran de l'oscilloscope, les réglages de la base de temps, des calibres (V/div), et les triggers.
* **Administration** : Cliquer sur le bouton d'engrenage pour afficher l'état du système, configurer le Wi-Fi, uploader des fichiers ou mettre à jour le firmware à distance (OTA).

## 📄 Licence

Distribué sous la licence MIT.
Voir la `LICENSE` pour plus d'informations.
