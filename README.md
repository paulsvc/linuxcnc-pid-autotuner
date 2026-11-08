# LinuxCNC PID & Feedforward Auto-Tuner

**[🇫🇷 Version Française](#-version-française) | [🇬🇧 English Version](#-english-version) | [🇩🇪 Deutsche Version](#-deutsche-version)**

---

## 🇫🇷 Version Française

Interface graphique légère (**Tkinter + Matplotlib**) conçue pour le réglage automatique des boucles **PID + Précommande (`FF0`, `FF1`, `FF2`, `deadband`, `bias`, `maxerrorI`)** sous **LinuxCNC**, particulièrement adaptée aux **servomoteurs en mode vitesse avec retour par règle linéaire**.

Le logiciel génère un programme G-code d'essai aller-retour sécurisé, enregistre les signaux temps réel de l'oscilloscope HAL (`joint.N.f-error`, `joint.N.pos-cmd`, `joint.N.pos-fb`, `joint.N.vel-cmd`, `pid.N.output`), analyse plus de 10 indicateurs de performance (KPIs) par phase de mouvement et propose automatiquement de nouveaux paramètres tout en **enregistrant la succession des essais** pour visualiser l'évolution des réglages.

### 1. Installation Rapide sur LinuxCNC (Debian Bookworm / Bullseye)

Installez les paquets Python standards requis :
```bash
sudo apt update
sudo apt install -y git python3-tk python3-numpy python3-matplotlib
```

Clonez le dépôt GitHub et lancez l'application :
```bash
git clone https://github.com/paulsvc/linuxcnc-pid-autotuner.git
cd linuxcnc-pid-autotuner
python3 main.py
```
*(Astuce : l'interface démarre en **Français** par défaut. Vous pouvez changer la langue à tout moment via le menu **`Langue / Language / Sprache`** ou avec `python3 main.py --lang de` ou `python3 main.py --lang en`. Si LinuxCNC n'est pas lancé, le logiciel bascule automatiquement en **Mode Simulation** avec un modèle physique à 1 kHz).*

### 2. Utilisation en 3 Étapes Simples

1. **Sélection de l'Axe et Réglage de la Course (Barre supérieure)**
   - Choisissez l'axe à régler (`X`, `Y`, `Z` ou `A`).
   - **Protection contre les fins de course :** Placez votre axe dans une zone dégagée et cliquez sur **`📍 Lire Pos`** pour définir votre position actuelle comme point de départ (`Départ (mm)`).
   - Réglez la **`Course (mm)`** (ex. `20 mm` ou `-20 mm`) et cliquez sur **`↔ Inverser`** si vous souhaitez inverser le sens du mouvement. Le logiciel vérifie automatiquement les limites logicielles (`MIN_LIMIT` / `MAX_LIMIT`) avant tout départ.

2. **Lancer le G-Code et l'Analyse (`▶ 1. Lancer Test (G-Code + Enreg.)`)**
   - Exécute l'aller-retour avec pauses de stabilisation, enregistre les signaux HAL, découpe le mouvement par phases (**Arrêt**, **Accélération**, **Vitesse Constante**, **Décélération**, **Stabilisation**) et affiche les **Valeurs Suggérées** à côté des **Valeurs Actives**.

3. **Appliquer et Suivre l'Évolution (`⚡ 2. Appliquer & Relancer` ou `🤖 Auto-Tune (5 Essais)`)**
   - Applique les valeurs suggérées dans HAL et relance immédiatement un essai pour observer l'amélioration.
   - Tous les essais sont enregistrés dans le tableau **Succession des Essais — Historique d'Évolution** en bas de l'écran.
   - **Cliquez sur n'importe quel essai de l'historique** pour superposer sa courbe `ferror` (en pointillés) sur l'oscilloscope, ou cliquez sur **`⏪ Restaurer les Paramètres de cet Essai`** / **`🏆 Charger le Meilleur Essai`**.
   - Une fois satisfait, cliquez sur **`💾 Sauver dans .INI`** (une sauvegarde `.ini.bak.<date>` est créée automatiquement).

### 3. Indicateurs KPI Évalués & Paramètres Associés

| Indicateur KPI | Phase du Mouvement | Paramètre Réglé |
| :--- | :--- | :--- |
| **Erreur `ferror` RMS & Crête** | Cycle complet | Raideur globale de poursuite (**`Pgain`**, **`FF1`**, **`FF2`**) |
| **Traînée en Vitesse Constante** | Palier à vitesse constante | **`FF1`** (Précommande de vitesse) |
| **Asymétrie Directionnelle (+/-)** | Palier $+$ vs Palier $-$ | **`bias`** (Compensation gravité / charge verticale, ex. axe Z) |
| **Pics d'Accélération / Décélération** | Rampes d'accél. et décél. | **`FF2`** (Précommande d'accélération — élimine le dépassement au freinage) |
| **Dépassement (Overshoot) & Temps de Stab.** | Fin de décélération | **`Dgain`** (Amortissement) & **`Pgain`** |
| **Bruit / Broutage Variateur RMS** | Cycle complet (`pid.output`) | Détecte l'amplification des pas de la règle linéaire ; réduit **`Dgain` / `Pgain`** |
| **Pompage à l'Arrêt (Crête-à-Crête)** | Pause / Arrêt | Détecte l'oscillation dans le jeu mécanique ; ajuste **`deadband`** & réduit **`Igain`** |
| **Erreur Statique à l'Arrêt** | Pause / Arrêt | **`Igain`** + **`maxerrorI`** (Limitation anti-windup) |

---

## 🇬🇧 English Version

A lightweight **Tkinter + Matplotlib GUI** designed for automatic tuning of **PID + Feedforward (`FF0`, `FF1`, `FF2`, `deadband`, `bias`, `maxerrorI`)** loops in **LinuxCNC**, specifically tailored for **velocity-mode servo drives with linear scale feedback**.

The software generates a safe back-and-forth test G-code move, records real-time HAL oscilloscope signals (`joint.N.f-error`, `joint.N.pos-cmd`, `joint.N.pos-fb`, `joint.N.vel-cmd`, `pid.N.output`), evaluates 10+ control engineering KPIs across motion phases, and recommends updated parameters while **recording the succession of attempts** to track parameter and performance evolution.

### 1. Quick Start on LinuxCNC (Debian Bookworm / Bullseye)

Install the required standard Python packages:
```bash
sudo apt update
sudo apt install -y git python3-tk python3-numpy python3-matplotlib
```

Clone from GitHub and launch the GUI:
```bash
git clone https://github.com/paulsvc/linuxcnc-pid-autotuner.git
cd linuxcnc-pid-autotuner
python3 main.py --lang en
```
*(Tip: You can switch between **English**, **Français**, and **Deutsch** anytime via the **`Language / Langue / Sprache`** top menu or toolbar dropdown. If LinuxCNC is not currently running, the GUI automatically starts in **Simulation Mode** with a built-in 1 kHz physics model).*

### 2. How to Tune an Axis in 3 Clicks

1. **Select Axis & Safe Travel Settings (Top Toolbar)**
   - Select your target axis (`X`, `Y`, `Z`, or `A`).
   - **Soft-Limit Protection:** Jog your machine to a safe position and click **`📍 Read Pos`** to set the current machine coordinate as `Start (mm)`.
   - Set **`Travel (mm)`** (e.g., `20 mm` or `-20 mm`) and use **`↔ Flip`** to reverse direction if you are near a travel limit. Before moving, the software automatically validates `[Start, Start + Travel]` against your LinuxCNC soft limits (`MIN_LIMIT` / `MAX_LIMIT`).

2. **Run Test & Analyze (`▶ 1. Run Test (G-Code + Record)`)**
   - Executes the back-and-forth trapezoidal move with dwell pauses, records HAL signals, segments the move into **Standstill**, **Acceleration**, **Constant Cruise**, **Deceleration**, and **Settling**, and displays **Suggested Values** next to your **Active Values**.

3. **Apply & Track Evolution (`⚡ 2. Apply Suggested & Re-Test` or `🤖 Auto-Tune (5 Attempts)`)**
   - Applies the recommended parameters to HAL and immediately runs the next attempt so you can watch the improvement.
   - Every run is logged in the **Succession of Attempts — Evolution Log** table at the bottom.
   - **Click any row in the Attempt History table** to overlay its `ferror` trace (dashed curve) on the oscilloscope against the latest attempt, or click **`⏪ Restore Selected Attempt Parameters`** / **`🏆 Load Best Attempt So Far`**.
   - Once satisfied, click **`💾 Save Active to .INI`** (automatically creates a timestamped `.ini.bak` backup before updating your `.ini` file).

### 3. Evaluated KPIs & Tuned Parameters

| KPI | Motion Phase | Parameter Tuned |
| :--- | :--- | :--- |
| **RMS & Peak `ferror`** | Full Cycle | Overall tracking stiffness (**`Pgain`**, **`FF1`**, **`FF2`**) |
| **Cruise Lag (`cruise_error_mean`)** | Constant Velocity | **`FF1`** (Velocity Feedforward) |
| **Directional Asymmetry (+/-)** | $+$ Cruise vs $-$ Cruise | **`bias`** (Gravity / directional load compensation, e.g., Z axis) |
| **Acceleration / Deceleration Spikes** | Accel & Decel Ramps | **`FF2`** (Acceleration Feedforward — eliminates braking overshoot) |
| **Overshoot & Settling Time** | Post-Decel Settling | **`Dgain`** (Damping) & **`Pgain`** |
| **Drive Output Chatter RMS** | Full Cycle (`pid.output`) | Detects linear-scale quantization buzz; backs off **`Dgain` / `Pgain`** |
| **Standstill Hunting Peak-to-Peak** | Dwell / Standstill | Detects backlash limit-cycling; widens **`deadband`** & reduces **`Igain`** |
| **Steady-State Error** | Dwell / Standstill | **`Igain`** + **`maxerrorI`** (Anti-windup clamp) |

---

## 🇩🇪 Deutsche Version

Schlanke grafische Benutzeroberfläche (**Tkinter + Matplotlib**) zur automatischen Optimierung von **PID- und Vorsteuerungsregelkreisen (`FF0`, `FF1`, `FF2`, `deadband`, `bias`, `maxerrorI`)** in **LinuxCNC**, speziell ausgelegt für **Drehzahl-Servoverstärker mit Linearmaßstab-Rückführung**.

Die Software generiert ein sicheres G-Code-Hin- und Rückfahrprogramm, zeichnet HAL-Oszilloskopsignale in Echtzeit auf (`joint.N.f-error`, `joint.N.pos-cmd`, `joint.N.pos-fb`, `joint.N.vel-cmd`, `pid.N.output`), wertet über 10 regelungstechnische Kennzahlen (KPIs) je Bewegungsphase aus und schlägt verbesserte Parameter vor. Gleichzeitig wird eine **Reihe von Versuchen protokolliert**, um den Verlauf und die Konvergenz der Regelung direkt nachzuvollziehen.

### 1. Schnellstart auf LinuxCNC (Debian Bookworm / Bullseye)

Erforderliche Standard-Python-Pakete installieren:
```bash
sudo apt update
sudo apt install -y git python3-tk python3-numpy python3-matplotlib
```

Repository von GitHub klonen und GUI starten:
```bash
git clone https://github.com/paulsvc/linuxcnc-pid-autotuner.git
cd linuxcnc-pid-autotuner
python3 main.py --lang de
```
*(Tipp: Sie können jederzeit über das Menü **`Sprache / Language / Langue`** oder das Dropdown oben rechts zwischen **Deutsch**, **Französisch** und **Englisch** wechseln. Läuft LinuxCNC gerade nicht, startet automatisch der integrierte **Simulationsmodus** mit einem 1-kHz-Physikmodell).*

### 2. Achse einstellen in 3 einfachen Schritten

1. **Achse wählen & Verfahrweg einstellen (Obere Leiste)**
   - Wählen Sie die Zielachse (`X`, `Y`, `Z` oder `A`).
   - **Software-Endlagenschutz:** Fahren Sie die Achse in einen freien Bereich und klicken Sie auf **`📍 Pos lesen`**, um die aktuelle Koordinate als Startpunkt (`Start (mm)`) zu übernehmen.
   - Stellen Sie den **`Weg (mm)`** ein (z. B. `20 mm` oder `-20 mm`) und nutzen Sie **`↔ Richtung umkehren`**, falls Sie nahe an einer Endlage stehen. Die Software prüft vor jedem Start automatisch die LinuxCNC-Softwaregrenzen (`MIN_LIMIT` / `MAX_LIMIT`).

2. **G-Code-Test starten & auswerten (`▶ 1. Test starten (G-Code + Aufn.)`)**
   - Führt die Hin- und Rückfahrt mit Einschwingpausen aus, zeichnet HAL-Signale auf, segmentiert die Bewegung in Phasen (**Stillstand**, **Beschleunigung**, **Konstantfahrt**, **Verzögerung**, **Einschwingen**) und zeigt die **Vorgeschlagenen Werte** neben den **Aktuellen Werten** an.

3. **Werte übernehmen & Verlauf verfolgen (`⚡ 2. Übernehmen & Neustart` oder `🤖 Auto-Tune (5 Versuche)`)**
   - Schreibt die vorgeschlagenen Werte direkt in HAL und startet sofort den nächsten Versuch, um die Verbesserung zu prüfen.
   - Alle Durchläufe werden in der Tabelle **Versuchsreihe — Verlaufsprotokoll** unten protokolliert.
   - **Klicken Sie auf eine beliebige Zeile im Verlauf**, um die Schleppfehlerkurve (gestrichelt) im Oszilloskop zum direkten Vergleich zu überlagern, oder klicken Sie auf **`⏪ Parameter dieses Versuchs wiederherstellen`** / **`🏆 Besten Versuch laden`**.
   - Nach erfolgreicher Optimierung klicken Sie auf **`💾 In .INI speichern`** (eine Sicherungskopie `.ini.bak.<Zeitstempel>` wird automatisch erstellt).

### 3. Bewertete Kennzahlen (KPIs) & zugehörige Parameter

| Kennzahl (KPI) | Bewegungsphase | Optimierter Parameter |
| :--- | :--- | :--- |
| **RMS- & Max.-Schleppfehler** | Gesamter Zyklus | Gesamte Führungssteifigkeit (**`Pgain`**, **`FF1`**, **`FF2`**) |
| **Schleppfehler bei Konstantfahrt** | Konstantfahrt | **`FF1`** (Geschwindigkeits-Vorsteuerung) |
| **Richtungsasymmetrie (+/-)** | Vor- vs. Rückfahrt | **`bias`** (Schwerkraft- / Vertikallastkompensation, z. B. Z-Achse) |
| **Beschleunigungs- & Bremsspitzen** | Rampen (Beschl./Verzög.) | **`FF2`** (Beschleunigungs-Vorsteuerung — verhindert Bremsüberschwinger) |
| **Überschwingen & Einschwingzeit** | Nach Verzögerungsende | **`Dgain`** (Dämpfung) & **`Pgain`** |
| **Regelungsrauschen RMS** | Gesamter Zyklus (`pid.output`) | Erkennt Maßstabsquantisierungsrauschen; nimmt **`Dgain` / `Pgain`** zurück |
| **Stillstand-Pendeln (Spitze-Spitze)** | Pause / Stillstand | Erkennt Schwingen im Umkehrspiel; passt **`deadband`** an & senkt **`Igain`** |
| **Statischer Schleppfehler** | Pause / Stillstand | **`Igain`** + **`maxerrorI`** (Anti-Windup-Begrenzung) |
