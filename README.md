# LinuxCNC PID & Feedforward Auto-Tuner

A lightweight **Tkinter + Matplotlib GUI** designed for **LinuxCNC velocity-mode servo drives with linear scale feedback**.

It automatically generates a back-and-forth reference G-code move, records real-time HAL oscilloscope signals (`joint.N.f-error`, `joint.N.pos-cmd`, `joint.N.pos-fb`, `joint.N.vel-cmd`, `pid.N.output`), evaluates 10+ control engineering KPIs across motion phases, and recommends updated **`P`, `I`, `D`, `FF0`, `FF1`, `FF2`, `deadband`, `bias`, and `maxerrorI`** values while tracking the **evolution across multiple tuning attempts**.

---

## Quick Start on LinuxCNC (Debian Bookworm / Bullseye)

### 1. Install System Packages
LinuxCNC's Python environment requires `python3-tk`, `python3-matplotlib`, and `python3-numpy`:
```bash
sudo apt update
sudo apt install -y git python3-tk python3-numpy python3-matplotlib
```

### 2. Clone from GitHub & Run
```bash
git clone https://github.com/paulsvc/linuxcnc-pid-autotuner.git
cd linuxcnc-pid-autotuner
python3 main.py
```
*(If LinuxCNC is not currently running, the GUI automatically starts in **Simulation Mode** with a built-in 1 kHz physics model of a velocity-mode servo + ballscrew backlash + $1\ \mu\text{m}$ quantized linear scale so you can test the workflow offline).*

---

## How to Tune an Axis in 3 Clicks

1. **Select Axis & Move Settings (Top Bar)**
   - Pick your axis (`X`, `Y`, `Z`, or `A`).
   - Set a safe **Travel (mm)** (e.g., `50 mm`) and **Feed (mm/min)** (e.g., `1500 mm/min`).
   - Make sure the axis is positioned with clear travel in the $+$ direction and LinuxCNC Machine Power is **ON**.

2. **Click `▶ 1. Run Test (G-Code + Record)`**
   - The software executes a back-and-forth trapezoidal move with dwell pauses, records the HAL signals, segments the move into **Standstill**, **Acceleration**, **Constant-Velocity Cruise**, **Deceleration**, and **Settling**, and displays the **Suggested Values** next to your **Active Values**.

3. **Click `⚡ 2. Apply Suggested & Re-Test` (or `🤖 Auto-Tune (5 Attempts)`)**
   - Applies the recommended parameters to HAL and immediately runs the next attempt.
   - Every attempt is logged in the **Succession of Attempts — Evolution Log** table at the bottom.
   - **Click any row in the Attempt History table** to overlay its `ferror` trace (dashed curve) on the oscilloscope against the latest attempt, or click **`⏪ Restore Selected Attempt Parameters`** / **`🏆 Load Best Attempt So Far`**.
   - Once satisfied, click **`💾 Save Active to .INI`** (automatically creates a `.ini.bak.<timestamp>` backup before updating your `.ini` file).

---

## KPIs Evaluated by the Analyzer

| KPI | Motion Phase | Parameter Tuned |
| :--- | :--- | :--- |
| **RMS & Peak `ferror`** | Full Cycle | Overall tracking stiffness (`Pgain`, `FF1`, `FF2`) |
| **Cruise Lag (`cruise_error_mean`)** | Constant Velocity | **`FF1`** (Velocity Feedforward) |
| **Directional Asymmetry** | $+$ Cruise vs $-$ Cruise | **`bias`** (Gravity / directional load compensation, e.g., Z axis) |
| **Acceleration / Deceleration Spikes** | Accel & Decel Ramps | **`FF2`** (Acceleration Feedforward) |
| **Overshoot & Settling Time** | Post-Decel Settling | **`Dgain`** (Damping) & **`Pgain`** |
| **Drive Output Chatter RMS** | Full Cycle (2nd diff of `pid.output`) | Detects linear-scale quantization buzz; backs off **`Dgain` / `Pgain`** |
| **Standstill Hunting Peak-to-Peak** | Dwell / Standstill | Detects backlash limit-cycling; widens **`deadband`** & reduces **`Igain`** |
| **Steady-State Error** | Dwell / Standstill | **`Igain`** + **`maxerrorI`** (Anti-windup clamp) |
