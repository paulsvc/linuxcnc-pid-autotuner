"""Internationalization (i18n) support for English (en) and French (fr)."""

from __future__ import annotations

from typing import Dict

SUPPORTED_LANGUAGES = {
    "en": "English",
    "fr": "Français",
}

TRANSLATIONS: Dict[str, Dict[str, str]] = {
    "en": {
        # Window & Menu
        "app_title": "LinuxCNC PID & Feedforward Auto-Tuner (Velocity Servo + Linear Scale)",
        "menu_file": "File",
        "menu_load_ini": "Load .INI Configuration...",
        "menu_save_ini": "Save Active Parameters to .INI...",
        "menu_export_history": "Export Attempts History (CSV / JSON)...",
        "menu_exit": "Exit",
        "menu_language": "Language / Langue",
        "lang_label": "Language:",
        # Top Toolbar
        "step1_frame": "Step 1: Axis, Motion Profile & Test Execution",
        "axis_label": "Axis:",
        "start_label": "Start (mm):",
        "btn_read_pos": "📍 Read Pos",
        "travel_label": "Travel (mm):",
        "btn_flip": "↔ Flip",
        "feed_label": "Feed (mm/min):",
        "accel_label": "Accel (mm/s²):",
        "dwell_label": "Dwell (s):",
        "btn_run_test": "▶ 1. Run Test (G-Code + Record)",
        "btn_apply_retest": "⚡ 2. Apply Suggested & Re-Test",
        "btn_autotune": "🤖 Auto-Tune (5 Attempts)",
        "btn_stop": "🛑 STOP",
        "chk_sim_mode": "Simulation Mode",
        # Left Panel: Parameters
        "step2_frame": "Step 2: PID & Feedforward Values (Current vs Suggested)",
        "hdr_param": "Parameter",
        "hdr_active": "Active Value",
        "hdr_suggested": "Suggested",
        "hdr_delta": "Delta",
        "unchanged": "(unchanged)",
        "param_P": "Pgain (Stiffness)",
        "param_I": "Igain (Steady-State)",
        "param_D": "Dgain (Damping)",
        "param_FF0": "FF0 (Position FF)",
        "param_FF1": "FF1 (Velocity FF)",
        "param_FF2": "FF2 (Accel FF)",
        "param_deadband": "Deadband (mm)",
        "param_bias": "Bias (Offset)",
        "param_maxerrorI": "MaxErrorI (mm)",
        "btn_copy_suggested": "✓ Copy Suggested -> Active",
        "btn_load_ini": "📂 Load .INI",
        "btn_save_ini": "💾 Save Active to .INI",
        # Left Panel: KPIs
        "kpi_frame": "Measured Oscilloscope KPIs (Latest Attempt)",
        "kpi_ferror_rms": "RMS ferror:",
        "kpi_ferror_max": "Max Peak |ferror|:",
        "kpi_cruise_error_mean": "Cruise Lag (FF1):",
        "kpi_accel_peak_error": "Accel Spike (FF2):",
        "kpi_overshoot_mm": "Overshoot (D/P):",
        "kpi_settling_time_ms": "Settling Time:",
        "kpi_standstill_hunting_p2p": "Standstill Hunting:",
        "kpi_output_chatter_rms": "Drive Chatter RMS:",
        "kpi_cruise_asymmetry": "+/- Asymmetry (Bias):",
        "kpi_overall_score": "Overall Quality Score:",
        # Left Panel: Rationale
        "rationale_frame": "Engineering Diagnosis & Tuning Recommendation",
        "rationale_welcome": (
            "Click '▶ 1. Run Test (G-Code + Record)' to execute the reference move, "
            "capture HAL oscilloscope signals, and compute optimal PID + FF parameters."
        ),
        "comparison_vs_attempt": (
            "Comparison vs Attempt #{prev_id}: RMS ferror changed from "
            "{rms_prev:.1f} µm -> {rms_now:.1f} µm ({impr:+.1f}% improvement)."
        ),
        # Notebook Tabs & Plots
        "tab_scope": "📈 Integrated Oscilloscope (Halscope View)",
        "tab_evolution": "📊 Evolution Across Attempts (KPIs & Parameters)",
        "tab_gcode": "📝 Test G-Code Program",
        "scope_empty_title_top": "LinuxCNC Integrated Oscilloscope — Following Error (joint.N.f-error)",
        "scope_ylabel_top": "Following Error (µm)",
        "scope_empty_title_bot": "Commanded Velocity (mm/s) & PID Drive Output",
        "scope_xlabel_bot": "Time (s)",
        "scope_ylabel_bot": "mm/s | Output",
        "scope_title_top": "Axis {axis} — Following Error `joint.{joint}.f-error` (µm)",
        "scope_title_bot": "Commanded Velocity & PID Drive Command (`pid.N.output`)",
        "phase_accel_decel": "Accel/Decel",
        "phase_cruise": "Constant Cruise (FF1)",
        "phase_settling": "Settling (P/D)",
        "evol_title_kpi": "Axis {axis} — Evolution of Following Error KPIs Across Attempts",
        "evol_ylabel_kpi": "Error (µm)",
        "evol_title_param": "Axis {axis} — Evolution of PID & Feedforward Parameters Across Attempts",
        "evol_xlabel_param": "Attempt #",
        "evol_ylabel_param": "Scaled Parameter Value",
        # History Panel
        "history_frame": "Succession of Attempts — Evolution Log (Click any row to Overlay on Scope or Restore)",
        "col_attempt": "#",
        "col_time": "Time",
        "col_axis": "Axis",
        "col_stage": "Tuning Stage",
        "col_deadband": "Deadband",
        "col_bias": "Bias",
        "col_rms_um": "RMS (µm)",
        "col_max_um": "Peak (µm)",
        "col_cruise_um": "Cruise (µm)",
        "col_accel_um": "Accel (µm)",
        "col_overshoot": "Overshoot",
        "col_settling_ms": "Settle (ms)",
        "col_chatter": "Chatter",
        "col_score": "Score",
        "btn_overlay": "🔍 Overlay Selected Attempt on Oscilloscope",
        "btn_restore": "⏪ Restore Selected Attempt Parameters",
        "btn_load_best": "🏆 Load Best Attempt So Far",
        "btn_clear_history": "🗑 Clear History",
        "btn_export_history": "📤 Export History (CSV / JSON)",
        # Status & Dialogs
        "status_ready": "Selected Axis {axis} (Limits: [{min_lim:.1f}, {max_lim:.1f} mm]). Adjust Start/Travel (mm) and click '▶ 1. Run Test'.",
        "status_running": "Running G-code test on Axis {axis} and recording HAL signals...",
        "status_attempt_done": "Attempt #{num} complete — RMS ferror: {rms:.1f} µm | Peak: {peak:.1f} µm | Score: {score:.1f}.",
        "status_overlay": "Overlaying Attempt #{num} (dashed gray line) against latest attempt on Oscilloscope.",
        "status_restored": "Restored active parameters from Attempt #{num} (RMS={rms:.1f} µm).",
        "status_best_loaded": "Loaded best attempt for Axis {axis}: Attempt #{num} (Score={score:.1f}, RMS={rms:.1f} µm).",
        "status_read_pos": "Set Start (mm) to current Axis {axis} position: {pos:.3f} mm.",
        "status_flipped": "Flipped travel direction: {start:.1f} mm -> {end:.1f} mm -> {start:.1f} mm.",
        "dlg_no_suggestion_title": "No Suggestion Yet",
        "dlg_no_suggestion_msg": "Please click '▶ 1. Run Test (G-Code + Record)' first to generate suggested values.",
        "dlg_select_attempt_title": "Select an Attempt",
        "dlg_select_attempt_msg": "Click a row in the Attempt History table first to restore its parameters.",
        "dlg_clear_title": "Clear History",
        "dlg_clear_msg": "Clear all recorded tuning attempts?",
        "dlg_soft_limit_title": "Soft Limit Protection",
        "dlg_soft_limit_ask": "{msg}\n\nAutomatically adjust Travel to {safe_travel:+.1f} mm and continue?",
    },
    "fr": {
        # Window & Menu
        "app_title": "Auto-Tuner PID & Précommande LinuxCNC (Servo Mode Vitesse + Règle Linéaire)",
        "menu_file": "Fichier",
        "menu_load_ini": "Charger un fichier .INI...",
        "menu_save_ini": "Enregistrer les paramètres dans .INI...",
        "menu_export_history": "Exporter l'historique des essais (CSV / JSON)...",
        "menu_exit": "Quitter",
        "menu_language": "Langue / Language",
        "lang_label": "Langue :",
        # Top Toolbar
        "step1_frame": "Étape 1 : Sélection de l'Axe, Profil G-Code & Lancement",
        "axis_label": "Axe :",
        "start_label": "Départ (mm) :",
        "btn_read_pos": "📍 Lire Pos",
        "travel_label": "Course (mm) :",
        "btn_flip": "↔ Inverser",
        "feed_label": "Avance (mm/min) :",
        "accel_label": "Accél (mm/s²) :",
        "dwell_label": "Pause (s) :",
        "btn_run_test": "▶ 1. Lancer Test (G-Code + Enreg.)",
        "btn_apply_retest": "⚡ 2. Appliquer & Relancer",
        "btn_autotune": "🤖 Auto-Tune (5 Essais)",
        "btn_stop": "🛑 ARRÊT",
        "chk_sim_mode": "Mode Simulation",
        # Left Panel: Parameters
        "step2_frame": "Étape 2 : Paramètres PID & Précommande (Actuels vs Suggérés)",
        "hdr_param": "Paramètre",
        "hdr_active": "Valeur Active",
        "hdr_suggested": "Suggérée",
        "hdr_delta": "Écart (Δ)",
        "unchanged": "(inchangé)",
        "param_P": "Pgain (Raideur)",
        "param_I": "Igain (Statique)",
        "param_D": "Dgain (Amortissement)",
        "param_FF0": "FF0 (Précom. Position)",
        "param_FF1": "FF1 (Précom. Vitesse)",
        "param_FF2": "FF2 (Précom. Accél.)",
        "param_deadband": "Deadband / Zone morte",
        "param_bias": "Bias / Décalage",
        "param_maxerrorI": "MaxErrorI (mm)",
        "btn_copy_suggested": "✓ Copier Suggérés -> Actifs",
        "btn_load_ini": "📂 Charger .INI",
        "btn_save_ini": "💾 Sauver dans .INI",
        # Left Panel: KPIs
        "kpi_frame": "Indicateurs KPI de l'Oscilloscope (Dernier Essai)",
        "kpi_ferror_rms": "Erreur ferror RMS :",
        "kpi_ferror_max": "Erreur Crête |ferror| :",
        "kpi_cruise_error_mean": "Traînée Vitesse (FF1) :",
        "kpi_accel_peak_error": "Pic Accél. (FF2) :",
        "kpi_overshoot_mm": "Dépassement (D/P) :",
        "kpi_settling_time_ms": "Temps de stab. :",
        "kpi_standstill_hunting_p2p": "Pompage à l'arrêt :",
        "kpi_output_chatter_rms": "Bruit / Broutage RMS :",
        "kpi_cruise_asymmetry": "Asymétrie +/- (Bias) :",
        "kpi_overall_score": "Score Global Qualité :",
        # Left Panel: Rationale
        "rationale_frame": "Diagnostic Ingénieur & Explication des Modifications",
        "rationale_welcome": (
            "Cliquez sur '▶ 1. Lancer Test (G-Code + Enreg.)' pour exécuter le mouvement G-code, "
            "enregistrer l'oscilloscope HAL et calculer les nouveaux paramètres PID + FF."
        ),
        "comparison_vs_attempt": (
            "Comparaison vs Essai #{prev_id} : l'erreur ferror RMS est passée de "
            "{rms_prev:.1f} µm -> {rms_now:.1f} µm ({impr:+.1f}% d'amélioration)."
        ),
        # Notebook Tabs & Plots
        "tab_scope": "📈 Oscilloscope Intégré (Vue Halscope)",
        "tab_evolution": "📊 Évolution des Essais (KPIs & Paramètres)",
        "tab_gcode": "📝 Programme G-Code de Test",
        "scope_empty_title_top": "Oscilloscope Intégré LinuxCNC — Erreur de poursuite (joint.N.f-error)",
        "scope_ylabel_top": "Erreur de poursuite (µm)",
        "scope_empty_title_bot": "Vitesse Commandée (mm/s) & Sortie Variateur PID",
        "scope_xlabel_bot": "Temps (s)",
        "scope_ylabel_bot": "mm/s | Sortie",
        "scope_title_top": "Axe {axis} — Erreur de poursuite `joint.{joint}.f-error` (µm)",
        "scope_title_bot": "Vitesse Commandée & Commande Variateur (`pid.N.output`)",
        "phase_accel_decel": "Accél/Décél",
        "phase_cruise": "Vitesse Constante (FF1)",
        "phase_settling": "Stabilisation (P/D)",
        "evol_title_kpi": "Axe {axis} — Évolution des KPIs d'erreur au fil des essais",
        "evol_ylabel_kpi": "Erreur (µm)",
        "evol_title_param": "Axe {axis} — Évolution des paramètres PID & FF au fil des essais",
        "evol_xlabel_param": "N° d'Essai",
        "evol_ylabel_param": "Valeur Mise à l'Échelle",
        # History Panel
        "history_frame": "Succession des Essais — Historique d'Évolution (Cliquez sur une ligne pour Comparer ou Restaurer)",
        "col_attempt": "#",
        "col_time": "Heure",
        "col_axis": "Axe",
        "col_stage": "Étape de Réglage",
        "col_deadband": "Zone morte",
        "col_bias": "Bias",
        "col_rms_um": "RMS (µm)",
        "col_max_um": "Crête (µm)",
        "col_cruise_um": "Palier (µm)",
        "col_accel_um": "Accél (µm)",
        "col_overshoot": "Dépass.",
        "col_settling_ms": "Stab (ms)",
        "col_chatter": "Bruit",
        "col_score": "Score",
        "btn_overlay": "🔍 Superposer l'Essai Sélectionné sur l'Oscilloscope",
        "btn_restore": "⏪ Restaurer les Paramètres de cet Essai",
        "btn_load_best": "🏆 Charger le Meilleur Essai",
        "btn_clear_history": "🗑 Effacer l'Historique",
        "btn_export_history": "📤 Exporter l'Historique (CSV / JSON)",
        # Status & Dialogs
        "status_ready": "Axe {axis} sélectionné (Limites : [{min_lim:.1f}, {max_lim:.1f} mm]). Ajustez Départ/Course (mm) et cliquez sur '▶ 1. Lancer Test'.",
        "status_running": "Exécution du G-code sur l'Axe {axis} et enregistrement des signaux HAL...",
        "status_attempt_done": "Essai #{num} terminé — ferror RMS : {rms:.1f} µm | Crête : {peak:.1f} µm | Score : {score:.1f}.",
        "status_overlay": "Superposition de l'Essai #{num} (courbe grise pointillée) sur l'Oscilloscope.",
        "status_restored": "Paramètres restaurés depuis l'Essai #{num} (RMS={rms:.1f} µm).",
        "status_best_loaded": "Meilleur essai chargé pour l'Axe {axis} : Essai #{num} (Score={score:.1f}, RMS={rms:.1f} µm).",
        "status_read_pos": "Position de départ (mm) réglée sur la position actuelle de l'Axe {axis} : {pos:.3f} mm.",
        "status_flipped": "Direction de course inversée : {start:.1f} mm -> {end:.1f} mm -> {start:.1f} mm.",
        "dlg_no_suggestion_title": "Aucune suggestion",
        "dlg_no_suggestion_msg": "Veuillez d'abord cliquer sur '▶ 1. Lancer Test (G-Code + Enreg.)' pour générer des valeurs suggérées.",
        "dlg_select_attempt_title": "Sélectionnez un essai",
        "dlg_select_attempt_msg": "Cliquez d'abord sur une ligne du tableau d'historique pour restaurer ses paramètres.",
        "dlg_clear_title": "Effacer l'historique",
        "dlg_clear_msg": "Voulez-vous effacer tous les essais enregistrés ?",
        "dlg_soft_limit_title": "Protection des Limites de Course",
        "dlg_soft_limit_ask": "{msg}\n\nVoulez-vous ajuster automatiquement la course à {safe_travel:+.1f} mm et continuer ?",
    },
}


STAGE_TRANSLATIONS_FR: Dict[str, str] = {
    "Stability Recovery (Reduce Chatter / Buzz)": "Récupération de Stabilité (Réduction Vibration / Bruit)",
    "Anti-Hunting Protection (Linear Scale Backlash)": "Protection Anti-Pompage (Jeu / Règle Linéaire)",
    "Stage 1: Velocity Feedforward (FF1) & Bias Calibration": "Étape 1 : Précommande de Vitesse (FF1) & Calibration Bias",
    "Stage 2: Acceleration Feedforward (FF2) & Fine FF1": "Étape 2 : Précommande d'Accélération (FF2) & Affinage FF1",
    "Stage 3: Proportional (P) Stiffness & Damping Optimization": "Étape 3 : Gain Proportionnel (P - Raideur) & Amortissement (D)",
    "Stage 4: Integral (Igain), Anti-Windup & Final Polish": "Étape 4 : Gain Intégral (I), Anti-Saturation & Finition",
}


def tr_text(lang: str, key: str, **kwargs: object) -> str:
    """Return translated string for `lang` ('en' or 'fr') formatted with `kwargs`."""
    table = TRANSLATIONS.get(lang, TRANSLATIONS["en"])
    template = table.get(key, TRANSLATIONS["en"].get(key, key))
    if kwargs:
        try:
            return template.format(**kwargs)
        except Exception:
            return template
    return template


def translate_stage_name(stage: str, lang: str) -> str:
    """Translate tuning stage title into French when `lang == 'fr'`."""
    if lang == "fr":
        return STAGE_TRANSLATIONS_FR.get(stage, stage)
    return stage


def translate_rationale_line(line: str, lang: str) -> str:
    """Translate technical diagnosis lines into French when `lang == 'fr'`."""
    if lang != "fr":
        return line
    replacements = [
        ("High-frequency drive chatter detected", "Broutage haute fréquence détecté sur la commande"),
        ("Linear scale quantization or mechanical resonance is being amplified.", "La quantification de la règle linéaire ou une résonance mécanique est amplifiée."),
        ("Reduced Dgain", "Réduction de Dgain"),
        ("because D amplifies linear scale step quantization", "car le gain D amplifie les pas de la règle linéaire"),
        ("Backed off Pgain", "Réduction de Pgain"),
        ("by 22% to restore stability margin.", "de 22% pour restaurer la marge de stabilité."),
        ("Standstill hunting detected", "Pompage à l'arrêt détecté"),
        ("peak-to-peak", "crête-à-crête"),
        ("Integral action is cycling across mechanical backlash.", "L'action intégrale oscille dans le jeu mécanique."),
        ("Reduced Igain", "Réduction de Igain"),
        ("and widened deadband to", "et élargissement de la zone morte (deadband) à"),
        ("to stop backlash limit-cycling.", "pour stopper le pompage à l'arrêt."),
        ("Set deadband to", "Réglage de la zone morte (deadband) à"),
        ("(1.5x linear scale resolution) to prevent encoder quantization dither at standstill.", "(1,5x la résolution de la règle) pour éviter les micro-oscillations à l'arrêt."),
        ("Reset FF0 to 0.0 (position feedforward must be 0 on linear axes; velocity-mode servos use FF1 and FF2).", "Remise à 0.0 de FF0 (la précommande de position doit rester à 0 sur un axe linéaire ; utiliser FF1 et FF2)."),
        ("Cruise following error averaged", "Erreur de poursuite moyenne en vitesse constante :"),
        ("at ", "à "),
        ("Updated FF1", "Mise à jour de FF1"),
        ("so the velocity command matches axis speed directly.", "pour que la consigne de vitesse corresponde directement à la vitesse de l'axe."),
        ("Detected directional asymmetry", "Asymétrie directionnelle détectée"),
        ("between + and - moves", "entre les mouvements + et -"),
        ("Adjusted bias", "Ajustement du Bias"),
        ("to counter constant load/gravity.", "pour compenser la charge constante / gravité."),
        ("Raised Pgain", "Augmentation de Pgain"),
        ("to tighten position tracking.", "pour rigidifier l'asservissement de position."),
        ("Refined FF1", "Affinage de FF1"),
        ("to zero out", "pour annuler"),
        ("residual cruise lag.", "de traînée résiduelle à vitesse constante."),
        ("Accel/Decel transient lag averaged", "Retard transitoire en accél./décél. mesuré à"),
        ("Updated FF2", "Mise à jour de FF2"),
        ("to pre-compensate servo velocity-loop lag and eliminate braking overshoot.", "pour pré-compenser le retard de la boucle de vitesse et éliminer le dépassement au freinage."),
        ("Refined bias", "Affinage du Bias"),
        ("for symmetric tracking.", "pour un suivi symétrique."),
        ("Output chatter is low", "Le bruit de commande est faible"),
        ("Increased Pgain", "Augmentation de Pgain"),
        ("to reduce reversal spikes and increase stiffness.", "pour réduire les pics d'inversion et augmenter la raideur."),
        ("Added light Dgain", "Ajout d'un léger Dgain"),
        ("to damp", "pour amortir"),
        ("settling overshoot without amplifying scale quantization.", "de dépassement sans amplifier la quantification de la règle linéaire."),
        ("Micro-trimmed FF1", "Micro-ajustement de FF1"),
        ("cruise accuracy.", "de précision en palier de vitesse."),
        ("Micro-trimmed FF2", "Micro-ajustement de FF2"),
        ("ramp accuracy.", "de précision en rampe d'accélération."),
        ("Added conservative Igain", "Ajout d'un gain intégral prudent Igain"),
        ("with maxerrorI=", "avec limitation maxerrorI="),
        ("to eliminate standstill residual error safely.", "pour annuler l'erreur statique à l'arrêt sans risque de pompage."),
        ("Axis is converged", "L'axe est parfaitement réglé"),
        ("Ready to save to .INI!", "Prêt à être sauvegardé dans le fichier .INI !"),
        ("Drive saturated for", "Variateur saturé pendant"),
        ("of the move", "du mouvement"),
        ("Consider lowering feedrate/acceleration or checking OUTPUT_SCALE.", "Réduisez l'avance/accélération ou vérifiez OUTPUT_SCALE."),
    ]
    out = line
    for en_sub, fr_sub in replacements:
        out = out.replace(en_sub, fr_sub)
    return out
