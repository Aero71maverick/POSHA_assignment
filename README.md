# POSHA Systems Engineering Internship Assignment

**Kunal Paroda** · Roll No: 24B0071  
2nd Year, B.Tech. Aerospace Engineering — IIT Bombay  
Submitted to: POSHA Engineering Team · April 9, 2026

---

## Overview

This repository contains the complete submission for the POSHA Systems Engineering internship assignment, covering three interconnected tasks around servo motor testing, reliability analysis, and robot system design.

| Task | Title |
|------|-------|
| Task 1 | Servo Motor QC Jig Design |
| Task 2 | Servo System Reliability & Safety Analysis |
| Task 3 | POSHA System Design Improvements |

---

## Task 1: Servo Motor QC Jig Design

A custom quality-control test jig designed to characterise servo motors — primarily the **Dynamixel XL430-W250-T** — across their full operating envelope: stall torque, no-load speed, efficiency, and current draw at multiple supply voltages.

### Key Design Decisions

- **Target servo:** Dynamixel XL430-W250-T (1.5 N·m stall @ 12 V, 4096 counts/rev absolute encoder, TTL serial bus)
- **Load mechanism:** JGA25-370 DC gear motor with PI current-control loop for precise, commanded torque
- **Frame:** Aluminium 2020 T-slot extrusion (vs. DIN rail) — adjustable, corrosion-resistant, handles stall reaction loads
- **Torque measurement:** Dual load cells (HX711, 24-bit ADC) in a differential configuration that cancels gravitational bias
- **Speed/position:** AS5600 magnetic rotary encoder (12-bit, contactless, I²C)
- **Controller:** Arduino Uno (5 V native — avoids 3.3 V compatibility issues with HX711, ACS712, and servo signal lines)

### CAD Model & Assembly

The full jig was designed in **Fusion 360** as a parametric, jointed assembly. All components are linked by kinematic joints — not just static positions — so the gear mesh, encoder sweep, and reaction arm deflection can all be verified in simulation before physical assembly.

> **📹 CAD Joint Motion Video**  
> *[Insert screen recording here — drag the servo horn in Fusion 360 to show the full kinematic chain: gear mesh → encoder magnet sweep → load cell arms deflecting]*  
> Suggested clip: ~30–60 seconds, exported as MP4 or GIF, showing the jig scrubbing through full servo angular travel.  
> To record: Fusion 360 → `File` → `New Animation` → scrub timeline → `Publish Video`.

![Jig Assembly Isometric](assets/jig_cad.png)
*Full jig assembly at nominal position.*

![No-Load Configuration](assets/collage.png)
*Motor-side gearbox half disengaged — no-load test configuration.*

### FOC Approach Considered and Rejected

A BLDC + field-oriented control approach was sketched (using ODrive/VESC) that would have swept the full performance curve autonomously in ~60 seconds. It was rejected on three grounds:

1. **Cost** — ODrive 3.6 alone costs ₹8,000–12,000; the full DC motor + load cell approach costs under ₹3,000
2. **Calibration bootstrap** — Kt of the BLDC still needs a reference load cell to calibrate, eliminating the key advantage
3. **Systematic error** — a mis-tuned FOC loop introduces unobservable torque error; a load cell is an independent physical measurement

### Electronics

Two separate current measurement strategies are used deliberately:

| Measurement | Sensor | Reason |
|---|---|---|
| Servo current (0–2 A, DUT) | INA219 (shunt, I²C) | High precision, simultaneous voltage → on-chip power calc |
| DC motor current (module) | ACS712 + ADS1115 | Electrically isolated — motor switching noise can't corrupt logic rail |
| DC motor current (PCB) | 2nd INA219 @ 0x41 | Full digital path, eliminates analogue signal routing |

All three I²C devices share SDA/SCL (A4/A5) with unique addresses: AS5600 @ 0x36, INA219 @ 0x40, ADS1115 @ 0x48.

### Firmware

Two firmware variants are provided:

- **`firmware/module_version/`** — Arduino Uno with all breakout modules (ACS712 + ADS1115 for motor current)
- **`firmware/pcb_version/`** — ATmega328P with discrete ICs (dual INA219, fully digital)

Both stream a timestamped CSV over Serial at 20 Hz:

```
t_ms, enc_raw, speed_dps, F1_N, F2_N, torque_Nm, servo_mA, servo_mV, motor_A, tau_cmd_Nm
```

The DC motor torque control loop uses a **PI current controller** (Kp = 8.0, Ki = 0.5) with anti-windup. The outer sweep advances the torque setpoint by 0.05 N·m every 500 ms; the inner loop converges in under 200 ms.

### Bill of Materials

Total build cost (module version, excluding Dynamixel DUT): **≈ ₹4,550**

| Item | Qty | Unit (INR) |
|------|-----|-----------|
| DS3218 servo (DUT proxy) | 1 | ₹900 |
| JGA25-370 DC gear motor | 1 | ₹450 |
| 2020 extrusion, 500 mm × 4 | 4 | ₹120 |
| 50 kg load cells (matched pair) | 2 | ₹180 |
| HX711 24-bit ADC module | 1 | ₹80 |
| AS5600 encoder module | 1 | ₹150 |
| INA219 module | 1 | ₹80 |
| ACS712 (5 A) module | 1 | ₹70 |
| ADS1115 16-bit ADC | 1 | ₹150 |
| Arduino Uno R3 | 1 | ₹500 |
| BTS7960 H-bridge | 1 | ₹180 |
| 12 V 5 A PSU | 1 | ₹600 |
| 3D-printed parts (PETG) | — | ₹360 |
| Hardware, wiring, consumables | — | ₹510 |

PCB upgrade path (replacing breakout modules): **≈ ₹1,000 additional**

### Calibration

Load cells are calibrated using slotted disc weights (100 g → 2000 g) and a least-squares regression:

$$\text{scale} = \frac{\sum (R_i - \bar{R})(F_i - \bar{F})}{\sum (R_i - \bar{R})^2}$$

Linearity is verified with a one-sample t-test (α = 0.05, n = 5, t_crit = 2.776). Scale factors are stored in EEPROM.

---

## Task 2: Servo System Reliability & Safety

### Failure Modes

| Failure Mode | Root Cause | Detection on Jig | Mitigation |
|---|---|---|---|
| Position sensor wear | Potentiometer wiper erosion | Jitter / dead spots in position output | Magnetic encoder (Dynamixel XL430) |
| Motor winding burnout | Sustained stall current | ↑ no-load current, ↓ stall torque | Stall watchdog (200 ms timeout) |
| Gear backlash | Plastic tooth surface deformation | Hysteresis in oscillation test (> 0.5°) | Metal gear upgrade |
| PWM noise corruption | Commutator switching EMI | Erratic position steps | Shielded cable + 100 nF bypass cap |
| Voltage rail droop | Simultaneous servo actuation | Voltage dip at connector | 470–1000 µF bulk cap per connector |

### Debugging Flow

1. **Isolate mechanically** — run in free air; problem disappears → mechanical binding in robot
2. **Check signal** — oscilloscope on signal pin; noise present → cable routing or shielding
3. **Check supply voltage under load** — > 0.5 V drop → cable too thin or missing bulk cap
4. **Check internal sensors** — no-load current vs. datasheet; elevated → shorted winding turn
5. **Check backlash on jig** — oscillation test between ±5° target angles

### Improvement Suggestions

- **Metal output gears** for any joint running near stall repeatedly (brass/steel yield strength ~10× ABS)
- **470 µF bulk capacitor** at each servo connector
- **Stall watchdog in firmware** — cut torque if position error > threshold for > 200 ms
- **PTC resettable fuse** on each servo power line
- **Migrate to Dynamixel TTL serial** — eliminates PWM noise sensitivity entirely; adds real-time position, velocity, current, and temperature telemetry

---

## Task 3: POSHA System Design Improvements

### Reducing Cooking Time

| Technique | Mechanism | Implementation cost |
|---|---|---|
| **Temperature overshoot** | Pre-store thermal energy before liquid addition; faster phase transition | Software only (thermal model + recipe scheduler) |
| **Pre-heat before user confirms** | Begin heating as soon as recipe is selected | Recipe parser + UI confirmation prompt |
| **Overlap dispense + heating** | Dispense water/stock while heating to next setpoint | Recipe-specific flag for temperature-sensitive ingredients |
| **Torque-based stir frequency** | Motor current = resistance proxy; stir only when needed | Monitor stirring motor current; adaptive threshold |

### Physical & UX Improvements

**Spice dispenser:**
- Single-use sealed cartridges (solenoid/cam puncture) — eliminates cross-contamination and wash cycles
- Alternative: vibratory feeder + load cell for closed-loop mass dispensing (standard in pharmaceutical manufacturing)

**Accessibility:**
- All user interfaces at hip-to-chest height
- Front-loading spice drawer (printer tray style) with positive click latch
- Split-unit architecture — computation/power electronics shelf-mounted separately from the cooking module (same principle as split-system AC)
- Direct tap connection for water inlet, eliminating manual tank fills

**Adaptive recipe learning:**
- Log actual time-to-temperature for each thermal milestone
- Compare against recipe nominal timing
- Adjust preheat and overshoot parameters for subsequent runs
- After 3–4 runs: kitchen-specific thermal model (ambient temperature, altitude, vessel condition)
- Runs entirely on-device — no cloud connectivity required

---

## Repository Structure

```
posha-internship/
├── README.md
├── FInal_submission.pdf          # Full written report
├── assets/
│   ├── jig_cad.png               # Full assembly isometric
│   ├── collage.png               # No-load configuration
│   ├── schematic.png             # Module-based wiring schematic
│   ├── perf_image.jpg            # OEM performance graph
│   ├── bldc_based_approach.jpeg  # FOC concept sketch
│   └── videos/
│       └── cad_joint_motion.mp4  # ← ADD: Fusion 360 kinematic demo
├── firmware/
│   ├── module_version/
│   │   └── qc_jig_module.ino     # Arduino Uno + breakout modules
│   └── pcb_version/
│       └── qc_jig_pcb.ino        # ATmega328P + discrete ICs
├── cad/
│   ├── jig_assembly.f3d          # Fusion 360 parametric assembly
│   └── stl/
│       ├── servo_mount.stl
│       ├── output_horn.stl
│       ├── encoder_bracket.stl
│       ├── gearbox_half_output.stl
│       └── gearbox_half_motor.stl
└── pcb/
    └── pcb_explainer.pdf         # IC selection rationale & layout notes
```

---

## How to Run the Jig

1. **Build the frame** — assemble 2020 extrusion with corner brackets; mount servo, gearbox, and load cells
2. **Print parts** — export STLs from Fusion 360; print in PETG at ≥ 40% infill
3. **Wire electronics** — follow schematic in `assets/schematic.png`; verify I²C addresses with `i2c_scanner.ino`
4. **Calibrate** — run the tare routine; hang weights 100 g → 2000 g; store scale factors in EEPROM
5. **No-load test** — disengage motor-side gearbox half; run firmware; verify speed curve vs. OEM
6. **Load sweep** — re-engage gearbox; firmware auto-sweeps 0 → 1.5 N·m in 0.05 N·m steps; export CSV
7. **Analyse** — plot torque vs. speed, torque vs. current, torque vs. efficiency; compare against OEM curves

---

## Notes

- PCB layout was not completed within the assignment window; the module-based implementation is fully functional
- 3D-printed STLs export directly from the Fusion 360 assembly — no additional CAD preparation needed
- The FOC/BLDC approach was considered and documented but set aside on cost and calibration grounds — see Section 1.2 of the report

---

*IIT Bombay · POSHA Assignment · April 2026*
