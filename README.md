# 🤖 Robotics Journal

I'm Anna. By day I work on AI integration; in the evenings I'm teaching myself robotics, from physics simulation to solder.
This repo is my lab notebook: courses, parts, builds, mistakes, and handwritten notes.

**Now building:** [Sesame](https://github.com/dorianborian/sesame-robot), a small ESP32 quadruped, while learning simulation with MuJoCo.

_Last updated: 2026-11-16_

## Contents

| | |
|---|---|
| 📓 [Log](log/) | one entry per session |
| 🎓 [Course](course/) | lesson notes, key code explained |
| 🧪 [Notebooks](notebooks/) | Colab notebooks (outputs stripped) |
| 🔧 [Builds](builds/sesame/) | firmware, schematics, STL, photos |
| ✍️ [Notes](notes/) | photos of my paper notebook |
| 🧾 [BOM](bom.csv) | every part and tool I've bought |
| 🪦 [Graveyard](graveyard.md) | what I broke and what it taught me |

## Sesame build status

| Stage | Status |
|---|---|
| Tools (iron, multimeter) | ✅ done |
| Electronics (ESP32-S2 Mini, OLED, perfboard, wire) | ✅ delivered |
| Servos (MG90S, 8 needed) | 🚚 in transit |
| 3D-printed parts | ⏳ to order |
| Simulation (MuJoCo) | 🟡 in progress |
| Firmware | ⬜ not started |
| Assembly | ⬜ not started |
| First steps | ⬜ not started |

## Spent so far

From [`bom.csv`](bom.csv), delivered items only (items still marked TODO are not counted):

| | € |
|---|---:|
| 🛠️ Tools | 23.58 |
| 🔌 Components | 38.15 |
| 🔩 Fasteners | 17.87 |
| 🧴 Consumables | 15.59 |
| **Robot parts subtotal** (components + fasteners + consumables) | **71.61** |
| **Total** | **95.19** |

## Course

| # | Lesson | Notebook |
|---|---|---|
| 01 | [MuJoCo foundations](course/01-mujoco-foundations.md): MJCF, model vs data, generalized coordinates, PD servo, Sesame's kinematic tree | [ipynb](notebooks/01-mujoco-foundations.ipynb) |

## Log

- 2026-11-11 · [Parts week](log/2026-11-11-parts-arrive.md)
- 2026-11-10 · [MuJoCo, part 1](log/2026-11-10-mujoco-part-1.md)
- 2026-11-05 · [The soldering kit is here](log/2026-11-05-soldering-kit-arrives.md)
- 2026-11-02 · [Picking a first robot: Sesame](log/2026-11-02-choosing-sesame.md)

New entries start from [`templates/log-entry.md`](templates/log-entry.md).
