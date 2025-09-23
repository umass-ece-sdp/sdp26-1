# Capstone 2025 – Team XX

This repository contains all project artifacts for our year-long capstone project.  
It is organized to clearly separate **hardware** and **software** development, while also supporting shared documentation, data, and results.

---

## 📂 Repository Structure

---

## 🛠 Usage Notes

- **docs/**  
  All project documentation, design specs, and final reports live here. Keep it updated regularly.

- **hardware/**  
  Contains circuit schematics, PCB layouts, CAD models, and firmware. If firmware grows large, it can move into `software/`.

- **software/**  
  Contains the project’s codebase, with unit tests, libraries, and scripts. Follow good coding practices and include test coverage.

- **data/** vs **results/**  
  Keep **raw data** (inputs, logs, calibration files) separate from **processed results** (plots, analysis outputs). This ensures reproducibility.

- **tools/**  
  Place any helper scripts, configuration files, or utilities used by both HW and SW here.

---

## 📑 Conventions

- **Empty folders** contain a `.gitkeep` placeholder to ensure they are tracked by Git.
- **Branching**: Use feature branches (`feature/hw-schematics`, `feature/sw-api`) and merge via pull requests.
- **Commits**: Write descriptive commit messages, e.g.,  
  `Add PCB schematic v1.0` or `Implement sensor data parser`.

---

## 👥 Team Notes

- Update the **meeting_minutes/** after every weekly meeting.  
- Keep the **README.md** in `hardware/` and `software/` updated with build/setup instructions.  
- Document all design decisions in `docs/design_specs/`.  

---