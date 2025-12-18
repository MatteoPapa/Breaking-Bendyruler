# Breaking BendyRuler Simulator
2D (pygame) simulator to test *defender* drone formations aimed at diverting a *intruder* drone toward a hijack point.  
The leader runs the **OA BendyRuler** obstacle avoidance algorithm from ArduPilot, reimplemented in Python.

<img width="1269" height="715" alt="Schermata del 2025-12-18 12-47-13" src="https://github.com/user-attachments/assets/cc73adef-9d97-476d-9996-28eae6faabc4" />

## Structure
- **main.py**  
  Pygame loop, user input, rendering, intruder/target/obstacle management, and formation selection.
- **bendyruler.py**  
  Python reimplementation of the `AP_OABendyRuler` algorithm (two-step path probing, resist bearing change, dynamic lookahead).
- **config.py**  
  Simulation constants and BendyRuler tuning parameters (lookahead, angle, margin, ratio, etc.).
- **c_shape.py**  
  C-shaped formation behind the intruder to push it toward the hijack target.
- **u_shape.py**  
  U-shaped formation with dynamic role reassignment near the target (avoids crossings).
- **smart_u_shape.py**  
  Advanced U-shape featuring:
  - passive *recon* phase (destination estimation),
  - *trap / lockdown* phase near the hijack point.

## Requirements
- Python 3.x
- pygame

```
pip install -r requirements.txt
```

## Run
```
python main.py  
python main.py --formation CSHAPE  
python main.py --formation USHAPE  
python main.py --formation SMART_USHAPE  
```

## Notes
- Original C++ implementation (ArduPilot – AP_OABendyRuler):
  https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_Avoidance/AP_OABendyRuler.cpp
