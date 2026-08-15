BeyondHome ALFA 0.10 — Bogon Zones / Tracker Lock

CAMERA ONLY. No GPS, depth sensor, IMU, ARCore, WebXR or external libraries.

Changes from 0.08:
- Scanner now works by 3 spatial screen zones (3x2 grid), not by requiring large movement.
- A small lateral micro-displacement is enough to create parallax evidence.
- Triangulation accepts a wider but still bounded reprojection geometry, allowing cyan 3D samples to appear on ordinary RGB cameras.
- Green points are visual tracking only.
- CYAN points are consolidated 3D samples.
- Progress is based on actual 3D zones + samples rather than raw coverage cells.
- Save requires 3 of 6 zones, 12+ 3D samples, 3 consolidated samples and 2 keyframes.
- App version/cache-busting updated to 0.10; Netlify no-cache headers retained.

Recommended use:
1. Hold the phone still for about 1 second.
2. Aim at a textured area (books, furniture, frame, edge).
3. Move the phone sideways a few centimetres without rotating much.
4. Wait for cyan points.
5. Move to another screen zone and repeat. Three zones are enough to save.


ALFA 0.10: ciclo visual de referencias RED/ORANGE/YELLOW/GREEN/BLUE. El azul representa puntos triangulados que entran en la malla Bogon; no se exige precisión métrica absoluta.
