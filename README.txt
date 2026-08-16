BeyondHome ALFA 0.13.8 — General Room / Bogon Spatial AR

Esta versión mantiene la cámara RGB como fuente principal y elimina por completo la dependencia de DeviceOrientation/giroscopio/brújula; la pose se obtiene únicamente mediante movimiento visual y reproyección de referencias. La profundidad del mapa es relativa: se obtiene por paralaje visual y triangulación cuando existe evidencia suficiente, con una hipótesis de profundidad gruesa como fallback.

En AR, los Bogones se almacenan en coordenadas de mundo relativas y se proyectan mediante la pose de cámara. El tamaño visual se adapta a la profundidad: los Bogones cercanos se muestran mayores y los lejanos menores. El tracking visual corrige la traslación cuando encuentra referencias 3D.

No se requiere LiDAR, ARCore, WebXR, GPS ni una API de profundidad.

Versión: 0.13.8
Build: 2026-08-16.13.6


Cambios 0.13.8:
- IMU/giroscopio/brújula desactivados.
- Pose AR actualizada mediante movimiento visual RGB y refinada con referencias 3D.
- Mapa de representación reducido mediante LOD espacial: celdas Bogon de 0,55 unidades y máximo 40 primitivas visibles/guardadas para la primera pasada.
- Geometría Bogon adaptativa: perímetro 12 líneas, superficie con crosshatch reducido y volumen mediante muestras mínimas; se evita generar miles de puntos por cubo.
- El mapa lógico puede conservar más evidencia durante el escaneo, pero la representación final es deliberadamente gruesa.


ALFA 0.13.8 FIX: el botón ENTRAR tiene un fallback inline independiente del bundle JS.
La clave local es beyondHome.v37 y se conserva la migración desde v36.


ALFA 0.13.8: fixed entry deadlock caused by show() calling an undefined sensor teardown function. Entry is now independent of scanner/AR teardown and has a URL-fragment fallback (#home) so it remains usable even before app.js finishes booting. Gyroscope, compass and DeviceOrientation calls removed from scanner startup.
