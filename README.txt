BeyondHome ALFA 0.13.5 — General Room / Bogon Spatial AR

Esta versión mantiene la cámara RGB como fuente principal y añade fusión opcional de DeviceOrientation (giroscopio/brújula expuestos por el navegador). La profundidad del mapa es relativa: se obtiene por paralaje visual y triangulación cuando existe evidencia suficiente, con una hipótesis de profundidad gruesa como fallback.

En AR, los Bogones se almacenan en coordenadas de mundo relativas y se proyectan mediante la pose de cámara. El tamaño visual se adapta a la profundidad: los Bogones cercanos se muestran mayores y los lejanos menores. El tracking visual corrige la traslación cuando encuentra referencias 3D.

No se requiere LiDAR, ARCore, WebXR, GPS ni una API de profundidad.

Versión: 0.13.5
Build: 2026-08-16.13.4
