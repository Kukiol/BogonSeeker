BeyondHome ALFA 0.06

Camera-only monocular spatial mapper / Bogon AR.

Cambios 0.06:
- Cache del sitio desactivada mediante headers Netlify y assets versionados ?v=0.06.
- Versión visible y metadatos actualizados a ALFA 0.06.
- AR muestra marcadores sobre la imagen real: cyan = textura/referencia visual, verde = referencia 3D reconocida.
- Se resalta "ENFOCA AQUÍ" sobre la mejor referencia visible.
- La UI explica explícitamente que se usa una cámara RGB normal: no hay depth sensor ni detección de planos por hardware.
- Persistencia local migra v25 -> v26.
