BeyondHome ALFA 0.13.3 — World-space Bogon Map

Esta versión corrige la colocación espacial de los Bogones del entorno.

- Los anclajes azules se generan a partir de posiciones 3D fusionadas, no de coordenadas de pantalla.
- Si un tracker verde todavía no tiene triangulación fiable, se crea una hipótesis espacial en el mismo marco de coordenadas de la cámara y se refina con observaciones posteriores.
- El mapa guardado conserva los anclajes Bogon como puntos espaciales.
- Al entrar en AR, esos anclajes se dibujan como cubos Bogon en sus coordenadas espaciales; no son decoraciones 2D.
- La escala sigue siendo relativa/abstracta porque una cámara RGB monocular no proporciona metros absolutos.
- Se mantiene el enfoque general de habitación: solidez espacial primero, detalle después.
