# Waypoint de trayectoria cartesiana.
# El primer waypoint de una trayectoria DEBE ser la pose actual del TCP.
float64   x             # posicion objetivo (m)
float64   y
float64   z
float64[] r             # matriz rotacion 3x3 row-major (9 elementos)
float64   duration      # segundos para llegar desde el waypoint anterior (>= 0.05)
int32     n_steps       # pasos de interpolacion hacia este waypoint (>= 1)
