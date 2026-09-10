# Estado en tiempo real del controlador cartesiano.
# Publicado en /cartesian/state a ~10 Hz, tambien en reposo y preflight.
std_msgs/Header header
float64  x               # posicion TCP actual (m); NaN sin feedback_valid
float64  y
float64  z
float64  pos_error       # error de posicion vs referencia activa (m)
float64  rot_error       # error de orientacion vs referencia activa (rad)
float64  progress        # fraccion de la trayectoria completada [0, 1]
bool     in_progress     # true durante validacion completa o ejecucion
bool     maintenance     # bloqueo central de comandos normales por mantenimiento
bool     feedback_valid  # seis articulaciones recibidas hace menos de 1 s
bool     within_path_tol # false si se supero la tolerancia de ruta
int32    waypoint_idx    # indice del waypoint actual (0-based)
int32    total_waypoints # total de waypoints en la trayectoria
