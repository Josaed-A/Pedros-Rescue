"""
test_local_cameras.launch.py
─────────────────────────────
SOLO PRUEBA LOCAL — no representa el hardware real del robot.

El robot real usa dos camaras distintas conectadas a la Raspberry Pi: la
Orbbec Astra Pro (profundidad + color) y la Logitech frontal. Cuando no hay
Pi/robot a mano, este launch permite validar en el PC el mismo mecanismo de
streaming dual + deteccion que usa la interfaz real (dos publishers
CompressedImage + object_detector + dashboard), sustituyendo esas dos camaras
por las que existen en este equipo:

  - GENERAL WEBCAM (USB, /dev/video2 en esta maquina) → toma el lugar de la
    Logitech frontal → topic /robot/camera/front/image_raw/compressed
  - Camara local integrada (/dev/video0 en esta maquina) → toma el lugar de
    la Astra → topic /robot/camera/astra/color/image_raw/compressed
    (sin profundidad real: solo color)

Usa el mismo nodo publisher que corre en la Pi (`rescue_bringup/logitech_pub`)
para ambos dispositivos — es el mismo metodo de streaming, solo apuntado a
otro /dev/videoN.

Deteccion en AMBAS camaras al mismo tiempo: se lanzan DOS instancias de
`object_detector` (una por camara), cada una con su propio `camera_id` y su
propio feed anotado — el dashboard muestra los dos paneles con cajas de
deteccion en vivo, no solo el frontal. AprilTag y YOLO-objetos los sigue
corriendo cada instancia localmente, sin cambios.

Hazmat va aparte, por un `hazmat_worker` COMPARTIDO (un solo nodo, un solo
modelo cargado en memoria) — las dos instancias de object_detector NO cargan
su propio modelo hazmat, solo le reenvian sus frames y reciben los
resultados de vuelta (ver hazmat_mode='worker' en object_detector.py). El
worker prioriza: en estado normal muestrea ambas camaras por turnos: si una
confirma una señal, esa camara pasa a inferirse en (casi) todos los ciclos y
la otra baja a un piso de vigilancia (nunca se detiene del todo); la
prioridad vuelve a normal si la camara boosteada deja de detectar durante
`priority_release_sec`. Ninguna captura/streaming de camara se ve afectada
por esto — ese muestreo es solo hacia el worker, el streaming crudo y el
resto de detectores siguen a su ritmo normal.

Alertas hazmat: una deteccion parpadea entre frames por ruido del modelo, asi
que cada object_detector solo la confirma (y dispara una captura para
revision humana) cuando la misma señal se sostiene durante
`alert_confirm_frames` frames seguidos. Cada captura (frame anotado .jpg +
descripcion .json: clase, confianza, bbox, camara) se guarda en
hazmat/alertas_detectadas/<front|astra>/.

IMPORTANTE: esto es una prueba de escritorio. No tocar pi_sensors.launch.py /
pedro_pi.launch.py / logitech_vision.launch.py / command_station.launch.py —
esos siguen apuntando a la Astra y a la Logitech de la Pi (direcciones reales
del robot) y son los que se deben usar al commitear avances reales.

Uso:
  ros2 launch rescue_bringup test_local_cameras.launch.py
  ros2 launch rescue_bringup test_local_cameras.launch.py general_webcam_device:=2 local_camera_device:=0
  ros2 launch rescue_bringup test_local_cameras.launch.py alert_confirm_frames:=5 alert_cooldown_sec:=30
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

FRONT_TOPIC = '/robot/camera/front/image_raw/compressed'
ASTRA_COLOR_TOPIC = '/robot/camera/astra/color/image_raw/compressed'
FRONT_ANNOTATED_TOPIC = '/camera/color/image_annotated/compressed'
ASTRA_ANNOTATED_TOPIC = '/camera/astra/image_annotated/compressed'


def _find_alerts_dir(launch_file: str) -> str:
    """
    Resuelve hazmat/alertas_detectadas junto al checkout del repo. Con
    symlink-install, realpath(__file__) sigue el enlace hasta el archivo
    fuente real en src/rescue_bringup/launch/, asi que subiendo desde ahi se
    llega a la raiz del repo. Si el install no es symlink (copia normal) no
    hay forma generica de ubicar el repo desde ahi, asi que se usa un default
    razonable para este checkout de desarrollo.
    """
    d = os.path.dirname(os.path.realpath(launch_file))
    for _ in range(6):
        if os.path.isdir(os.path.join(d, 'hazmat')) and os.path.isdir(os.path.join(d, 'src')):
            return os.path.join(d, 'hazmat', 'alertas_detectadas')
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return os.path.join(os.path.expanduser('~'), 'Pedros-Rescue', 'hazmat', 'alertas_detectadas')


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rescue_bringup')
    default_hazmat_model = os.path.join(pkg_bringup, 'models', 'best.pt')
    default_alerts_dir = _find_alerts_dir(__file__)

    general_webcam_device = LaunchConfiguration('general_webcam_device', default='2')
    local_camera_device   = LaunchConfiguration('local_camera_device',   default='0')
    hazmat_model          = LaunchConfiguration('hazmat_model',          default=default_hazmat_model)
    enable_yolo           = LaunchConfiguration('enable_yolo',           default='true')
    alerts_dir            = LaunchConfiguration('alerts_dir',            default=default_alerts_dir)
    alert_confirm_frames  = LaunchConfiguration('alert_confirm_frames',  default='3')
    alert_cooldown_sec    = LaunchConfiguration('alert_cooldown_sec',    default='20.0')

    # Scheduler de prioridad del hazmat_worker compartido.
    priority_confirm_frames = LaunchConfiguration('priority_confirm_frames', default='2')
    priority_release_sec    = LaunchConfiguration('priority_release_sec',    default='8.0')

    # ── GENERAL WEBCAM → sustituye la Logitech frontal ──────────────────
    general_webcam_pub = Node(
        package='rescue_bringup',
        executable='logitech_pub',
        name='general_webcam_pub',
        output='screen',
        parameters=[{
            'device':       general_webcam_device,
            'topic':        FRONT_TOPIC,
            'fps':          15,
            'jpeg_quality': 80,
        }],
    )

    # ── Camara local integrada → sustituye la Astra (solo color) ────────
    local_camera_pub = Node(
        package='rescue_bringup',
        executable='logitech_pub',
        name='local_camera_pub',
        output='screen',
        parameters=[{
            'device':       local_camera_device,
            'topic':        ASTRA_COLOR_TOPIC,
            'fps':          15,
            'jpeg_quality': 80,
        }],
    )

    def _detector_params(camera_id: str, color_topic: str) -> dict:
        return {
            'output_dir':      os.path.join(os.path.expanduser('~'), 'maps'),
            'team_name':       'SabanaHerons',
            'mission':         'M1',
            'robot_name':      'Pedro',
            'mode':            'T',
            'use_compressed':  True,
            'color_topic':     color_topic,
            'require_depth':   False,
            'enable_apriltag': True,
            'enable_hazmat':   True,
            'enable_yolo':     enable_yolo,
            'yolo_model':      'yolov8n.pt',
            # Hazmat corre en el hazmat_worker compartido, no aqui — esta
            # instancia no carga ningun modelo hazmat propio, solo reenvia
            # frames y recibe resultados (ver hazmat_worker mas abajo).
            'hazmat_mode':     'worker',
            'hazmat_conf':     0.40,
            'camera_id':             camera_id,
            'alerts_dir':            alerts_dir,
            'alert_confirm_frames':  alert_confirm_frames,
            'alert_cooldown_sec':    alert_cooldown_sec,
        }

    # ── object_detector sobre la GENERAL WEBCAM (frontal) ────────────────
    # AprilTag + YOLO-objetos locales (sin cambios); hazmat via worker.
    detector_front = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector_front',
                output='screen',
                parameters=[_detector_params('front', FRONT_TOPIC)],
                # annotated/detections/markers en sus topics por defecto —
                # coincide con lo que ya espera el dashboard para el panel frontal.
            )
        ],
    )

    # ── object_detector sobre la camara local (ocupa el slot Astra) ──────
    detector_astra = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='rescue_bringup',
                executable='object_detector',
                name='object_detector_astra',
                output='screen',
                parameters=[_detector_params('astra', ASTRA_COLOR_TOPIC)],
                remappings=[
                    # Topic anotado propio para no pisar el del frontal.
                    (FRONT_ANNOTATED_TOPIC, ASTRA_ANNOTATED_TOPIC),
                    # Servicio propio: /save_detection_csv sin remapear solo
                    # sirve a una instancia; esta usa uno separado.
                    ('/save_detection_csv', '/save_detection_csv_astra'),
                ],
            )
        ],
    )

    # ── hazmat_worker: UNICO modelo hazmat cargado, compartido por front+astra ──
    hazmat_worker = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rescue_bringup',
                executable='hazmat_worker',
                name='hazmat_worker',
                output='screen',
                parameters=[{
                    'camera_ids':   ['front', 'astra'],
                    'hazmat_model': hazmat_model,
                    'hazmat_conf':  0.40,
                    'priority_confirm_frames': priority_confirm_frames,
                    'priority_release_sec':    priority_release_sec,
                }],
            )
        ],
    )

    # ── Dashboard: mismo panel/mecanismo que en el robot real, con ambos
    #    feeds anotados (frontal y "astra") ────────────────────────────
    dashboard_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='rescue_command_station',
                executable='dashboard_node',
                name='drive_dashboard_node',
                output='screen',
                parameters=[{
                    'front_camera_topic':    FRONT_TOPIC,
                    'astra_color_topic':     ASTRA_COLOR_TOPIC,
                    'front_annotated_topic': FRONT_ANNOTATED_TOPIC,
                    'astra_annotated_topic': ASTRA_ANNOTATED_TOPIC,
                }],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('general_webcam_device', default_value='2',
                              description='Indice /dev/videoN de la GENERAL WEBCAM (sustituye a la Logitech)'),
        DeclareLaunchArgument('local_camera_device', default_value='0',
                              description='Indice /dev/videoN de la camara local (sustituye a la Astra, solo color)'),
        DeclareLaunchArgument('hazmat_model', default_value=default_hazmat_model,
                              description='Ruta al modelo YOLO hazmat (.pt). Vacio = HSV fallback'),
        DeclareLaunchArgument('enable_yolo', default_value='true',
                              description='Habilitar YOLO COCO (objetos de mision)'),
        DeclareLaunchArgument('alerts_dir', default_value=default_alerts_dir,
                              description='Carpeta base para capturas automaticas hazmat (subcarpeta por camara)'),
        DeclareLaunchArgument('alert_confirm_frames', default_value='3',
                              description='Frames consecutivos que una señal debe sostenerse antes de capturarla'),
        DeclareLaunchArgument('alert_cooldown_sec', default_value='20.0',
                              description='Segundos minimos entre dos capturas de la misma señal'),
        DeclareLaunchArgument('priority_confirm_frames', default_value='2',
                              description='Inferencias seguidas con deteccion para que una camara tome prioridad en el hazmat_worker'),
        DeclareLaunchArgument('priority_release_sec', default_value='8.0',
                              description='Segundos sin deteccion antes de que el hazmat_worker vuelva a prioridad normal'),
        general_webcam_pub,
        local_camera_pub,
        hazmat_worker,
        detector_front,
        detector_astra,
        dashboard_node,
    ])
