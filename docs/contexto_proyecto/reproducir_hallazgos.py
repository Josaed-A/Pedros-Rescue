"""Pruebas de auditoría: extraen lógica sin importar ROS ni abrir hardware.
Ejecutar desde la raíz con el Python temporal que tenga numpy.
No modifica código ni archivos de ejecución del proyecto.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np

ROOT = Path(__file__).resolve().parents[2]

def extract(file, name, symbols):
    p = ROOT / file
    tree = ast.parse(p.read_text())
    node = next(n for n in tree.body if getattr(n, 'name', None) == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(p), 'exec'), symbols)
    return symbols[name]

# Error de lectura: poll conserva posición y publish emite aun sin fd.
os_fake = NS(read=Mock(side_effect=OSError('desconexion simulada')), close=Mock())
Joy = extract('src/dependencias/joy/joy/joy_node.py', 'JoyNode', {
    'Node': object, 'os': os_fake, 'EVENT_SIZE': 8,
    'Joy': lambda: NS(header=NS()),
})
j = object.__new__(Joy)
j.dev='dispositivo_falso'; j.fd=8; j.axes=[0.0, 1.0]; j.buttons=[1]; j.changed=False
j.open_device=lambda: True
j.get_logger=lambda: NS(warn=lambda *_:None)
j.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:'HORA_NUEVA'))
sent=[]; j.publisher=NS(publish=sent.append)
j.poll(); j.publish()
assert j.fd is None and sent[0].axes == [0.0,1.0]
print('H04 REPRODUCIDO: tras OSError se publica eje=1 con hora nueva y fd=None')

# La API local carece del encoding que el consumidor solicita.
Bridge = extract('src/dependencias/cv_bridge/cv_bridge/core.py', 'CvBridge', {
    'np': np, 'CvBridgeError': ValueError,
})
try:
    Bridge()._encoding_info('32FC1')
except ValueError:
    print('H08 REPRODUCIDO: CvBridge local rechaza 32FC1')
else:
    raise AssertionError('32FC1 ya soportado')

# RGB de un pixel por fila con padding de un byte (step 4): reshape de estación falla.
converter = extract('src/rescue_command_station/rescue_command_station/vision/ros_image.py',
    'image_msg_to_numpy', {'np':np, 'sys':NS(byteorder='little'),
                         'get_encoding_layout':lambda _: (np.uint8,3)})
msg=NS(encoding='rgb8',height=2,width=1,step=4,is_bigendian=False,data=bytes([1,2,3,0,4,5,6,0]))
try:
    converter(msg)
except ValueError:
    print('H09 REPRODUCIDO: decodificador estación falla con padding RGB válido step=4 width=1')
else:
    raise AssertionError('Padding ya soportado')

# Parser PointCloud2 no admite padding row_step.
parser=extract('src/rescue_bringup/rescue_bringup/pointcloud_accumulator.py', '_pc2_to_xyz_rgb', {
    'np':np, 'PointCloud2':object, 'Tuple':__import__('typing').Tuple,
    'Optional':__import__('typing').Optional,
})
msg=NS(fields=[NS(name=n,offset=i*4) for i,n in enumerate('xyz')],
       width=1,height=2,point_step=12,row_step=16,is_bigendian=False,data=bytes(32))
try:
    parser(msg)
except ValueError:
    print('H16 REPRODUCIDO: parser nube falla con padding row_step=16 point_step=12')
else:
    raise AssertionError('Padding de nube ya soportado')

# Frescura de scans: stamp cero permanece válido indefinidamente.
Scan=extract('src/rescue_bringup/rescue_bringup/scan_merger.py','ScanMerger',{'Node':object,'LaserScan':object})
scan=NS(header=NS(stamp=NS(sec=0,nanosec=0)))
assert object.__new__(Scan)._fresh(scan,None,.3) is scan
print('H17 REPRODUCIDO: scan con stamp cero pasa frescura sin comparar edad')

# Evidencia geométrica, sin presumir montaje físico.
rear=(math.cos(math.pi),math.sin(math.pi))
assert rear[0]<0
print('H07: rayo cero LiDAR bajo yaw=pi apunta a -X de la base')
