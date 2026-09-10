"""One configuration loader for ROS nodes and the standalone simulator."""
from pathlib import Path
import numpy as np
import yaml
from .kinematics import Arm6DOF, ArmParams


def config_path(package, filename, *, prefer_installed=False):
    if prefer_installed:
        # ROS must not accidentally prefer a source checkout to the active overlay.
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package)) / 'config' / filename
    source = Path(__file__).resolve().parents[3] / package / 'config' / filename
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package)) / 'config' / filename


def _parameter(node, name, default):
    from rcl_interfaces.msg import ParameterDescriptor
    node.declare_parameter(name, default, descriptor=ParameterDescriptor(read_only=True))
    return node.get_parameter(name).value


def settings(node=None):
    """One immutable snapshot per ROS node; no YAML reads at module import time."""
    if node is not None and hasattr(node, '_arm_configuration'):
        return dict(node._arm_configuration)
    path = config_path('rescue_command_station', 'arm.yaml', prefer_installed=node is not None)
    if node is not None:
        path = Path(_parameter(node, 'arm_config_path', str(path)))
    cfg = yaml.safe_load(path.read_text(encoding='utf-8'))['/**']['ros__parameters']
    keys = list(ArmParams.__dataclass_fields__) + ['joint_order', 'model_sign', 'joint_min', 'joint_max']
    if node is not None:
        cfg = {key: _parameter(node, key, cfg[key]) for key in keys}
    names = cfg['joint_order']
    if len(names) != 6 or len(set(names)) != 6 or not all(isinstance(n, str) and n for n in names):
        raise ValueError('joint_order requiere seis nombres unicos, en orden J1..J6')
    if node is not None:
        node._arm_configuration = dict(cfg)
    return dict(cfg)


def joint_drivers(node=None):
    path = config_path('rescue_robot_core', 'servos.yaml', prefer_installed=node is not None)
    if node is not None:
        path = Path(_parameter(node, 'servos_config_path', str(path)))
    data=yaml.safe_load(path.read_text(encoding='utf-8'))
    out={}
    for node_name, driver in [('ax12a_driver','ax'),('ex106_driver','ex')]:
        params = data[node_name]['ros__parameters']
        ids = set()
        # Use exactly the enabled names read by leer_servos_params in the drivers.
        for name in params['servo_names']:
            servo = params['servos'][name]
            sid = servo['id']
            if name in out or type(sid) is not int or not 0 <= sid <= 253 or sid in ids:
                raise ValueError('Nombre o ID de servo duplicado/invalido: '+str(name))
            ids.add(sid)
            out[name]=(driver,sid)
        for name in params.get('wheel_names', []):
            sid = params['wheels'][name]['id']
            if sid in ids:
                raise ValueError('Un ID de brazo y patas colisiona en el bus '+driver)
            ids.add(sid)
    expected = set(settings(node)['joint_order'])
    if set(out) != expected:
        raise ValueError('servo_names y joint_order no coinciden: '+str(sorted(set(out) ^ expected)))
    return out


def configured_arm(node=None, overrides=None):
    cfg=settings(node) if node is not None else settings()
    keys=list(ArmParams.__dataclass_fields__)
    if overrides:
        for k,v in overrides.items():
            if k not in keys: raise ValueError('Parametro desconocido: '+k)
            cfg[k]=v
    signs=np.asarray(cfg['model_sign'],float)
    if signs.shape!=(6,) or not np.all(np.isin(signs,[-1,1])):
        raise ValueError('model_sign requiere seis valores +/-1')
    bounds=np.column_stack((cfg['joint_min'],cfg['joint_max']))
    if (bounds.shape != (6, 2) or not np.all(np.isfinite(bounds))
            or np.any(bounds[:, 0] >= bounds[:, 1])):
        raise ValueError('joint_min/joint_max requieren seis limites finitos y min < max')
    model_bounds=np.sort(bounds*signs[:,None],axis=1)
    return Arm6DOF(ArmParams(**{k:cfg[k] for k in keys}),model_bounds),signs
