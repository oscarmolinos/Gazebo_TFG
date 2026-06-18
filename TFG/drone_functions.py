#!/usr/bin/env python3

"""
drone_functions.py — Modulo reutilizable para misiones multi-dron en Aerostack2.

Contiene TODA la logica de ejecucion (control de drones, gimbal, camara, hilos)
para que un script de problema solo tenga que aportar los DATOS del escenario y
llamar a run().

------------------------------------------------------------------------------
COMO USARLO DESDE UN SCRIPT DE PROBLEMA
------------------------------------------------------------------------------
Los datos del escenario llegan en un diccionario `scenario` (cargado del YAML
del problema) y los planes en un diccionario `plans` (de plan_parser). Ambos se
pasan como ARGUMENTO a las funciones: este modulo NO usa variables globales que
el script tenga que rellenar.

    import drone_functions as df

    scenario = { 'COORDS': {...}, 'DRONES': {...}, 'CAN_PHOTOGRAPH': {...}, ... }
    plans    = load_plans('/home/oscar/MA-LAMA')   # {dron: [acciones]}

    df.run(scenario, plans)        # crea drones, lanza hilos y muestra fotos

    # o, si necesitas los objetos drone entre medias:
    drones = df.create_drones(plans)
    df.execute_mission(drones, scenario, plans)
    df.shutdown_drones(drones)

Asi, toda la logica (estas ~250 lineas) vive aqui una sola vez, y cada problema
nuevo son solo los datos (scenario) + el plan + la llamada a run().

Por que un hilo por dron: cada DroneInterface es un nodo ROS 2 independiente con
su propio executor. Cada hilo toca UNICAMENTE la interfaz de su dron (no se
comparte estado ROS entre hilos), asi que no hay condiciones de carrera. Que un
dron este bloqueado fotografiando no frena a los demas.
"""

import argparse
import math
import threading
from time import sleep

from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import GimbalControl
from geometry_msgs.msg import Vector3Stamped, Vector3, QuaternionStamped
from std_msgs.msg import Header
import rclpy

import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

import glob
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


# =============================================================================
#  FORMA DE LOS DATOS DEL ESCENARIO
#  Los datos llegan en el dict `scenario` (del YAML) y los planes en `plans`
#  (de plan_parser); ambos se pasan como argumento a las funciones de abajo.
#  De `scenario` se usan estas claves:
#     COORDS         -> nombre de waypoint -> (x, y, z) en metros
#     CAN_PHOTOGRAPH -> viewpoint -> target que fotografia desde el
#     DRONES[ns]['speed'] -> velocidad de crucero (m/s) de cada dron
#  Y `plans` es: nombre de dron -> lista de acciones (tuplas).
#  A continuación se muestra la forma que deben tener:
    # COORDS = {
    #     # ---- ZONA A ----        x      y     z
    #     "base1_ground": (-5.0,  1.5, 0.0),
    #     "base1_air":    (-5.0,  1.5, 2.0),
    #     "vp1":          (-2.4,  2.4, 4.1),
    #     "vp2":          ( 2.4,  2.4, 4.1),
    #     "tgt1":         (-1.4,  1.4, 3.1),
    #     "tgt2":         ( 1.4,  1.4, 3.1),

    #     # ---- ZONA B ----        x      y     z
    #     "base2_ground": (-5.0, -1.5, 0.0),
    #     "base2_air":    (-5.0, -1.5, 2.0),
    #     "vp3":          (-2.4, -2.4, 4.1),
    #     "vp4":          ( 2.4, -2.4, 4.1),
    #     "tgt3":         (-1.4, -1.4, 3.1),
    #     "tgt4":         ( 1.4, -1.4, 3.1),
    # }

    # velocidad por dron: scenario['DRONES']["drone1"]["speed"] = 1.0, ...

    # CAN_PHOTOGRAPH = {
    #     "vp1": "tgt1",
    #     "vp2": "tgt2",
    #     "vp3": "tgt3",
    #     "vp4": "tgt4",
    # }

    # plans = {
    #     "drone1": [
    #         ("takeoff",    "base1_ground", "base1_air"),
    #         ("fly",        "base1_air",    "vp1"),
    #         ("take_photo", "vp1",          "tgt1"),
    #         ("fly",        "vp1",          "vp2"),
    #         ("take_photo", "vp2",          "tgt2"),
    #         ("fly",        "vp2",          "vp1"),
    #         ("fly",        "vp1",          "base1_air"),
    #         ("land",       "base1_air",    "base1_ground"),
    #     ],
    #     "drone2": [
    #         ("takeoff",    "base2_ground", "base2_air"),
    #         ("fly",        "base2_air",    "vp3"),
    #         ("take_photo", "vp3",          "tgt3"),
    #         ("fly",        "vp3",          "vp4"),
    #         ("take_photo", "vp4",          "tgt4"),
    #         ("fly",        "vp4",          "vp3"),
    #         ("fly",        "vp3",          "base2_air"),
    #         ("land",       "base2_air",    "base2_ground"),
    #     ],
    # }
    # Los planes vienen de MA-LAMA:
    # from plan_parser import load_plans
    # plans = load_plans('/home/oscar/MA-LAMA')

# Parametros de vuelo (constantes; iguales para todos los problemas).
DEFAULT_SPEED = 1.0
LAND_SPEED = 0.5
TAKE_OFF_SPEED = 1.0

# Instante de simulacion que marca el t=0.0 (primer takeoff de cualquier dron).
# Se comparte entre hilos; el lock protege su escritura inicial.
T0 = {'value': None}
T0_LOCK = threading.Lock()


# =============================================================================
#  FUNCIONES AUXILIARES
# =============================================================================
def sim_time(drone) -> float:
    """Tiempo de SIMULACION en segundos (reloj ROS, respeta use_sim_time)."""
    return drone.get_clock().now().nanoseconds * 1e-9


def elapsed(drone) -> float:
    """Segundos de simulacion desde el t=0 global (primer takeoff)."""
    with T0_LOCK:
        if T0['value'] is None:
            T0['value'] = sim_time(drone)
    return sim_time(drone) - T0['value']


def yaw_to_face(origin: list, target: list) -> float:
    """Yaw (rad) en frame mundo para que el dron en 'origin' encare 'target'."""
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    return math.atan2(dy, dx)


def take_photo(drone_interface, filename: str, timeout: float = 5.0,
               photos_dir: str = "./TFG/photos") -> bool:
    """Capturar un frame de la camara y guardarlo como PNG en photos_dir (bloqueante)."""
    topic = f'/{drone_interface.drone_id}/sensor_measurements/gimbal/camera/image_raw'

    success, msg = wait_for_message(
        Image, drone_interface, topic,
        qos_profile=qos_profile_sensor_data,
        time_to_wait=timeout)

    if not success:
        print(f'No se recibio imagen en {topic}')
        return False

    # crear la carpeta de fotos si no existe (evita que save() falle)
    os.makedirs(photos_dir, exist_ok=True)
    path = os.path.join(photos_dir, filename)

    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    PILImage.fromarray(array, mode='RGB').save(path)
    print(f'Foto guardada: {path} ({msg.width}x{msg.height})')
    return True


def gimbal_orientation(drone_interface: DroneInterface, viewpoint, target,
                       tol_deg: float = 1, pitch_rate: float = 0.5,
                       timeout: float = 3.0) -> bool:
    """Inclinar el gimbal en pitch hasta enfocar el target, en lazo cerrado."""
    dx = target[0] - viewpoint[0]
    dy = target[1] - viewpoint[1]
    dz = target[2] - viewpoint[2]
    horizontal = math.hypot(dx, dy)
    pitch_target = -math.atan2(dz, horizontal)

    cmd_topic = f'/{drone_interface.drone_id}/platform/gimbal/gimbal_command'
    publisher = drone_interface.create_publisher(
        GimbalControl, cmd_topic, qos_profile_system_default)
    sleep(0.5)

    def publish(px, py, pz):
        msg = GimbalControl()
        msg.control_mode = 1
        msg.target = Vector3Stamped(
            header=Header(frame_id=f'{drone_interface.drone_id}/gimbal'),
            vector=Vector3(x=float(px), y=float(py), z=float(pz)))
        publisher.publish(msg)

    att_topic = f'/{drone_interface.drone_id}/sensor_measurements/gimbal/attitude'
    tol_rad = math.radians(tol_deg)

    while True:
        success, msg = wait_for_message(
            QuaternionStamped, drone_interface, att_topic,
            qos_profile=qos_profile_sensor_data, time_to_wait=timeout)
        if not success:
            publish(0.0, 0.0, 0.0)
            drone_interface.destroy_publisher(publisher)
            return False

        q = msg.quaternion
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch_now = math.asin(sinp)

        error = pitch_target - pitch_now
        if abs(error) < tol_rad:
            break

        rate = pitch_rate if error > 0 else -pitch_rate
        publish(0.0, rate, 0.0)
        sleep(0.01)

    for _ in range(10):
        publish(0.0, 0.0, 0.0)
        sleep(0.05)

    drone_interface.destroy_publisher(publisher)
    return True


def clear_photos(photos_dir: str = "./TFG/photos", pattern: str = "photo_*.png") -> None:
    """Borrar las fotos de la mision anterior antes de empezar una nueva."""
    for path in glob.glob(os.path.join(photos_dir, pattern)):
        os.remove(path)


def show_all_photos(photos_dir: str = "./TFG/photos", pattern: str = "photo_*.png") -> None:
    """Mostrar todas las fotos de la mision (de photos_dir) en una sola figura."""
    # buscar los PNG dentro de la carpeta de fotos
    files = sorted(glob.glob(os.path.join(photos_dir, pattern)))
    if not files:
        print(f'No se encontraron fotos en {photos_dir}')
        return

    n = len(files)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3 * rows))
    if n == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    for ax, filename in zip(axes, files):
        img = mpimg.imread(filename)
        ax.imshow(img)
        ax.set_title(os.path.basename(filename), fontsize=9)  # solo el nombre, sin la ruta
        ax.axis('off')

    for ax in axes[n:]:
        ax.axis('off')

    fig.suptitle(f'Fotos de la mision ({n})', fontsize=14)
    fig.tight_layout()
    plt.show()


# =============================================================================
#  EJECUCION DE CADA TIPO DE ACCION
# =============================================================================
def do_takeoff(drone: DroneInterface, ground: str, air: str, coords: dict) -> None:
    """Despegar de la base de suelo a la aerea (altura = z del punto air)."""
    height = coords[air][2]
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] takeoff -> {air} (h={height} m)')
    drone.arm()
    drone.offboard()
    drone.takeoff(height=height, speed=TAKE_OFF_SPEED)


def do_fly(drone: DroneInterface, origin: str, dest: str, speed: float,
           coords: dict, viewpoint_target: dict) -> None:
    """
    Volar a 'dest'. Si 'dest' es un viewpoint con target asociado, el dron llega
    orientado hacia ese target; si no, sigue el path encarado al destino.
    """
    p_dest = list(coords[dest])
    t = elapsed(drone)

    if dest in viewpoint_target:
        face_point = list(coords[viewpoint_target[dest]])   # mirar al target
        face_name = viewpoint_target[dest]
        yaw = yaw_to_face(p_dest, face_point)
        print(f'[{t:7.3f}][{drone.drone_id}] fly {origin} -> {dest} facing {face_name} '
              f'(yaw={math.degrees(yaw):.1f} deg)')
        drone.go_to.go_to_point_with_yaw(p_dest, angle=yaw, speed=speed)
    else:
        print(f'[{t:7.3f}][{drone.drone_id}] fly {origin} -> {dest} (path facing)')
        drone.go_to.go_to_point_path_facing(p_dest, speed=speed)

    print(f'[{drone.drone_id}] in {dest}')


def do_land(drone: DroneInterface, air: str, ground: str) -> None:
    """Aterrizar de la base aerea a la de suelo (bloqueante)."""
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] land -> {ground}')
    drone.land(speed=LAND_SPEED)


def do_take_photo(drone: DroneInterface, viewpoint: str, target: str, coords: dict) -> None:
    """Orientar el gimbal hacia el target y fotografiar exactamente 5 segundos."""
    p_vp = list(coords[viewpoint])
    p_tgt = list(coords[target])
    t_start = elapsed(drone)
    print(f'[{t_start:7.3f}][{drone.drone_id}] take_photo {viewpoint} -> {target}')
    gimbal_orientation(drone, p_vp, p_tgt)
    take_photo(drone, f'photo_{drone.drone_id}_{target}.png')

    while elapsed(drone) - t_start < 5.0:
        sleep(0.01)


# =============================================================================
#  MISION DE UN DRON  (esto es lo que corre en CADA HILO)
# =============================================================================
def drone_mission(drone: DroneInterface, plan: list, scenario: dict) -> None:
    """Ejecutar la mision completa de UN dron, secuencial y bloqueante.
    Los datos del escenario (coordenadas, viewpoint->target, velocidad) se leen
    del dict 'scenario'."""
    ns = drone.drone_id
    coords = scenario['COORDS']
    viewpoint_target = scenario['CAN_PHOTOGRAPH']
    speed = scenario['DRONES'].get(ns, {}).get('speed', DEFAULT_SPEED)

    for action in plan:
        kind = action[0]
        if kind == "takeoff":
            do_takeoff(drone, action[1], action[2], coords)
        elif kind == "fly":
            do_fly(drone, action[1], action[2], speed, coords, viewpoint_target)
        elif kind == "take_photo":
            do_take_photo(drone, action[1], action[2], coords)
        elif kind == "land":
            do_land(drone, action[1], action[2])
        else:
            print(f'[{ns}] accion desconocida: {kind}')

    drone.manual()
    t = elapsed(drone)
    print(f'[{t:7.3f}][{ns}] mision completada')


# =============================================================================
#  CREACION Y EJECUCION  (separadas para que el script tenga los objetos drone)
#
#  El flujo se parte en tres pasos que el script puede usar por separado:
#    1) create_drones()    -> crea las DroneInterface y TE LAS DEVUELVE
#    2) execute_mission()  -> lanza los hilos y ejecuta los planes
#    3) shutdown_drones()  -> cierre ordenado
#
#  Asi el script tiene el diccionario de drones en su mano (para leer odometria,
#  bateria, etc.) entre la creacion y la ejecucion, o despues.
# =============================================================================
def create_drones(plans: dict, use_sim_time: bool = True, verbose: bool = False) -> dict:
    """
    Inicializar rclpy y crear una DroneInterface por dron de 'plans'.
    Devuelve un dict {nombre: DroneInterface} para que el script lo use.
    """
    rclpy.init()

    drones = {}
    for ns in plans:
        print(f'Creando interfaz de {ns}')
        drones[ns] = DroneInterface(
            drone_id=ns, use_sim_time=use_sim_time, verbose=verbose)
    return drones


def arm_offboard(drones: dict) -> None:
    """Armar y poner en offboard todos los drones (antes de volar)."""
    for drone in drones.values():
        print(f'[{drone.drone_id}] Arm')
        drone.arm()
        print(f'[{drone.drone_id}] Offboard')
        drone.offboard()


def execute_mission(drones: dict, scenario: dict, plans: dict) -> None:
    """
    Lanzar un hilo por dron y ejecutar su plan. Espera a que todos terminen.
    'drones' es el dict devuelto por create_drones(); 'scenario' aporta los datos
    del escenario y 'plans' el plan de acciones de cada dron.
    """
    clear_photos()

    threads = []
    for ns, plan in plans.items():
        t = threading.Thread(target=drone_mission, args=(drones[ns], plan, scenario), name=ns)
        threads.append(t)

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print('Todas las misiones han terminado')


def shutdown_drones(drones: dict) -> None:
    """Cierre ordenado de cada dron y de rclpy (en el hilo principal)."""
    for drone in drones.values():
        drone.shutdown()
    rclpy.shutdown()
    print('Clean exit')


def run(scenario: dict, plans: dict, use_sim_time: bool = True,
        verbose: bool = False, show_photos: bool = True) -> None:
    """
    Atajo que hace todo el flujo de una vez (crear, armar, ejecutar, cerrar).
    Util cuando NO necesitas acceder a los drones desde el script. Si si los
    necesitas (odometria, bateria...), usa create_drones() / execute_mission()
    por separado desde tu script.
    """
    drones = create_drones(plans, use_sim_time, verbose)
    arm_offboard(drones)
    execute_mission(drones, scenario, plans)
    shutdown_drones(drones)
    if show_photos:
        show_all_photos()