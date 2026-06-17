#!/usr/bin/env python3

"""
Ejecutor multi-dron con UN HILO POR DRON.

Cada dron ejecuta su propia mision en un hilo independiente de Python. Dentro de
cada hilo, la secuencia es exactamente la misma que en la mision de un solo dron
(mission_problem1.py): las acciones se ejecutan una tras otra de forma BLOQUEANTE
(despegar, volar y esperar a llegar, orientar gimbal, fotografiar, aterrizar).

La independencia entre drones la da el hilo: como cada dron corre en su propio
hilo, que un dron este bloqueado fotografiando NO frena a los demas, que siguen
avanzando en sus propios hilos. Asi se reutilizan tal cual las funciones del caso
de un dron, sin reescribirlas en version no bloqueante.

Por que es seguro usar hilos aqui: cada DroneInterface es un nodo ROS 2
independiente, con su propio executor. Cada hilo toca UNICAMENTE la DroneInterface
de su dron (no se comparte estado ROS entre hilos), por lo que no hay condiciones
de carrera entre ellos.
"""

import argparse
import math
import threading
from time import sleep

from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import GimbalControl
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
import rclpy

import numpy as np
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

import glob
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Instante de simulacion que marca el t=0.0 (primer takeoff de cualquier dron).
# Se comparte entre hilos; el lock protege su escritura inicial.
T0 = {'value': None}
T0_LOCK = threading.Lock()

# =============================================================================
#  GEOMETRIA: nombre simbolico del waypoint -> coordenadas (x, y, z) en metros.
#  El plan habla de 'vp1', 'base1_air'...; AS2 necesita coordenadas. Este
#  diccionario es el puente (las mismas coordenadas del generador de PDDL).
# =============================================================================
COORDS = {
    # ---- ZONA A ----        x      y     z
    "base1_ground": (-5.0,  1.5, 0.0),
    "base1_air":    (-5.0,  1.5, 2.0),
    "vp1":          (-2.4,  2.4, 4.1),
    "vp2":          ( 2.4,  2.4, 4.1),
    "tgt1":         (-1.4,  1.4, 3.1),
    "tgt2":         ( 1.4,  1.4, 3.1),

    # ---- ZONA B ----        x      y     z
    "base2_ground": (-5.0, -1.5, 0.0),
    "base2_air":    (-5.0, -1.5, 2.0),
    "vp3":          (-2.4, -2.4, 4.1),
    "vp4":          ( 2.4, -2.4, 4.1),
    "tgt3":         (-1.4, -1.4, 3.1),
    "tgt4":         ( 1.4, -1.4, 3.1),
}

# Velocidad de crucero por dron (m/s).
SPEEDS = {"drone1": 1.0, "drone2": 2.0}
DEFAULT_SPEED = 1.0

LAND_SPEED = 0.5
TAKE_OFF_SPEED = 1.0

# Que target fotografia cada viewpoint (de CAN_PHOTOGRAPH del generador PDDL).
VIEWPOINT_TARGET = {
    "vp1": "tgt1",
    "vp2": "tgt2",
    "vp3": "tgt3",
    "vp4": "tgt4",
}

# =============================================================================
#  PLANES POR DRON 
#  Cada acción es una tupla con nombres simbolicos de waypoint:
#    ('takeoff', ground, air)
#    ('fly', origen, destino)
#    ('take_photo', viewpoint, target)
#    ('land', air, ground)
# =============================================================================
PLANS = {
    "drone1": [
        ("takeoff",    "base1_ground", "base1_air"),
        ("fly",        "base1_air",    "vp1"),
        ("take_photo", "vp1",          "tgt1"),
        ("fly",        "vp1",          "vp2"),
        ("take_photo", "vp2",          "tgt2"),
        ("fly",        "vp2",          "vp1"),
        ("fly",        "vp1",          "base1_air"),
        ("land",       "base1_air",    "base1_ground"),
    ],
    "drone2": [
        ("takeoff",    "base2_ground", "base2_air"),
        ("fly",        "base2_air",    "vp3"),
        ("take_photo", "vp3",          "tgt3"),
        ("fly",        "vp3",          "vp4"),
        ("take_photo", "vp4",          "tgt4"),
        ("fly",        "vp4",          "vp3"),
        ("fly",        "vp3",          "base2_air"),
        ("land",       "base2_air",    "base2_ground"),
    ],
}


# =============================================================================
#  FUNCIONES AUXILIARES
# =============================================================================

def sim_time(drone) -> float:
    """Tiempo de SIMULACION en segundos (reloj ROS, respeta use_sim_time)."""
    return drone.get_clock().now().nanoseconds * 1e-9


def elapsed(drone) -> float:
    """Segundos de simulacion transcurridos desde el t=0 global (primer takeoff)."""
    # fijar T0 la primera vez que alguien lo pide (el primer takeoff)
    with T0_LOCK:
        if T0['value'] is None:
            T0['value'] = sim_time(drone)
    return sim_time(drone) - T0['value']


def yaw_to_face(origin: list, target: list) -> float:
    """Yaw (rad) en frame mundo para que el dron en 'origin' encare 'target'."""
    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    return math.atan2(dy, dx)


def take_photo(drone_interface, filename: str, timeout: float = 5.0) -> bool:
    """Capturar un frame de la camara y guardarlo como PNG (bloqueante)."""
    topic = f'/{drone_interface.drone_id}/sensor_measurements/gimbal/camera/image_raw'

    success, msg = wait_for_message(
        Image, drone_interface, topic,
        qos_profile=qos_profile_sensor_data,
        time_to_wait=timeout)

    if not success:
        print(f'No se recibio imagen en {topic}')
        return False

    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    PILImage.fromarray(array, mode='RGB').save(filename)
    print(f'Foto guardada: {filename} ({msg.width}x{msg.height})')
    return True


def gimbal_orientation(drone_interface: DroneInterface, viewpoint, target,
                       tol_deg: float = 1, pitch_rate: float = 0.5,
                       timeout: float = 3.0) -> bool:
    """Inclinar el gimbal en pitch hasta enfocar el target, en lazo cerrado."""
    from geometry_msgs.msg import QuaternionStamped

    dx = target[0] - viewpoint[0]
    dy = target[1] - viewpoint[1]
    dz = target[2] - viewpoint[2]
    horizontal = math.hypot(dx, dy)
    pitch_target = -math.atan2(dz, horizontal)
    #print(f'  pitch objetivo: {math.degrees(pitch_target):.1f} deg')

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
    # print('gimbal orientado')
    return True

def show_all_photos(pattern: str = "photo_*.png") -> None:
    """
    Mostrar todas las fotos de la mision en una sola figura (cuadricula).
    Busca en disco los archivos que casan con 'pattern' (las guardadas por take_photo).
    """
    files = sorted(glob.glob(pattern))
    if not files:
        print('No se encontraron fotos para mostrar')
        return

    n = len(files)
    # cuadricula lo mas cuadrada posible: cols = techo(raiz(n)), filas las necesarias
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 3 * rows))
    # axes puede ser un solo Axes (n=1) o un array; lo normalizamos a lista plana
    if n == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    for ax, filename in zip(axes, files):
        img = mpimg.imread(filename)
        ax.imshow(img)
        ax.set_title(filename, fontsize=9)
        ax.axis('off')          # ocultar ejes, no aportan en una foto

    # apagar las celdas sobrantes de la cuadricula (si n no llena el grid)
    for ax in axes[n:]:
        ax.axis('off')

    fig.suptitle(f'Fotos de la mision ({n})', fontsize=14)
    fig.tight_layout()
    plt.show()

# =============================================================================
#  EJECUCIÓN DE CADA TIPO DE ACCIÓN
# =============================================================================
def do_takeoff(drone: DroneInterface, ground: str, air: str) -> None:
    """Despegar de la base de suelo a la aerea (altura = z del punto air)."""
    height = COORDS[air][2]
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] takeoff -> {air} (h={height} m)')
    # llamada BLOQUEANTE (sin wait=False): el hilo espera a que termine el despegue
    drone.takeoff(height=height, speed=TAKE_OFF_SPEED)


def do_fly(drone: DroneInterface, origin: str, dest: str, speed: float) -> None:
    """
    Volar a 'dest'. Si 'dest' es un viewpoint con target asociado, el dron llega
    orientado hacia ese target (listo para fotografiar); si no, se orienta hacia
    el propio destino.
    """
    p_dest = list(COORDS[dest])
    # ¿el destino es un viewpoint que fotografia algun target?
    if dest in VIEWPOINT_TARGET:
        face_point = list(COORDS[VIEWPOINT_TARGET[dest]])   # mirar al target
        face_name = VIEWPOINT_TARGET[dest]
    else:
        face_point = p_dest # mirar al destino
        face_name = dest
    # yaw calculado DESDE el destino HACIA el punto a encarar (igual que el caso 1 dron)
    yaw = yaw_to_face(p_dest, face_point)
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] fly {origin} -> {dest} facing {face_name} '
          f'(yaw={math.degrees(yaw):.1f} deg)')

    drone.go_to.go_to_point_with_yaw(p_dest, angle=yaw, speed=speed)

    print(f'[{drone.drone_id}] in {dest}')


def do_land(drone: DroneInterface, air: str, ground: str) -> None:
    """Aterrizar de la base aerea a la de suelo (bloqueante)."""
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] land -> {ground}')
    drone.land(speed=LAND_SPEED)


def do_take_photo(drone: DroneInterface, viewpoint: str, target: str) -> None:
    """Orientar el gimbal hacia el target y fotografiar (bloqueante)."""
    p_vp = list(COORDS[viewpoint])
    p_tgt = list(COORDS[target])
    t = elapsed(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] take_photo {viewpoint} -> {target}')
    gimbal_orientation(drone, p_vp, p_tgt)
    take_photo(drone, f'photo_{drone.drone_id}_{target}.png')


# =============================================================================
#  MISION DE UN DRON  (esto es lo que corre en CADA HILO)
# =============================================================================
def drone_mission(drone: DroneInterface, plan: list) -> None:
    """
    Ejecutar la mision completa de UN dron de forma secuencial y bloqueante.
    Identico en espiritu a mission_problem1.py, pero recorriendo el 'plan' en vez
    de una lista fija de viewpoints. Corre dentro del hilo propio del dron.
    """
    ns = drone.drone_id
    speed = SPEEDS.get(ns, DEFAULT_SPEED)

    # --- recorrer las acciones del plan, una tras otra (bloqueante) ---
    for action in plan:
        kind = action[0]
        if kind == "takeoff":
            do_takeoff(drone, action[1], action[2])
        elif kind == "fly":
            do_fly(drone, action[1], action[2], speed)
        elif kind == "take_photo":
            do_take_photo(drone, action[1], action[2])
        elif kind == "land":
            do_land(drone, action[1], action[2])
        else:
            print(f'[{ns}] accion desconocida: {kind}')

    # --- devolver a modo manual al terminar (igual que drone_end) ---
    drone.manual()
    t = elapsed(drone)
    print(f'[{t:7.3f}][{ns}] mision completada')


# =============================================================================
#  MAIN: crear drones, lanzar un hilo por dron, esperar a todos.
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description='Multi-drone threaded executor')
    parser.add_argument('-v', '--verbose', action='store_true', default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time', action='store_true', default=True,
                        help='Use simulation time')
    args = parser.parse_args()

    rclpy.init()

    # Crear todas las DroneInterface en el hilo PRINCIPAL (inicializacion segura).
    drones = {}
    for ns in PLANS:
        print(f'Creando interfaz de {ns}')
        drones[ns] = DroneInterface(
            drone_id=ns, use_sim_time=args.use_sim_time, verbose=args.verbose)
        
    # --- armado y offboard (igual que drone_start de mission_problem1) ---    
    for drone in drones.values():
        print(f'[{drone.drone_id}] Arm')
        drone.arm()
        print(f'[{drone.drone_id}] Offboard')
        drone.offboard()

    # Un hilo por dron: cada uno ejecuta su mision de forma independiente.
    threads = []
    for ns, plan in PLANS.items():
        t = threading.Thread(target=drone_mission, args=(drones[ns], plan), name=ns)
        threads.append(t)

    # Arrancar todos los hilos (los drones empiezan a la vez).
    for t in threads:
        t.start()

    # Esperar a que TODOS los hilos terminen su mision.
    for t in threads:
        t.join()

    print('Todas las misiones han terminado')

    # Cierre ordenado de cada dron y de rclpy (en el hilo principal).
    for drone in drones.values():
        drone.shutdown()
        
    rclpy.shutdown()
    print('Clean exit')

    show_all_photos()


if __name__ == '__main__':
    main()