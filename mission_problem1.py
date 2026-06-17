
import argparse
import math
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

# =============================================================================
#  PUNTOS DEL ESCENARIO  (coordenadas en el frame 'earth': x, y, z en metros)
# =============================================================================
BASE_GROUND = [-5.0, 0.0, 0.0]   # base en suelo (punto de despegue / aterrizaje)
BASE_AIR = [-5.0, 0.0, 2.0]      # base aérea (sobre la base de suelo)

# Cada viewpoint se visita orientando el dron hacia su target correspondiente:
#   vp1 -> mira a tgt1     vp2 -> mira a tgt2
INSPECTION = [
    {'name': 'vp1', 'viewpoint': [-2.4, -2.4, 4.1], 'target': [-1.4, -1.4, 3.1]},
    {'name': 'vp2', 'viewpoint': [2.4, 2.4, 4.1], 'target': [1.4, 1.4, 3.1]},
]

# =============================================================================
#  PARÁMETROS DE VUELO
# =============================================================================
TAKE_OFF_HEIGHT = BASE_AIR[2]  # subir desde la base de suelo hasta la base aérea (2.0 m)
TAKE_OFF_SPEED = 1.0           # m/s
SPEED = 1.0                    # m/s (velocidad de crucero)
SLEEP_TIME = 0.5               # s entre behaviors
LAND_SPEED = 0.5               # m/s


def yaw_to_face(origin: list, target: list) -> float:

    dx = target[0] - origin[0]
    dy = target[1] - origin[1]
    yaw = math.atan2(dy, dx)

    return yaw


def take_photo(drone_interface, filename: str, timeout: float = 5.0) -> bool:

    topic = f'/{drone_interface.drone_id}/sensor_measurements/gimbal/camera/image_raw'

    success, msg = wait_for_message(
        Image, drone_interface, topic,
        qos_profile=qos_profile_sensor_data,
        time_to_wait=timeout)

    if not success:
        print(f'No se recibió imagen en {topic}')
        return False

    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

    PILImage.fromarray(array, mode='RGB').save(filename)
    print(f'Foto guardada: {filename} ({msg.width}x{msg.height})')
    return True


def gimbal_orientation(drone_interface: DroneInterface, viewpoint, target,
                       tol_deg: float = 1, pitch_rate: float = 0.5,
                       timeout: float = 3.0) -> bool:
    """
    Inclinar el gimbal en pitch hasta enfocar el target, en lazo cerrado.
    Gira con pitch_rate (rad/s) hasta que el error sea menor que tol_deg.

    :param viewpoint: [x, y, z] posicion del dron / viewpoint
    :param target: [x, y, z] punto a enfocar
    :param tol_deg: tolerancia en grados para considerar que ya apunta
    :param pitch_rate: velocidad angular de pitch (rad/s)
    :return: True si converge, False si no llega attitude
    """
    from geometry_msgs.msg import QuaternionStamped

    # --- pitch objetivo por geometria (angulo de depresion hacia el target) ---
    dx = target[0] - viewpoint[0]
    dy = target[1] - viewpoint[1]
    dz = target[2] - viewpoint[2]
    horizontal = math.hypot(dx, dy)
    pitch_target = -math.atan2(dz, horizontal)   # negativo si el target esta mas abajo
    print(f'  pitch objetivo: {math.degrees(pitch_target):.1f} deg')

    # --- publisher de comandos del gimbal ---
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
    converged = False

    while True:
        # leer pitch actual del gimbal
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

        # girar en el sentido del error (ajusta el signo si gira al reves)
        rate = pitch_rate if error > 0 else - pitch_rate
        publish(0.0, rate, 0.0)
        sleep(0.01)

    # parar el gimbal
    for _ in range(10):
        publish(0.0, 0.0, 0.0)
        sleep(0.05)

    drone_interface.destroy_publisher(publisher)
    print('gimbal orientado ')
    return True
    

def drone_start(drone_interface: DroneInterface) -> bool:
    """
    Take off the drone from the ground base up to the air base.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the take off was successful
    """

    print('Start mission')

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')

    # Take Off: de la base de suelo a la base aérea
    print(f'Take Off (ground -> air, height={TAKE_OFF_HEIGHT} m)')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone_interface: DroneInterface) -> bool:
    """
    Run the inspection mission: visit each viewpoint facing its target,
    then return to the air base.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """

    print('Run mission')

    # Visitar cada viewpoint orientando el dron hacia su target con go_to_point_with_yaw
    for wp in INSPECTION:
        viewpoint = wp['viewpoint']
        target = wp['target']
        name = wp["name"]

        yaw = yaw_to_face(viewpoint, target)
        print(f'Go to {name} {viewpoint} facing {target} '
              f'(yaw={yaw:.3f} rad / {math.degrees(yaw):.1f} deg)')
        success = drone_interface.go_to.go_to_point_with_yaw(
            viewpoint, angle=yaw, speed=SPEED)
        print(f'Go to success: {success}')
        if not success:
            return success
        print('Go to done')

        gimbal_orientation(drone_interface, viewpoint, target)

        nombre_archivo = f"photo_{name.replace('vp', 'tgt')}.png"
        take_photo(drone_interface, nombre_archivo)

        sleep(SLEEP_TIME)

    # Regreso a la base aérea antes de aterrizar
    print(f'Return to air base {BASE_AIR}')
    success = drone_interface.go_to.go_to_point(BASE_AIR, speed=SPEED)
    print(f'Go to success: {success}')
    if not success:
        return success
    print('Back at air base')
    sleep(SLEEP_TIME)

    return success


def drone_end(drone_interface: DroneInterface) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone inspection mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f'Running mission for drone {drone_namespace}')

    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)

    success = drone_start(uav)

    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)