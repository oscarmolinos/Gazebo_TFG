#!/usr/bin/env python3

'''
drone_functions.py — Módulo reutilizable para misiones multi-dron en Aerostack2.

Contiene toda la lógica de ejecución (control de drones, gimbal, cámara, hilos,
bloqueo de waypoints, detección de averías) para que un script de problema solo
tenga que aportar los datos del escenario y el plan.

------------------------------------------------------------------------------
CÓMO USARLO DESDE UN SCRIPT DE PROBLEMA
------------------------------------------------------------------------------
Los datos del escenario llegan en un diccionario 'scenario' (cargado del YAML
del problema) y los planes en un diccionario 'plans' (de plan_parser, formato
{dron: [(acción, arg1, arg2), ...]}). Ambos se pasan como ARGUMENTO a las
funciones. 'run_problem.py' es el punto de entrada real y sigue este flujo:

    import drone_functions as df
    import replanner as rp

    drones = df.create_drones(plans)          # rclpy.init() + una DroneInterface por dron
    rp.run_mission(drones, scenario, plans)    # arma, ejecuta y replanifica si hace falta
    df.shutdown_drones(drones)
    df.show_all_photos()

'rp.run_mission' (en replanner.py) es quien arma los drones y llama en bucle a
'execute_phase'/'execute_mission' de este módulo, generando un plan nuevo cada
vez que una fase termina en REPLAN. Si no hace falta replanificar, basta con:

    drones = df.create_drones(plans)
    df.arm_offboard(drones)
    df.execute_mission(drones, scenario, plans)
    df.shutdown_drones(drones)

Así, toda la lógica de ejecución vive aquí una sola vez, y cada problema nuevo
son solo los datos (scenario) + el plan.

Por qué un hilo por dron: las acciones de DroneInterface son bloqueantes, es decir
si se ejecuta una acción drone.go_to.go_to_point_with_yaw(), el código se quedaría 
ejecutando esa linea hasta que terminase la acción, haciendo imposible el movimiento 
de varios drones a la vez.

Estado compartido entre hilos (todo protegido por lock):
  - WP_OCUPIED: qué dron ocupa cada waypoint. Antes de cada desplazamiento el
    dron reserva su destino y libera su origen; si el destino está ocupado
    espera en WP_COND hasta que quede libre o salte WP_WAIT_TIMEOUT.
  - REPLAN: petición de replanificación (p.ej. timeout de waypoint o avería),
    comparte lock con WP_OCUPIED para poder despertar a los drones dormidos.
  - PHOTOGRAPHED / FAILED: qué se arrastra entre fases al replanificar
    (targets ya fotografiados, drones averiados).
  - PHASE / T0: reloj de misión, se reinicia en cada fase mientras PHASE
    acumula el offset para que tiempos_mision.txt tenga los tiempos totales.
'''

import math
import threading
from dataclasses import dataclass
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


# Parámetros de vuelo (constantes; iguales para todos los problemas).
DEFAULT_SPEED = 1.0
DEFAULT_LAND_SPEED = 0.5
DEFAULT_TAKEOFF_SPEED = 1.0

# Instante de simulación que marca el t=0.0 (primer takeoff de cualquier dron).
# Se comparte entre hilos; el lock protege su escritura inicial.
T0 = {'value': None}
T0_LOCK = threading.Lock()

# Registro de acciones ejecutadas (tiempos reales) para tiempos_mision.txt.
# Se comparte entre hilos (un hilo por dron), de ahi el lock.
MISSION_LOG = []
MISSION_LOG_LOCK = threading.Lock()

# Waypoints ocupados: {waypoint: dron_que_lo_ocupa}. Evita que dos drones se
# dirijan al mismo punto: antes de cualquier desplazamiento (takeoff/fly/land)
# el dron reserva su destino y libera su origen, todo bajo el mismo lock.
# Si el destino está ocupado, el hilo espera en la Condition hasta que se libere.
WP_OCUPIED = {}
WP_OCUPIED_LOCK = threading.Lock()
WP_COND = threading.Condition(WP_OCUPIED_LOCK)

# Tiempo máximo de simulación que un dron espera a que su destino quede libre.
# Pasado ese plazo se pide replanificar (el caso tipico es un bloqueo
# mutuo entre drones).
WP_WAIT_TIMEOUT = 10.0

# Intervalo de sondeo (tiempo real) para comprobar WP_WAIT_TIMEOUT contra el
# reloj de simulación. No decide el timeout, solo la frecuencia con la que se
# reevalua; el notify_all() sigue despertando el wait_for al instante.
WP_POLL = 0.5

# Petición de replanificación. Se protege con WP_COND (el mismo lock que
# WP_OCUPIED) a propósito: asi el notify_all() despierta también a los drones
# dormidos esperando un waypoint, que de otro modo seguirían bloqueados hasta
# agotar WP_WAIT_TIMEOUT sin enterarse de que hay que replanificar.
REPLAN = {'flag': False, 'reason': None}

# Fase de ejecución: 'index' es el número de plan ejecutado (0 = plan inicial,
# 1 o + = replanificaciones) y 'offset' el tiempo de misión acumulado por las fases
# anteriores. El reloj T0 se reinicia en cada fase, porque los t_start del plan
# nuevo vuelven a empezar en 0; 'offset' permite seguir informando de tiempos
# absolutos de misión en tiempos_mision.txt.
PHASE = {'index': 0, 'offset': 0.0}

# Estado que hay que arrastrar de una fase a la siguiente al replanificar:
#   PHOTOGRAPHED -> targets ya fotografiados (no hay que repetirlos)
#   FAILED -> drones averiados, que salen de la misión y vuelven a base
# La bateria NO se arrastra: cada fase parte del battery_level del escenario.
PHOTOGRAPHED = set()
FAILED = set()
STATE_LOCK = threading.Lock()

# Altura del corredor de vuelta a casa cuando CONFIG no la fija: por encima del
# punto más alto del escenario, para no invadir ninguna ruta.
RTL_MARGIN = 1.0


@dataclass
class ActionRecord:
    """Tiempos reales (medidos, no planificados) de una acción ya ejecutada."""
    drone: str
    kind: str
    arg1: str
    arg2: str
    t_start: float
    t_end: float
    phase: int = 0

    @property
    def duration(self) -> float:
        return self.t_end - self.t_start


# =============================================================================
#  FUNCIONES AUXILIARES
# =============================================================================
def sim_time(drone) -> float:
    """Tiempo de simulación en segundos (reloj ROS, respeta use_sim_time)."""
    return drone.get_clock().now().nanoseconds * 1e-9


def elapsed(drone) -> float:
    """Segundos de simulación desde el t=0 de la fase actual.

    Es el reloj contra el que se comparan los t_start del plan, que en cada
    replanificación vuelven a empezar en 0. Si el reloj no se ha fijado
    explicitamente (start_phase), se fija en la primera llamada.
    """
    with T0_LOCK:
        if T0['value'] is None:
            T0['value'] = sim_time(drone)
    return sim_time(drone) - T0['value']


def mission_time(drone) -> float:
    """Segundos desde el arranque de la misión (suma de todas las fases).

    Es lo que se imprime en los logs y se guarda en tiempos_mision.txt, para que
    los tiempos sigan acumulándose aunque el reloj de fase se reinicie.
    """
    return PHASE['offset'] + elapsed(drone)


def reset_mission_clock() -> None:
    """Volver al principio de la misión (fase 0, sin tiempo acumulado)."""
    with T0_LOCK:
        T0['value'] = None
        PHASE['index'] = 0
        PHASE['offset'] = 0.0


def start_phase(drones: dict) -> None:
    """Fijar el t=0 de la fase y incrementar la fase actual.

    Si ya había una fase en marcha, su tiempo se acumula en PHASE['offset'] antes 
    de reiniciar el reloj.
    """
    any_drone = next(iter(drones.values()))
    with T0_LOCK:
        now = sim_time(any_drone)
        if T0['value'] is not None:
            PHASE['offset'] += now - T0['value']
            PHASE['index'] += 1
        T0['value'] = now


def wait_until(drone, t_target: float) -> None:
    """Bloquear hasta que el reloj de misión alcance t_target (s desde t=0)."""
    while elapsed(drone) < t_target:
        sleep(0.01)


def record_action(drone: str, kind: str, arg1: str, arg2: str,
                   t_start: float, t_end: float) -> None:
    """Registrar los tiempos reales de una acción ya ejecutada."""
    with MISSION_LOG_LOCK:
        MISSION_LOG.append(
            ActionRecord(drone, kind, arg1, arg2, t_start, t_end, PHASE['index']))


def request_replan(reason: str) -> None:
    """Pedir que se interrumpa la fase actual y se replanifique.

    Los hilos de los drones terminan la acción que estuvieran ejecutando y salen.
    Los que estuvieran dormidos esperando un waypoint despiertan aquí mismo por
    el notify_all(), sin agotar WP_WAIT_TIMEOUT.
    """
    with WP_COND:
        if REPLAN['flag']:
            return                      # ya habia una petición en curso
        REPLAN['flag'] = True
        REPLAN['reason'] = reason
        WP_COND.notify_all()
    print(f'*** REPLANIFICACIÓN solicitada: {reason} ***')


def replan_requested() -> bool:
    """True si hay una replanificación pendiente."""
    with WP_COND:
        return REPLAN['flag']


def clear_replan() -> None:
    """Limpiar la petición de replanificación (al arrancar una fase nueva)."""
    with WP_COND:
        REPLAN['flag'] = False
        REPLAN['reason'] = None


def write_mission_times(path: str = "./TFG/tiempos_mision.txt") -> None:
    """Crear (o sobreescribir) el informe de tiempos reales de la misión.

    Una fila por acción ejecutada (t_inicio, acción, t_fin, duración) agrupada
    por dron, más un resumen con el tiempo total de cada dron y el makespan
    real de la misión completa.
    """
    with MISSION_LOG_LOCK:
        records = list(MISSION_LOG)

    if not records:
        print('No hay tiempos que guardar (MISSION_LOG vacio)')
        return

    by_drone = {}
    for r in records:
        by_drone.setdefault(r.drone, []).append(r)

    lines = ['TIEMPOS DE MISION', '=================', '']

    # Solo se muestra la columna de fase si de verdad hubo replanificación.
    con_fases = any(r.phase for r in records)

    for drone in sorted(by_drone):
        actions = by_drone[drone]
        lines.append(f'Dron: {drone}')
        lines.append('-' * (6 + len(drone)))
        cab_fase = f"{'fase':>5}" if con_fases else ''
        lines.append(f"{cab_fase}{'t_inicio':>10}  {'acción':<12}{'arg1':<16}{'arg2':<16}"
                      f"{'t_fin':>10}{'duración':>12}")
        for r in actions:
            fase = f'{r.phase:5d}' if con_fases else ''
            lines.append(
                f'{fase}{r.t_start:10.3f}  {r.kind:<12}{r.arg1:<16}{r.arg2:<16}'
                f'{r.t_end:10.3f}{r.duration:12.3f}')
        lines.append('')

    lines.append('RESUMEN')
    lines.append('-------')
    total_mision = 0.0
    for drone in sorted(by_drone):
        t_total_dron = by_drone[drone][-1].t_end
        total_mision = max(total_mision, t_total_dron)
        lines.append(f'  {drone}: tiempo total = {t_total_dron:.3f} s')
    lines.append('')
    if con_fases:
        lines.append(f'Fases ejecutadas: {max(r.phase for r in records) + 1} '
                      f'(0 = plan inicial, 1 o + = replanificaciones)')
    lines.append(f'Tiempo total de la misión (makespan): {total_mision:.3f} s')

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')

    print(f'Tiempos de misión guardados en: {path}')


def init_wp_ocupied(scenario: dict, plans: dict) -> None:
    """Estado inicial de WP_OCUPIED: cada dron ocupa su punto de partida.

    El punto de partida se lee de scenario['DRONES'][dron]['start']; si no
    estuviera, se usa el origen (arg1) de la primera acción de su plan.
    """
    with WP_OCUPIED_LOCK:
        WP_OCUPIED.clear()
        for ns, plan in plans.items():
            start = scenario.get('DRONES', {}).get(ns, {}).get('start')
            if not start and plan:
                start = plan[0].arg1
            if start:
                WP_OCUPIED[start] = ns


def reserve_wp(drone: DroneInterface, origin: str, dest: str) -> bool:
    """Reservar 'dest' para este dron y liberar 'origin' (bloqueante).

    Si 'dest' está ocupado por otro dron, el hilo se queda esperando hasta que
    lo libere. La reserva del destino y la liberación del origen ocurren en la
    misma sección critica, para que nadie pueda colarse entre las dos.

    Devuelve False si hay que abortar (replanificación pedida, o timeout que la
    provoca). En ese caso NO se toca WP_OCUPIED: el dron sigue registrado en su
    origen, que es donde realmente está, y ese estado es el que usara el
    escenario replanificado.
    """
    ns = drone.drone_id
    with WP_COND:
        if REPLAN['flag']:
            return False

        ocupante = WP_OCUPIED.get(dest)
        if ocupante is not None and ocupante != ns:
            print(f'[{mission_time(drone):7.3f}][{ns}] espera: {dest} ocupado por {ocupante}')

            # WP_WAIT_TIMEOUT se mide en tiempo de SIMULACION: un unico
            # wait_for con ese timeout usaria el reloj real, que depende
            # del real time factor de Gazebo y rompe la reproducibilidad. Se
            # sondea cada WP_POLL segundos reales y se compara contra el reloj
            # de misión; el notify_all() sigue despertando al instante en el
            # caso normal, el sondeo solo decide cuando darse por vencido.
            limite = mission_time(drone) + WP_WAIT_TIMEOUT
            libre = False
            while not libre and not REPLAN['flag'] and mission_time(drone) < limite:
                libre = WP_COND.wait_for(
                    lambda: REPLAN['flag'] or WP_OCUPIED.get(dest) in (None, ns),
                    timeout=WP_POLL)

            if REPLAN['flag']:
                print(f'[{mission_time(drone):7.3f}][{ns}] espera cancelada por replanificación')
                return False

            if libre:
                print(f'[{mission_time(drone):7.3f}][{ns}] {dest} libre, continua')
            else:
                # Nadie ha soltado el waypoint en todo el plazo: lo normal es un
                # bloqueo mutuo, que solo se rompe replanificando.
                motivo = (f'timeout de {WP_WAIT_TIMEOUT:.0f} s de {ns} esperando '
                          f'{dest} (ocupado por {WP_OCUPIED.get(dest)})')
                REPLAN['flag'] = True
                REPLAN['reason'] = motivo
                WP_COND.notify_all()
                print(f'*** REPLANIFICACIÓN solicitada: {motivo} ***')
                return False

        WP_OCUPIED[dest] = ns
        if origin and WP_OCUPIED.get(origin) == ns:
            del WP_OCUPIED[origin]
        WP_COND.notify_all()
        return True


def release_wp(ns: str, wp: str) -> None:
    """Liberar 'wp' si lo ocupa 'ns'."""
    with WP_COND:
        if WP_OCUPIED.get(wp) == ns:
            del WP_OCUPIED[wp]
            WP_COND.notify_all()


def drone_waypoints() -> dict:
    """Copia de {dron: waypoint} para reconstruir el escenario al replanificar."""
    with WP_OCUPIED_LOCK:
        return {ns: wp for wp, ns in WP_OCUPIED.items()}


# -----------------------------------------------------------------------------
#  ESTADO QUE SE GUARDA ENTRE FASES (targets fotografiados / drones averiados)
# -----------------------------------------------------------------------------
def init_photographed(scenario: dict) -> None:
    """Cargar los targets ya fotografiados que declare el escenario."""
    with STATE_LOCK:
        PHOTOGRAPHED.clear()
        PHOTOGRAPHED.update(scenario.get('PHOTOGRAPHED') or [])


def mark_photographed(target: str) -> None:
    """Anotar que 'target' ya está fotografiado (no se repetira al replanificar)."""
    with STATE_LOCK:
        PHOTOGRAPHED.add(target)


def photographed_targets() -> list:
    """Lista ordenada de targets fotografiados hasta ahora."""
    with STATE_LOCK:
        return sorted(PHOTOGRAPHED)


def mark_failed(ns: str) -> None:
    """Marcar un dron como averiado: sale de la misión y vuelve a su base."""
    with STATE_LOCK:
        FAILED.add(ns)


def failed_drones() -> set:
    """Copia del conjunto de drones averiados."""
    with STATE_LOCK:
        return set(FAILED)


def clear_failed() -> None:
    """Olvidar las averias (al arrancar una misión nueva)."""
    with STATE_LOCK:
        FAILED.clear()


# -----------------------------------------------------------------------------
#  FUNCIONES PARA LA TOMA Y LA REPRESENTACIÓN DE FOTOGRAFÍAS
# -----------------------------------------------------------------------------
def yaw_to_face(viewpoint: list, target: list) -> float:
    """Ángulo (rad) entre 'viewpoint' y 'target'."""
    dx = target[0] - viewpoint[0]
    dy = target[1] - viewpoint[1]
    return math.atan2(dy, dx)


def take_photo(drone_interface, filename: str, timeout: float = 5.0,
               photos_dir: str = "./TFG/photos") -> bool:
    """Capturar un frame de la camara y guardarlo como PNG en photos_dir."""
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
    """Inclinar el gimbal en pitch hasta enfocar el target."""
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
    """Borrar las fotos de la misión anterior antes de empezar una nueva."""
    for path in glob.glob(os.path.join(photos_dir, pattern)):
        os.remove(path)


def show_all_photos(photos_dir: str = "./TFG/photos", pattern: str = "photo_*.png") -> None:
    """Mostrar todas las fotos de la misión (de photos_dir) en una sola figura."""
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

    fig.suptitle(f'Fotos de la misión ({n})', fontsize=14)
    fig.tight_layout()
    plt.show()


# =============================================================================
#  EJECUCIÓN DE CADA TIPO DE ACCIÓN
# =============================================================================
def do_takeoff(drone: DroneInterface, ground: str, air: str, speed: float,
               coords: dict) -> None:
    """Despegar de la base de suelo a la aerea (altura = z del punto air)."""
    height = coords[air][2]
    t = mission_time(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] takeoff -> {air} (h={height} m)')
    drone.arm()
    drone.offboard()
    drone.takeoff(height=height, speed=speed)


def do_fly(drone: DroneInterface, origin: str, dest: str, speed: float,
           coords: dict, viewpoint_target: dict) -> None:
    """
    Volar a 'dest'. Si 'dest' es un viewpoint con target asociado, el dron llega
    orientado hacia ese target; si no, sigue el path encarado al destino.
    """
    p_dest = list(coords[dest])
    t = mission_time(drone)

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


def do_land(drone: DroneInterface, air: str, ground: str, speed: float) -> None:
    """Aterrizar de la base aerea a la de suelo."""
    t = mission_time(drone)
    print(f'[{t:7.3f}][{drone.drone_id}] land -> {ground}')
    drone.land(speed=speed)


def do_recharge(drone: DroneInterface, waypoint: str, duration: float) -> None:
    """Recargar bateria en 'waypoint': el dron está parado, solo se espera.

    No hay bateria fÍsica que cargar en simulacion, asi que la acción se limita
    a consumir el tiempo que el planificador le asigno (duration segundos).
    """
    t_start = elapsed(drone)
    print(f'[{mission_time(drone):7.3f}][{drone.drone_id}] recharge en {waypoint} ({duration:.3f} s)')

    while elapsed(drone) - t_start < duration:
        sleep(0.1)

    print(f'[{mission_time(drone):7.3f}][{drone.drone_id}] recharge completada')


def do_take_photo(drone: DroneInterface, viewpoint: str, target: str, coords: dict) -> None:
    """Orientar el gimbal hacia el target y fotografiar exactamente 5 segundos."""
    p_vp = list(coords[viewpoint])
    p_tgt = list(coords[target])
    t_start = elapsed(drone)
    print(f'[{mission_time(drone):7.3f}][{drone.drone_id}] take_photo {viewpoint} -> {target}')
    gimbal_orientation(drone, p_vp, p_tgt)
    take_photo(drone, f'photo_{drone.drone_id}_{target}.png')

    while elapsed(drone) - t_start < 5.0:
        sleep(0.01)


# =============================================================================
#  RETORNO A BASE DE UN DRON AVERIADO
#
#  No usa el grafo de waypoints: el dron sube a una altura por encima de todas
#  las rutas, cruza hasta la vertical de su base y aterriza. Así no compite por
#  ningún waypoint ni interfiere con los drones que siguen trabajando, y puede
#  volar mientras el planificador calcula el plan nuevo.
# =============================================================================
def rtl_altitude(scenario: dict) -> float:
    """Altura del corredor de vuelta a casa.

    Si CONFIG no la fija (rtl_altitude), se toma el punto más alto del escenario
    más RTL_MARGIN, para pasar por encima de cualquier ruta y de los obstaculos.
    """
    config = scenario.get('CONFIG', {})
    if config.get('rtl_altitude') is not None:
        return float(config['rtl_altitude'])
    max_z = max(p[2] for p in scenario['COORDS'].values())
    return max_z + float(config.get('rtl_margin', RTL_MARGIN))


def return_to_base(drone: DroneInterface, base_xy: list, altitude: float,
                   speed: float = DEFAULT_SPEED,
                   land_speed: float = DEFAULT_LAND_SPEED) -> None:
    """Llevar un dron averiado a su base: subir, cruzar y aterrizar.

    Solo se necesitan las coordenadas (x, y) de la base.
    """
    ns = drone.drone_id

    # Sale del grafo de waypoints: libera el suyo para que otros puedan usarlo.
    wp_actual = drone_waypoints().get(ns)
    if wp_actual:
        release_wp(ns, wp_actual)
        print(f'[{mission_time(drone):7.3f}][{ns}] RTL: libera {wp_actual}')

    pos = drone.position
    print(f'[{mission_time(drone):7.3f}][{ns}] RTL: sube a {altitude:.1f} m')
    # KEEP_YAW en los tramos verticales: sin avance horizontal, el modo path
    # facing no tiene direccion que encarar.
    drone.go_to.go_to_point([pos[0], pos[1], altitude], speed=speed)

    print(f'[{mission_time(drone):7.3f}][{ns}] RTL: crucero a '
          f'({base_xy[0]:.1f}, {base_xy[1]:.1f})')
    drone.go_to.go_to_point_path_facing([base_xy[0], base_xy[1], altitude], speed=speed)

    print(f'[{mission_time(drone):7.3f}][{ns}] RTL: aterrizando')
    drone.land(speed=land_speed)
    drone.manual()
    print(f'[{mission_time(drone):7.3f}][{ns}] RTL: en base, fuera de la misión')


def start_failure_watcher(drones: dict, scenario: dict) -> threading.Thread:
    """Lanzar el vigilante de averías programadas en el escenario.

    El escenario puede declararlas asi:

        FAILURES:
          - {drone: drone3, at_time: 20.0}

    Al llegar ese instante de misión se marca el dron como averiado y se pide
    replanificar. Tener el fallo preprogramado hace el experimento reproducible.
    Devuelve None si el escenario no declara ninguna avería.
    """
    failures = [f for f in (scenario.get('FAILURES') or [])
                if f.get('drone') in drones and f.get('drone') not in failed_drones()]
    if not failures:
        return None

    failures.sort(key=lambda f: float(f['at_time']))
    reloj = next(iter(drones.values()))

    def watcher():
        for f in failures:
            ns, t_fallo = f['drone'], float(f['at_time'])
            while mission_time(reloj) < t_fallo:
                if replan_requested():
                    return          # otra causa se adelanto; se reintenta en la fase siguiente
                sleep(0.05)
            mark_failed(ns)
            request_replan(f'averia simulada en {ns} (t={t_fallo:.1f} s)')
            return                  # la fase termina aquí; el resto se verá en la siguiente

    th = threading.Thread(target=watcher, name='failure_watcher', daemon=True)
    th.start()
    return th


# =============================================================================
#  MISION DE UN DRON  (esto es lo que corre en CADA HILO)
# =============================================================================
def drone_mission(drone: DroneInterface, plan: list, scenario: dict) -> None:
    """Ejecutar la misión completa de un dron, secuencial y bloqueante.
    Los datos del escenario (coordenadas, viewpoint->target, velocidad) se leen
    del dict 'scenario'."""
    ns = drone.drone_id
    coords = scenario['COORDS']
    viewpoint_target = scenario['CAN_PHOTOGRAPH']
    speed = scenario['DRONES'].get(ns, {}).get('speed', DEFAULT_SPEED)
    config = scenario.get('CONFIG', {})
    takeoff_speed = float(config.get('takeoff_speed', DEFAULT_TAKEOFF_SPEED))
    land_speed = float(config.get('land_speed', DEFAULT_LAND_SPEED))

    for action in plan:
        # Gate de replanificación: se comprueba al finalizar una acción, nunca a
        # mitad de una. El dron queda en un waypoint bien definido, que es lo que
        # necesita el escenario replanificado.
        if replan_requested():
            print(f'[{mission_time(drone):7.3f}][{ns}] fase interrumpida, a la espera de plan nuevo')
            return

        # Gate de agenda: no empezar la acción antes de su instante programado.
        # Un único mecanismo cubre los dos casos:
        #   - hueco de inactividad del planner -> t_start futuro, se espera.
        #   - la acción real anterior tardó menos -> se llega antes, se espera.
        # Es solo una cota inferior: si se tardó de más, t_start ya paso y no espera.
        wait_until(drone, action.t_start)

        kind = action.kind

        # Gate de ocupación: un desplazamiento solo empieza si su destino está
        # libre. Reserva el destino y libera el origen; si está ocupado, espera.
        # (take_photo y recharge no mueven al dron, no tocan WP_OCUPIED.)
        if kind in ('takeoff', 'fly', 'land'):
            if not reserve_wp(drone, action.arg1, action.arg2):
                print(f'[{mission_time(drone):7.3f}][{ns}] fase interrumpida en {action.arg1}, '
                      f'a la espera de plan nuevo')
                return

        t_real_start = mission_time(drone)
        if kind == "takeoff":
            do_takeoff(drone, action.arg1, action.arg2, takeoff_speed, coords)
        elif kind == "fly":
            do_fly(drone, action.arg1, action.arg2, speed, coords, viewpoint_target)
        elif kind == "take_photo":
            do_take_photo(drone, action.arg1, action.arg2, coords)
            mark_photographed(action.arg2)   # no habrá que repetirlo si se replanifica
        elif kind == "land":
            do_land(drone, action.arg1, action.arg2, land_speed)
        elif kind == "recharge":
            do_recharge(drone, action.arg1, action.duration)
        else:
            print(f'[{ns}] acción desconocida: {kind}')
            continue

        record_action(ns, kind, action.arg1, action.arg2, t_real_start, mission_time(drone))

    # Solo se sale de offboard cuando el plan se ha completado de verdad. Si la
    # fase se interrumpe (return de arriba) el dron sigue en offboard, quieto y
    # listo para recibir el plan siguiente.
    drone.manual()
    print(f'[{mission_time(drone):7.3f}][{ns}] misión completada')


# =============================================================================
#  CREACIÓN Y EJECUCIÓN  (separadas para que el script tenga los objetos drone)
#
#  El flujo se parte en tres pasos que el script puede usar por separado:
#    1) create_drones()    -> crea las DroneInterface y las devuelve
#    2) execute_mission()  -> lanza los hilos y ejecuta los planes
#    3) shutdown_drones()  -> cierre ordenado
#
#  Así el script tiene el diccionario de drones en su mano (para leer odometría,
#  bateria, etc.) entre la creación y la ejecución, o después.
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


def execute_phase(drones: dict, scenario: dict, plans: dict) -> str:
    """
    Ejecutar una fase (un plan) y esperar a que todos los hilos terminen.

    Devuelve None si la fase se completó entera, o el motivo (str) por el que
    hay que replanificar. No limpia fotos ni el registro de tiempos: es la
    función que usa el bucle de replanificación, que puede llamarla varias
    veces sobre los mismos drones.
    """
    clear_replan()

    # Cada dron ocupa su punto de partida antes de que arranque ningun hilo.
    init_wp_ocupied(scenario, plans)
    init_photographed(scenario)

    # Fijar el t=0 de la fase aquí, para que los t_start del plan (que vuelven a
    # empezar en 0 en cada replanificación) tomen este cero como referencia.
    start_phase(drones)

    # Averías programadas (FAILURES en el escenario), si las hay.
    start_failure_watcher(drones, scenario)

    threads = []
    for ns, plan in plans.items():
        t = threading.Thread(target=drone_mission, args=(drones[ns], plan, scenario), name=ns)
        threads.append(t)

    for t in threads:
        t.start()
    for t in threads:
        t.join()

    with WP_COND:
        return REPLAN['reason'] if REPLAN['flag'] else None


def execute_mission(drones: dict, scenario: dict, plans: dict) -> str:
    """
    Ejecutar la misión de una sola fase (sin replanificar) y guardar los tiempos.

    'drones' es el dict devuelto por create_drones(); 'scenario' aporta los datos
    del escenario y 'plans' el plan de acciones de cada dron. Devuelve el motivo
    de replanificación, o None si la misión se completó.
    """
    clear_photos()

    with MISSION_LOG_LOCK:
        MISSION_LOG.clear()
    reset_mission_clock()

    motivo = execute_phase(drones, scenario, plans)

    write_mission_times()
    if motivo:
        print(f'Fase interrumpida: {motivo}')
    else:
        print('Todas las misiones han terminado')
    return motivo


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
    Útil cuando NO necesitas acceder a los drones desde el script. Si si los
    necesitas (odometria, bateria...), usa create_drones() / execute_mission()
    por separado desde tu script.
    """
    drones = create_drones(plans, use_sim_time, verbose)
    arm_offboard(drones)
    execute_mission(drones, scenario, plans)
    shutdown_drones(drones)
    if show_photos:
        show_all_photos()