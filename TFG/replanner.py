#!/usr/bin/env python3

"""
replanner.py — Replanificacion sobre la marcha de una mision multi-dron.

La mision deja de ser "un plan que se ejecuta entero" y pasa a ser una
secuencia de FASES. Cada fase ejecuta un plan hasta que ocurre algo que lo
invalida; entonces se reconstruye el escenario con el estado REAL de los drones
y se vuelve a planificar desde ahi.

Causas que interrumpen una fase (todas acaban en df.request_replan):
  - bloqueo mutuo: dos drones se esperan mutuamente y salta WP_WAIT_TIMEOUT
  - averia programada: la clave FAILURES del escenario (fallo reproducible)

Ciclo completo
--------------
    1. execute_phase()      -> ejecuta el plan; los hilos salen en frontera de
                               accion, asi que cada dron queda en un waypoint
                               bien definido
    2. return_to_base()     -> los drones averiados vuelven a casa por un
                               corredor de altitud, EN PARALELO con el paso 3
    3. rebuild_scenario()   -> escenario nuevo: posiciones reales, targets ya
                               fotografiados, y sin los drones averiados
    4. generate + planner   -> plan nuevo
    5. vuelta al paso 1

Uso desde un script de problema:

    import replanner as rp
    rp.run_mission(drones, scenario, plans, planner='MA-LAMA')
"""

import copy
import os
import re

import yaml

import drone_functions as df
import problem_generator as pg
from plan_parser import load_malama_plans, load_optic_plan
from run_planner import run_malama, run_optic


# Tope de replanificaciones: si el estado que provoca el fallo se reprodujera
# igual, el bucle podria no terminar nunca.
MAX_REPLANS = 3

# Donde se dejan los escenarios de cada fase (para depurar y para la memoria).
GENERATED_DIR = './TFG/problems/generated'


# =============================================================================
#  RECONSTRUCCION DEL ESCENARIO
# =============================================================================
def drone_bases(scenario: dict, ns: str) -> tuple:
    """(base_ground, base_air) de un dron, a partir de su goal y LANDING_PADS.

    Se usa 'goal' (su base de destino) y no 'start', porque tras la primera fase
    'start' ya es el waypoint donde se quedo, no su casa.
    """
    datos = scenario.get('DRONES', {}).get(ns, {})
    ground = datos.get('goal') or datos.get('start')
    for pad in scenario.get('LANDING_PADS', []):
        if pad[0] == ground:
            return pad[0], pad[1]
    return ground, None


def rebuild_scenario(scenario: dict, positions: dict, photographed: list,
                     failed: set, phase: int) -> dict:
    """Escenario nuevo a partir del estado REAL de la mision.

    - cada dron arranca donde se quedo (positions, sacado de WP_OCUPIED)
    - los targets ya fotografiados pasan a PHOTOGRAPHED
    - los drones averiados desaparecen, junto con sus bases y las conexiones de
      esas bases: asi el planificador no puede mandar a nadie a ese espacio
    - la bateria NO se arrastra: se mantiene la del escenario original

    Se trabaja sobre una copia; el escenario que se esta volando no se toca
    (los drones averiados todavia necesitan las coordenadas de su base).
    """
    nuevo = copy.deepcopy(scenario)

    # --- 1. Quitar los drones averiados, sus bases y sus conexiones ---
    wps_fuera = set()
    for ns in failed:
        if ns not in nuevo.get('DRONES', {}):
            continue
        ground, air = drone_bases(nuevo, ns)
        wps_fuera.update(wp for wp in (ground, air) if wp)
        del nuevo['DRONES'][ns]

    if wps_fuera:
        for wp in wps_fuera:
            nuevo.get('COORDS', {}).pop(wp, None)
            nuevo.get('WAYPOINTS', {}).pop(wp, None)
        nuevo['LANDING_PADS'] = [p for p in nuevo.get('LANDING_PADS', [])
                                 if not wps_fuera & set(p[:2])]
        nuevo['VALID_PATHS'] = [p for p in nuevo.get('VALID_PATHS', [])
                                if not wps_fuera & set(p)]
        print(f'Fuera de la mision: {sorted(failed)} (waypoints {sorted(wps_fuera)})')

    # Las averias ya disparadas no deben volver a dispararse.
    if nuevo.get('FAILURES'):
        nuevo['FAILURES'] = [f for f in nuevo['FAILURES']
                             if f.get('drone') in nuevo.get('DRONES', {})]

    # --- 2. Cada dron arranca donde esta de verdad ---
    for ns, datos in nuevo.get('DRONES', {}).items():
        if ns in positions:
            datos['start'] = positions[ns]

    # --- 3. Targets ya fotografiados ---
    nuevo['PHOTOGRAPHED'] = sorted(set(nuevo.get('PHOTOGRAPHED') or []) | set(photographed))

    # --- 4. Nombres de fichero propios de la fase (no pisar los anteriores) ---
    config = nuevo['CONFIG']
    base_nombre = re.sub(r'_fase\d+$', '', config['problem_name'])
    base_fichero = re.sub(r'_fase\d+$', '', os.path.splitext(config['output_file'])[0])
    config['problem_name'] = f'{base_nombre}_fase{phase}'
    config['output_file'] = f'{base_fichero}_fase{phase}.pddl'

    return nuevo


def dump_scenario(scenario: dict, path: str) -> None:
    """Guardar el escenario de la fase en YAML (trazabilidad y depuracion)."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(scenario, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    print(f'Escenario de la fase guardado en: {path}')


# =============================================================================
#  PLANIFICACION
# =============================================================================
def generate_and_plan(scenario: dict, planner: str) -> dict:
    """Generar el PDDL de este escenario, planificar y devolver los planes.

    Devuelve None si el planificador no encuentra solucion.
    """
    pg.generate_problem(scenario)

    config = scenario['CONFIG']
    if planner == 'MA-LAMA':
        # Si tras las averias solo queda un dron, hay que salir del modo
        # multiagente: con un unico agente MA-LAMA entra en un bucle infinito.
        multiagente = len(scenario.get('DRONES', {})) > 1
        if not run_malama(config, multiagente=multiagente):
            return None
        return load_malama_plans(os.path.expanduser('~/MA-LAMA'))

    if not run_optic(config):
        return None
    return load_optic_plan(os.path.expanduser('~/OPTIC/optic_plan.txt'))


def pending_targets(scenario: dict) -> list:
    """Targets que aun quedan por fotografiar en este escenario."""
    hechos = set(scenario.get('PHOTOGRAPHED') or [])
    return [t for t in scenario.get('TARGETS', []) if t not in hechos]


# =============================================================================
#  BUCLE DE FASES
# =============================================================================
def start_returns(drones: dict, scenario: dict, failed: set) -> list:
    """Lanzar el retorno a base de los drones averiados (hilos, no bloquea).

    Se llama ANTES de reconstruir el escenario, porque necesita las coordenadas
    de las bases que el escenario nuevo ya no tendra. Vuelan mientras el
    planificador trabaja.
    """
    import threading

    altitud = df.rtl_altitude(scenario)
    hilos = []
    for ns in sorted(failed):
        if ns not in drones:
            continue
        ground, _ = drone_bases(scenario, ns)
        if not ground or ground not in scenario['COORDS']:
            print(f'[{ns}] sin base conocida, no se puede volver: se deja donde esta')
            continue
        base_xy = scenario['COORDS'][ground][:2]
        speed = scenario['DRONES'].get(ns, {}).get('speed', df.DEFAULT_SPEED)
        t = threading.Thread(target=df.return_to_base,
                             args=(drones[ns], base_xy, altitud, speed),
                             name=f'rtl_{ns}')
        t.start()
        hilos.append(t)
    return hilos


def run_mission(drones: dict, scenario: dict, plans: dict,
                planner: str = 'MA-LAMA', max_replans: int = MAX_REPLANS) -> None:
    """Ejecutar la mision completa, replanificando cuando haga falta.

    'plans' es el plan de la fase 0 (el que ya venia calculado). A partir de ahi,
    cada interrupcion genera un escenario y un plan nuevos.
    """
    # Preparacion de la MISION (una sola vez, no por fase).
    df.clear_photos()
    with df.MISSION_LOG_LOCK:
        df.MISSION_LOG.clear()
    df.reset_mission_clock()
    df.clear_failed()

    escenario = scenario
    fase = 0
    retirados = set()       # averiados que YA volvieron a base en fases anteriores
    todos_hilos_rtl = []    # se unen al final: no bloquean el resto de la mision

    while True:
        print(f'\n===== FASE {fase} =====')
        motivo = df.execute_phase(drones, escenario, plans)

        if motivo is None:
            print('\nMision completada: no queda nada que replanificar')
            break

        if fase >= max_replans:
            print(f'\nLimite de {max_replans} replanificaciones alcanzado; se aborta')
            break

        fase += 1
        print(f'\n===== REPLANIFICACION {fase}: {motivo} =====')

        # Los averiados se van a casa en PARALELO con el resto de la mision: no
        # se espera a que aterricen para seguir. El corredor de altitud (por
        # encima de todas las rutas) es lo que hace esto seguro. Solo los que
        # aun no han vuelto: failed_drones() acumula, y los de fases anteriores
        # ya estan en tierra.
        averiados = df.failed_drones() - retirados
        todos_hilos_rtl.extend(start_returns(drones, escenario, averiados))
        retirados |= averiados

        escenario = rebuild_scenario(escenario, df.drone_waypoints(),
                                      df.photographed_targets(), averiados, fase)
        dump_scenario(escenario, os.path.join(
            GENERATED_DIR, f"{os.path.splitext(escenario['CONFIG']['output_file'])[0]}.yaml"))

        if not escenario.get('DRONES'):
            print('No quedan drones operativos; se termina la mision')
            break

        plans = generate_and_plan(escenario, planner)
        if not plans:
            print('El planificador no encontro solucion; se termina la mision')
            break

    # Se espera aqui, no en cada transicion de fase: los drones sanos ya
    # terminaron, pero antes de devolver el control (shutdown_drones cierra
    # rclpy) hay que asegurarse de que ningun RTL siga en el aire.
    for t in todos_hilos_rtl:
        t.join()

    df.write_mission_times()
