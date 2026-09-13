#!/usr/bin/env python3

"""
replanner.py — Replanificación sobre la marcha de una misión multi-dron.

La misión deja de ser "un plan que se ejecuta entero" y pasa a ser una
secuencia de fases. Cada fase ejecuta un plan hasta que ocurre algo que lo
invalida; entonces se reconstruye el escenario con el estado real de los drones
y se vuelve a planificar desde ahí.

Causas que interrumpen una fase (todas acaban en df.request_replan):
  - bloqueo mutuo: dos drones se esperan mutuamente y salta WP_WAIT_TIMEOUT
  - avería programada: la clave FAILURES del escenario (fallo reproducible)

Ciclo completo
--------------
    1. execute_phase()      -> ejecuta el plan; los hilos salen en frontera de
                               acción, así que cada dron queda en un waypoint
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
# igual, el bucle podría no terminar nunca.
MAX_REPLANS = 10

# Dónde se dejan los escenarios de cada fase (para depurar y para la memoria).
GENERATED_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'problems', 'generated')


# =============================================================================
#  RECONSTRUCCIÓN DEL ESCENARIO
# =============================================================================
def drone_bases(scenario: dict, ns: str) -> tuple:
    """(base_ground, base_air) de un dron, a partir de su goal y LANDING_PADS.

    Se usa 'goal' (su base de destino) y no 'start', porque tras la primera fase
    'start' es el waypoint donde se quedó, no su base.
    """
    datos = scenario.get('DRONES', {}).get(ns, {})
    ground = datos.get('goal') or datos.get('start')
    for pad in scenario.get('LANDING_PADS', []):
        if pad[0] == ground:
            return pad[0], pad[1]
    return ground, None


def rebuild_scenario(scenario: dict, positions: dict, photographed: list,
                     failed: set, phase: int) -> dict:
    """Escenario nuevo a partir del estado real de la misión.

    - cada dron arranca donde se quedó (positions, sacado de WP_OCUPIED)
    - los targets ya fotografiados pasan a PHOTOGRAPHED
    - los drones averiados desaparecen, junto con sus bases y las conexiones de
      esas bases: así el planificador no puede mandar a nadie a ese espacio
    - la batería no se arrastra: cada dron replanifica con el depósito lleno
      (battery_level = battery_capacity)

    Se trabaja sobre una copia; el escenario que se está volando no se toca
    (los drones averiados todavía necesitan las coordenadas de su base).
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
        print(f'Fuera de la misión: {sorted(failed)} (waypoints {sorted(wps_fuera)})')

    # Las averías ya disparadas no deben volver a dispararse.
    if nuevo.get('FAILURES'):
        nuevo['FAILURES'] = [f for f in nuevo['FAILURES']
                             if f.get('drone') in nuevo.get('DRONES', {})]

    # --- 2. Cada dron arranca donde está de verdad y con la batería al 100% ---
    for ns, datos in nuevo.get('DRONES', {}).items():
        if ns in positions:
            datos['start'] = positions[ns]
        datos['battery_level'] = datos.get('battery_capacity', datos.get('battery_level'))

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
    """Guardar el escenario de la fase en YAML (trazabilidad y depuración)."""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(scenario, f, allow_unicode=True, sort_keys=False,
                       default_flow_style=False)
    print(f'Escenario de la fase guardado en: {path}')


# =============================================================================
#  PLANIFICACIÓN
# =============================================================================
def generate_and_plan(scenario: dict, planner: str) -> dict:
    """Generar el PDDL de este escenario, planificar y devolver los planes.

    Devuelve None si el planificador no encuentra solución.
    """
    pg.generate_problem(scenario)

    config = scenario['CONFIG']
    if planner == 'MA-LAMA':
        # Si tras las averías solo queda un dron, hay que salir del modo
        # multiagente: con un único agente MA-LAMA entra en un bucle infinito.
        multiagente = len(scenario.get('DRONES', {})) > 1
        if not run_malama(config, multiagente=multiagente):
            return None
        return load_malama_plans(os.path.expanduser('~/MA-LAMA'))

    if not run_optic(config):
        return None
    return load_optic_plan(os.path.expanduser('~/OPTIC/optic_plan.txt'))


def pending_targets(scenario: dict) -> list:
    """Targets que aún quedan por fotografiar en este escenario."""
    hechos = set(scenario.get('PHOTOGRAPHED') or [])
    return [t for t in scenario.get('TARGETS', []) if t not in hechos]


# =============================================================================
#  BUCLE DE FASES
# =============================================================================
def start_returns(drones: dict, scenario: dict, failed: set) -> list:
    """Lanzar el retorno a base de los drones averiados (hilos, no bloquea).

    Se llama antes de reconstruir el escenario, porque necesita las coordenadas
    de las bases que el escenario nuevo ya no tendrá. Vuelan mientras el
    planificador trabaja.
    """
    import threading

    altitud = df.rtl_altitude(scenario)
    land_speed = float(scenario.get('CONFIG', {}).get('land_speed', df.DEFAULT_LAND_SPEED))
    hilos = []
    for ns in sorted(failed):
        if ns not in drones:
            continue
        ground, _ = drone_bases(scenario, ns)
        if not ground or ground not in scenario['COORDS']:
            print(f'[{ns}] sin base conocida, no se puede volver: se deja donde está')
            continue
        base_xy = scenario['COORDS'][ground][:2]
        speed = scenario['DRONES'].get(ns, {}).get('speed', df.DEFAULT_SPEED)
        t = threading.Thread(target=df.return_to_base,
                             args=(drones[ns], base_xy, altitud, speed, land_speed),
                             name=f'rtl_{ns}')
        t.start()
        hilos.append(t)
    return hilos


def run_mission(drones: dict, scenario: dict, plans: dict,
                planner: str = 'MA-LAMA', max_replans: int = MAX_REPLANS) -> None:
    """Ejecutar la misión completa, replanificando cuando haga falta.

    'plans' es el plan de la fase 0 (el que ya venía calculado). A partir de ahí,
    cada interrupción genera un escenario y un plan nuevos.
    """
    # Preparación de la MISIÓN (una sola vez, no por fase).
    df.clear_photos()
    with df.MISSION_LOG_LOCK:
        df.MISSION_LOG.clear()
    df.reset_mission_clock()
    df.clear_failed()

    escenario = scenario
    fase = 0
    retirados = set()       # averiados que YA volvieron a base en fases anteriores
    todos_hilos_rtl = []    # se unen al final: no bloquean el resto de la misión

    while True:
        print(f'\n===== FASE {fase} =====')
        motivo = df.execute_phase(drones, escenario, plans)

        if motivo is None:
            print('\nMisión completada: no queda nada que replanificar')
            break

        if fase >= max_replans:
            print(f'\nLímite de {max_replans} replanificaciones alcanzado; se aborta')
            break

        fase += 1
        print(f'\n===== REPLANIFICACIÓN {fase}: {motivo} =====')

        # Los averiados se van a casa en paralelo con el resto de la misión: no
        # se espera a que aterricen para seguir. El corredor de altitud (por
        # encima de todas las rutas) es lo que hace esto seguro. Solo los que
        # aún no han vuelto: failed_drones() acumula, y los de fases anteriores
        # ya están en tierra.
        averiados = df.failed_drones() - retirados
        todos_hilos_rtl.extend(start_returns(drones, escenario, averiados))
        retirados |= averiados

        escenario = rebuild_scenario(escenario, df.drone_waypoints(),
                                      df.photographed_targets(), averiados, fase)
        dump_scenario(escenario, os.path.join(
            GENERATED_DIR, f"{os.path.splitext(escenario['CONFIG']['output_file'])[0]}.yaml"))

        if not escenario.get('DRONES'):
            print('No quedan drones operativos; se termina la misión')
            break

        plans = generate_and_plan(escenario, planner)
        if not plans:
            print('El planificador no encontró solución; se termina la misión')
            break

    # Se espera aquí, no en cada transición de fase: los drones sanos ya
    # terminaron, pero antes de devolver el control (shutdown_drones cierra
    # rclpy) hay que asegurarse de que ningún RTL siga en el aire.
    for t in todos_hilos_rtl:
        t.join()

    df.write_mission_times()
