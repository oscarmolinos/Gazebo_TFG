#!/usr/bin/env python3

"""
Parser de los planes de MA-LAMA al diccionario PLANS que consume el ejecutor.

MA-LAMA genera un fichero por agente (plan_agent0.txt, plan_agent1.txt, ...).
Cada linea tiene el formato:

    <accion> <arg1> <arg2> <drone> [t_inicio, t_inicio, duracion]

Ejemplos:
    takeoff base1_ground base1_air drone1 [0.010, 0.010, 2.000]
    fly base1_air vp1 drone1 [2.010, 2.010, 3.460]
    take_photo vp1 drone1 tgt1 [5.470, 5.470, 5.000]
    land base1_air base1_ground drone1 [28.530, 28.530, 4.000]

Particularidad: en 'take_photo' el dron va EN MEDIO (posicion 3) y el target al
final -> 'take_photo <viewpoint> <drone> <target>'. En el resto, el dron va al
final -> '<accion> <arg1> <arg2> <drone>'.

Los TIEMPOS entre corchetes se ignoran (solo importa la secuencia de acciones).

Salida: dict { 'drone1': [ (tipo, arg1, arg2), ... ], 'drone2': [ ... ] }
  - takeoff/fly/land -> (tipo, arg1, arg2)
  - take_photo       -> ('take_photo', viewpoint, target)
"""

import glob
import os
import re
from typing import Dict, List, Tuple

Action = Tuple[str, str, str]


def parse_malama_line(line: str) -> Tuple[str, Action]:
    """
    Convertir UNA linea de plan de MA-LAMA en (drone, accion).

    :param line: linea cruda del fichero de plan
    :return: (nombre_dron, tupla_accion) o (None, None) si la linea esta vacia
    """
    line = line.strip()
    if not line:
        return None, None

    # quitar la parte de tiempos entre corchetes: '... [0.010, 0.010, 2.000]'
    line = re.sub(r'\[.*?\]', '', line).strip()

    tokens = line.split()
    kind = tokens[0]

    if kind == 'take_photo':
        # formato: take_photo <viewpoint> <drone> <target>
        viewpoint = tokens[1]
        drone = tokens[2]
        target = tokens[3]
        action = ('take_photo', viewpoint, target)
    else:
        # formato: <accion> <arg1> <arg2> <drone>
        arg1 = tokens[1]
        arg2 = tokens[2]
        drone = tokens[3]
        action = (kind, arg1, arg2)

    return drone, action


def parse_malama_file(path: str) -> Dict[str, List[Action]]:
    """
    Parsear un fichero de plan de MA-LAMA completo y agrupar las acciones por dron.

    :param path: ruta al fichero plan_agentN.txt
    :return: dict {drone: [acciones en orden]}
    """
    plans: Dict[str, List[Action]] = {}
    with open(path, 'r') as f:
        for line in f:
            drone, action = parse_malama_line(line)
            if drone is None:
                continue
            plans.setdefault(drone, []).append(action)
    return plans


def load_malama_plans(plan_dir: str, pattern: str = 'plan_agent*.txt') -> Dict[str, List[Action]]:
    """
    Cargar TODOS los ficheros de plan de MA-LAMA de un directorio y fundirlos en un unico
    diccionario PLANS (un fichero puede contener uno o varios drones).

    :param plan_dir: directorio donde estan los plan_agentN.txt
    :param pattern: patron de los ficheros de plan
    :return: dict {drone: [acciones]} listo para el ejecutor
    """
    plans: Dict[str, List[Action]] = {}
    files = sorted(glob.glob(os.path.join(plan_dir, pattern)))
    if not files:
        raise FileNotFoundError(
            f'No se encontraron planes ({pattern}) en {plan_dir}')

    for path in files:
        file_plans = parse_malama_file(path)
        for drone, actions in file_plans.items():
            # si un dron apareciera en varios ficheros, se concatenan en orden
            plans.setdefault(drone, []).extend(actions)
    return plans


# Alias para compatibilidad con código existente
load_plans = load_malama_plans


_OPTIC_LINE_RE = re.compile(r'^\d+\.\d+:\s*\((\S+)\s+([^)]+)\)\s*\[[\d.]+\]')
_KNOWN_ACTIONS = {'takeoff', 'fly', 'take_photo', 'land'}


def parse_optic_line(line: str) -> Tuple[str, Action]:
    """
    Convertir UNA linea de salida de OPTIC en (drone, accion).

    Formato OPTIC: T: (accion arg1 arg2 drone)  [duracion]
    Acciones no reconocidas (recharge, etc.) se ignoran -> (None, None).
    """
    m = _OPTIC_LINE_RE.match(line.strip())
    if not m:
        return None, None

    kind = m.group(1)
    if kind not in _KNOWN_ACTIONS:
        return None, None

    args = m.group(2).split()

    if kind == 'take_photo':
        # take_photo vp drone target
        viewpoint, drone, target = args[0], args[1], args[2]
        action = ('take_photo', viewpoint, target)
    else:
        # takeoff/fly/land: arg1 arg2 drone
        arg1, arg2, drone = args[0], args[1], args[2]
        action = (kind, arg1, arg2)

    return drone, action


def load_optic_plan(plan_file: str) -> Dict[str, List[Action]]:
    """
    Parsear el fichero de salida de OPTIC (optic_plan.txt) en el mismo formato
    {drone: [acciones]} que usa el ejecutor.

    OPTIC vuelca todas las acciones ordenadas por tiempo en un unico fichero;
    agruparlas por drone en ese orden produce la secuencia correcta por hilo.
    """
    plans: Dict[str, List[Action]] = {}
    with open(plan_file, 'r', encoding='utf-8') as f:
        for line in f:
            drone, action = parse_optic_line(line)
            if drone is None:
                continue
            plans.setdefault(drone, []).append(action)

    if not plans:
        raise ValueError(f'No se encontraron acciones reconocidas en {plan_file}')
    return plans


# Prueba rapida al ejecutar el módulo directamente
if __name__ == '__main__':
    from pprint import pprint

    print("=== Probando MA-LAMA ===\n")
    try:
        plan_dir = '/home/oscar/MA-LAMA'
        result = load_malama_plans(plan_dir)
        pprint(result)
    except FileNotFoundError as e:
        print(f"Error: {e}\n")

    print("\n=== Probando OPTIC ===\n")
    try:
        plan_file = '/home/oscar/OPTIC/optic_plan.txt'
        result = load_optic_plan(plan_file)
        pprint(result)
    except (FileNotFoundError, ValueError) as e:
        print(f"Error: {e}\n")