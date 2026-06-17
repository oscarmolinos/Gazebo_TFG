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


def parse_plan_line(line: str) -> Tuple[str, Action]:
    """
    Convertir UNA linea de plan en (drone, accion).

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


def parse_plan_file(path: str) -> Dict[str, List[Action]]:
    """
    Parsear un fichero de plan completo y agrupar las acciones por dron.

    :param path: ruta al fichero plan_agentN.txt
    :return: dict {drone: [acciones en orden]}
    """
    plans: Dict[str, List[Action]] = {}
    with open(path, 'r') as f:
        for line in f:
            drone, action = parse_plan_line(line)
            if drone is None:
                continue
            plans.setdefault(drone, []).append(action)
    return plans


def load_plans(plan_dir: str, pattern: str = 'plan_agent*.txt') -> Dict[str, List[Action]]:
    """
    Cargar TODOS los ficheros de plan de un directorio y fundirlos en un unico
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
        file_plans = parse_plan_file(path)
        for drone, actions in file_plans.items():
            # si un dron apareciera en varios ficheros, se concatenan en orden
            plans.setdefault(drone, []).extend(actions)
    return plans


# Prueba rapida al ejecutar el módulo directamente
if __name__ == '__main__':
    from pprint import pprint

    plan_dir = '/home/oscar/MA-LAMA'
    result = load_plans(plan_dir)
    pprint(result)