#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
problem_generator.py
====================
Generador automático de problemas PDDL de inspección con drones.

Filosofía de diseño
--------------------
Los DATOS (geometría, física, drones, conectividad y objetivos) llegan en un
diccionario 'scenario' (cargado del YAML del problema) que se pasa como
ARGUMENTO a generate_problem(). La LÓGICA de escritura es completamente
genérica: no conoce ningún nombre de waypoint ni de dron concreto, y no usa
variables globales de módulo.

Esto significa que para crear un problema nuevo basta con:
    - añadir un waypoint en COORDS + WAYPOINTS,
    - añadir una conexión en VALID_PATHS,
    - añadir un dron en DRONES,
(todo dentro del YAML del escenario) y el PDDL se regenera solo, recalculando
distancias euclídeas y costes de vuelo.

Un dron puede arrancar (DRONES.*.start) en cualquier waypoint, ya sea una base
de tierra o directamente un punto aéreo: el generador calcula qué waypoints
quedan libres a partir de las posiciones de inicio de todos los drones.

La clave TARGETS puede complementarse con PHOTOGRAPHED (opcional, lista de
nombres de target) para arrancar el problema con algunos targets ya
fotografiados de antemano.

Reglas físicas automatizadas
----------------------------
    distance(x, y) = sqrt((x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2)  (euclídea 3D, desde COORDS)
    fly_cost(x, y) = distance(x, y) * FLY_COST_FACTOR          (consumo de batería)

Coordenadas en 3D: cada punto es (x, y, z), donde z es la ALTITUD en metros.
Las transiciones suelo<->aire (takeoff / land) toman su 'distance' de la
geometría vertical (diferencia de altitud); solo el takeoff_cost / land_cost
se mantienen como costes fijos definidos en LANDING_PADS.
"""

import math

# =============================================================================
#  LÓGICA DE GENERACIÓN (genérica, recibe los datos por argumento)
# =============================================================================

def fmt(x, decimals):
    """Formatea un número para PDDL con 'decimals' decimales, sin ceros sobrantes."""
    return f"{round(float(x), decimals):.{decimals}f}"


def euclidean(a, b, coords):
    """Distancia euclídea entre dos puntos de 'coords'."""
    return math.dist(coords[a], coords[b])


def write_objects(scenario):
    """Genera la sección (:objects) agrupando por tipo."""
    waypoints = " ".join(scenario['WAYPOINTS'].keys())
    targets   = " ".join(scenario['TARGETS'])
    drones    = " ".join(scenario['DRONES'].keys())
    return (
        "(:objects\n"
        f"    {waypoints} - waypoint\n"
        f"    {targets} - target\n"
        f"    {drones} - drone\n"
        ")"
    )


def write_init(scenario):
    """Genera la sección (:init): estados booleanos y funciones numéricas."""
    config         = scenario['CONFIG']
    coords         = scenario['COORDS']
    waypoints      = scenario['WAYPOINTS']
    drones         = scenario['DRONES']
    landing_pads   = scenario['LANDING_PADS']
    valid_paths    = scenario['VALID_PATHS']
    can_photograph = scenario['CAN_PHOTOGRAPH']
    decimals       = config['decimals']

    L = []

    # --- Parámetros globales ---
    L.append("    ; CONFIGURACIÓN DE PARÁMETROS")
    L.append(f"    (= (photo_time) {fmt(config['photo_time'], decimals)}) ; segundos para tomar una fotografía")
    L.append(f"    (= (recharge_rate) {fmt(config['recharge_rate'], decimals)}) ; cantidad de carga por segundo")
    L.append("")

    # --- Parámetros de velocidad de despegue y aterrizaje ---
    L.append("    ; parámetros de velocidad de despegue y aterrizaje")
    L.append(f"    (= (takeoff_speed) {fmt(config['takeoff_speed'], decimals)})")
    L.append(f"    (= (land_speed) {fmt(config['land_speed'], decimals)})")
    L.append("")

    # --- Margen de batería por waypoint ---
    L.append("    ; parámetros para no quedarnos sin batería")
    for wp, data in waypoints.items():
        L.append(f"    (= (battery_safety_margin {wp}) {fmt(data['safety_margin'], decimals)})")
    L.append("")

    # --- Configuración de cada dron ---
    for name, d in drones.items():
        L.append(f"    ; CONFIGURACIÓN DE {name.upper()}")
        L.append(f"    (drone_at {name} {d['start']}) (available {name})")
        L.append(f"    (= (battery_level {name}) {fmt(d['battery_level'], decimals)})")
        L.append(f"    (= (battery_capacity {name}) {fmt(d['battery_capacity'], decimals)})")
        L.append(f"    (= (drone_speed {name}) {fmt(d['speed'], decimals)}) ; m/s")
        L.append("")

    # --- Tipos de waypoint ---
    ground = [wp for wp, data in waypoints.items() if data["type"] == "ground"]
    air    = [wp for wp, data in waypoints.items() if data["type"] == "air"]
    L.append("    ; TIPOS DE WAYPOINT")
    for wp in ground:
        L.append(f"    (is_ground {wp})")
    L.append("    " + " ".join(f"(is_air {wp})" for wp in air))
    L.append("")

    # --- Configuración de las bases (recarga) ---
    L.append("    ; CONFIGURACIÓN DE LAS BASES")
    for wp, data in waypoints.items():
        if data.get("is_recharge"):
            L.append(f"    (is_recharge {wp})")
    L.append("")

    # --- Waypoints libres: cualquier waypoint que ningún dron ocupe al inicio ---
    # (un dron puede empezar en una base de tierra o directamente en un punto
    # aéreo; ese waypoint de partida NUNCA está libre, esté donde esté)
    occupied_wps = {d['start'] for d in drones.values()}
    free_wps = [wp for wp in waypoints if wp not in occupied_wps]
    L.append("    ; WAYPOINTS LIBRES (todo waypoint no ocupado por un dron al inicio)")
    L.append("    " + " ".join(f"(free {wp})" for wp in free_wps))
    L.append("")

    # --- Targets pendientes de fotografiar / ya fotografiados ---
    targets      = scenario['TARGETS']
    photographed = set(scenario.get('PHOTOGRAPHED', []))
    L.append("    ; TARGETS PENDIENTES DE FOTOGRAFIAR")
    L.append("    " + " ".join(f"(pending {tgt})" for tgt in targets if tgt not in photographed))
    if photographed:
        L.append("    ; TARGETS YA FOTOGRAFIADOS (dados de antemano en el escenario)")
        L.append("    " + " ".join(f"(photographed {tgt})" for tgt in targets if tgt in photographed))
    L.append("")

    # --- Relación suelo <-> aire (takeoff / land) ---
    L.append("    ; RELACIÓN SUELO <-> AIRE (solo se transita con takeoff / land)")
    margin = config["action_margin"]
    for ground_wp, air_wp, takeoff, land in landing_pads:
        dist = euclidean(ground_wp, air_wp, coords)*margin
        L.append(f"    ; {ground_wp} <-> {air_wp}  ({fmt(dist, decimals)} m de altitud)")
        L.append(f"    (landing_pad {ground_wp} {air_wp})")
        L.append(f"    (= (distance {ground_wp} {air_wp}) {fmt(dist, decimals)}) "
                 f"(= (takeoff_cost {ground_wp} {air_wp}) {fmt(takeoff, decimals)})")
        L.append(f"    (= (distance {air_wp} {ground_wp}) {fmt(dist, decimals)}) "
                 f"(= (land_cost {air_wp} {ground_wp}) {fmt(land, decimals)})")
    L.append("")

    # --- Conectividad aire <-> aire (geometría automática, bidireccional) ---
    L.append("    ; CONECTIVIDAD ENTRE PUNTOS (distancia y coste calculados por geometría)")
    factor = config["fly_cost_factor"]
    margin = config["action_margin"]
    for a, b in valid_paths:
        dist = euclidean(a, b, coords)
        cost = dist * factor
        dist *= margin
        L.append(f"    ; {a} <-> {b}  ({fmt(dist, decimals)} m)")
        L.append(f"    (valid_path {a} {b}) (= (distance {a} {b}) {fmt(dist, decimals)}) "
                 f"(= (fly_cost {a} {b}) {fmt(cost, decimals)})")
        L.append(f"    (valid_path {b} {a}) (= (distance {b} {a}) {fmt(dist, decimals)}) "
                 f"(= (fly_cost {b} {a}) {fmt(cost, decimals)})")
        L.append("")

    # --- Capacidad de fotografía ---
    L.append("    ; CONFIGURACIÓN DE LOS PUNTOS DE FOTOGRAFÍA")
    for vp, tgt in can_photograph.items():
        L.append(f"    (can_photograph {vp} {tgt})")

    return "(:init\n" + "\n".join(L) + "\n)"


def write_goal(scenario):
    """Genera la sección (:goal): todos los targets fotografiados y drones en su base."""
    targets = scenario['TARGETS']
    drones  = scenario['DRONES']
    L = ["(:goal", "    (and"]
    for tgt in targets:
        L.append(f"        (photographed {tgt})")
    L.append("")
    for name, d in drones.items():
        L.append(f"        (drone_at {name} {d['goal']})")
    L.append("    )")
    L.append(")")
    return "\n".join(L)


def write_metric():
    """Genera la sección (:metric)."""
    return "(:metric minimize (total-time))"


def write_header(scenario):
    """Cabecera con comentarios descriptivos del problema."""
    drones    = scenario['DRONES']
    waypoints = scenario['WAYPOINTS']
    targets   = scenario['TARGETS']
    return (
        "; --- PROBLEMA GENERADO AUTOMÁTICAMENTE ---\n"
        f"; {len(drones)} drones, {len(waypoints)} waypoints, {len(targets)} targets\n"
        "; Distancias y fly_cost calculados por geometría euclídea desde COORDS.\n"
    )


def generate_problem(scenario):
    """Ensambla el problema PDDL completo (a partir del dict 'scenario') y lo
    escribe a disco en ~/MA-LAMA/domains/."""
    config = scenario['CONFIG']
    parts = [
        write_header(scenario),
        f"(define (problem {config['problem_name']})",
        "",
        f"(:domain {config['domain_name']})",
        "",
        write_objects(scenario),
        "",
        "; --- INIT ---",
        write_init(scenario),
        "",
        "; --- GOAL ---",
        write_goal(scenario),
        "",
        "; --- METRIC ---",
        write_metric(),
        "",
        ")",
        "",
    ]

    problem_pddl = "\n".join(parts)

    import os

    # expandir ~ a la ruta real del home del usuario
    output_dir = os.path.expanduser("~/MA-LAMA/domains")
    os.makedirs(output_dir, exist_ok=True)  # crear la carpeta si no existe
    output_path = os.path.join(output_dir, config["output_file"])

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(problem_pddl)

    print(f'Problema guardado en: {output_path}')
    print(f"PDDL generado correctamente -> {config['output_file']}")
    print(f"  drones:    {len(scenario['DRONES'])}")
    print(f"  waypoints: {len(scenario['WAYPOINTS'])}")
    print(f"  targets:   {len(scenario['TARGETS'])}")
    print(f"  conexiones aire-aire: {len(scenario['VALID_PATHS'])} (x2 bidireccionales)")


def main():
    """Permite generar un problema directamente desde un YAML de escenario."""
    import argparse
    import yaml

    parser = argparse.ArgumentParser(
        description="Generar un problema PDDL a partir de un escenario YAML.")
    parser.add_argument("scenario", help="ruta al YAML del problema")
    args = parser.parse_args()

    with open(args.scenario, "r", encoding="utf-8") as f:
        scenario = yaml.safe_load(f)
    generate_problem(scenario)


if __name__ == "__main__":
    main()
