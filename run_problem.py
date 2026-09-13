#!/usr/bin/env python3

"""
run_problem.py — Punto de entrada genérico para problemas PDDL.

Carga el escenario (geometría, drones, conectividad, objetivos...) desde un
fichero YAML, genera el PDDL con problem_generator, pregunta (via inquirer)
qué planificador usar (MA-LAMA u OPTIC), carga el plan resultante y lanza la
misión en el simulador a través de replanner.run_mission, que se encarga de
replanificar si hace falta.

Para crear un problema nuevo basta con añadir un YAML en problems/ (copiando
uno existente como plantilla); este script no necesita cambios.

Uso (en la ruta ~/Aerostack2/project_gazebo/):
    python3 run_problem.py problems/problem2.yaml
    python3 run_problem.py problems/problem3.yaml
"""

import argparse
import os
import time

import inquirer
import yaml

import problem_generator as pg
import drone_functions as df
import replanner as rp
from run_planner import run_malama, run_optic
from plan_parser import load_malama_plans, load_optic_plan


def confirm(msg: str = 'Continuar') -> bool:
    """Pedir confirmación al usuario usando inquirer."""
    respuesta = inquirer.prompt([
        inquirer.Confirm(
            'confirmar',
            message=msg,
            default=True,
        )
    ])
    return respuesta.get('confirmar', False)


def load_scenario(path: str) -> dict:
    """Cargar el escenario (CONFIG, COORDS, WAYPOINTS, ...) desde un YAML."""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(
        description='Generar PDDL, planificar y ejecutar una misión a partir de un escenario YAML.')
    parser.add_argument('scenario', help='ruta al YAML del problema (p.ej. problems/problem2.yaml)')
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    config = scenario['CONFIG']

    malama_dir = os.path.expanduser("~/MA-LAMA")
    optic_dir = os.path.expanduser("~/OPTIC")
    problem_path = os.path.join(malama_dir, "domains", config["output_file"])

    # --- 1. GENERAR PROBLEMA PDDL ---
    generar = True
    if os.path.exists(problem_path):
        generar = confirm(f"Ya existe '{config['output_file']}'. ¿Generarlo de nuevo?")

    if generar:
        pg.generate_problem(scenario)
        print("Problema PDDL generado.\n")
    else:
        print("Reutilizando el problema existente.\n")

    # --- 2. ELEGIR PLANIFICADOR ---
    respuesta = inquirer.prompt([
        inquirer.List(
            'planner',
            message='¿Qué planificador usar?',
            choices=['MA-LAMA', 'OPTIC'],
        )
    ])
    planner = respuesta['planner']

    if planner == 'MA-LAMA':
        plan_path = os.path.join(malama_dir, "final_plan.txt")
    else:
        plan_path = os.path.join(optic_dir, "optic_plan.txt")

    # --- 3. EJECUTAR PLANIFICADOR ---
    planificar = True
    if os.path.exists(plan_path):
        planificar = confirm("Ya existe un plan. ¿Ejecutar el planificador de nuevo?")

    if planificar:
        if planner == 'MA-LAMA':
            # Con un solo dron hay que desactivar el modo multiagente, o MA-LAMA
            # se queda en un bucle infinito.
            ok = run_malama(config, multiagente=len(scenario['DRONES']) > 1)
        else:
            ok = run_optic(config)
        if not ok:
            print("Error en el planificador. Abortando.")
            return
        print("Planificador completado.\n")
    else:
        print("Reutilizando el plan existente.\n")

    # --- 4. IMPORTAR PLAN ---
    if not confirm("¿Cargar el plan generado?"):
        return

    if planner == 'MA-LAMA':
        plans = load_malama_plans(malama_dir)
    else:
        plans = load_optic_plan(plan_path)

    if not plans:
        print("No se cargaron planes. Abortando.")
        return
    print("Plan cargado.\n")

    # --- 5. EJECUTAR MISION EN SIMULADOR ---
    if not confirm("¿Ejecutar misión en el simulador?"):
        return

    drones = df.create_drones(plans, use_sim_time=True, verbose=False)
    time.sleep(0.1)
    rp.run_mission(drones, scenario, plans, planner=planner)
    df.shutdown_drones(drones)
    df.show_all_photos()
    print("Misión completada.\n")


if __name__ == "__main__":
    main()
