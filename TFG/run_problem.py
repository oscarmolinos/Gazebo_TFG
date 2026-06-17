#!/usr/bin/env python3

"""
run_problem.py — Punto de entrada genérico para problemas PDDL.

Carga el escenario (geometría, drones, conectividad, objetivos...) desde un
fichero YAML, genera el PDDL con problem_generator, ejecuta el planificador
MA-LAMA, carga el plan resultante y lanza la mision en el simulador.

Para crear un problema nuevo basta con añadir un YAML en problems/ (copiando
uno existente como plantilla); este script no necesita cambios.

Uso:
    python3 run_problem.py problems/problem2.yaml
    python3 run_problem.py problems/problem3.yaml
"""

import argparse
import os

import yaml

import problem_generator as pg
import drone_functions as df
from run_planner import run_planner
from plan_parser import load_plans


def confirm(msg: str = 'Continuar') -> bool:
    """Pedir confirmación al usuario (y/n)."""
    while True:
        respuesta = input(f"{msg} (y/n): ").strip().lower()
        if respuesta in ['y', 'yes', 's', 'si']:
            return True
        elif respuesta in ['n', 'no']:
            return False
        else:
            print("Respuesta inválida. Escribe 'y' o 'n'.")


def load_scenario(path: str) -> dict:
    """Cargar el escenario (CONFIG, COORDS, WAYPOINTS, ...) desde un YAML."""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(
        description='Generar PDDL, planificar y ejecutar una mision a partir de un escenario YAML.')
    parser.add_argument('scenario', help='ruta al YAML del problema (p.ej. problems/problem2.yaml)')
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    config = scenario['CONFIG']

    malama_dir = os.path.expanduser("~/MA-LAMA")
    problem_path = os.path.join(malama_dir, "domains", config["output_file"])
    plan_path = os.path.join(malama_dir, "final_plan.txt")

    # --- 1. GENERAR PROBLEMA PDDL ---
    generar = True
    if os.path.exists(problem_path):
        # el problema ya existe: preguntar si rehacerlo
        generar = confirm(f"Ya existe '{config['output_file']}'. ¿Generarlo de nuevo?")

    if generar:
        pg.generate_problem(scenario)
        print("Problema PDDL generado.\n")
    else:
        print("Reutilizando el problema existente.\n")

    # --- 2. EJECUTAR PLANIFICADOR ---
    planificar = True
    if os.path.exists(plan_path):
        # ya hay un plan: preguntar si replanificar
        planificar = confirm("Ya existe un plan. ¿Ejecutar el planificador de nuevo?")

    if planificar:
        if not run_planner(config):
            print("Error en el planificador. Abortando.")
            return
        print("Planificador completado.\n")
    else:
        print("Reutilizando el plan existente.\n")

    # --- 3. IMPORTAR PLAN ---
    if not confirm("¿Cargar el plan generado?"):
        return

    plans = load_plans(malama_dir)
    if not plans:
        print("No se cargaron planes. Abortando.")
        return
    print("Plan cargado.\n")

    # --- 4. EJECUTAR MISION EN SIMULADOR ---
    if not confirm("¿Ejecutar misión en el simulador?"):
        return

    drones = df.create_drones(plans, use_sim_time=True, verbose=False)
    df.execute_mission(drones, scenario, plans)
    df.shutdown_drones(drones)
    df.show_all_photos()
    print("Misión completada.\n")


if __name__ == "__main__":
    main()
