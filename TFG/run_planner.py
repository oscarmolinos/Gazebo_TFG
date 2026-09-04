#!/usr/bin/env python3
"""
run_planner.py — Lanza los planificadores externos (MA-LAMA y OPTIC) como
procesos independientes sobre el PDDL ya generado en ~/MA-LAMA/domains/.

run_malama() invoca ./launchMALama.sh dentro de ~/MA-LAMA y comprueba que
final_plan.txt se haya generado; run_optic() copia el PDDL a ~/OPTIC/domains/,
ejecuta optic-clp y guarda toda la salida en ~/OPTIC/optic_plan.txt. Ambas
funciones devuelven True/False según si el planificador encontró solución;
el plan resultante lo interpreta después plan_parser.py.
"""


import os
import shutil
import subprocess


def run_malama(config, timeout_arg: str = "20", multiagente: bool = True) -> bool:
    """
    Ejecutar MA-LAMA con el dominio y el problema de CONFIG, e imprimir el plan.

    Equivale a lanzar en terminal (desde ~/MA-LAMA):
        ./launchMALama.sh domains/<dominio>.pddl domains/<problema>.pddl 20 y y h

    :param timeout_arg: el argumento numérico del planificador
    :param multiagente: True fuerza el modo multiagente ('y'). Con un SOLO dron
        hay que pasar False ('n'): forzar multiagente con un único agente deja
        al planificador en un bucle infinito.
    :return: True si el planificador terminó bien y existe el plan, False si no
    """
    malama_dir = os.path.expanduser("~/MA-LAMA")

    domain_file = os.path.join("domains", f'{config["domain_name"]}.pddl')
    problem_file = os.path.join("domains", config["output_file"])

    modo = "y" if multiagente else "n"
    cmd = ["bash", "./launchMALama.sh", domain_file, problem_file, timeout_arg, modo, "y", "h"]

    print(f'Ejecutando planificador: {" ".join(cmd)}')
    resultado = subprocess.run(
        cmd,
        cwd=malama_dir,
        capture_output=True,
        text=True)

    if resultado.returncode != 0:
        print('El planificador devolvió error:')
        print(resultado.stderr)
        return False

    plan_path = os.path.join(malama_dir, "final_plan.txt")
    if not os.path.exists(plan_path):
        print(f'No se encontró el plan en {plan_path}')
        return False

    print('\n===== PLAN GENERADO (final_plan.txt) =====')
    with open(plan_path, "r", encoding="utf-8") as f:
        print(f.read())
    print('==========================================\n')

    return True


def run_optic(config) -> bool:
    """
    Ejecutar OPTIC con el dominio y el problema de CONFIG.

    Copia el .pddl generado (en ~/MA-LAMA/domains/) a ~/OPTIC/domains/ y lanza:
        ./build/src/optic/optic-clp -N ./domains/<dominio>.pddl ./domains/<problema>.pddl

    La salida completa se guarda en ~/OPTIC/optic_plan.txt.

    :return: True si OPTIC encontró solución, False si no
    """
    optic_dir = os.path.expanduser("~/OPTIC")
    malama_dir = os.path.expanduser("~/MA-LAMA")

    problem_file = config["output_file"]
    src = os.path.join(malama_dir, "domains", problem_file)
    dst_dir = os.path.join(optic_dir, "domains")
    dst = os.path.join(dst_dir, problem_file)

    os.makedirs(dst_dir, exist_ok=True)
    shutil.copy2(src, dst)

    domain_rel = f'./domains/{config["domain_name"]}.pddl'
    problem_rel = f'./domains/{problem_file}'

    cmd = ["./build/src/optic/optic-clp", "-N", domain_rel, problem_rel]
    print(f'Ejecutando planificador: {" ".join(cmd)}')

    resultado = subprocess.run(
        cmd,
        cwd=optic_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True)

    output = resultado.stdout
    print(output)

    plan_path = os.path.join(optic_dir, "optic_plan.txt")
    with open(plan_path, "w", encoding="utf-8") as f:
        f.write(output)

    if "Solution Found" not in output:
        print("OPTIC no encontró solución.")
        return False

    print(f'Plan guardado en {plan_path}\n')
    return True


# Alias para compatibilidad con código existente.
run_planner = run_malama


if __name__ == "__main__":
    import argparse
    import yaml

    parser = argparse.ArgumentParser(description='Ejecutar MA-LAMA para el escenario dado.')
    parser.add_argument('scenario', help='ruta al YAML del problema (p.ej. problems/problem2.yaml)')
    args = parser.parse_args()

    with open(args.scenario, 'r', encoding='utf-8') as f:
        scenario = yaml.safe_load(f)

    run_malama(scenario['CONFIG'])
    run_optic(scenario['CONFIG'])
