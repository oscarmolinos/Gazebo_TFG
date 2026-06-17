import os
import subprocess

def run_planner(config, timeout_arg: str = "100") -> bool:
    """
    Ejecutar MA-LAMA con el dominio y el problema de CONFIG, e imprimir el plan.

    Equivale a lanzar en terminal (desde ~/MA-LAMA):
        ./launchMALama.sh domains/<dominio>.pddl domains/<problema>.pddl 10 y y

    :param timeout_arg: el argumento numerico del planificador (p.ej. "10")
    :return: True si el planificador termino bien y existe el plan, False si no
    """
    malama_dir = os.path.expanduser("~/MA-LAMA")

    # construir las rutas tal como las espera el script (carpeta domains/, .pddl)
    domain_file = os.path.join("domains", f'{config["domain_name"]}.pddl')
    problem_file = os.path.join("domains", config["output_file"])

    # comando completo: cada token es un elemento de la lista
    cmd = ["bash", "./launchMALama.sh", domain_file, problem_file, timeout_arg, "y", "y", "h"]

    print(f'Ejecutando planificador: {" ".join(cmd)}')
    resultado = subprocess.run(
        cmd,
        cwd=malama_dir,        # ejecutar DENTRO de ~/MA-LAMA (como si hicieras cd)
        capture_output=True,
        text=True)

    # comprobar que el planificador no fallo
    if resultado.returncode != 0:
        print('El planificador devolvio error:')
        print(resultado.stderr)
        return False

    # leer e imprimir el plan final
    plan_path = os.path.join(malama_dir, "final_plan.txt")
    if not os.path.exists(plan_path):
        print(f'No se encontro el plan en {plan_path}')
        return False

    print('\n===== PLAN GENERADO (final_plan.txt) =====')
    with open(plan_path, "r", encoding="utf-8") as f:
        print(f.read())
    print('==========================================\n')

    return True


if __name__ == "__main__":
    import argparse
    import yaml

    parser = argparse.ArgumentParser(description='Ejecutar MA-LAMA para el escenario dado.')
    parser.add_argument('scenario', help='ruta al YAML del problema (p.ej. problems/problem2.yaml)')
    args = parser.parse_args()

    with open(args.scenario, 'r', encoding='utf-8') as f:
        scenario = yaml.safe_load(f)

    run_planner(scenario['CONFIG'])