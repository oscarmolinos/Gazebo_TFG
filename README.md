# TFG — Planificación y ejecución de misiones multi-dron

Proyecto basado en [aerostack2/project_gazebo](https://github.com/aerostack2/project_gazebo).
Convierte un escenario YAML en un problema PDDL, lo resuelve con MA-LAMA u OPTIC
y ejecuta el plan en Gazebo con Aerostack2, replanificando si hace falta.

## Estructura

| Ruta | Contenido |
|------|-----------|
| `run_problem.py` | Punto de entrada: genera el PDDL, planifica y ejecuta |
| `problem_generator.py` | YAML → PDDL (se escribe en `~/MA-LAMA/domains/`) |
| `run_planner.py` | Lanza MA-LAMA / OPTIC |
| `plan_parser.py` | Unifica la salida de ambos planificadores |
| `replanner.py` | Ejecución por fases y replanificación |
| `drone_functions.py` | Control de drones, gimbal, cámara y bloqueo de waypoints |
| `problems/` | Escenarios `problemN.yaml` y `problem_visualizer.py` |
| `config/world_problemN.yaml` | Configuración de drones de cada mundo |
| `worlds/world_problemN.sdf` | Mundos de Gazebo |
| `photos/`, `tiempos_mision.txt`, `problems/generated/` | Salidas de la misión (ignoradas por git) |

## Ejecución

Desde la raíz del proyecto:

```bash
# 1. Simulación y nodos de Aerostack2
./launch_as2.bash -c config/world_problemN.yaml

# 2. Estación de tierra (opcional: -v rviz, -t teleoperación, -r rosbag)
./launch_ground_station.bash -c config/world_problemN.yaml

# 3. Misión
python3 run_problem.py problems/problemN.yaml

# Visualizar un escenario
python3 problems/problem_visualizer.py

# Parar todo
./stop.bash
```

Requiere los planificadores externos en `~/MA-LAMA/` y `~/OPTIC/`.
