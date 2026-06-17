# Refactor: eliminar las variables globales de módulo en el pipeline PDDL

## Resumen

Se ha eliminado el patrón de **rellenar variables globales de módulo desde fuera**
en `TFG/problem_generator.py` y `TFG/drone_functions.py`. Ahora los datos del
escenario se pasan **como argumento** a las funciones:

- `pg.generate_problem(scenario)`
- `df.execute_mission(drones, scenario, plans)` (y `df.create_drones(plans, ...)`,
  `df.run(scenario, plans, ...)`)

## De dónde venía el problema

En las **primeras versiones** del TFG el código no era modular: cada problema era
un único script que *definía* sus propias variables (`COORDS`, `SPEEDS`, `PLANS`…)
en el ámbito global del archivo y las funciones de ese mismo script las leían
directamente. En ese contexto, usar variables globales era natural: vivían en el
mismo fichero que las usaba.

Al **modularizar** el código (separar la lógica reutilizable en
`problem_generator.py` y `drone_functions.py`, y los datos en los YAML de
`problems/`), esas variables globales se mantuvieron por inercia. Pero ahora vivían
en un módulo distinto al que las rellenaba, así que `run_problem.py` tenía que
hacer esto antes de llamar a cada función:

```python
# ANTES — run_problem.py inyectaba estado en los módulos
pg.CONFIG = config
pg.COORDS = scenario['COORDS']
pg.WAYPOINTS = scenario['WAYPOINTS']
pg.TARGETS = scenario['TARGETS']
pg.LANDING_PADS = scenario['LANDING_PADS']
pg.VALID_PATHS = scenario['VALID_PATHS']
pg.CAN_PHOTOGRAPH = scenario['CAN_PHOTOGRAPH']
pg.DRONES = scenario['DRONES']
pg.generate_problem()          # ← lee todo eso de globales del módulo

...

df.COORDS = scenario['COORDS']
df.SPEEDS = {name: d["speed"] for name, d in scenario['DRONES'].items()}
df.VIEWPOINT_TARGET = scenario['CAN_PHOTOGRAPH']
df.PLANS = load_plans(malama_dir)
drones = df.create_drones(use_sim_time=True, verbose=False)
df.execute_mission(drones)     # ← lee todo eso de globales del módulo
```

Esto es un caso de **estado global mutable compartido entre módulos**, que arrastra
varios problemas.

## Por qué no era correcto

1. **Acoplamiento oculto.** La firma `generate_problem()` no dice que necesita 8
   datos: la dependencia está escondida en globales del módulo. Hay que conocer el
   "ritual" de asignaciones previas o el código falla en tiempo de ejecución.
2. **Orden frágil.** Si olvidas una asignación o cambias el orden, la función usa
   un valor vacío o el de una ejecución anterior, sin error claro.
3. **Estado residual entre ejecuciones.** Al ser globales del módulo, conservan el
   valor de la llamada anterior. Ejecutar dos escenarios en el mismo proceso podía
   mezclar datos.
4. **Difícil de testear y reutilizar.** No puedes llamar a `generate_problem()` con
   dos escenarios distintos sin reescribir globales por el medio.

## Qué se ha cambiado

Los datos viajan ahora **explícitamente por parámetro**; los módulos no guardan
estado de escenario:

```python
# DESPUÉS — run_problem.py pasa los datos como argumento
pg.generate_problem(scenario)

...

plans = load_plans(malama_dir)
drones = df.create_drones(plans, use_sim_time=True, verbose=False)
df.execute_mission(drones, scenario, plans)
```

Internamente:

- **`problem_generator.py`**: `generate_problem(scenario)` recibe el dict y lo pasa
  a las funciones auxiliares (`write_objects`, `write_init`, `write_goal`,
  `write_header`). Los ayudantes puros `fmt(x, decimals)` y
  `euclidean(a, b, coords)` reciben solo el dato que necesitan. Se eliminaron las
  variables de módulo `CONFIG/COORDS/WAYPOINTS/...`.
- **`drone_functions.py`**: `execute_mission(drones, scenario, plans)` y
  `drone_mission(drone, plan, scenario)` leen del `scenario` las coordenadas, el
  mapa viewpoint→target (`CAN_PHOTOGRAPH`) y la velocidad de cada dron
  (`DRONES[ns]['speed']`). `create_drones(plans, ...)` y `run(scenario, plans, ...)`
  reciben el plan por argumento. Se eliminaron las variables de módulo
  `COORDS/SPEEDS/VIEWPOINT_TARGET/PLANS`.

> Nota: `plans` se pasa **aparte** de `scenario` porque no forma parte del escenario:
> es la salida del planificador MA-LAMA (`plan_parser.load_plans()`), no un dato de
> entrada del problema. Mantenerlos separados deja claro qué es entrada (escenario)
> y qué es resultado (plan).

## Por qué el código ha mejorado

- **Dependencias explícitas.** La firma de cada función declara exactamente qué
  necesita. Se entiende sin conocer rituales previos.
- **Sin estado residual.** Cada llamada es autocontenida; se pueden generar/ejecutar
  varios escenarios en el mismo proceso sin contaminación entre ellos.
- **Más fácil de testear y reutilizar.** Se puede llamar a `generate_problem(otro)`
  directamente, pasando datos de prueba.
- **Funciones más puras.** Reciben entrada y devuelven (o escriben) salida, sin leer
  estado mutable compartido: menos efectos colaterales y razonamiento más simple.

## Comprobación

Los tres ficheros siguen compilando (`python3 -m py_compile`) y `run_problem.py`
mantiene el mismo flujo de 4 pasos (generar PDDL → planificar → cargar plan →
ejecutar misión). El comportamiento externo no cambia; solo la forma de pasar los
datos.
