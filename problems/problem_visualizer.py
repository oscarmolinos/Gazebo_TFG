#!/usr/bin/env python3
"""Visualizador 2D/3D interactivo de problemas PDDL multi-dron."""

import glob
import os
import re
import sys
import xml.etree.ElementTree as ET

import inquirer
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import proj3d
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection
import numpy as np
import yaml


def load_problem(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def classify_point(name: str) -> str:
    if name.startswith("vp"):
        return "vp"
    if name.startswith("wp"):
        return "wp"
    if name.startswith("tgt"):
        return "tgt"
    if "base" in name and "air" in name:
        return "base_air"
    if "base" in name and "ground" in name:
        return "base_ground"
    return "other"


STYLE = {
    "vp":          {"marker": "^", "color": "#2196F3", "s": 120, "zorder": 4},
    "wp":          {"marker": "o", "color": "#9E9E9E", "s": 80,  "zorder": 3},
    "tgt":         {"marker": "*", "color": "#F44336", "s": 180, "zorder": 5},
    "base_air":    {"marker": "s", "color": "#4CAF50", "s": 100, "zorder": 4},
    "base_ground": {"marker": "D", "color": "#FF9800", "s": 100, "zorder": 4},
    "other":       {"marker": "o", "color": "#607D8B", "s": 60,  "zorder": 3},
}

DRONE_COLORS = ["#E91E63", "#3F51B5", "#009688", "#FF5722", "#673AB7",
                "#00BCD4", "#8BC34A", "#FFC107"]

LABELS_MAP = {
    "vp": "Viewpoint", "wp": "Waypoint", "tgt": "Target",
    "base_air": "Base (aire)", "base_ground": "Base (suelo)",
}


# =========================================================================
#  PARSEO DEL SDF
# =========================================================================
def _find_sdf(problem_yaml_path: str) -> str | None:
    """Dado el path del YAML (problemN.yaml), busca worlds/world_problemN.sdf."""
    basename = os.path.basename(problem_yaml_path)
    m = re.search(r'(\d+)', basename)
    if not m:
        return None
    n = m.group(1)
    project_root = os.path.abspath(os.path.join(
        os.path.dirname(problem_yaml_path), '..'))
    sdf_path = os.path.join(project_root, 'worlds', f'world_problem{n}.sdf')
    return sdf_path if os.path.isfile(sdf_path) else None


def parse_sdf_boxes(sdf_path: str) -> list[dict]:
    """Extraer cajas del SDF: pose, size y color diffuse de cada <model> con <box>."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    boxes = []

    for model in root.iter('model'):
        name = model.get('name', '')
        if name == 'ground_plane':
            continue

        pose_el = model.find('pose')
        if pose_el is None:
            continue
        pose_vals = list(map(float, pose_el.text.split()))
        cx, cy, cz = pose_vals[0], pose_vals[1], pose_vals[2]

        box_el = model.find('.//geometry/box/size')
        if box_el is None:
            continue
        sx, sy, sz = map(float, box_el.text.split())

        diffuse_el = model.find('.//material/diffuse')
        if diffuse_el is not None:
            rgba = list(map(float, diffuse_el.text.split()))
            color = (rgba[0], rgba[1], rgba[2], 1.0)
        else:
            color = (0.8, 0.8, 0.8, 1.0)

        boxes.append({
            'name': name, 'cx': cx, 'cy': cy, 'cz': cz,
            'sx': sx, 'sy': sy, 'sz': sz, 'color': color,
        })

    return boxes


# =========================================================================
#  LEYENDA
# =========================================================================
def _build_legend(plotted_types):
    items = []
    for kind in ["base_ground", "base_air", "wp", "vp", "tgt"]:
        if kind in plotted_types:
            st = STYLE[kind]
            items.append(
                plt.scatter([], [], marker=st["marker"], c=st["color"],
                            s=st["s"], edgecolors="black", linewidths=0.5,
                            label=LABELS_MAP[kind]))
    items.append(
        plt.scatter([], [], marker="P", c=DRONE_COLORS[0], s=160,
                    edgecolors="black", linewidths=1.0, label="Dron (inicio)"))
    items.append(
        plt.Line2D([0], [0], color="#BDBDBD", linewidth=1.2,
                   label="Ruta valida"))
    items.append(
        plt.Line2D([0], [0], color="#A5D6A7", linewidth=1.2,
                   linestyle="--", label="Landing pad"))
    items.append(
        plt.Line2D([0], [0], color="#EF9A9A", linewidth=1.0,
                   linestyle="--", label="Puede fotografiar"))
    return items


def _extract(data):
    return (data["COORDS"],
            data.get("VALID_PATHS", []),
            data.get("LANDING_PADS", []),
            data.get("CAN_PHOTOGRAPH", {}),
            data.get("DRONES", {}))


# =========================================================================
#  VISTA 2D
# =========================================================================
def visualize_2d(data: dict, title: str, boxes: list[dict]) -> None:
    coords, valid_paths, landing_pads, can_photo, drones = _extract(data)

    fig, ax = plt.subplots(figsize=(12, 10))

    all_xs = [c[0] for c in coords.values()]
    all_ys = [c[1] for c in coords.values()]
    for b in boxes:
        all_xs += [b['cx'] - b['sx']/2, b['cx'] + b['sx']/2]
        all_ys += [b['cy'] - b['sy']/2, b['cy'] + b['sy']/2]
    margin = 2
    ax.set_xlim(min(all_xs) - margin, max(all_xs) + margin)
    ax.set_ylim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_aspect("equal")
    ax.grid(True, which="both", linewidth=0.5, alpha=0.4)
    ax.xaxis.set_major_locator(plt.MultipleLocator(1))
    ax.yaxis.set_major_locator(plt.MultipleLocator(1))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"{title} (2D)", fontsize=14)

    for b in boxes:
        rect = mpatches.FancyBboxPatch(
            (b['cx'] - b['sx']/2, b['cy'] - b['sy']/2),
            b['sx'], b['sy'],
            boxstyle="round,pad=0.02",
            facecolor=b['color'], edgecolor='black', linewidth=0.8, zorder=2)
        ax.add_patch(rect)
        ax.text(b['cx'], b['cy'], b['name'].replace('_', '\n'),
                fontsize=5, ha='center', va='center', zorder=3, alpha=0.7)

    for a, b in valid_paths:
        if a in coords and b in coords:
            ax.plot([coords[a][0], coords[b][0]],
                    [coords[a][1], coords[b][1]],
                    color="#BDBDBD", linewidth=1.2, zorder=3)

    for pad in landing_pads:
        g, a = pad[0], pad[1]
        if g in coords and a in coords:
            ax.plot([coords[g][0], coords[a][0]],
                    [coords[g][1], coords[a][1]],
                    color="#A5D6A7", linewidth=1.2, linestyle="--", zorder=3)

    for vp_name, tgt_name in can_photo.items():
        if vp_name in coords and tgt_name in coords:
            ax.annotate("",
                        xy=(coords[tgt_name][0], coords[tgt_name][1]),
                        xytext=(coords[vp_name][0], coords[vp_name][1]),
                        arrowprops=dict(arrowstyle="->", color="#EF9A9A",
                                        lw=1.0, linestyle="--"),
                        zorder=4)

    plotted_types = set()
    for name, (x, y, _z) in coords.items():
        kind = classify_point(name)
        st = STYLE[kind]
        ax.scatter(x, y, marker=st["marker"], c=st["color"],
                   s=st["s"], zorder=st["zorder"] + 2, edgecolors="black",
                   linewidths=0.5)
        ax.annotate(name, (x, y), textcoords="offset points",
                    xytext=(6, 6), fontsize=7, zorder=8)
        plotted_types.add(kind)

    for i, (ns, info) in enumerate(drones.items()):
        start = info["start"]
        if start in coords:
            x, y = coords[start][0], coords[start][1]
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            ax.scatter(x, y, marker="P", c=color, s=220, zorder=9,
                       edgecolors="black", linewidths=1.0)
            ax.annotate(ns, (x, y), textcoords="offset points",
                        xytext=(-10, -14), fontsize=8, fontweight="bold",
                        color=color, zorder=9)

    ax.legend(handles=_build_legend(plotted_types), loc="upper left",
              fontsize=8, framealpha=0.9)
    fig.tight_layout()
    plt.show()


# =========================================================================
#  VISTA 3D
# =========================================================================
def _box_faces(cx, cy, cz, sx, sy, sz):
    """Devuelve las 6 caras de una caja como listas de vertices."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    corners = np.array([
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz],
    ])
    idx = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # front
        [2, 3, 7, 6],  # back
        [0, 3, 7, 4],  # left
        [1, 2, 6, 5],  # right
    ]
    return [corners[face].tolist() for face in idx]

_FACE_SHADE = [0.55, 1.00, 0.78, 0.78, 0.88, 0.88]

_SEG_LEN = 1.0
_MAX_SEGS = 16

_MARKER_ZORDER = 10000


def _shaded_face_colors(rgb):
    """Aplicar un sombreado distinto a cada cara para dar sensacion de volumen."""
    return [tuple(min(1.0, c * f) for c in rgb[:3]) for f in _FACE_SHADE]


class _Box3D(Poly3DCollection):
    """Caja opaca ordenada por su centro en vez de por su vertice mas cercano.

    Poly3DCollection se ordena por el vertice mas proximo a la camara, lo que
    "acerca" la caja y hace que tape cosas que en realidad pasan por delante o
    por encima de ella.
    """

    def __init__(self, faces, center, **kwargs):
        super().__init__(faces, **kwargs)
        self._center = np.asarray(center, dtype=float)

    def do_3d_projection(self):
        super().do_3d_projection()
        return proj3d.proj_transform(*self._center, self.axes.M)[2]


def _plot_marker_3d(ax, p, marker, color, size, edgewidth=0.5):
    """Dibujar un marcador 3D que nunca queda tapado por las cajas.

    Se usa ax.plot en vez de ax.scatter justo por lo contrario que en
    _plot_line_3d: el Line3D resultante queda fuera de la ordenacion por
    profundidad y conserva el zorder alto que le damos.
    """
    ax.plot([p[0]], [p[1]], [p[2]], linestyle="none", marker=marker,
            markersize=np.sqrt(size), markerfacecolor=color,
            markeredgecolor="black", markeredgewidth=edgewidth,
            zorder=_MARKER_ZORDER)


def _plot_line_3d(ax, p0, p1, **style):
    """Dibujar una linea 3D que respeta la oclusion de las cajas.

    ax.plot crea un Line3D, que al heredar de Line2D no es una Collection y por
    tanto matplotlib lo deja fuera de la ordenacion por profundidad: conserva un
    zorder fijo y acaba siempre por debajo de las cajas. Un Line3DCollection si
    entra en esa ordenacion, y partir la linea en tramos hace que solo se oculte
    el trozo que realmente pasa por detras.
    """
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    n = int(np.clip(np.ceil(np.linalg.norm(p1 - p0) / _SEG_LEN), 1, _MAX_SEGS))
    pts = np.linspace(p0, p1, n + 1)
    for a, b in zip(pts[:-1], pts[1:]):
        ax.add_collection3d(Line3DCollection([[a, b]], **style))


def visualize_3d(data: dict, title: str, boxes: list[dict]) -> None:
    coords, valid_paths, landing_pads, can_photo, drones = _extract(data)

    fig = plt.figure(figsize=(13, 10))
    ax = fig.add_subplot(111, projection="3d")

    all_xs = [c[0] for c in coords.values()]
    all_ys = [c[1] for c in coords.values()]
    all_zs = [c[2] for c in coords.values()]
    for b in boxes:
        all_xs += [b['cx'] - b['sx']/2, b['cx'] + b['sx']/2]
        all_ys += [b['cy'] - b['sy']/2, b['cy'] + b['sy']/2]
        all_zs += [b['cz'] - b['sz']/2, b['cz'] + b['sz']/2]
    margin = 2
    xlim = (min(all_xs) - margin, max(all_xs) + margin)
    ylim = (min(all_ys) - margin, max(all_ys) + margin)
    zlim = (max(min(all_zs) - margin, -0.5), max(all_zs) + margin)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_zlim(*zlim)
    # Misma escala en los tres ejes: la caja de dibujo se hace proporcional al
    # rango de datos, de modo que un metro ocupa lo mismo en X, Y y Z y los
    # obstaculos no salen deformados. (set_aspect('equal') no vale en 3D
    # hasta matplotlib 3.6.)
    ax.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(f"{title} (3D)", fontsize=14)

    # Cajas opacas: ocultan lineas y puntos que queden detras
    for b in boxes:
        faces = _box_faces(b['cx'], b['cy'], b['cz'],
                           b['sx'], b['sy'], b['sz'])
        poly = _Box3D(faces, (b['cx'], b['cy'], b['cz']),
                      facecolors=_shaded_face_colors(b['color']),
                      edgecolor='black', linewidth=0.4, zsort='average')
        poly.set_alpha(None)
        ax.add_collection3d(poly)

    for a, b in valid_paths:
        if a in coords and b in coords:
            _plot_line_3d(ax, coords[a], coords[b],
                          color="#BDBDBD", linewidth=1.2)

    for pad in landing_pads:
        g, a = pad[0], pad[1]
        if g in coords and a in coords:
            _plot_line_3d(ax, coords[g], coords[a],
                          color="#A5D6A7", linewidth=1.2, linestyle="--")

    for vp_name, tgt_name in can_photo.items():
        if vp_name in coords and tgt_name in coords:
            _plot_line_3d(ax, coords[vp_name], coords[tgt_name],
                          color="#EF9A9A", linewidth=1.0, linestyle="--")

    plotted_types = set()
    for name, (x, y, z) in coords.items():
        kind = classify_point(name)
        st = STYLE[kind]
        _plot_marker_3d(ax, (x, y, z), st["marker"], st["color"], st["s"])
        ax.text(x, y, z + 0.3, name, fontsize=6, ha="center",
                zorder=_MARKER_ZORDER + 1)
        plotted_types.add(kind)

    for i, (ns, info) in enumerate(drones.items()):
        start = info["start"]
        if start in coords:
            x, y, z = coords[start]
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            _plot_marker_3d(ax, (x, y, z), "P", color, 220, edgewidth=1.0)
            ax.text(x, y, z - 0.6, ns, fontsize=8, fontweight="bold",
                    color=color, ha="center", zorder=_MARKER_ZORDER + 1)

    ax.legend(handles=_build_legend(plotted_types), loc="upper left",
              fontsize=8, framealpha=0.9)
    fig.tight_layout()
    plt.show()


# =========================================================================
#  MAIN
# =========================================================================
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    files = sorted(glob.glob(os.path.join(script_dir, "problem*.yaml")))
    if not files:
        print("No se encontraron archivos problemN.yaml")
        sys.exit(1)

    choices = [os.path.basename(f) for f in files]
    questions = [
        inquirer.List(
            "problem",
            message="Selecciona un problema para visualizar",
            choices=choices,
        ),
        inquirer.List(
            "view",
            message="Tipo de vista",
            choices=["2D (planta X-Y)", "3D (X-Y-Z)"],
        ),
    ]
    answers = inquirer.prompt(questions)
    if not answers:
        sys.exit(0)

    selected = os.path.join(script_dir, answers["problem"])
    data = load_problem(selected)

    boxes = []
    sdf_path = _find_sdf(selected)
    if sdf_path:
        boxes = parse_sdf_boxes(sdf_path)
        print(f"SDF cargado: {sdf_path} ({len(boxes)} cajas)")
    else:
        print("No se encontro SDF asociado, se dibuja sin cajas")

    if answers["view"].startswith("3D"):
        visualize_3d(data, answers["problem"], boxes)
    else:
        visualize_2d(data, answers["problem"], boxes)


if __name__ == "__main__":
    main()
