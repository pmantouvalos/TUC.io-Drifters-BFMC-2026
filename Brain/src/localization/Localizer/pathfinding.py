"""

BFMC Competition Track — Pathfinding

=====================================

Finds a path between two nodes in the GraphML track using one of three

algorithms:  A* (default) · Dijkstra · BFS


Node coordinates (x, y) are stored on every node and are used to compute

Euclidean edge weights for A* and Dijkstra.


USAGE

-----

  # Interactive (prompts for source / target):

    python3 pathfinding.py


  # One-shot:

    python3 pathfinding.py --source 1 --target 42


  # Choose algorithm (astar | dijkstra | bfs):

    python3 pathfinding.py --source 1 --target 42 --algo dijkstra


  # Exclude dotted edges (shortcut / alternative routes):

    python3 pathfinding.py --source 1 --target 42 --no-dotted


  # Visualise result with Pygame (reuses visualization.py colour scheme):

    python3 pathfinding.py --source 1 --target 42 --visualize \

            --map Competition_track_graph.graphml


REQUIRES

--------

  pip install networkx pygame          (pygame only needed for --visualize)

"""


import argparse

import math

import sys

from typing import List, Optional


import networkx as nx


# ── Colours (match visualization.py) ─────────────────────────────────────────

BG = (15, 15, 25)

EDGE_SOLID = (40, 65, 100)

EDGE_DOTTED = (60, 130, 55)

NODE_COL = (50, 120, 220)

NODE_START = (80, 220, 100)

NODE_END = (220, 40, 40)

NODE_PATH = (255, 200, 0)

PATH_COL = (255, 200, 0)

TEXT = (200, 200, 200)

TEXT_DIM = (110, 110, 130)

TEXT_GOOD = (80, 220, 100)

TEXT_BAD = (220, 60, 60)

TEXT_WARN = (220, 180, 40)


WINDOW_W = 1280

WINDOW_H = 800

NODE_R = 4

FPS = 30

DEFAULT_MAP_PATH = "/home/pi/Documents/Brain/src/utils/Competition_track_graph.graphml"

# ═══════════════════════════════════════════════════════════════════════════════

# Graph helpers

# ═══════════════════════════════════════════════════════════════════════════════



def load_graph(path: str, exclude_dotted: bool = False) -> nx.DiGraph:

    """Load the GraphML file and attach Euclidean weights to every edge."""

    G: nx.DiGraph = nx.read_graphml(path)


    if exclude_dotted:

        dotted = [(u, v) for u, v, d in G.edges(data=True) if d.get("dotted")]

        G.remove_edges_from(dotted)

        print(f"[GRAPH] Removed {len(dotted)} dotted edges.")


    # Attach weight = Euclidean distance between node positions

    for u, v in G.edges():

        x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]

        x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]

        G[u][v]["weight"] = math.dist((x1, y1), (x2, y2))


    print(

        f"[GRAPH] Loaded: {G.number_of_nodes()} nodes, "

        f"{G.number_of_edges()} edges  "

        f"({'directed' if G.is_directed() else 'undirected'})"

    )

    return G



def euclidean_heuristic(G: nx.DiGraph):

    """Return an A* heuristic function that uses node (x, y) positions."""


    def h(u, v):

        x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]

        x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]

        return math.dist((x1, y1), (x2, y2))


    return h



# ═══════════════════════════════════════════════════════════════════════════════

# Pathfinding algorithms

# ═══════════════════════════════════════════════════════════════════════════════



def find_path(

    G: nx.DiGraph,

    source: str,

    target: str,

    algo: str = "astar",

) -> Optional[List[str]]:

    """

    Return the shortest path as a list of node IDs, or None if unreachable.


    algo: "astar" | "dijkstra" | "bfs"

    """

    if source not in G:

        print(f"[ERROR] Source node '{source}' not found in graph.")

        return None

    if target not in G:

        print(f"[ERROR] Target node '{target}' not found in graph.")

        return None


    try:

        if algo == "astar":

            path = nx.astar_path(

                G,

                source,

                target,

                heuristic=euclidean_heuristic(G),

                weight="weight",

            )

        elif algo == "dijkstra":

            path = nx.dijkstra_path(G, source, target, weight="weight")

        elif algo == "bfs":

            path = nx.shortest_path(G, source, target)  # unweighted BFS

        else:

            raise ValueError(

                f"Unknown algorithm '{algo}'. " "Choose: astar | dijkstra | bfs"

            )

        return path


    except nx.NetworkXNoPath:

        print(f"[PATHFIND] No path from '{source}' to '{target}'.")

        return None

    except nx.NodeNotFound as e:

        print(f"[PATHFIND] Node not found: {e}")

        return None



def path_length(G: nx.DiGraph, path: List[str]) -> float:

    """Total weighted length of a path."""

    return sum(G[u][v]["weight"] for u, v in zip(path, path[1:]))



def print_path(G: nx.DiGraph, path: List[str], algo: str) -> None:

    """Pretty-print the path to stdout."""

    length = path_length(G, path)

    print(f"\n{'─'*55}")

    print(f"  Algorithm : {algo.upper()}")

    print(f"  From      : {path[0]}")

    print(f"  To        : {path[-1]}")

    print(f"  Hops      : {len(path) - 1}")

    print(f"  Distance  : {length:.4f} map-units")

    print(f"{'─'*55}")

    print("  Path:", " → ".join(path))

    print(f"{'─'*55}\n")



# ═══════════════════════════════════════════════════════════════════════════════

# Pygame visualiser

# ═══════════════════════════════════════════════════════════════════════════════



def visualize(G: nx.DiGraph, path: Optional[List[str]]) -> None:

    """Render the graph and the computed path with Pygame."""

    try:

        import pygame

    except ImportError:

        print("[VIZ] pygame not installed — run:  pip install pygame")

        return


    # ── Camera ────────────────────────────────────────────────────────────────
    class Camera:
        def __init__(self, w, h):
            self.w = w
            self.h = h
            self.scale = 50.0
            # Default offset starts at bottom-left
            self.offset = [50, h - 50] 

        def w2s(self, wx, wy):
            # wx * scale added to X-offset
            # wy * scale subtracted from Y-offset (to flip Y axis up)
            return (
                int(self.offset[0] + wx * self.scale),
                int(self.offset[1] - wy * self.scale),
            )

        def s2w(self, sx, sy):
            return (
                (sx - self.offset[0]) / self.scale,
                (self.offset[1] - sy) / self.scale,
            )

        def pan(self, dx, dy):
            self.offset[0] += dx
            self.offset[1] += dy

        def zoom(self, f, cx, cy):
            wx, wy = self.s2w(cx, cy)
            self.scale = max(8.0, min(600.0, self.scale * f))
            # Recalculate offset so the world point under the mouse stays fixed
            self.offset[0] = cx - wx * self.scale
            self.offset[1] = cy + wy * self.scale

        def fit(self, mnx, mxx, mny, mxy, mg=60):
            rw = mxx - mnx
            rh = mxy - mny
            if rw == 0 or rh == 0:
                return
            
            # Determine scale to fit the bounding box
            self.scale = min((self.w - 2 * mg) / rw, (self.h - 2 * mg) / rh)
            
            # Center the map
            cx = (mnx + mxx) / 2
            cy = (mny + mxy) / 2
            
            self.offset[0] = self.w / 2 - cx * self.scale
            self.offset[1] = self.h / 2 + cy * self.scale


    # ── Setup ─────────────────────────────────────────────────────────────────

    pygame.init()

    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))

    pygame.display.set_caption("BFMC Pathfinding Visualizer")

    clock = pygame.time.Clock()

    font = pygame.font.SysFont("monospace", 13)

    font_sm = pygame.font.SysFont("monospace", 11)

    font_lg = pygame.font.SysFont("monospace", 16, bold=True)


    xs = [d["x"] for _, d in G.nodes(data=True)]

    ys = [d["y"] for _, d in G.nodes(data=True)]

    cam = Camera(WINDOW_W, WINDOW_H)

    cam.fit(min(xs), max(xs), min(ys), max(ys))


    path_set = set(path) if path else set()

    path_edges = set(zip(path, path[1:])) if path else set()


    source = path[0] if path else None

    target = path[-1] if path else None


    drag = False

    drag_last = (0, 0)

    hover_node = None

    show_dotted = True


    # ── Main loop ─────────────────────────────────────────────────────────────

    running = True

    while running:

        # Events

        mx, my = pygame.mouse.get_pos()

        wx, wy = cam.s2w(mx, my)

        thresh = max(0.3, 12 / cam.scale)

        hover_node = None

        best = thresh

        for nid, nd in G.nodes(data=True):

            d = math.dist((nd["x"], nd["y"]), (wx, wy))

            if d < best:

                best = d

                hover_node = nid


        for event in pygame.event.get():

            if event.type == pygame.QUIT:

                running = False

            elif event.type == pygame.KEYDOWN:

                if event.key == pygame.K_ESCAPE:

                    running = False

                elif event.key == pygame.K_f:

                    cam.fit(min(xs), max(xs), min(ys), max(ys))

                elif event.key == pygame.K_d:

                    show_dotted = not show_dotted

            elif event.type == pygame.MOUSEBUTTONDOWN:

                if event.button == 1:

                    drag = True

                    drag_last = event.pos

            elif event.type == pygame.MOUSEBUTTONUP:

                if event.button == 1:

                    drag = False

            elif event.type == pygame.MOUSEMOTION:

                if drag:

                    dx = event.pos[0] - drag_last[0]

                    dy = event.pos[1] - drag_last[1]

                    cam.pan(dx, dy)

                    drag_last = event.pos

            elif event.type == pygame.MOUSEWHEEL:

                cam.zoom(1.15 if event.y > 0 else 1 / 1.15, mx, my)


        # Draw

        screen.fill(BG)


        # Edges

        for u, v, d in G.edges(data=True):

            sx, sy = cam.w2s(G.nodes[u]["x"], G.nodes[u]["y"])

            ex, ey = cam.w2s(G.nodes[v]["x"], G.nodes[v]["y"])

            if (

                max(sx, ex) < -20

                or min(sx, ex) > WINDOW_W + 20

                or max(sy, ey) < -20

                or min(sy, ey) > WINDOW_H + 20

            ):

                continue


            is_path_edge = (u, v) in path_edges

            dotted = d.get("dotted", False)


            if is_path_edge:

                pygame.draw.line(screen, PATH_COL, (sx, sy), (ex, ey), 3)

            elif dotted and show_dotted:

                length = math.dist((sx, sy), (ex, ey))

                if length < 1:

                    continue

                steps = max(2, int(length / 8))

                for i in range(0, steps, 2):

                    ax = sx + (ex - sx) * i / steps

                    ay = sy + (ey - sy) * i / steps

                    bx = sx + (ex - sx) * min(i + 1, steps) / steps

                    by = sy + (ey - sy) * min(i + 1, steps) / steps

                    pygame.draw.line(

                        screen, EDGE_DOTTED, (int(ax), int(ay)), (int(bx), int(by)), 1

                    )

            else:

                pygame.draw.line(screen, EDGE_SOLID, (sx, sy), (ex, ey), 1)


        # Nodes

        for nid, nd in G.nodes(data=True):

            sx, sy = cam.w2s(nd["x"], nd["y"])

            if sx < -10 or sx > WINDOW_W + 10 or sy < -10 or sy > WINDOW_H + 10:

                continue

            if nid == source:

                col = NODE_START

                r = NODE_R + 4

            elif nid == target:

                col = NODE_END

                r = NODE_R + 4

            elif nid in path_set:

                col = NODE_PATH

                r = NODE_R + 2

            elif nid == hover_node:

                col = (255, 255, 255)

                r = NODE_R + 2

            else:

                col = NODE_COL

                r = NODE_R

            pygame.draw.circle(screen, col, (sx, sy), r)


            if nid == hover_node:

                lbl = font_sm.render(

                    f" {nid} ({nd['x']:.2f}, {nd['y']:.2f})", True, (255, 255, 255)

                )

                screen.blit(lbl, (sx + 8, sy - 8))


        # HUD

        y = 10


        def hud(txt, col=TEXT):

            nonlocal y

            screen.blit(font.render(txt, True, col), (10, y))

            y += 17


        if path:

            length = path_length(G, path)

            hud("BFMC Pathfinding Visualizer", TEXT)

            hud(f"Source : {source}", NODE_START)

            hud(f"Target : {target}", NODE_END)

            hud(f"Hops   : {len(path) - 1}", NODE_PATH)

            hud(f"Dist   : {length:.4f}", TEXT_GOOD)

        else:

            hud("No path found", TEXT_BAD)


        y += 8

        hud("─" * 26, TEXT_DIM)

        hud("[F]  Fit map", TEXT_DIM)

        hud("[D]  Toggle dotted", TEXT_DIM)

        hud("[Esc] Exit", TEXT_DIM)


        # Legend

        for col, label, lx, ly in [

            (NODE_START, "Start", WINDOW_W - 140, 10),

            (NODE_END, "End", WINDOW_W - 140, 28),

            (NODE_PATH, "Path", WINDOW_W - 140, 46),

        ]:

            pygame.draw.circle(screen, col, (lx, ly + 5), 6)

            screen.blit(font.render(label, True, TEXT), (lx + 14, ly))


        pygame.display.flip()

        clock.tick(FPS)


    pygame.quit()



# ═══════════════════════════════════════════════════════════════════════════════

# CLI

# ═══════════════════════════════════════════════════════════════════════════════



def prompt_node(G: nx.DiGraph, label: str) -> str:

    while True:

        val = input(f"  {label} node ID: ").strip()

        if val in G:

            return val

        print(

            f"  [!] Node '{val}' not found. " f"Example valid IDs: {list(G.nodes)[:5]}"

        )


def print_nodes_in_rectangle(G: nx.DiGraph, x_min=0, y_min=0, x_max=6.5, y_max=7):
    """Prints all nodes located within the bounding box in a comma-separated list."""
    found_nodes = []
    
    # Collect all IDs that meet the criteria
    for nid, data in G.nodes(data=True):
        x, y = data.get("x"), data.get("y")
        if x is not None and y is not None:
            if x_min <= x <= x_max and y_min <= y <= y_max:
                found_nodes.append(f'"{nid}"')
    
    # Print in the requested format
    if found_nodes:
        print(f"NODES: {', '.join(found_nodes)}")
    else:
        print("NODES: None found")
        
    print(f"Total nodes found: {len(found_nodes)}")


def main():

    ap = argparse.ArgumentParser(

        description="Pathfinding on the BFMC Competition Track GraphML",

        formatter_class=argparse.RawTextHelpFormatter,

    )

    ap.add_argument(
        "--map",

        default=DEFAULT_MAP_PATH,
        
        help="Path to the .graphml file",
    )

    ap.add_argument(

        "--source", default=None, help="Source node ID (interactive prompt if omitted)"

    )

    ap.add_argument(

        "--target", default=None, help="Target node ID (interactive prompt if omitted)"

    )

    ap.add_argument(

        "--algo",

        default="bfs",

        choices=["astar", "dijkstra", "bfs"],

        help="Pathfinding algorithm  (default: astar)",

    )

    ap.add_argument(

        "--no-dotted",

        action="store_true",

        help="Exclude dotted (shortcut) edges from the graph",

    )

    ap.add_argument(

        "--visualize",

        action="store_true",

        help="Open a Pygame window to visualise the result",

    )

    args = ap.parse_args()


    G = load_graph(args.map, exclude_dotted=args.no_dotted)

    print_nodes_in_rectangle(G, 0, 0, 6.5, 7)


    # ── Node selection ────────────────────────────────────────────────────────

    if args.source and args.target:

        source, target = str(args.source), str(args.target)

    else:

        print(f"\n  Graph has {G.number_of_nodes()} nodes.")

        print(f"  Example IDs: {list(G.nodes)[:10]}\n")

        source = args.source or prompt_node(G, "Source")

        target = args.target or prompt_node(G, "Target")


    # ── Pathfinding ───────────────────────────────────────────────────────────

    print(f"\n[PATHFIND] {args.algo.upper()}  {source} → {target}")

    path = find_path(G, source, target, algo=args.algo)


    if path:

        print_path(G, path, args.algo)

    else:

        print(f"[PATHFIND] Could not find a path from {source} to {target}.")


    # ── Optional visualisation ────────────────────────────────────────────────

    if args.visualize:

        visualize(G, path)



if __name__ == "__main__":

    main() 