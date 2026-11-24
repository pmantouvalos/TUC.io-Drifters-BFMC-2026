"""
BFMC Live Track Visualization  ─  ROLLING TARGET VERSION
=========================================================

Διαφορές από την προηγούμενη έκδοση:
  ✦ Δεν υπάρχει πλέον το [T] Set Target
  ✦ Το target node και το path έρχονται αυτόματα από τον
    threadcontroller μέσω telemetry UDP (port 9002)
  ✦ Εμφανίζει τον τρέχοντα checkpoint αριθμό
  ✦ Εμφανίζει το heading (κατεύθυνση) του αμαξιού

── ΑΠΑΙΤΗΣΕΙΣ ──────────────────────────────────────────
    pip install pygame

── ΤΡΕΞΙΜΟ ─────────────────────────────────────────────
  Χωρίς αμάξι (demo):
    python3 visualization.py --no-car --map Competition_track_graph.graphml

  Με αμάξι (lab):
    python3 visualization.py --map Competition_track_graph.graphml

── SHORTCUTS ────────────────────────────────────────────
  C     Calibration (κάθε φορά που αλλάζεις κομμάτι)
  A     Toggle auto-follow
  D     Toggle dotted edges
  F     Fit ολόκληρος χάρτης
  S     Save run JSON
  R     Toggle recording
  L     Toggle log panel
  M     Toggle mini-map
  Esc   Έξοδος
"""

import sys, math, threading, time, argparse, json, socket
import xml.etree.ElementTree as ET
from collections import deque
from datetime import datetime
from typing import Dict, List, Tuple, Optional

try:
    import pygame
except ImportError:
    print("pip install pygame"); sys.exit(1)

SEM_UDP_PORT       = 5007
TELEMETRY_UDP_PORT = 9002

# ── Χρώματα ───────────────────────────────────────────────────────────────────
BG           = (15,  15,  25)
EDGE_SOLID   = (40,  65, 100)
EDGE_DOTTED  = (60, 130,  55)
NODE_COL     = (50, 120, 220)
NODE_PATH    = (255, 200,   0)
NODE_REF     = (0,  220, 120)
NODE_HOVER   = (255, 255, 255)
NODE_CP      = (255, 100, 200)   # checkpoint node
PATH_COL     = (255, 200,   0)
TRAIL_COL    = (180,  60,  60)
CAR_COL      = (220,  40,  40)
CAR_HALO     = (255, 255, 255)
SEM_RED      = (220,  40,  40)
SEM_YEL      = (220, 180,  40)
SEM_GRN      = (40,  200,  80)
SEM_UNK      = (100, 100, 100)
TEXT         = (200, 200, 200)
TEXT_DIM     = (110, 110, 130)
TEXT_GOOD    = (80,  220, 100)
TEXT_WARN    = (220, 180,  40)
TEXT_BAD     = (220,  60,  60)
CALIB_BG     = (30,  30,  50)
LOG_BG       = (18,  18,  32)
MINIMAP_BG   = (18,  18,  32)
MINIMAP_EDGE = (20,  50,  20)
MINIMAP_BDR  = (60,  60, 100)
HEADING_COL  = (100, 200, 255)

DET_COLORS = {
    "Stop":        (220,  40,  40),
    "STOP_LINE":   (220,  40,  40),
    "Crosswalk":   (255, 200,   0),
    "Pedestrian":  (255, 140,   0),
    "Car":         (140, 200, 255),
    "Parking":     (180,  80, 220),
    "HighwayEntry":(80,  220, 220),
    "HighwayExit": (80,  220, 220),
    "Priority":    (100, 220, 100),
}

MODE_COLORS = {
    "AUTO":        (80,  220, 100),
    "MANUAL":      (100, 150, 220),
    "IDLE":        (110, 110, 130),
    "INIT":        (110, 110, 130),
    "STOP":        (220,  40,  40),
    "FIND":        (220, 140,  40),
    "PARKING":     (180,  80, 220),
    "PARKED":      (140,  60, 200),
    "UNPARKING":   (200, 100, 220),
    "HIGHWAY":     (80,  220, 220),
    "ENTER":       (80,  220, 220),
    "EXIT":        (80,  220, 220),
    "PEDESTRIAN":  (255, 140,   0),
    "CROSSWALK":   (255, 200,   0),
    "INTERSECTION":(255, 160,  60),
    "MOVE":        (100, 200, 100),
}

WINDOW_W = 1280; WINDOW_H = 800
LOG_W    = 265;  MAIN_W   = WINDOW_W - LOG_W
MINI_W   = 220;  MINI_H   = 160
NODE_R   = 4;    CAR_R    = 10
TRAIL_LEN= 300;  FPS      = 30


# ═════════════════════════════════════════════════════════════════════════════
class TrackMap:
    def __init__(self, path: str):
        self.nodes: Dict[str, Tuple[float,float]] = {}
        self.edges: List[Tuple[str,str,bool]]     = []
        self._load(path)

    def _load(self, path: str):
        try:    tree = ET.parse(path)
        except FileNotFoundError:
            print(f"[MAP] Not found: {path}"); sys.exit(1)
        root = tree.getroot()
        tag  = root.tag
        ns   = {"g": tag[1:tag.index("}")]} if tag.startswith("{") else {}

        def fa(el,t): return el.findall(f"g:{t}",ns) if ns else el.findall(t)
        def f1(el,t): return el.find(f"g:{t}",ns)    if ns else el.find(t)

        key_map = {k.get("attr.name",""):k.get("id","") for k in fa(root,"key")}
        x_key   = key_map.get("x","d0")
        y_key   = key_map.get("y","d1")
        dot_key = key_map.get("dotted","d2")

        graph = f1(root,"graph") or root.find("graph")
        if graph is None: print("[MAP] No <graph>"); sys.exit(1)

        for node in fa(graph,"node"):
            nid=node.get("id"); x=y=0.0
            for d in fa(node,"data"):
                if d.text is None: continue
                k=d.get("key","")
                try:
                    if k==x_key: x=float(d.text)
                    if k==y_key: y=float(d.text)
                except ValueError: pass
            self.nodes[nid]=(x,y)

        for edge in fa(graph,"edge"):
            src=edge.get("source"); dst=edge.get("target"); dotted=False
            for d in fa(edge,"data"):
                if d.get("key","")==dot_key and d.text:
                    dotted=d.text.strip().upper()=="TRUE"
            if src in self.nodes and dst in self.nodes:
                self.edges.append((src,dst,dotted))

        nd=sum(1 for _,_,d in self.edges if d)
        print(f"[MAP] {len(self.nodes)} nodes, {len(self.edges)} edges "
              f"({nd} dotted)")

    def bounds(self):
        xs=[v[0] for v in self.nodes.values()]; ys=[v[1] for v in self.nodes.values()]
        return min(xs),max(xs),min(ys),max(ys)

    def find_nearest(self,x,y):
        if not self.nodes: return None
        return min(self.nodes,key=lambda n:(self.nodes[n][0]-x)**2+(self.nodes[n][1]-y)**2)


# ═════════════════════════════════════════════════════════════════════════════
class DWMReader:
    def __init__(self,port):
        self.port=port; self._pos=None; self._lock=threading.Lock()
        self._running=False

    def start(self):
        self._running=True
        threading.Thread(target=self._loop,daemon=True).start()

    def stop(self): self._running=False

    def get(self):
        with self._lock: return dict(self._pos) if self._pos else None

    def _loop(self):
        try: import serial
        except ImportError: print("[DWM] pip install pyserial"); return
        while self._running:
            try:
                ser=serial.Serial(self.port,115200,timeout=1.0)
                ser.write(b"\r\r"); time.sleep(0.5)
                ser.write(b"lep\r"); time.sleep(0.5)
                print(f"[DWM] Connected on {self.port}")
                while self._running:
                    line=ser.readline().decode("utf-8",errors="ignore").strip()
                    if line.startswith("POS"):
                        p=line.split(",")
                        if len(p)>=5:
                            with self._lock:
                                self._pos={"x":float(p[1]),"y":float(p[2]),
                                           "z":float(p[3]),"quality":int(p[4])}
                ser.close()
            except Exception as e: print(f"[DWM] {e} — retry 3s"); time.sleep(3)


# ═════════════════════════════════════════════════════════════════════════════
class SemaphoreReceiver:
    def __init__(self):
        self._data:Dict[str,dict]={}; self._lock=threading.Lock()
        self._running=False

    def start(self):
        self._running=True
        threading.Thread(target=self._loop,daemon=True).start()

    def stop(self): self._running=False

    def get_all(self):
        with self._lock: return dict(self._data)

    def _loop(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try: sock.bind(("",SEM_UDP_PORT)); print(f"[SEM] UDP {SEM_UDP_PORT}")
        except Exception as e: print(f"[SEM] {e}"); return
        sock.settimeout(1.0)
        while self._running:
            try:
                data,_=sock.recvfrom(4096)
                msg=json.loads(data.decode("utf-8"))
                sid=str(msg.get("id","?"))
                with self._lock:
                    self._data[sid]={"state":int(msg.get("state",0)),
                                     "x":float(msg.get("x",0.0)),
                                     "y":float(msg.get("y",0.0)),
                                     "ts":time.time()}
            except socket.timeout: continue
            except Exception: continue
        sock.close()


# ═════════════════════════════════════════════════════════════════════════════
class TelemetryReceiver:
    """
    Λαμβάνει από threadcontroller:
    {
      "mode":        "AUTO",
      "detections":  [...],
      "car_x":       1.23,
      "car_y":       4.56,
      "checkpoint":  2,
      "target_node": "42",
      "path_len":    15,
      "heading":     90.0
    }
    """
    def __init__(self):
        self._data={"mode":"UNKNOWN","detections":[],"car_x":None,"car_y":None,
                    "checkpoint":0,"target_node":"","path_len":0,"heading":None}
        self._lock=threading.Lock(); self._running=False

    def start(self):
        self._running=True
        threading.Thread(target=self._loop,daemon=True).start()

    def stop(self): self._running=False

    def get(self):
        with self._lock: return dict(self._data)

    def _loop(self):
        sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        try: sock.bind(("",TELEMETRY_UDP_PORT)); print(f"[TELEM] UDP {TELEMETRY_UDP_PORT}")
        except Exception as e: print(f"[TELEM] {e}"); return
        sock.settimeout(1.0)
        while self._running:
            try:
                data,_=sock.recvfrom(8192)
                with self._lock: self._data=json.loads(data.decode("utf-8"))
            except socket.timeout: continue
            except Exception: continue
        sock.close()


# ═════════════════════════════════════════════════════════════════════════════
class Camera:
    def __init__(self,w,h):
        self.w=w; self.h=h; self.scale=50.0; self.offset=[w/2,h/2]

    def w2s(self,wx,wy):
        return(int(self.offset[0]+wx*self.scale),int(self.offset[1]-wy*self.scale))

    def s2w(self,sx,sy):
        return((sx-self.offset[0])/self.scale,-(sy-self.offset[1])/self.scale)

    def pan(self,dx,dy): self.offset[0]+=dx; self.offset[1]+=dy

    def zoom(self,f,cx,cy):
        wx,wy=self.s2w(cx,cy); self.scale=max(8.0,min(600.0,self.scale*f))
        self.offset[0]=cx-wx*self.scale; self.offset[1]=cy+wy*self.scale

    def fit(self,mnx,mxx,mny,mxy,mg=60):
        rw=mxx-mnx; rh=mxy-mny
        if rw==0 or rh==0: return
        self.scale=min((self.w-2*mg)/rw,(self.h-2*mg)/rh)
        cx=(mnx+mxx)/2; cy=(mny+mxy)/2
        self.offset=[self.w/2-cx*self.scale,self.h/2+cy*self.scale]

    def center_on(self,wx,wy):
        self.offset=[self.w/2-wx*self.scale,self.h/2+wy*self.scale]


# ═════════════════════════════════════════════════════════════════════════════
class EventLog:
    def __init__(self,maxlen=14):
        self._entries=deque(maxlen=maxlen); self._lock=threading.Lock()

    def add(self,msg:str,color=None):
        ts=datetime.now().strftime("%H:%M:%S")
        with self._lock:
            self._entries.append({"ts":ts,"msg":msg,"color":color or TEXT})
        print(f"[LOG] {ts}  {msg}")

    def get(self):
        with self._lock: return list(self._entries)


# ═════════════════════════════════════════════════════════════════════════════
class Visualizer:
    def __init__(self,track,dwm,sem,telem):
        self.track=track; self.dwm=dwm; self.sem=sem; self.telem=telem

        # Calibration
        self.offset_x=0.0; self.offset_y=0.0
        self.ref_node=None; self.calibrating=False; self.calib_input=""

        # Car state (da DWM o da telemetry)
        self.car_map_pos:Optional[Tuple[float,float]]=None
        self.trail=deque(maxlen=TRAIL_LEN); self._last_trail=None

        # Path (dal telemetry del threadcontroller)
        self.path_nodes:List[str]=[]    # da telemetry — solo per visualizzazione
        self.target_node=""
        self.hover_node:Optional[str]=None

        # UI
        self.auto_follow=False; self.show_dotted=True
        self.show_log=True; self.show_minimap=True

        # Log
        self.log=EventLog()
        self.log.add("Visualizer started",TEXT_GOOD)
        self.log.add(f"{len(track.nodes)} nodes loaded",TEXT_DIM)
        self.log.add("Rolling target — auto from controller",TEXT_GOOD)

        # Recording
        self._run_data:List[dict]=[]; self._recording=False

        # Change detection per log
        self._prev_mode=""; self._prev_dets:List[str]=[]
        self._prev_cp=-1; self._sem_states:Dict[str,int]={}

        pygame.init()
        self.screen=pygame.display.set_mode((WINDOW_W,WINDOW_H))
        pygame.display.set_caption("BFMC Track Visualizer — Rolling Target")
        self.clock=pygame.time.Clock()
        self.font   =pygame.font.SysFont("monospace",13)
        self.font_sm=pygame.font.SysFont("monospace",11)
        self.font_lg=pygame.font.SysFont("monospace",16,bold=True)

        self.cam=Camera(MAIN_W,WINDOW_H)
        mn,mx,mny,mxy=track.bounds(); self.cam.fit(mn,mx,mny,mxy)

        self._main_surf=pygame.Surface((MAIN_W, WINDOW_H))
        self._log_surf =pygame.Surface((LOG_W,  WINDOW_H))
        self._mini_surf=pygame.Surface((MINI_W, MINI_H))
        self._drag=False; self._drag_last=(0,0)

    # ── Calibration ───────────────────────────────────────────────────────────

    def apply_calibration(self,node_id:str):
        if node_id not in self.track.nodes:
            self.log.add(f"Node '{node_id}' not found",TEXT_BAD); return
        nx,ny=self.track.nodes[node_id]
        pos=self.dwm.get() if self.dwm else None
        if pos is None:
            self.cam.center_on(nx,ny); self.ref_node=node_id
            self.log.add(f"Pan → node {node_id}",TEXT_GOOD)
        else:
            self.offset_x=nx-pos["x"]; self.offset_y=ny-pos["y"]
            self.ref_node=node_id
            self.log.add(
                f"Calibrated {node_id} Δ=({self.offset_x:.2f},{self.offset_y:.2f})",
                TEXT_GOOD)

    def _dwm_to_map(self,x,y): return x+self.offset_x, y+self.offset_y

    # ── Update ────────────────────────────────────────────────────────────────

    def update(self):
        td=self.telem.get()

        # Car position — prendi da telemetry se disponibile, altrimenti da DWM
        tx=td.get("car_x"); ty=td.get("car_y")
        if tx is not None and ty is not None:
            self.car_map_pos=(float(tx),float(ty))
        elif self.dwm:
            pos=self.dwm.get()
            if pos and pos["quality"]>=15:
                self.car_map_pos=self._dwm_to_map(pos["x"],pos["y"])

        if self.car_map_pos:
            mx,my=self.car_map_pos
            if (self._last_trail is None or
                    math.dist((mx,my),self._last_trail)>0.05):
                self.trail.append((mx,my)); self._last_trail=(mx,my)
            if self.auto_follow: self.cam.center_on(mx,my)
            if self._recording:
                self._run_data.append({"t":time.time(),"x":mx,"y":my})

        # Rolling target info from telemetry
        self.target_node=str(td.get("target_node",""))
        cp=td.get("checkpoint",0)

        # Checkpoint change log
        if cp!=self._prev_cp:
            self.log.add(
                f"Checkpoint {cp} → node {self.target_node}",NODE_CP)
            self._prev_cp=cp

        # Mode log
        mode=td.get("mode","")
        if mode!=self._prev_mode and mode:
            col=TEXT
            for k,v in MODE_COLORS.items():
                if mode.startswith(k): col=v; break
            self.log.add(f"Mode → {mode}",col)
            self._prev_mode=mode

        # Detections log
        dets=td.get("detections",[])
        labels=sorted(set(d.get("label","") for d in dets if d))
        if labels!=self._prev_dets:
            if labels: self.log.add("Det: "+", ".join(labels),TEXT_WARN)
            self._prev_dets=labels

        # Semaphore log
        for sid,sd in self.sem.get_all().items():
            if time.time()-sd["ts"]<0.3:
                if self._sem_states.get(sid)!=sd["state"]:
                    self._sem_states[sid]=sd["state"]
                    name={0:"RED",1:"YELLOW",2:"GREEN"}.get(sd["state"],"?")
                    col=[SEM_RED,SEM_YEL,SEM_GRN][sd["state"]] \
                        if sd["state"]<3 else SEM_UNK
                    self.log.add(f"Semaphore {sid} → {name}",col)

    # ── Draw ──────────────────────────────────────────────────────────────────

    def draw(self):
        self._main_surf.fill(BG)
        self._draw_edges()
        self._draw_trail()
        self._draw_nodes()
        self._draw_semaphores()
        self._draw_car()
        self._draw_heading()
        self._draw_detections()
        self._draw_hud()
        if self.calibrating: self._draw_calib_dialog()
        self.screen.blit(self._main_surf,(0,0))

        if self.show_log:
            self._draw_log_panel()
            self.screen.blit(self._log_surf,(MAIN_W,0))

        if self.show_minimap:
            self._draw_minimap()
            self.screen.blit(self._mini_surf,(MAIN_W-MINI_W-10,10))

        pygame.display.flip()

    def _draw_edges(self):
        for src,dst,dotted in self.track.edges:
            sx,sy=self.cam.w2s(*self.track.nodes[src])
            dx,dy=self.cam.w2s(*self.track.nodes[dst])
            if(max(sx,dx)<-20 or min(sx,dx)>MAIN_W+20 or
               max(sy,dy)<-20 or min(sy,dy)>WINDOW_H+20): continue
            color=EDGE_DOTTED if(dotted and self.show_dotted) else EDGE_SOLID
            if dotted and self.show_dotted:
                length=math.dist((sx,sy),(dx,dy))
                if length<1: continue
                steps=max(2,int(length/8))
                for i in range(0,steps,2):
                    ax=sx+(dx-sx)*i/steps; ay=sy+(dy-sy)*i/steps
                    bx=sx+(dx-sx)*min(i+1,steps)/steps
                    by=sy+(dy-sy)*min(i+1,steps)/steps
                    pygame.draw.line(self._main_surf,color,
                                     (int(ax),int(ay)),(int(bx),int(by)),1)
            else:
                pygame.draw.line(self._main_surf,color,(sx,sy),(dx,dy),1)

    def _draw_trail(self):
        pts=list(self.trail)
        if len(pts)<2: return
        n=len(pts)
        for i in range(1,n):
            r=min(255,TRAIL_COL[0]); g=min(255,TRAIL_COL[1]+int(40*i/n))
            b=TRAIL_COL[2]
            pygame.draw.line(self._main_surf,(r,g,b),
                             self.cam.w2s(*pts[i-1]),self.cam.w2s(*pts[i]),2)

    def _draw_nodes(self):
        td=self.telem.get()
        target=str(td.get("target_node",""))
        for nid,(nx,ny) in self.track.nodes.items():
            sx,sy=self.cam.w2s(nx,ny)
            if sx<-10 or sx>MAIN_W+10 or sy<-10 or sy>WINDOW_H+10: continue
            if   nid==self.ref_node:   col=NODE_REF;   r=NODE_R+3
            elif nid==target:          col=NODE_CP;    r=NODE_R+4
            elif nid==self.hover_node: col=NODE_HOVER; r=NODE_R+2
            else:                      col=NODE_COL;   r=NODE_R
            pygame.draw.circle(self._main_surf,col,(sx,sy),r)
            # Target node — disegna anello
            if nid==target:
                pygame.draw.circle(self._main_surf,NODE_CP,(sx,sy),r+3,2)
            if nid==self.hover_node:
                lbl=self.font_sm.render(f" {nid} ({nx:.2f},{ny:.2f})",
                                        True,NODE_HOVER)
                self._main_surf.blit(lbl,(sx+8,sy-8))

    def _draw_semaphores(self):
        for sid,sd in self.sem.get_all().items():
            sx,sy=self.cam.w2s(sd["x"],sd["y"])
            if sx<-10 or sx>MAIN_W+10 or sy<-10 or sy>WINDOW_H+10: continue
            col=[SEM_RED,SEM_YEL,SEM_GRN][sd["state"]] \
                if sd["state"]<3 else SEM_UNK
            pygame.draw.circle(self._main_surf,(255,255,255),(sx,sy),11)
            pygame.draw.circle(self._main_surf,col,(sx,sy),9)
            self._main_surf.blit(
                self.font_sm.render(f" {sid}",True,TEXT),(sx+12,sy-6))

    def _draw_car(self):
        if self.car_map_pos is None: return
        sx,sy=self.cam.w2s(*self.car_map_pos)
        pygame.draw.circle(self._main_surf,CAR_HALO,(sx,sy),CAR_R+3)
        pygame.draw.circle(self._main_surf,CAR_COL,  (sx,sy),CAR_R)
        pygame.draw.circle(self._main_surf,(255,255,255),(sx,sy),CAR_R,2)

    def _draw_heading(self):
        """Freccia che mostra il heading verso il prossimo waypoint."""
        if self.car_map_pos is None: return
        td=self.telem.get()
        heading=td.get("heading")
        if heading is None: return
        sx,sy=self.cam.w2s(*self.car_map_pos)
        length=40
        ex=int(sx+length*math.sin(math.radians(heading)))
        ey=int(sy-length*math.cos(math.radians(heading)))
        pygame.draw.line(self._main_surf,HEADING_COL,(sx,sy),(ex,ey),2)
        pygame.draw.circle(self._main_surf,HEADING_COL,(ex,ey),4)

    def _draw_detections(self):
        if self.car_map_pos is None: return
        td=self.telem.get(); dets=td.get("detections",[])
        if not dets: return
        sx,sy=self.cam.w2s(*self.car_map_pos); oy=sy-CAR_R-16
        for det in dets[:6]:
            label=det.get("label",""); prox=det.get("proximity","")
            col=DET_COLORS.get(label,TEXT_WARN)
            self._main_surf.blit(
                self.font_sm.render(f"▲{label}:{prox}",True,col),(sx+14,oy))
            oy-=14

    def _draw_hud(self):
        td=self.telem.get()
        mode=td.get("mode","—"); cp=td.get("checkpoint",0)
        target=str(td.get("target_node","?")); plen=td.get("path_len",0)
        heading=td.get("heading")

        # Mode badge
        mcol=TEXT
        for k,v in MODE_COLORS.items():
            if mode.startswith(k): mcol=v; break
        self._main_surf.blit(self.font_lg.render(f"● {mode}",True,mcol),(10,10))

        y=38
        def line(txt,col=TEXT_DIM):
            nonlocal y
            self._main_surf.blit(self.font.render(txt,True,col),(10,y)); y+=16

        line("BFMC Track Visualizer",TEXT)
        line("Rolling Target Mode",TEXT_GOOD)
        line("─"*28)

        # DWM info
        if self.dwm is None:
            line("DEMO — no DWM1001",TEXT_WARN)
        else:
            pos=self.dwm.get()
            if pos:
                q=pos["quality"]
                qc=TEXT_GOOD if q>70 else(TEXT_WARN if q>40 else TEXT_BAD)
                mx2,my2=self._dwm_to_map(pos["x"],pos["y"])
                line(f"MAP: ({mx2:.2f},{my2:.2f})",TEXT)
                line(f"Quality: {q}%",qc)
                bx,by,bw,bh=10,y,140,8; y+=12
                pygame.draw.rect(self._main_surf,(40,40,60),(bx,by,bw,bh),
                                 border_radius=4)
                fw=int(bw*q/100)
                bc=(40,200,80) if q>70 else((220,180,40) if q>40 else(220,60,60))
                pygame.draw.rect(self._main_surf,bc,(bx,by,fw,bh),border_radius=4)
            else:
                line("DWM: waiting...",TEXT_WARN)

        y+=4; line("─"*28)

        # Rolling target info
        line(f"Checkpoint: {cp}",NODE_CP)
        line(f"Target node: {target}",NODE_CP)
        line(f"Path length: {plen} nodes",TEXT)
        if heading is not None:
            line(f"Heading: {heading:.1f}°",HEADING_COL)

        if self.ref_node:
            line(f"Ref node: {self.ref_node}",NODE_REF)
        if self._recording:
            line(f"● REC  {len(self._run_data)} pts",TEXT_BAD)

        y+=4; line("─"*28)
        for k,txt in [("C","Calibration"),("A","Auto-follow"),
                      ("D","Dotted edges"),("F","Fit map"),
                      ("S","Save JSON"),("R","Record"),
                      ("L","Log"),("M","Mini-map"),("Esc","Exit")]:
            col=TEXT_GOOD if(k=="A" and self.auto_follow)else TEXT_DIM
            line(f"[{k}] {txt}",col)

    def _draw_log_panel(self):
        self._log_surf.fill(LOG_BG)
        pygame.draw.line(self._log_surf,(50,50,80),(0,0),(0,WINDOW_H),2)
        self._log_surf.blit(
            self.font_lg.render("EVENT LOG",True,(100,100,200)),(10,10))
        pygame.draw.line(self._log_surf,(50,50,80),(4,30),(LOG_W-4,30),1)
        y=36
        for e in reversed(self.log.get()):
            self._log_surf.blit(
                self.font_sm.render(e["ts"],True,TEXT_DIM),(8,y))
            self._log_surf.blit(
                self.font_sm.render(e["msg"][:30],True,e["color"]),(8,y+12))
            y+=30
            if y>WINDOW_H-10: break

    def _draw_minimap(self):
        surf=self._mini_surf; surf.fill(MINIMAP_BG)
        pygame.draw.rect(surf,MINIMAP_BDR,(0,0,MINI_W,MINI_H),2,border_radius=4)
        mn_x,mx_x,mn_y,mx_y=self.track.bounds()
        rw=mx_x-mn_x; rh=mx_y-mn_y
        if rw==0 or rh==0: return
        mg=8; sc=min((MINI_W-2*mg)/rw,(MINI_H-2*mg)/rh)

        def mm(wx,wy):
            return(int(mg+(wx-mn_x)*sc),int(MINI_H-mg-(wy-mn_y)*sc))

        for src,dst,_ in self.track.edges:
            pygame.draw.line(surf,MINIMAP_EDGE,
                             mm(*self.track.nodes[src]),
                             mm(*self.track.nodes[dst]),1)

        for _,sd in self.sem.get_all().items():
            col=[SEM_RED,SEM_YEL,SEM_GRN][sd["state"]] \
                if sd["state"]<3 else SEM_UNK
            pygame.draw.circle(surf,col,mm(sd["x"],sd["y"]),3)

        # Target node
        td=self.telem.get(); target=str(td.get("target_node",""))
        if target and target in self.track.nodes:
            tx,ty=mm(*self.track.nodes[target])
            pygame.draw.circle(surf,NODE_CP,(tx,ty),5)
            pygame.draw.circle(surf,NODE_CP,(tx,ty),5,2)

        if self.car_map_pos:
            cx,cy=mm(*self.car_map_pos)
            pygame.draw.circle(surf,(255,255,255),(cx,cy),5)
            pygame.draw.circle(surf,CAR_COL,(cx,cy),4)

        # Viewport rectangle
        tl=self.cam.s2w(0,0); br=self.cam.s2w(MAIN_W,WINDOW_H)
        sx1,sy1=mm(*tl); sx2,sy2=mm(*br)
        vw=abs(sx2-sx1); vh=abs(sy2-sy1)
        pygame.draw.rect(surf,(80,80,140),
                         (min(sx1,sx2),min(sy1,sy2),max(vw,4),max(vh,4)),1)

        surf.blit(self.font_sm.render("OVERVIEW",True,TEXT_DIM),(4,2))

    def _draw_calib_dialog(self):
        w,h=440,165; x=(MAIN_W-w)//2; y=(WINDOW_H-h)//2
        pygame.draw.rect(self._main_surf,CALIB_BG,(x,y,w,h),border_radius=8)
        pygame.draw.rect(self._main_surf,(80,80,120),(x,y,w,h),2,border_radius=8)
        self._main_surf.blit(
            self.font_lg.render("CALIBRATION",True,(255,200,0)),(x+150,y+10))
        for i,msg in enumerate(["Βάλε αμάξι στην αρχή του κομματιού.",
                                 "Hover πάνω στον κόμβο → δες το ID.",
                                 "Γράψε Node ID και πάτα Enter:"]):
            self._main_surf.blit(self.font.render(msg,True,TEXT),(x+12,y+38+i*18))
        self._main_surf.blit(
            self.font_lg.render(f"> {self.calib_input}_",True,(0,220,120)),
            (x+12,y+106))
        self._main_surf.blit(
            self.font.render("Esc: Ακύρωση",True,TEXT_DIM),(x+12,y+140))

    # ── Events ────────────────────────────────────────────────────────────────

    def handle_events(self)->bool:
        mx,my=pygame.mouse.get_pos()
        if mx<MAIN_W:
            wx,wy=self.cam.s2w(mx,my); self.hover_node=None
            thresh=max(0.3,12/self.cam.scale); best=thresh
            for nid,(nx,ny) in self.track.nodes.items():
                d=math.dist((nx,ny),(wx,wy))
                if d<best: best=d; self.hover_node=nid

        for event in pygame.event.get():
            if event.type==pygame.QUIT: return False

            if self.calibrating:
                if event.type==pygame.KEYDOWN:
                    if   event.key==pygame.K_RETURN:
                        self.apply_calibration(self.calib_input.strip())
                        self.calibrating=False; self.calib_input=""
                    elif event.key==pygame.K_ESCAPE:
                        self.calibrating=False; self.calib_input=""
                    elif event.key==pygame.K_BACKSPACE:
                        self.calib_input=self.calib_input[:-1]
                    else: self.calib_input+=event.unicode
                continue

            if event.type==pygame.KEYDOWN:
                k=event.key
                if   k==pygame.K_c:
                    self.calibrating=True; self.calib_input=self.hover_node or ""
                elif k==pygame.K_a:
                    self.auto_follow=not self.auto_follow
                    self.log.add(f"Auto-follow {'ON' if self.auto_follow else 'OFF'}")
                elif k==pygame.K_d: self.show_dotted=not self.show_dotted
                elif k==pygame.K_f:
                    mn,mx2,mny,mxy=self.track.bounds(); self.cam.fit(mn,mx2,mny,mxy)
                elif k==pygame.K_l: self.show_log=not self.show_log
                elif k==pygame.K_m: self.show_minimap=not self.show_minimap
                elif k==pygame.K_s: self._save_run()
                elif k==pygame.K_r:
                    self._recording=not self._recording
                    self.log.add(f"Recording {'ON' if self._recording else 'OFF'}",
                                 TEXT_GOOD if self._recording else TEXT_DIM)
                elif k==pygame.K_ESCAPE: return False

            elif event.type==pygame.MOUSEBUTTONDOWN and mx<MAIN_W:
                if event.button==1: self._drag=True; self._drag_last=event.pos
            elif event.type==pygame.MOUSEBUTTONUP:
                if event.button==1: self._drag=False
            elif event.type==pygame.MOUSEMOTION:
                if self._drag and mx<MAIN_W:
                    dx=event.pos[0]-self._drag_last[0]
                    dy=event.pos[1]-self._drag_last[1]
                    self.cam.pan(dx,dy); self._drag_last=event.pos
            elif event.type==pygame.MOUSEWHEEL and mx<MAIN_W:
                self.cam.zoom(1.15 if event.y>0 else 1/1.15,mx,my)

        return True

    def _save_run(self):
        fname=f"run_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(fname,"w") as f:
            json.dump({"recorded_at":datetime.now().isoformat(),
                       "trail":list(self.trail),
                       "positions":self._run_data},f,indent=2)
        self.log.add(f"Saved: {fname}",TEXT_GOOD)

    def run(self):
        running=True
        while running:
            self.update(); running=self.handle_events(); self.draw()
            self.clock.tick(FPS)
        pygame.quit()


# ═════════════════════════════════════════════════════════════════════════════
def main():
    ap=argparse.ArgumentParser(description="BFMC Track Visualizer")
    ap.add_argument("--map",    default="src/pathplanning/Competition_track_graph.graphml")
    ap.add_argument("--port",   default="/dev/ttyACM1")
    ap.add_argument("--no-car", action="store_true")
    args=ap.parse_args()

    track=TrackMap(args.map)
    dwm=None
    if not args.no_car:
        dwm=DWMReader(args.port); dwm.start()
        print(f"[DWM] Waiting signal on {args.port}...")
    else:
        print("[INFO] Demo mode")

    sem  =SemaphoreReceiver(); sem.start()
    telem=TelemetryReceiver(); telem.start()

    viz=Visualizer(track,dwm,sem,telem)

    print("\n── Shortcuts ─────────────────────────────────")
    print("  C     Calibration (κάθε φορά που αλλάζεις κομμάτι)")
    print("  A     Auto-follow car")
    print("  D     Toggle dotted edges")
    print("  F     Fit full map")
    print("  S     Save run to JSON")
    print("  R     Toggle recording")
    print("  L     Toggle log panel")
    print("  M     Toggle mini-map")
    print("  Scroll Zoom  │  Drag: Pan  │  Esc: Exit")
    print("  Δεν χρειάζεται [T] — target αυτόματα από controller")
    print("──────────────────────────────────────────────\n")

    viz.run()
    if dwm: dwm.stop()
    sem.stop(); telem.stop()


if __name__=="__main__":
    main()
