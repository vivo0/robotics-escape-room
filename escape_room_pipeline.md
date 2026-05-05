# Escape Room Pipeline — RoboMaster EP

## Goal

Il robot è chiuso in una stanza con ostacoli. Deve trovare una **chiave** (cubo colorato), portarla su una **pressure plate** colorata per aprire una **porta** colorata, e uscire. Tre landmark distinti per colore, ciascuno scoperto una volta sola e poi ricordato.

## Architettura a due fasi

**Fase 1 — Discovery.** Il robot esplora reattivamente, costruisce in memoria una mappa degli ostacoli, e annota la posizione dei tre landmark man mano che li vede.

**Fase 2 — Execution.** Quando tutti e tre i landmark sono noti, il robot smette di esplorare e si muove "ad occhi chiusi" usando solo la mappa: pianifica con A*, segue il path con pure pursuit, raccoglie il cubo, lo deposita, esce.

## Nodi ROS

```
┌─────────────────┐     ┌──────────────┐     ┌─────────────────┐
│ color_detector  │────►│   mission    │◄────│     mapper      │
│                 │     │ (state mach.)│     │                 │
│ camera → poses  │     │              │     │ ToF → /map      │
└─────────────────┘     └──────┬───────┘     └─────────────────┘
                               │                       ▲
                               ▼                       │
                        ┌──────────────┐               │
                        │   planner    │───────────────┘
                        │  A* + follow │
                        └──────────────┘
                               │
                               ▼ cmd_vel, gripper, leds
```

| Nodo | Responsabilità | Subscribe | Publish |
|---|---|---|---|
| `color_detector` | Vede i 3 colori, stima pose in `odom`, una sola volta | `camera/image_color`, `camera/camera_info`, TF | `targets/cube`, `targets/plate`, `targets/door` (latched) |
| `mapper` | Occupancy grid 2D via ray-casting dai ToF | `range_*`, `odom` | `/map` (`OccupancyGrid`) |
| `planner` | A* su `/map` con inflation, pure pursuit follower | `/map`, goal via servizio | `cmd_vel` durante l'execution |
| `mission` | State machine globale | tutti i topic sopra | comandi gripper/LEDs, attiva/disattiva planner |

## Fase 1 — Discovery in dettaglio

**Esplorazione**: random walk reattivo. Avanti finché ToF anteriori sono liberi, ostacolo → ruota di angolo casuale → riparti.

**Mapping**: ad ogni lettura ToF, ray-casting nella grid. Celle attraversate dal raggio = libere, cella alla distanza misurata = occupata, mai osservate = ignote. Risoluzione 10 cm, stanza 5×4 m → 50×40 celle.

**Color detection**: ogni frame della camera viene convertito in HSV, applicate maschere `cv2.inRange` per ciascun colore, trovati i blob, calcolato il centroide. Il pixel del centroide viene proiettato nel mondo intersecando il raggio con il piano `z = z_landmark` (noto per ciascun tipo). La posa viene trasformata da camera a `odom` via TF, e pubblicata su un topic latched `transient_local` — chi subscribe dopo riceve comunque l'ultimo valore.

**Criterio di terminazione**: tutti e tre i landmark visti almeno una volta. Possibile timeout di sicurezza (es. 90s) per non bloccarsi.

## Fase 2 — Execution in dettaglio

**Path planning**: A* sulla occupancy grid, dopo aver applicato **inflation** (dilatazione morfologica di ~3 celle = raggio del robot) per tenere il path lontano dai muri. Ritorna una lista di waypoint.

**Path following**: pure pursuit. Sceglie un punto sul path a ~30 cm davanti al robot, calcola angolo verso quel punto, comanda velocità lineare e angolare. Quando il punto finale è raggiunto entro tolleranza (es. 10 cm), passa allo stato successivo.

**Manipolazione**: arrivato vicino al cubo, switch a visual servoing (centra il cubo nell'immagine, avvicinati lentamente, chiudi gripper). Stesso pattern per il rilascio sulla plate.

## State machine

```
DISCOVERY ─────► tutti i landmark visti ──► PLAN_TO_CUBE
PLAN_TO_CUBE ──► path calcolato ─────────► FOLLOW_TO_CUBE
FOLLOW_TO_CUBE ► arrivato ────────────────► GRASP
GRASP ─────────► cubo in mano ────────────► PLAN_TO_PLATE
PLAN_TO_PLATE ─► path calcolato ─────────► FOLLOW_TO_PLATE
FOLLOW_TO_PLATE► arrivato ────────────────► RELEASE
RELEASE ───────► cubo rilasciato ─────────► PLAN_TO_DOOR
PLAN_TO_DOOR ──► path calcolato ─────────► FOLLOW_TO_DOOR
FOLLOW_TO_DOOR ► arrivato ────────────────► ESCAPED (LED + dance)
```

## Persistenza dei dati

Tutto vive in memoria nei rispettivi nodi:

- **Mappa**: `numpy.ndarray` dentro `mapper`, ripubblicata su `/map` ad ogni update.
- **Landmark**: `PoseStamped` con QoS `TRANSIENT_LOCAL` dentro `color_detector`. Il publisher si "ricorda" l'ultimo valore e lo invia automaticamente a chi subscribe dopo.
- **State**: variabile interna in `mission`.

A fine missione, `mission` può salvare `map.npy` + `targets.json` per il report.

## Convenzioni colore

| Landmark | Colore | Hue HSV (~) | Z nel mondo |
|---|---|---|---|
| Cubo (chiave) | Rosso saturo | 0–10 e 170–180 | 0.05 m |
| Pressure plate | Verde saturo | 40–80 | 0.001 m |
| Porta | Blu saturo | 100–130 | 0.50 m |

Nel JSON dello scenario, i `color` dei tre oggetti devono essere coerenti con queste fasce.

## Stack tecnico

- **CoppeliaSim 4.10** + plugin `simExtROS2` per la simulazione
- **ROS2** + driver `robomaster_ros` per il controllo del robot
- **Python**: `rclpy`, `opencv-python` (color + visual servoing), `numpy` (grid + A*), `tf2_ros` (frame transforms), `cv_bridge` (ROS Image ↔ OpenCV)
- **RViz2** per visualizzare mappa e path durante demo/video

## Roadmap incrementale

1. **MVP reattivo** (~1 settimana): no mapping, robot esplora a caso e usa visual servoing per andare sui landmark direttamente. Demo end-to-end funzionante.
2. **+ Mapper passivo** (~4 giorni): aggiungi `mapper` in parallelo, visualizzi `/map` in RViz nel video. Controllo ancora reattivo.
3. **+ Planner attivo** (~4 giorni): A* + pure pursuit. Discovery → Execution come descritto. "Ad occhi chiusi" realizzato.

Ogni tappa è consegnabile per conto suo. Se rimani indietro alla tappa 2, hai comunque un progetto solido.

## Scope esplicito di cosa NON è incluso

- SLAM (l'odometria di Coppelia in simulazione è perfetta, non serve).
- Mappatura dinamica con oggetti che si muovono.
- Distrattori dello stesso colore (un solo oggetto per colore).
- Gestione di più stanze o porte multiple.
- Recovery sofisticate da fallimenti del grasping.
