# Lidar Gap Analysis

Pipeline ROS2 pour l'analyse d'un scan laser 2D (Hokuyo URG). Permet de filtrer une zone d'intérêt, de détecter des groupes de points par clustering et de mesurer la distance minimale entre ces groupes en temps réel dans RViz2.

---

## Architecture

```
/scan  (urg_node2)
  └──▶  scan_filter_node  ──▶  /filteredScan
                │                    └──▶  scan_analysis_node  ──▶  /clusters_markers
                │                                                     /distance_markers
                └──▶  /roi_marker
```

### Packages

| Package | Rôle |
|---|---|
| `urg_node2` | Driver du lidar Hokuyo URG — publie `/scan` |
| `ScanFilter` | Filtrage par ROI cartésienne — publie `/filteredScan` + marker de la zone |
| `scanGapAnalysis` | Clustering DBSCAN sur buffer temporel — publie groupes colorés + distance minimale |

---

## Prérequis

- ROS2 Jazzy
- Python 3.12
- scikit-learn

```bash
pip install scikit-learn --break-system-packages
```

---

## Installation

```bash
mkdir -p ~/lidar_gap_ws/src
cd ~/lidar_gap_ws/src
git clone <url_du_repo> .
cd ~/lidar_gap_ws
colcon build
source install/setup.bash
```

---

## Lancement

### Tout en une commande

```bash
ros2 launch ScanFilter lidar_pipeline.launch.py
```

### Avec des paramètres personnalisés

```bash
ros2 launch ScanFilter lidar_pipeline.launch.py \
  x_min:=0.0 x_max:=0.30 \
  y_min:=-0.10 y_max:=0.10 \
  decay_time:=5.0 eps:=0.006 min_samples:=50
```

---

## Paramètres

### scan_filter_node

| Paramètre | Défaut | Description |
|---|---|---|
| `x_min` | `0.0` | Borne X minimale de la ROI (m) |
| `x_max` | `0.30` | Borne X maximale de la ROI (m) |
| `y_min` | `-0.10` | Borne Y minimale de la ROI (m) |
| `y_max` | `0.10` | Borne Y maximale de la ROI (m) |

### scan_analysis_node

| Paramètre | Défaut | Description |
|---|---|---|
| `decay_time` | `5.0` | Fenêtre temporelle du buffer (s) |
| `eps` | `0.006` | Rayon de voisinage DBSCAN (m) |
| `min_samples` | `50` | Nombre minimum de points par groupe |
| `marker_size` | `0.001` | Taille des points dans RViz2 (m) |

### Modification à chaud

Les paramètres sont dynamiques, modifiables sans redémarrer :

```bash
ros2 param set /scan_filter_node x_max 0.50
ros2 param set /scan_analysis_node eps 0.01
```

---

## Visualisation RViz2

| Topic | Type | Description |
|---|---|---|
| `/scan` | LaserScan | Scan brut du lidar |
| `/filteredScan` | LaserScan | Scan filtré par la ROI |
| `/roi_marker` | Marker | Rectangle vert délimitant la ROI |
| `/clusters_markers` | MarkerArray | Points colorés par groupe (rouge = G0, bleu = G1) |
| `/distance_markers` | MarkerArray | Trait blanc + distance minimale entre G0 et G1 |

---

## Structure du repo

```
lidar_gap_ws/src/
├── ScanFilter/
│   ├── ScanFilter/
│   │   ├── __init__.py
│   │   └── scan_filter_node.py
│   ├── launch/
│   │   └── lidar_pipeline.launch.py
│   ├── rviz/
│   │   └── lidar_pipeline.rviz
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
└── scanGapAnalysis/
    ├── scanGapAnalysis/
    │   ├── __init__.py
    │   └── scan_analysis_node.py
    ├── package.xml
    ├── setup.py
    └── setup.cfg
```
