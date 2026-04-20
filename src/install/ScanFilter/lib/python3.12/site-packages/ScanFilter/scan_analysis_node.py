import math
from collections import deque

import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# Palette de couleurs pour les groupes (cycle automatique)
COLORS = [
    (1.0, 0.2, 0.2),  # rouge
    (0.2, 0.6, 1.0),  # bleu
]


class ScanAnalysisNode(Node):
    """
    Node ROS2 qui s'abonne à /filteredScan, accumule les points sur une
    fenêtre glissante (decay_time secondes), détecte les groupes via DBSCAN,
    ne conserve que les 2 plus gros groupes, calcule la distance minimale
    entre eux et publie des MarkerArray pour visualisation RViz2.

    Topics publiés :
        /clusters_markers  : points colorés par groupe + label
        /distance_markers  : trait + texte distance minimale entre les 2 groupes

    Paramètres dynamiques :
        decay_time    (float) : fenêtre temporelle en secondes        (défaut : 5.0)
        eps           (float) : distance max entre voisins DBSCAN (m) (défaut : 0.006)
        min_samples   (int)   : points minimum par groupe DBSCAN      (défaut : 50)
        voxel_size    (float) : résolution de la grille (m)           (défaut : 0.02)
        marker_size   (float) : taille des points RViz (m)            (défaut : 0.001)
    """

    def __init__(self):
        super().__init__('scan_analysis_node')

        self.declare_parameter('decay_time',  10.0)
        self.declare_parameter('eps',         0.006)
        self.declare_parameter('min_samples', 50)
        self.declare_parameter('voxel_size',  0.02)
        self.declare_parameter('marker_size', 0.001)

        # Buffer glissant : deque de (timestamp_sec, [(x, y), ...])
        self.point_buffer = deque()
        self.last_frame_id = 'laser'

        self.sub = self.create_subscription(
            LaserScan,
            '/filteredScan',
            self.scan_callback,
            10
        )

        self.clusters_pub = self.create_publisher(MarkerArray, '/clusters_markers', 10)
        self.distance_pub = self.create_publisher(MarkerArray, '/distance_markers', 10)

        self.get_logger().info(
            'scan_analysis_node démarré (mode decay buffer, 2 groupes max).\n'
            '  Abonné à   : /filteredScan\n'
            '  Publie sur : /clusters_markers  (2 groupes colorés)\n'
            '               /distance_markers  (distance minimale)\n'
            '  Paramètres :\n'
            '    ros2 param set /scan_analysis_node decay_time  <val>\n'
            '    ros2 param set /scan_analysis_node eps         <val>\n'
            '    ros2 param set /scan_analysis_node min_samples <val>\n'
            '    ros2 param set /scan_analysis_node marker_size <val>'
        )

    # ------------------------------------------------------------------
    # Mise à jour du buffer glissant
    # ------------------------------------------------------------------
    def update_buffer(self, msg: LaserScan, now: float, decay_time: float):
        """Ajoute les points du scan courant et purge les points trop vieux."""
        new_points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            new_points.append((x, y))

        if new_points:
            self.point_buffer.append((now, new_points))

        # Purge des entrées expirées
        while self.point_buffer and (now - self.point_buffer[0][0]) > decay_time:
            self.point_buffer.popleft()

    # ------------------------------------------------------------------
    # Clustering DBSCAN — retourne uniquement les 2 plus gros groupes
    # ------------------------------------------------------------------
    def extract_clusters(self, all_points, eps: float, min_samples: int):
        """
        Retourne les 2 clusters les plus grands (par nombre de points).
        """
        if len(all_points) < min_samples:
            return []

        xy = np.array(all_points)
        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(xy)

        # Construire tous les clusters valides (label != -1)
        clusters = []
        for label in sorted(set(labels)):
            if label == -1:
                continue
            cluster = [tuple(xy[i]) for i in range(len(xy)) if labels[i] == label]
            clusters.append(cluster)

        # Trier par taille décroissante et ne garder que les 2 plus gros
        clusters.sort(key=lambda c: len(c), reverse=True)
        return clusters[:2]

    # ------------------------------------------------------------------
    # Distance minimale entre deux groupes
    # ------------------------------------------------------------------
    def min_distance_between(self, cluster_a, cluster_b, k=5):
        """
        Retourne la moyenne des k plus petites distances entre deux clusters,
        ainsi que les points correspondants.
        """
        arr_a = np.array(cluster_a)
        arr_b = np.array(cluster_b)

        # Calculer toutes les distances entre chaque paire de points
        distances = []
        for i, pa in enumerate(arr_a):
            dists = np.sqrt(np.sum((arr_b - pa) ** 2, axis=1))
            for j, d in enumerate(dists):
                distances.append((d, tuple(pa), tuple(arr_b[j])))

        # Trier les distances par ordre croissant
        distances.sort(key=lambda x: x[0])

        # Prendre les k plus petites distances
        top_k = distances[:k]

        # Calculer la moyenne des k distances
        avg_distance = sum(d for d, _, _ in top_k) / k

        # Retourner la moyenne et les points associés à la première distance (pour la visualisation)
        return avg_distance, top_k[0][1], top_k[0][2]

    # ------------------------------------------------------------------
    # Markers : points colorés par groupe + label
    # ------------------------------------------------------------------
    def build_cluster_markers(self, clusters, frame_id, marker_size):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, cluster in enumerate(clusters):
            r, g, b = COLORS[idx % len(COLORS)]

            # Points du groupe
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'clusters'
            marker.id = idx
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = marker_size
            marker.scale.y = marker_size
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            for x, y in cluster:
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = 0.0
                marker.points.append(p)

            marker_array.markers.append(marker)

            # Label centroïde
            cx = sum(x for x, _ in cluster) / len(cluster)
            cy = sum(y for _, y in cluster) / len(cluster)

            label = Marker()
            label.header.frame_id = frame_id
            label.header.stamp = stamp
            label.ns = 'cluster_labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = float(cx)
            label.pose.position.y = float(cy)
            label.pose.position.z = 0.15
            label.scale.z = 0.012
            label.color.r = r
            label.color.g = g
            label.color.b = b
            label.color.a = 1.0
            label.text = f'G{idx} ({len(cluster)} pts)'

            marker_array.markers.append(label)

        return marker_array

    # ------------------------------------------------------------------
    # Markers : trait + texte de distance minimale entre les 2 groupes
    # ------------------------------------------------------------------
    def build_distance_markers(self, clusters, frame_id):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        if len(clusters) < 2:
            return marker_array

        d_min, pa, pb = self.min_distance_between(clusters[0], clusters[1])

        if pa is None or pb is None:
            return marker_array

        ri, gi, bi = COLORS[0]

        # Trait entre les deux points les plus proches
        line = Marker()
        line.header.frame_id = frame_id
        line.header.stamp = stamp
        line.ns = 'distances_lines'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.0015
        line.color.r = 1.0
        line.color.g = 1.0
        line.color.b = 1.0
        line.color.a = 0.9

        for px, py in [pa, pb]:
            p = Point()
            p.x = float(px)
            p.y = float(py)
            p.z = 0.0
            line.points.append(p)

        marker_array.markers.append(line)

        # Texte au milieu du trait
        mx = (pa[0] + pb[0]) / 2.0
        my = (pa[1] + pb[1]) / 2.0

        text = Marker()
        text.header.frame_id = frame_id
        text.header.stamp = stamp
        text.ns = 'distances_labels'
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(mx)
        text.pose.position.y = float(my)
        text.pose.position.z = 0.1
        text.scale.z = 0.010
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f'{d_min:.4f}-m-(G0-G1)'

        marker_array.markers.append(text)

        return marker_array

    # ------------------------------------------------------------------
    # Callback principal
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        decay_time  = self.get_parameter('decay_time').value
        eps         = self.get_parameter('eps').value
        min_samples = self.get_parameter('min_samples').value
        marker_size = self.get_parameter('marker_size').value

        self.last_frame_id = msg.header.frame_id
        now = self.get_clock().now().nanoseconds / 1e9

        # Mise à jour du buffer glissant
        self.update_buffer(msg, now, decay_time)

        # Fusion de tous les points du buffer
        all_points = [pt for (_, pts) in self.point_buffer for pt in pts]

        if not all_points:
            return

        # Clustering DBSCAN — 2 plus gros groupes uniquement
        clusters = self.extract_clusters(all_points, eps, min_samples)

        if not clusters:
            self.get_logger().warn(
                'Aucun groupe détecté — ajuste eps ou min_samples',
                throttle_duration_sec=2.0
            )
            return

        self.get_logger().info(
            f'{len(clusters)} groupe(s) retenu(s) | {len(all_points)} points dans le buffer',
            throttle_duration_sec=1.0
        )

        frame_id = msg.header.frame_id
        self.clusters_pub.publish(self.build_cluster_markers(clusters, frame_id, marker_size))
        self.distance_pub.publish(self.build_distance_markers(clusters, frame_id))


def main(args=None):
    rclpy.init(args=args)
    node = ScanAnalysisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()