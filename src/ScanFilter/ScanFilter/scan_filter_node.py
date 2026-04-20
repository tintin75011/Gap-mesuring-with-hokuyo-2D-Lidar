import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class ScanFilterNode(Node):
    """
    Node ROS2 qui s'abonne à /scan, filtre les points selon une ROI
    (boîte rectangulaire en coordonnées cartésiennes XY) et republie
    le LaserScan filtré sur /filteredScan.
    Publie également un Marker visuel de la ROI sur /roi_marker.

    Paramètres dynamiques (modifiables avec ros2 param set) :
        x_min (float) : borne X minimale en mètres  (défaut : -00.0)
        x_max (float) : borne X maximale en mètres  (défaut :  30.0)
        y_min (float) : borne Y minimale en mètres  (défaut : -0.20)
        y_max (float) : borne Y maximale en mètres  (défaut :  0.10)
    """

    def __init__(self):
        super().__init__('scan_filter_node')

        # --- Déclaration des paramètres de la ROI ---
        self.declare_parameter('x_min', -0.0)
        self.declare_parameter('x_max',  0.50)
        self.declare_parameter('y_min', -0.050)
        self.declare_parameter('y_max',  0.05)

        # --- Subscriber / Publishers ---
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            LaserScan,
            '/filteredScan',
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/roi_marker',
            10
        )

        self.get_logger().info(
            'scan_filter_node démarré.\n'
            '  Abonné à   : /scan\n'
            '  Publie sur : /filteredScan\n'
            '  Marker ROI : /roi_marker\n'
            '  Modifier la ROI avec :\n'
            '    ros2 param set /scan_filter_node x_min <val>\n'
            '    ros2 param set /scan_filter_node x_max <val>\n'
            '    ros2 param set /scan_filter_node y_min <val>\n'
            '    ros2 param set /scan_filter_node y_max <val>'
        )

    def publish_roi_marker(self, x_min, x_max, y_min, y_max, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'roi'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Les 5 points qui forment le rectangle fermé
        corners = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min),  # fermeture
        ]
        for x, y in corners:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            marker.points.append(p)

        marker.scale.x = 0.002   # épaisseur du trait en mètres
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0    # opacité

        self.marker_pub.publish(marker)

    def scan_callback(self, msg: LaserScan):
        # Lecture des paramètres ROI (relus à chaque scan → dynamiques)
        x_min = self.get_parameter('x_min').value
        x_max = self.get_parameter('x_max').value
        y_min = self.get_parameter('y_min').value
        y_max = self.get_parameter('y_max').value

        # Publication du marker ROI avec le même frame que le scan
        self.publish_roi_marker(x_min, x_max, y_min, y_max, msg.header.frame_id)

        # Copie du message original pour conserver header + métadonnées
        filtered = LaserScan()
        filtered.header          = msg.header
        filtered.angle_min       = msg.angle_min
        filtered.angle_max       = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment  = msg.time_increment
        filtered.scan_time       = msg.scan_time
        filtered.range_min       = msg.range_min
        filtered.range_max       = msg.range_max

        filtered_ranges      = []
        filtered_intensities = []
        has_intensities = len(msg.intensities) == len(msg.ranges)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Point invalide (inf / nan / hors bornes capteur) → on l'exclut
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                filtered_ranges.append(float('inf'))
                if has_intensities:
                    filtered_intensities.append(0.0)
                continue

            # Conversion polaire → cartésien
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Test d'appartenance à la ROI
            if x_min <= x <= x_max and y_min <= y <= y_max:
                filtered_ranges.append(r)
                if has_intensities:
                    filtered_intensities.append(msg.intensities[i])
            else:
                # Point hors ROI : on met inf pour garder la structure angulaire
                filtered_ranges.append(float('inf'))
                if has_intensities:
                    filtered_intensities.append(0.0)

        filtered.ranges = filtered_ranges
        if has_intensities:
            filtered.intensities = filtered_intensities

        self.pub.publish(filtered)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()