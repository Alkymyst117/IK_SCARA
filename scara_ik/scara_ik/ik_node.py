import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class ScaraIKNode(Node):
    """
    Suscribe a /scara_target (x,y) en mm.
    Calcula IK, valida alcanzabilidad y límites articulares (en grados),
    y publica en /scara_angles [theta1_deg, theta2_deg].
    """
    def __init__(self):
        super().__init__('scara_ik_node')

        # Topicos
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'scara_target',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'scara_angles',
            10
        )

        # --- Parámetros del robot
        # Eslabones en mm
        self.L1 = 200.0
        self.L2 = 200.0

        # Limites articulares en grados
        self.min_theta1 = -90.0
        self.max_theta1 = 90.0
        self.min_theta2 = 0.0
        self.max_theta2 = 150.0

        self.get_logger().info('IK node listo. Esperando puntos en /scara_target (mm).')

    def listener_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            self.get_logger().warn("Mensaje inválido: se esperaban 2 valores (x, y).")
            return

        x_mm, y_mm = float(msg.data[0]), float(msg.data[1])
        self.get_logger().info(f'📥 Punto recibido: x={x_mm} mm, y={y_mm} mm')

        # Convertir a metros para trigonometría 
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        L1 = self.L1 / 1000.0
        L2 = self.L2 / 1000.0

        # Alcance físico (anillo)
        r = math.hypot(x, y)
        if r > (L1 + L2) or r < abs(L1 - L2):
            self.get_logger().warn('⚠️ Punto fuera del alcance físico (r = {:.3f} m)'.format(r))
            return

        # Calcular cos(theta2) — seguridad numérica
        cos_theta2 = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)

        theta2_options = [math.acos(cos_theta2), -math.acos(cos_theta2)]
        valid_solutions = []

        for theta2 in theta2_options:
            theta1 = math.atan2(y, x) - math.atan2(L2 * math.sin(theta2), L1 + L2 * math.cos(theta2))

            theta1_deg = math.degrees(theta1)
            theta2_deg = math.degrees(theta2)

            if self.is_within_limits(theta1_deg, theta2_deg):
                valid_solutions.append((theta1_deg, theta2_deg))

        if not valid_solutions:
            self.get_logger().warn('⚠️ Ninguna solución dentro de los límites articulares.')
            return

        # Selección de solución:
        # aquí escogemos la que tenga menor suma absoluta de ángulos
        chosen = min(valid_solutions, key=lambda s: abs(s[0]) + abs(s[1]))

        theta1_deg, theta2_deg = chosen
        out = Float32MultiArray()
        out.data = [theta1_deg, theta2_deg]
        self.publisher.publish(out)

        self.get_logger().info(f'✅ Solución: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°')

    def is_within_limits(self, theta1_deg, theta2_deg):
        return (self.min_theta1 <= theta1_deg <= self.max_theta1) and \
               (self.min_theta2 <= theta2_deg <= self.max_theta2)

def main(args=None):
    rclpy.init(args=args)
    node = ScaraIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
