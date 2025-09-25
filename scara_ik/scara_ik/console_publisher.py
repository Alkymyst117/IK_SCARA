import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ConsolePublisher(Node):
    """
    Publica en /scara_target puntos (x y) en milímetros leídos desde consola.
    Formato de entrada (ejemplos):
      200 100
      250,50
      (150, 150)
      exit    -> para salir
    """
    def __init__(self):
        super().__init__('scara_console_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'scara_target', 10)
        self.get_logger().info('Publisher listo. Ingresa puntos "x y" en mm. Escribe "exit" para salir.')

    def publish_point(self, x_mm: float, y_mm: float):
        msg = Float32MultiArray()
        msg.data = [float(x_mm), float(y_mm)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'📤 Publicado: x={x_mm} mm, y={y_mm} mm')

def parse_input_line(line: str):
    # Normalizar: quitar paréntesis, cambiar comas por espacios, etc.
    s = line.strip().replace('(', ' ').replace(')', ' ').replace(',', ' ')
    parts = s.split()
    if len(parts) < 2:
        raise ValueError("Se requieren al menos 2 valores (x y).")
    x = float(parts[0])
    y = float(parts[1])
    return x, y

def main(args=None):
    rclpy.init(args=args)
    node = ConsolePublisher()

    try:
        while rclpy.ok():
            try:
                line = input('Ingresa x y (mm) o "exit": ').strip()
            except (EOFError, KeyboardInterrupt):
                print()  # nueva línea bonita
                break

            if not line:
                continue
            if line.lower() in ('exit', 'quit', 'q'):
                node.get_logger().info('Saliendo del publicador por comando de usuario.')
                break

            try:
                x_mm, y_mm = parse_input_line(line)
            except ValueError as e:
                node.get_logger().warn(f'Entrada inválida: {e}')
                continue

            # publica (no necesitamos spin para publicar)
            node.publish_point(x_mm, y_mm)

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
