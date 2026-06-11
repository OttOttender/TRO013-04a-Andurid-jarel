import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


# ── Olekumasina olekud ──────────────────────────────────────────────────────
OLEK_EDASI_1  = 0   # Sõida 1.0 m otse ette
OLEK_POORDE_1 = 1   # Pöördu paremale 90°
OLEK_EDASI_2  = 2   # Sõida 1.0 m otse ette
OLEK_POORDE_2 = 3   # Pöördu paremale 90°
OLEK_VALMIS   = 4   # Teekond lõpetatud


class Marsruut(Node):
    """
    Sõidab L-kujulise teekonna odomeetria põhjal:
      1. Edasi 1.0 m
      2. Pööre paremale 90°
      3. Edasi 1.0 m
      4. Pööre paremale 90°
      5. VALMIS — robot peatub

    Kasutab olekumasinat ja /odom andmeid — mitte time.sleep().
    """

    # Teekonna parameetrid
    SIHTVAHEMAA   = 1.0   # meetrites
    SIHTPÖÖRDE    = math.pi / 2.0   # 90° radiaanides

    # Kiirused
    KIIRUS_EDASI  = 0.25  # m/s
    KIIRUS_POORDE = 0.4   # rad/s (paremale = negatiivne angular.z)

    # Lõpetamise lävedepiirid (odomeetria ei ole täpne)
    VAHEMAA_LAVIPIIR = 0.05   # 5 cm
    NURGA_LAVIPIIR   = 0.05   # ~3°

    def __init__(self):
        super().__init__('marsruut')

        # /odom subscription
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # /cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Olek ────────────────────────────────────────────────────────────
        self.olek = OLEK_EDASI_1

        # Praegune positsioon (täidetakse esimese odom sõnumiga)
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0   # radiaanides

        # Etapi alguspunkt (salvestatakse iga uue etapi alguses)
        self.etapi_algus_x   = None
        self.etapi_algus_y   = None
        self.etapi_algus_yaw = None

        # Juhtimine 10 Hz juhtimissilmaga
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Marsruut käivitatud. Ootan /odom andmeid...')

    # ── Abifunktsioonid ─────────────────────────────────────────────────────

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        """Teisendab quaternion'i yaw-nurgaks (radiaanides)."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def nurga_vahe(siht: float, praegune: float) -> float:
        """
        Arvutab nurga erinevuse [-π, π] vahemikus.
        Positiivne = vastupäeva (vasakule), negatiivne = päripäeva (paremale).
        """
        vahe = siht - praegune
        while vahe > math.pi:
            vahe -= 2.0 * math.pi
        while vahe < -math.pi:
            vahe += 2.0 * math.pi
        return vahe

    # ── Callback'id ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        """Uuendab roboti positsiooni ja orientatsiooni."""
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)

        # Salvesta esimene positsioon etapi alguspunktina
        if self.etapi_algus_x is None:
            self.etapi_algus_x   = self.x
            self.etapi_algus_y   = self.y
            self.etapi_algus_yaw = self.yaw
            self.get_logger().info(
                f'Alguspunkt salvestatud: ({self.x:.3f}, {self.y:.3f}), '
                f'yaw={math.degrees(self.yaw):.1f}°'
            )

    def alusta_uut_etappi(self):
        """Salvestab praeguse positsiooni uue etapi alguspunktina."""
        self.etapi_algus_x   = self.x
        self.etapi_algus_y   = self.y
        self.etapi_algus_yaw = self.yaw

    def labitud_vahemaa(self) -> float:
        """Tagastab läbitud vahemaa etapi alguspunktist."""
        if self.etapi_algus_x is None:
            return 0.0
        dx = self.x - self.etapi_algus_x
        dy = self.y - self.etapi_algus_y
        return math.sqrt(dx * dx + dy * dy)

    def labitud_nurk(self) -> float:
        """
        Tagastab pöördenurga etapi algusest (absoluutväärtus, radiaanides).
        """
        if self.etapi_algus_yaw is None:
            return 0.0
        return abs(self.nurga_vahe(self.yaw, self.etapi_algus_yaw))

    # ── Peamine juhtimissilmus ───────────────────────────────────────────────

    def control_loop(self):
        """
        Täidab olekumasina loogikat 10 Hz sagedusega.
        Iga olek saadab /cmd_vel käske kuni etapp on lõpetatud,
        siis lülitub järgmisele olekule.
        """
        if self.etapi_algus_x is None:
            # /odom andmed pole veel saabunud
            return

        cmd = Twist()

        # ── Olek 0: Edasi 1.0 m ─────────────────────────────────────────
        if self.olek == OLEK_EDASI_1:
            vahemaa = self.labitud_vahemaa()
            if vahemaa < self.SIHTVAHEMAA - self.VAHEMAA_LAVIPIIR:
                cmd.linear.x = self.KIIRUS_EDASI
                self.get_logger().info(
                    f'[EDASI_1] Läbitud: {vahemaa:.3f} / {self.SIHTVAHEMAA} m',
                    throttle_duration_sec=0.5
                )
            else:
                # Etapp lõpetatud — peatu ja lülitu pöördele
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f'[EDASI_1] Valmis! Läbitud: {vahemaa:.3f} m → pöördu paremale'
                )
                self.olek = OLEK_POORDE_1
                self.alusta_uut_etappi()

        # ── Olek 1: Pöördu paremale 90° ─────────────────────────────────
        elif self.olek == OLEK_POORDE_1:
            nurk = self.labitud_nurk()
            if nurk < self.SIHTPÖÖRDE - self.NURGA_LAVIPIIR:
                cmd.angular.z = -self.KIIRUS_POORDE  # negatiivne = paremale
                self.get_logger().info(
                    f'[POORDE_1] Pööratud: {math.degrees(nurk):.1f}° / 90°',
                    throttle_duration_sec=0.5
                )
            else:
                cmd.angular.z = 0.0
                self.get_logger().info(
                    f'[POORDE_1] Valmis! {math.degrees(nurk):.1f}° → sõida edasi'
                )
                self.olek = OLEK_EDASI_2
                self.alusta_uut_etappi()

        # ── Olek 2: Edasi 1.0 m ─────────────────────────────────────────
        elif self.olek == OLEK_EDASI_2:
            vahemaa = self.labitud_vahemaa()
            if vahemaa < self.SIHTVAHEMAA - self.VAHEMAA_LAVIPIIR:
                cmd.linear.x = self.KIIRUS_EDASI
                self.get_logger().info(
                    f'[EDASI_2] Läbitud: {vahemaa:.3f} / {self.SIHTVAHEMAA} m',
                    throttle_duration_sec=0.5
                )
            else:
                cmd.linear.x = 0.0
                self.get_logger().info(
                    f'[EDASI_2] Valmis! Läbitud: {vahemaa:.3f} m → pöördu paremale'
                )
                self.olek = OLEK_POORDE_2
                self.alusta_uut_etappi()

        # ── Olek 3: Pöördu paremale 90° ─────────────────────────────────
        elif self.olek == OLEK_POORDE_2:
            nurk = self.labitud_nurk()
            if nurk < self.SIHTPÖÖRDE - self.NURGA_LAVIPIIR:
                cmd.angular.z = -self.KIIRUS_POORDE  # negatiivne = paremale
                self.get_logger().info(
                    f'[POORDE_2] Pööratud: {math.degrees(nurk):.1f}° / 90°',
                    throttle_duration_sec=0.5
                )
            else:
                cmd.angular.z = 0.0
                self.get_logger().info(
                    '[POORDE_2] Valmis! → TEEKOND LÕPETATUD'
                )
                self.olek = OLEK_VALMIS
                self.timer.cancel()
                
        # ── Olek 4: VALMIS — seisa paigal ───────────────────────────────
        elif self.olek == OLEK_VALMIS:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                'OLEK_VALMIS: Robot on teekonna lõpetanud. Seisan paigal.'
            )

            self.timer.cancel()
            return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Marsruut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()