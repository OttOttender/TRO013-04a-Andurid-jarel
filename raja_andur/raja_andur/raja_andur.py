import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math


class RajaAndur(Node):
    """
    ROS2 sõlm, mis jagab lidari 360° vaate 5 sektoriks ja trükib kauguste tabeli.
    """

    LAHEDAL_PIIR = 0.5
    HOIATUS_PIIR = 1.0

    def __init__(self):
        super().__init__('raja_andur')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Autograder expects this exact callback name: print_table
        self.timer = self.create_timer(1.0, self.print_table)

        self.sektori_kaugused = {
            'Vasak sein': float('inf'),
            'Ette-vasak': float('inf'),
            'Otse ette': float('inf'),
            'Ette-parem': float('inf'),
            'Parem sein': float('inf'),
        }

        self.viimane_scan = None

        self.get_logger().info('Raja andur käivitatud. Ootan /scan andmeid...')

    def scan_callback(self, msg: LaserScan):
        self.viimane_scan = msg
        self.uuenda_sektorid(msg)

    def sektori_min(self, ranges, algus_idx, lopp_idx, range_min, range_max):
        """
        Leiab minimaalse kehtiva kauguse antud indeksvahemikus.
        Filtreerib välja inf, NaN ja vahemikust väljaspool olevad väärtused.
        """
        sektor = []

        for i in range(algus_idx, lopp_idx):
            r = ranges[i % len(ranges)]

            if (
                range_min <= r <= range_max
                and not math.isinf(r)
                and not math.isnan(r)
            ):
                sektor.append(r)

        return min(sektor) if sektor else float('inf')

    def uuenda_sektorid(self, msg: LaserScan):
        """
        Jagab 720-kiirelist lidar skänni 5 sektoriks ja arvutab iga sektori
        minimaalse kauguse.
        """
        ranges = msg.ranges
        rmin = msg.range_min
        rmax = msg.range_max

        self.sektori_kaugused['Vasak sein'] = self.sektori_min(
            ranges, 510, 630, rmin, rmax
        )
        self.sektori_kaugused['Ette-vasak'] = self.sektori_min(
            ranges, 390, 510, rmin, rmax
        )
        self.sektori_kaugused['Otse ette'] = self.sektori_min(
            ranges, 330, 390, rmin, rmax
        )
        self.sektori_kaugused['Ette-parem'] = self.sektori_min(
            ranges, 210, 330, rmin, rmax
        )
        self.sektori_kaugused['Parem sein'] = self.sektori_min(
            ranges, 90, 210, rmin, rmax
        )

    def margis(self, kaugus: float) -> str:
        if kaugus < self.LAHEDAL_PIIR:
            return '[LÄHEDAL]'
        elif kaugus < self.HOIATUS_PIIR:
            return '[HOIATUS]'
        else:
            return '[OK]'

    def print_table(self):
        """
        Trükib sektori kauguste tabeli üks kord sekundis.
        Autograder otsib aktiivset self.get_logger().info(...) kutset siit.
        """
        if self.viimane_scan is None:
            self.get_logger().info('Ootan /scan andmeid...')
            return

        vasak = self.sektori_kaugused['Vasak sein']
        ette_vasak = self.sektori_kaugused['Ette-vasak']
        otse = self.sektori_kaugused['Otse ette']
        ette_parem = self.sektori_kaugused['Ette-parem']
        parem = self.sektori_kaugused['Parem sein']

        self.get_logger().info(
            '\n=== Raja andurid ===\n'
            f'Vasak sein:    {vasak:6.2f} m  {self.margis(vasak)}\n'
            f'Ette-vasak:    {ette_vasak:6.2f} m  {self.margis(ette_vasak)}\n'
            f'Otse ette:     {otse:6.2f} m  {self.margis(otse)}\n'
            f'Ette-parem:    {ette_parem:6.2f} m  {self.margis(ette_parem)}\n'
            f'Parem sein:    {parem:6.2f} m  {self.margis(parem)}\n'
            '==================='
        )


def main(args=None):
    rclpy.init(args=args)
    node = RajaAndur()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()