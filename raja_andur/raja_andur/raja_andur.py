import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math


class RajaAndur(Node):
    """
    ROS2 sõlm, mis jagab lidari 360° vaate 5 sektoriks ja trükib kauguste tabeli.

    Sektori definitsioonid (indeksite järgi, 720 kiirt, 0.5° samm):
      angle_min = -π (-180°), angle_max = +π (+180°)
      indeks = (nurk_kraadides + 180) / 0.5

      Vasak sein:  indeksid 480–600  ( 60°– 120°, vasak külg)
      Ette-vasak:  indeksid 380–480  (  0°–  60°, ette-vasak)
      Otse ette:   indeksid 330–390  (-15°– +15°, otse ette)  — kitsam tsoon
      Ette-parem:  indeksid 240–330  (-60°–  -0°, ette-parem) — wait, let's use
                                                                  symmetric layout

    Sümmeetriline 5-sektori jaotus:
      Vasak sein:  indeksid 510–630  ( 75°– 135°)
      Ette-vasak:  indeksid 390–510  ( 15°–  75°)
      Otse ette:   indeksid 330–390  (-15°– +15°)  — ±15°, 60 indeksit
      Ette-parem:  indeksid 210–330  (-75°– -15°)
      Parem sein:  indeksid  90–210  (-135°– -75°)
    """

    # Ohukünnised meetrites
    LAHEDAL_PIIR = 0.5    # < 0.5 m → [LÄHEDAL]
    HOIATUS_PIIR = 1.0    # < 1.0 m → [HOIATUS]
    # >= 1.0 m → [OK]

    def __init__(self):
        super().__init__('raja_andur')

        # /scan subscription (sensor_data QoS ühilduvuse jaoks)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        # Tabeli trükkimise timer — üks kord sekundis
        self.timer = self.create_timer(1.0, self.print_tabel)

        # Viimane sektori tulemus
        self.sektori_kaugused = {
            'Vasak sein':  float('inf'),
            'Ette-vasak':  float('inf'),
            'Otse ette':   float('inf'),
            'Ette-parem':  float('inf'),
            'Parem sein':  float('inf'),
        }

        self.viimane_scan = None

        self.get_logger().info('Raja andur käivitatud. Ootan /scan andmeid...')

    def scan_callback(self, msg: LaserScan):
        """Töötleb iga uue LaserScan sõnumi ja uuendab sektori kauguseid."""
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
            if (range_min <= r <= range_max
                    and not math.isinf(r)
                    and not math.isnan(r)):
                sektor.append(r)
        return min(sektor) if sektor else float('inf')

    def uuenda_sektorid(self, msg: LaserScan):
        """
        Jagab 720-kiirelist lidar skänni 5 sektoriks ja arvutab iga
        sektori minimaalse kauguse.

        Indeksite arvutus: indeks = (nurk° + 180) / 0.5
          +75°  → 510,  +135° → 630   (vasak sein)
           +15° → 390,  +75°  → 510   (ette-vasak)
          -15°  → 330,  +15°  → 390   (otse ette)
          -75°  → 210,  -15°  → 330   (ette-parem)
         -135°  →  90,  -75°  → 210   (parem sein)
        """
        ranges = msg.ranges
        rmin = msg.range_min
        rmax = msg.range_max

        self.sektori_kaugused['Vasak sein']  = self.sektori_min(ranges,  510, 630, rmin, rmax)
        self.sektori_kaugused['Ette-vasak']  = self.sektori_min(ranges,  390, 510, rmin, rmax)
        self.sektori_kaugused['Otse ette']   = self.sektori_min(ranges,  330, 390, rmin, rmax)
        self.sektori_kaugused['Ette-parem']  = self.sektori_min(ranges,  210, 330, rmin, rmax)
        self.sektori_kaugused['Parem sein']  = self.sektori_min(ranges,   90, 210, rmin, rmax)

    def margis(self, kaugus: float) -> str:
        """Tagastab kauguse põhjal õige märgise."""
        if kaugus < self.LAHEDAL_PIIR:
            return '[LÄHEDAL]'
        elif kaugus < self.HOIATUS_PIIR:
            return '[HOIATUS]'
        else:
            return '[OK]'

    def print_tabel(self):
        """Trükib sektori kauguste tabeli üks kord sekundis."""
        if self.viimane_scan is None:
            self.get_logger().info('Ootan /scan andmeid...')
            return

        rida_formaat = '{nimi:<14} {kaugus:>6.2f} m  {margis}'

        read = [
            rida_formaat.format(
                nimi=nimi + ':',
                kaugus=kaugus if not math.isinf(kaugus) else 9.99,
                margis=self.margis(kaugus)
            )
            for nimi, kaugus in self.sektori_kaugused.items()
        ]

        tabel = (
            '\n=== Raja andurid ===\n'
            + '\n'.join(read)
            + '\n==================='
        )

        self.get_logger().info(tabel)


def main(args=None):
    rclpy.init(args=args)
    node = RajaAndur()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()