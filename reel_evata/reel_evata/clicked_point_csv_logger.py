#!/usr/bin/env python3

import csv
from pathlib import Path
from typing import TextIO

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class ClickedPointCsvLogger(Node):
    """
    RViz Publish Point aracından gönderilen PointStamped mesajlarını
    geliş sırasına göre CSV dosyasına kaydeder.
    """

    CSV_HEADER = [
        "point_id",
        "x",
        "y",
        "z",
        "frame_id",
        "stamp_sec",
        "stamp_nanosec",
    ]

    def __init__(self) -> None:
        super().__init__("clicked_point_csv_logger")

        # ROS parametreleri
        self.declare_parameter("topic_name", "/clicked_point")
        self.declare_parameter("output_file", "~/clicked_points.csv")
        self.declare_parameter("append", False)

        self.topic_name = str(
            self.get_parameter("topic_name").value
        )

        output_file_parameter = str(
            self.get_parameter("output_file").value
        )

        self.append_mode = bool(
            self.get_parameter("append").value
        )

        # ~ ifadesini /home/kullanici biçimine dönüştürür.
        self.output_path = Path(
            output_file_parameter
        ).expanduser().resolve()

        # CSV'nin yazılacağı klasör yoksa oluşturulur.
        self.output_path.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        # Append modundaysa mevcut CSV'deki son point_id bulunur.
        self.point_id = self._find_next_point_id()

        file_mode = "a" if self.append_mode else "w"

        # Append modunda dosya boşsa başlık yazılması gerekir.
        write_header = (
            not self.append_mode
            or not self.output_path.exists()
            or self.output_path.stat().st_size == 0
        )

        self.csv_file: TextIO = open(
            self.output_path,
            mode=file_mode,
            newline="",
            encoding="utf-8",
        )

        self.csv_writer = csv.writer(self.csv_file)

        if write_header:
            self.csv_writer.writerow(self.CSV_HEADER)
            self.csv_file.flush()

        # PointStamped aboneliği
        self.subscription = self.create_subscription(
            PointStamped,
            self.topic_name,
            self.clicked_point_callback,
            10,
        )

        self.get_logger().info(
            f"Topic dinleniyor: {self.topic_name}"
        )
        self.get_logger().info(
            f"CSV dosyası: {self.output_path}"
        )
        self.get_logger().info(
            f"Dosya modu: "
            f"{'mevcut dosyaya ekle' if self.append_mode else 'yeniden oluştur'}"
        )
        self.get_logger().info(
            "RViz üzerinden Publish Point ile noktaları seçebilirsiniz."
        )

    def _find_next_point_id(self) -> int:
        """
        Append modu açıksa mevcut CSV dosyasındaki son point_id değerini
        okuyup sonraki numarayı döndürür.

        Append kapalıysa numaralandırma 1'den başlar.
        """

        if not self.append_mode:
            return 1

        if not self.output_path.exists():
            return 1

        if self.output_path.stat().st_size == 0:
            return 1

        last_point_id = 0

        try:
            with open(
                self.output_path,
                mode="r",
                newline="",
                encoding="utf-8",
            ) as existing_file:

                reader = csv.DictReader(existing_file)

                for row in reader:
                    try:
                        current_id = int(row["point_id"])
                        last_point_id = max(
                            last_point_id,
                            current_id
                        )
                    except (
                        KeyError,
                        TypeError,
                        ValueError,
                    ):
                        continue

        except OSError as error:
            self.get_logger().warning(
                f"Mevcut CSV okunamadı: {error}. "
                "Numaralandırma 1'den başlayacak."
            )
            return 1

        return last_point_id + 1

    def clicked_point_callback(
        self,
        msg: PointStamped
    ) -> None:
        """
        Her /clicked_point mesajı geldiğinde çağrılır.
        Nokta bilgisi CSV dosyasına tek satır olarak yazılır.
        """

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        frame_id = msg.header.frame_id

        stamp_sec = msg.header.stamp.sec
        stamp_nanosec = msg.header.stamp.nanosec

        self.csv_writer.writerow(
            [
                self.point_id,
                f"{x:.6f}",
                f"{y:.6f}",
                f"{z:.6f}",
                frame_id,
                stamp_sec,
                stamp_nanosec,
            ]
        )

        # Her tıklamadan sonra veriyi disk tamponuna aktarır.
        # Program Ctrl+C ile kapatılmadan önce de veriler dosyada olur.
        self.csv_file.flush()

        self.get_logger().info(
            f"Nokta {self.point_id} kaydedildi | "
            f"frame={frame_id} | "
            f"x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

        self.point_id += 1

    def destroy_node(self) -> bool:
        """
        Node kapanırken CSV dosyasını güvenli biçimde kapatır.
        """

        if hasattr(self, "csv_file"):
            if not self.csv_file.closed:
                self.csv_file.flush()
                self.csv_file.close()

                self.get_logger().info(
                    f"CSV dosyası kapatıldı: {self.output_path}"
                )

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = ClickedPointCsvLogger()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().error(
                f"Beklenmeyen hata: {error}"
            )
        else:
            print(f"Node başlatılamadı: {error}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
