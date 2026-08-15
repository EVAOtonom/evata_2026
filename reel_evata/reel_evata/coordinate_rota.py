#!/usr/bin/env python3

import json
import os
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


# ============================================================
# AYARLAR
# ============================================================

TOPIC_NAME = "/initialpose"

SCRIPT_DIR = os.path.expanduser(
    "~/ros2_ws/src/evata_sim/evata_sim"
)

JSON_FILE = "rota.json"

JSON_PATH = os.path.join(
    SCRIPT_DIR,
    JSON_FILE
)


# ============================================================
# JSON BAŞLANGIÇ YAPISI
# ============================================================

DEFAULT_DATA = {
    "navigasyon": {
        "rota": [],
        "orta_seritler": []
    },

    "park": {
        "park_noktalari": [],
        "park_final": []
    }
}


# ============================================================
# JSON KAYDET
# ============================================================

def save_json(data):

    temp_path = JSON_PATH + ".tmp"

    with open(
        temp_path,
        "w",
        encoding="utf-8"
    ) as f:

        json.dump(
            data,
            f,
            indent=4,
            ensure_ascii=False
        )

    os.replace(
        temp_path,
        JSON_PATH
    )


# ============================================================
# JSON YÜKLE
# ============================================================

def load_json():

    if not os.path.exists(JSON_PATH):

        data = json.loads(
            json.dumps(DEFAULT_DATA)
        )

        save_json(data)

        return data


    try:

        with open(
            JSON_PATH,
            "r",
            encoding="utf-8"
        ) as f:

            data = json.load(f)


        # ----------------------------------------------------
        # Eksik bölümleri tamamla
        # ----------------------------------------------------

        if "navigasyon" not in data:

            data["navigasyon"] = {}


        if "rota" not in data["navigasyon"]:

            data["navigasyon"]["rota"] = []


        if "orta_seritler" not in data["navigasyon"]:

            data["navigasyon"]["orta_seritler"] = []


        if "park" not in data:

            data["park"] = {}


        if "park_noktalari" not in data["park"]:

            data["park"]["park_noktalari"] = []


        if "park_final" not in data["park"]:

            data["park"]["park_final"] = []


        save_json(data)

        return data


    except Exception as e:

        print()
        print(
            f"JSON okunamadı: {e}"
        )

        print(
            "Yeni rota.json oluşturuluyor."
        )

        data = json.loads(
            json.dumps(DEFAULT_DATA)
        )

        save_json(data)

        return data


# ============================================================
# ANA KATEGORİ SEÇ
# ============================================================

def select_main_category():

    print()
    print("=" * 60)
    print("                 ANA KATEGORİ")
    print("=" * 60)
    print()
    print("1 - Navigasyon")
    print("2 - Park")
    print()

    while True:

        choice = input(
            "Seçim [1-2]: "
        ).strip()

        if choice == "1":

            return "navigasyon"


        elif choice == "2":

            return "park"


        print(
            "Geçersiz seçim! Lütfen 1 veya 2 gir."
        )


# ============================================================
# NAVİGASYON ALT KATEGORİ
# ============================================================

def select_navigation_category():

    print()
    print("=" * 60)
    print("             NAVİGASYON KATEGORİSİ")
    print("=" * 60)
    print()
    print("1 - Rota")
    print("2 - Orta Şeritler")
    print()

    while True:

        choice = input(
            "Seçim [1-2]: "
        ).strip()

        if choice == "1":

            return "rota"


        elif choice == "2":

            return "orta_seritler"


        print(
            "Geçersiz seçim! Lütfen 1 veya 2 gir."
        )


# ============================================================
# PARK ALT KATEGORİ
# ============================================================

def select_park_category():

    print()
    print("=" * 60)
    print("                PARK KATEGORİSİ")
    print("=" * 60)
    print()
    print("1 - Park Noktaları")
    print("2 - Park Final")
    print()

    while True:

        choice = input(
            "Seçim [1-2]: "
        ).strip()

        if choice == "1":

            return "park_noktalari"


        elif choice == "2":

            return "park_final"


        print(
            "Geçersiz seçim! Lütfen 1 veya 2 gir."
        )


# ============================================================
# KATEGORİ SEÇ
# ============================================================

def select_category():

    main_category = select_main_category()


    if main_category == "navigasyon":

        sub_category = select_navigation_category()


    else:

        sub_category = select_park_category()


    return main_category, sub_category


# ============================================================
# ROS NODE
# ============================================================

class CoordinateLogger(Node):

    def __init__(
        self,
        data,
        main_category,
        sub_category
    ):

        super().__init__(
            "coordinate_logger"
        )


        self.data = data

        self.main_category = main_category

        self.sub_category = sub_category


        # ----------------------------------------------------
        # ENTER ile kategori değiştirme
        # ----------------------------------------------------

        self.change_category = False


        # ----------------------------------------------------
        # Mevcut sayaç
        # ----------------------------------------------------

        self.point_count = len(
            self.data[
                self.main_category
            ][
                self.sub_category
            ]
        )


        # ----------------------------------------------------
        # /initialpose subscriber
        # ----------------------------------------------------

        self.subscription = self.create_subscription(

            PoseWithCovarianceStamped,

            TOPIC_NAME,

            self.pose_callback,

            10
        )


        print()
        print("=" * 65)

        print(
            f"Ana kategori : {self.main_category}"
        )

        print(
            f"Alt kategori : {self.sub_category}"
        )

        print(
            f"Mevcut nokta: {self.point_count}"
        )

        print("=" * 65)

        print()

        print(
            "RViz'de 2D Pose Estimate ile noktaları seç."
        )

        print()

        print(
            "Kategori değiştirmek için ENTER'a bas."
        )

        print()

        print(
            "Programı tamamen kapatmak için CTRL+C."
        )

        print()


    # ========================================================
    # POSE CALLBACK
    # ========================================================

    def pose_callback(self, msg):

        position = msg.pose.pose.position

        orientation = msg.pose.pose.orientation


        # ----------------------------------------------------
        # Sayaç
        # ----------------------------------------------------

        self.point_count += 1


        # ====================================================
        # PARK
        #
        # SADECE X Y
        # ====================================================

        if (
            self.main_category == "park"
            and self.sub_category == "park_noktalari"
        ):
            
            goal = {

                "id": self.point_count,
                "x": position.x,
                "y": position.y

            }


        # ====================================================
        # NAVİGASYON
        #
        # X Y OZ OW WAIT
        # ====================================================

        else:

            goal = {

                "id": self.point_count,

                "x": position.x,

                "y": position.y,

                "oz": orientation.z,

                "ow": orientation.w,

                "wait": 0.0

            }


        # ----------------------------------------------------
        # JSON'A EKLE
        # ----------------------------------------------------

        self.data[
            self.main_category
        ][
            self.sub_category
        ].append(goal)


        # ----------------------------------------------------
        # JSON'A KAYDET
        # ----------------------------------------------------

        save_json(self.data)


        # ====================================================
        # TERMINAL BİLGİSİ
        # ====================================================

        print()
        print("=" * 65)

        print(
            f"  NOKTA KAYDEDİLDİ → #{self.point_count}"
        )

        print("=" * 65)

        print(
            f"  Ana kategori : {self.main_category}"
        )

        print(
            f"  Alt kategori : {self.sub_category}"
        )

        print(
            f"  X            : {position.x}"
        )

        print(
            f"  Y            : {position.y}"
        )


        if self.main_category == "navigasyon":

            print(
                f"  OZ           : {orientation.z}"
            )

            print(
                f"  OW           : {orientation.w}"
            )

            print(
                f"  WAIT         : 0.0"
            )


        print()

        print(
            f"  JSON         : {JSON_PATH}"
        )

        print("=" * 65)

        print()


# ============================================================
# ENTER DİNLEME THREAD'İ
# ============================================================

def wait_for_enter(node):

    while rclpy.ok():

        try:

            input()

            node.change_category = True

            break


        except EOFError:

            break


        except KeyboardInterrupt:

            break


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)


    # --------------------------------------------------------
    # JSON'u bir kere yükle
    # --------------------------------------------------------

    data = load_json()


    try:

        while rclpy.ok():

            # =================================================
            # EN BAŞTAKİ MENÜ
            # =================================================

            main_category, sub_category = select_category()


            # =================================================
            # NODE
            # =================================================

            node = CoordinateLogger(

                data,

                main_category,

                sub_category

            )


            # =================================================
            # ENTER THREAD
            # =================================================

            enter_thread = threading.Thread(

                target=wait_for_enter,

                args=(node,),

                daemon=True

            )

            enter_thread.start()


            # =================================================
            # ROS ÇALIŞTIR
            # =================================================

            while rclpy.ok() and not node.change_category:

                rclpy.spin_once(
                    node,
                    timeout_sec=0.1
                )


            # =================================================
            # NODE KAPAT
            # =================================================

            node.destroy_node()


            # =================================================
            # KATEGORİ DEĞİŞTİR
            # =================================================

            if node.change_category:

                print()
                print("=" * 65)

                print(
                    "  KATEGORİ DEĞİŞTİRİLİYOR..."
                )

                print("=" * 65)

                print()


                # ---------------------------------------------
                # Buradan tekrar en başa döner
                # ---------------------------------------------

                continue


            break


    except KeyboardInterrupt:

        print()

        print("=" * 65)

        print(
            "  Koordinat kaydı sonlandırıldı."
        )

        print("=" * 65)

        print()


    finally:

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# PROGRAM BAŞLANGICI
# ============================================================

if __name__ == "__main__":

    main()
