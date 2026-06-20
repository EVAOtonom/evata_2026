import rclpy
from rclpy.node import Node
import pexpect

class UsbSettingsNode(Node):
    def __init__(self):
        super().__init__('usb_settings_node')
        self.get_logger().info('USB ayarları node başlatıldı.')
        # Node başlar başlamaz sudo işlemini yap
        result = self.run_with_sudo()
        if result == 0:
            self.get_logger().info("İzinler başarıyla verildi.")
        else:
            self.get_logger().error("İzin verme sırasında hata oluştu.")

    def run_with_sudo(self):
        try:
            self.get_logger().info("Sudo işlemi başlatılıyor... Şifre otomatik giriliyor.")
            child = pexpect.spawn('sudo chmod 777 /dev/ttyUSB0')
            child.expect('password', timeout=5)
            child.sendline('040411')  # Buraya kendi şifrenizi yazın
            child.expect(pexpect.EOF, timeout=5)
            return 0
        except pexpect.exceptions.TIMEOUT:
            self.get_logger().error("Şifre sorusu gelmedi veya zaman aşımı oldu.")
            return 1
        except pexpect.exceptions.EOF:
            # EOF geldiğinde genellikle işlem tamamdır
            return 0
        except Exception as e:
            self.get_logger().error(f"Hata oluştu: {e}")
            return 1

def main(args=None):
    rclpy.init(args=args)
    node = UsbSettingsNode()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
