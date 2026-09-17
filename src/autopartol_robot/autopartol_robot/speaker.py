import espeakng
import rclpy
from rclpy.node import Node

from autopatol_interfaces.srv import SpeechText


class Speaker(Node):
    def __init__(self, node_name="speaker"):
        super().__init__(node_name)
        # 初始化语音合成器
        self.speaker = espeakng.Speaker()
        self.speaker.voice = "zh"  # 中文
        # 创建服务：服务名 speech_text，类型 SpeechText
        self.speech_service = self.create_service(
            SpeechText, "speech_text", self.speak_text_callback
        )

    def speak_text_callback(self, request, response):
        self.get_logger().info(f"正在准备朗读：{request.text}")
        # wait4prev=True 会等待上一次朗读结束，避免语音被打断
        self.speaker.say(request.text, wait4prev=True)
        response.result = True
        return response


def main():
    rclpy.init()
    node = Speaker()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
