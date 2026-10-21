#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        self.subscription = self.create_subscription(
            String,
            '/ai_response',
            self.speak_callback,
            10
        )
        self.get_logger().info('🔊 TTS Node đã sẵn sàng phát âm thanh phản hồi!')

    def speak_callback(self, msg):
        response_text = msg.data
        self.get_logger().info(f'Robot đọc: "{response_text}"')

        try:
            tts = gTTS(text=response_text, lang='vi', slow=False)
            audio_file = "/tmp/ai_response.mp3"
            tts.save(audio_file)

            # Phát file âm thanh ra loa qua mpg123
            os.system(f"mpg123 -q {audio_file} >/dev/null 2>&1")

        except Exception as e:
            self.get_logger().error(f'Lỗi phát âm thanh: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()