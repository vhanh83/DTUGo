#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ollama

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')
        self.subscription = self.create_subscription(
            String,
            '/ai_prompt',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(String, '/ai_response', 10)
        self.get_logger().info('🤖 AI Bridge Node đã khởi động và sẵn sàng kết nối Ollama.')

    def listener_callback(self, msg):
        prompt = msg.data
        self.get_logger().info(f'📥 Nhận prompt: "{prompt}"')

        try:
            response = ollama.chat(model='qwen2.5:7b', messages=[
                {
                    'role': 'system',
                    'content': 'You are an AI assistant in a robot. Keep answers concise, short, and helpful in Vietnamese.'
                },
                {
                    'role': 'user',
                    'content': prompt,
                },
            ])

            answer = response['message']['content']
            out_msg = String()
            out_msg.data = answer
            self.publisher_.publish(out_msg)
            self.get_logger().info(f'📤 AI Phản hồi: {answer}')

        except Exception as e:
            self.get_logger().error(f'Lỗi giao tiếp với Ollama: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AIBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()