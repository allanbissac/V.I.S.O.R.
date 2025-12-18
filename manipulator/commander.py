#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = Node('arm_commander')
    publisher = node.create_publisher(String, 'arm_command', 10)

    print("========================================")
    print("      Kitty的机械臂指挥台 (Pick & Place)")
    print("========================================")
    print("请输入指令，格式: [模式] [x] [y] [z]")
    print("模式可选: pick (抓), place (放)")
    print("\n👉 示例 1 (去抓):  pick 0.2 0.0 0.15")
    print("👉 示例 2 (去放):  place 0.2 -0.2 0.15")
    print("输入 'q' 退出")
    print("----------------------------------------")

    try:
        while rclpy.ok():
            user_input = input("\n请输入指令 >>> ")
            
            if user_input.lower() == 'q':
                break
            
            # 简单的格式检查
            parts = user_input.split()
            if len(parts) != 4:
                print("⚠️ 格式不对！请检查输入是否包含4个部分 (模式 x y z)")
                continue
            
            if parts[0] not in ['pick', 'place']:
                print("⚠️ 模式错误！第一个词必须是 pick 或 place")
                continue

            msg = String()
            msg.data = user_input
            publisher.publish(msg)
            print(f"📡 指令已发送: '{user_input}' -> 等待机械臂执行...")

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
