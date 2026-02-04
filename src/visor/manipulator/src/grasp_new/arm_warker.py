#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymycobot import MyCobot280

class ArmWorker(Node):
    def __init__(self):
        super().__init__('arm_worker_node')
        
        # --- 1. 硬件连接 ---
        # 树莓派版默认端口 /dev/ttyAMA0，波特率 1000000
        self.port = '/dev/ttyAMA0'
        self.baud = 1000000
        
        try:
            # 使用 MyCobot280 类
            self.mc = MyCobot280(self.port, self.baud)
            
            # --- 上电并等待 ---
            self.mc.power_on()
            time.sleep(0.5)
            
            self.get_logger().info(f"✅ 机械臂连接成功: {self.port}")
            
            # 初始化夹爪 (使用 value 模式，更通用)
            self.init_gripper()
            
        except Exception as e:
            self.get_logger().error(f"❌ 连接或初始化失败: {e}")
            self.mc = None

        # --- 2. 定义初始位置 (Home) ---
        self.home_angles = [0, 0, 0, 0, 0, 0] 

        # --- 3. 订阅指令话题 ---
        self.subscription = self.create_subscription(
            String, 'arm_command', self.command_callback, 10
        )
        
        self.is_busy = False
        self.has_object = False
        # 启动时先回初始位置
        self.go_home()

    def init_gripper(self):
        """初始化夹爪"""
        if self.mc:
            try:
                # 100=完全张开, 速度50
                self.mc.set_gripper_value(100, 50)
                time.sleep(1)
            except AttributeError:
                # 备用方案
                self.mc.set_gripper_state(0, 50) 
            self.has_object = False

    def go_home(self):
        """回到初始状态"""
        self.get_logger().info("🏠 正在回到初始位置 (Home)...")
        if self.mc:
            self.mc.send_angles(self.home_angles, 40)
            time.sleep(3)

    def move_to_xyz(self, x, y, z):
        """
        核心运动函数
        ⚠️ 输入单位现在是: 毫米 (mm)
        """
        if not self.mc:
            self.get_logger().error("机械臂未连接，无法移动")
            return False

        # --- 修改处：直接使用输入值，不再乘以 1000 ---
        target_x = x
        target_y = y
        target_z = z

        # 安全范围检查 (MyCobot 280 臂长约 280mm，给到 340mm 极限)
        distance = math.sqrt(target_x**2 + target_y**2 + (350-target_z)**2)
        if distance > 279: #350-70.42=279.58
            self.get_logger().error(f"❌ 目标太远了 ({distance:.1f}mm)！超出机械臂活动范围。")
            return False
        elif target_z < 110:
            self.get_logger().error("❌ 目标高度过低！请保持在0mm以上。")
            return False
        elif target_z > 350:
            self.get_logger().error("❌ 目标高度过高！请保持在350mm以下。")
            return False

        self.get_logger().info(f"执行移动 -> x={target_x:.1f}, y={target_y:.1f}, z={target_z:.1f} mm")
        
        # 设定末端姿态 [180, 0, 0] 垂直向下 
        rx, ry, rz = 180, 0, 0 
        
        # 发送指令
        self.mc.send_coords([target_x, target_y, target_z, rx, ry, rz], 30, 0)
        time.sleep(5) 
        
        return True

    def command_callback(self, msg):
        if self.is_busy:
            self.get_logger().warn("⏳ 忙碌中...")
            return

        command_str = msg.data.lower().strip()
        self.get_logger().info(f"收到指令: {command_str}")

        try:
            parts = command_str.split()
            mode = parts[0]
            # 这里接收到的已经是 mm 了
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])

            self.is_busy = True
            
            if mode == 'pick':
                self.perform_pick(x, y, z)
            elif mode == 'place':
                self.perform_place(x, y, z)
            else:
                self.get_logger().error("指令错误")

        except Exception as e:
            self.get_logger().error(f"执行出错: {e}")
        finally:
            self.is_busy = False

    def perform_pick(self, x, y, z):
        if self.has_object:
            self.get_logger().warn("⚠️ 手里已有物体")
            return

        self.get_logger().info(f"--- 执行抓取 ---")
        
        # 张开 (100)
        if self.mc: self.mc.set_gripper_value(100, 50)
        time.sleep(0.5)

        if self.move_to_xyz(x, y, z):
            self.get_logger().info("✊ 抓取中...")
            # 闭合 (20)
            if self.mc: self.mc.set_gripper_value(20, 50)
            time.sleep(1.5)

            self.go_home()
            self.has_object = True
            self.get_logger().info("✅ 抓取完成")

    def perform_place(self, x, y, z):
        if not self.has_object:
            self.get_logger().warn("⚠️ 手里没东西")
            return

        self.get_logger().info(f"--- 执行放置 ---")

        if self.move_to_xyz(x, y, z):
            self.get_logger().info("🖐 放下中...")
            # 张开 (100)
            if self.mc: self.mc.set_gripper_value(100, 50)
            time.sleep(1.5)

            self.go_home()
            self.has_object = False
            self.get_logger().info("✅ 放置完成")

def main(args=None):
    rclpy.init(args=args)
    node = ArmWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
