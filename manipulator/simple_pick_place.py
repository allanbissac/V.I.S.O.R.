#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from pymycobot import MyCobot280   # 使用官方 Python API 控制 myCobot 280Pi


class SimplePickPlace(Node):
    """
    非视觉版：固定三个位姿的“拿起-放下”示例。
    这里只是示例，具体角度/坐标要根据你的桌面重新调。
    """

    def __init__(self):
        super().__init__("simple_pick_place")

        # *** 串口参数：280Pi 默认是 /dev/ttyAMA0, 波特率 1000000 ***
        self.mc = MyCobot280("/dev/ttyAMA0", 1000000)

        self.get_logger().info("连接 myCobot 280Pi ...")
        self.mc.power_on()
        time.sleep(0.5)

        # 创建一个一次性的定时器，启动 2 秒后执行 demo
        self.timer = self.create_timer(2.0, self.run_demo)

    # -------- 可以按需要修改的几个姿态 --------
    def pose_home(self):
        """关节空间 home 姿态（单位：度）"""
        # 这只是一个大致安全的姿态，你可以根据实际情况调整
        return [0, -30, 30, 0, 40, 0]

    def pose_vertical(self):
        """机械臂竖直向上的姿态（六个关节全为 0 度）"""
        return [0, 0, 0, 0, 0, 0]

    def pose_above_pick(self):
        """末端坐标：在待抓取方块上方一点（单位：mm, deg）"""
        # x, y, z, rx, ry, rz —— 这里一定要根据你真实桌面重新调！
        return [200, 0, 200, 180, 0, 0]

    def pose_pick(self):
        """真正抓取高度"""
        return [200, 0, 120, 180, 0, 0]

    def pose_above_place(self):
        """放置点上方"""
        return [150, 150, 200, 180, 0, 90]

    def pose_place(self):
        """放置高度"""
        return [150, 150, 120, 180, 0, 90]

    # -------- 夹爪控制（根据你实际夹爪接口调整） --------
    def open_gripper(self):
        """
        打开夹爪。
        对 280Pi 自带电动夹爪，一般用 set_gripper_value / set_gripper_state 之类的接口。
        下面这一行的数值你可以自己试着改大/改小。
        """
        try:
            # 第一个参数：张开程度(0~100)；第二个参数：速度
            self.mc.set_gripper_value(80, 50)
        except Exception as e:
            self.get_logger().warn(f"打开夹爪失败：{e}")

    def close_gripper(self):
        """闭合夹爪"""
        try:
            self.mc.set_gripper_value(10, 50)
        except Exception as e:
            self.get_logger().warn(f"关闭夹爪失败：{e}")

    # -------- 主动作序列 --------
    def run_demo(self):
        # 只跑一次就停掉定时器
        self.timer.cancel()
        self.get_logger().info("开始执行 pick & place demo ...")

        # 1. 回 home
        self.get_logger().info("回到 home 姿态 ...")
        self.mc.send_angles(self.pose_home(), 40)
        time.sleep(3)

        # 2. 夹爪张开
        self.get_logger().info("夹爪张开 ...")
        self.open_gripper()
        time.sleep(1.0)

        # 3. 去抓取点上方
        self.get_logger().info("移动到抓取点上方 ...")
        self.mc.send_coords(self.pose_above_pick(), 40, 1)  # mode=1 直线插补
        time.sleep(3)

        # 4. 下到抓取高度
        self.get_logger().info("下到抓取高度 ...")
        self.mc.send_coords(self.pose_pick(), 30, 1)
        time.sleep(2)

        # 5. 合上夹爪
        self.get_logger().info("闭合夹爪 ...")
        self.close_gripper()
        time.sleep(1.5)

        # 6. 抬回上方
        self.get_logger().info("抬回抓取点上方 ...")
        self.mc.send_coords(self.pose_above_pick(), 40, 1)
        time.sleep(3)

        # 7. 移动到放置点上方
        self.get_logger().info("移动到放置点上方 ...")
        self.mc.send_coords(self.pose_above_place(), 40, 1)
        time.sleep(3)

        # 8. 下到放置高度
        self.get_logger().info("下到放置高度 ...")
        self.mc.send_coords(self.pose_place(), 30, 1)
        time.sleep(2)

        # 9. 松开夹爪
        self.get_logger().info("松开夹爪 ...")
        self.open_gripper()
        time.sleep(1.5)

        # 10. 回 home（可选）
        self.get_logger().info("回到 home 姿态 ...")
        self.mc.send_angles(self.pose_home(), 40)
        time.sleep(3)

        # 11. 回到竖直姿态，并保持扭矩
        self.get_logger().info("回到竖直姿态（全 0 度），保持扭矩，不关闭电机 ...")
        self.mc.send_angles(self.pose_vertical(), 30)
        time.sleep(3)

        # 注意：这里不再调用 power_off()，这样机械臂会保持在竖直姿态，不会塌下来
        # 如果以后你想手动关电机，可以单独写一个小脚本来 power_off

        self.get_logger().info("Demo 完成。")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
