#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from pymycobot import MyCobot280        # 控制 myCobot 280Pi
from arm_interfaces.srv import PickPlace  # 刚刚定义的自定义服务类型


class PickPlaceService(Node):
    """
    /pick_place 服务：
      - 请求: pick_x/y/z, place_x/y/z (单位：mm)
      - 响应: success(bool), message(string)
    """

    def __init__(self):
        super().__init__("pick_place_service")

        self._busy = False  # 防止并发请求

        # 连接机械臂
        self.get_logger().info("连接 myCobot 280Pi ...")
        # 如果你的串口不是 /dev/ttyAMA0，请改成实际的
        self.mc = MyCobot280("/dev/ttyAMA0", 1000000)
        self.mc.power_on()# 打开电源
        time.sleep(0.5)

        # 创建服务：名为 /pick_place
        self.srv = self.create_service(
            PickPlace,
            "pick_place",
            self.handle_pick_place
        )
        self.get_logger().info(
            "服务 /pick_place (arm_interfaces/srv/PickPlace) 已启动，等待调用..."
        )

    # ================== 一些固定姿态 ==================
    def pose_home(self):
        """home 姿态（关节角，单位：度），可以按需要调整"""
        return [0, -30, 30, 0, 40, 0]

    def pose_vertical(self):
        """竖直姿态（6 个关节全 0 度）"""
        return [0, 0, 0, 0, 0, 0]

    # ================== service 回调 ==================
    def handle_pick_place(self, request, response):
        """
        收到 PickPlace 请求时执行：
          - 如果当前在忙，就直接返回失败
          - 否则按顺序执行抓取流程
        """
        if self._busy:
            response.success = False
            response.message = "上一次抓取尚未结束，请稍后再试。"
            return response

        self._busy = True

        pick_x = request.pick_x
        pick_y = request.pick_y
        pick_z = request.pick_z

        place_x = request.place_x
        place_y = request.place_y
        place_z = request.place_z

        self.get_logger().info(
            f"收到抓取请求：pick=({pick_x:.1f}, {pick_y:.1f}, {pick_z:.1f}), "
            f"place=({place_x:.1f}, {place_y:.1f}, {place_z:.1f}) (mm)"
        )

        try:
            # 相对目标点上方的高度偏移（避免直接撞上去）
            offset_z = 80.0  # mm，可以根据实际情况改

            # 1. 回 home
            self.get_logger().info("回到 home 姿态 ...")
            self.mc.send_angles(self.pose_home(), 40)
            time.sleep(3)

            # 2. 夹爪张开
            self.get_logger().info("夹爪张开 ...")
            self.open_gripper()
            time.sleep(1.0)

            # 3. 去抓取点上方
            above_pick = [pick_x, pick_y, pick_z + offset_z, 180, 0, 0]
            self.get_logger().info(f"移动到抓取点上方: {above_pick} ...")
            self.mc.send_coords(above_pick, 40, 1)  # mode=1 直线
            time.sleep(3)

            # 4. 下到抓取高度
            pick_pose = [pick_x, pick_y, pick_z, 180, 0, 0]
            self.get_logger().info(f"下到抓取高度: {pick_pose} ...")
            self.mc.send_coords(pick_pose, 30, 1)
            time.sleep(2)

            # 5. 闭合夹爪
            self.get_logger().info("闭合夹爪 ...")
            self.close_gripper()
            time.sleep(1.5)

            # 6. 抬回抓取点上方
            self.get_logger().info("抬回抓取点上方 ...")
            self.mc.send_coords(above_pick, 40, 1)
            time.sleep(3)

            # 7. 移动到放置点上方
            above_place = [place_x, place_y, place_z + offset_z, 180, 0, 90]
            self.get_logger().info(f"移动到放置点上方: {above_place} ...")
            self.mc.send_coords(above_place, 40, 1)
            time.sleep(3)

            # 8. 下到放置高度
            place_pose = [place_x, place_y, place_z, 180, 0, 90]
            self.get_logger().info(f"下到放置高度: {place_pose} ...")
            self.mc.send_coords(place_pose, 30, 1)
            time.sleep(2)

            # 9. 松开夹爪
            self.get_logger().info("松开夹爪 ...")
            self.open_gripper()
            time.sleep(1.5)

            # 10. 回 home（可选）
            self.get_logger().info("回到 home 姿态 ...")
            self.mc.send_angles(self.pose_home(), 40)
            time.sleep(3)

            # 11. 回到竖直姿态并保持扭矩
            self.get_logger().info("回到竖直姿态 (0,0,0,0,0,0)，保持扭矩 ...")
            self.mc.send_angles(self.pose_vertical(), 30)
            time.sleep(3)

            response.success = True
            response.message = "抓取流程执行完成。"

        except Exception as e:
            self.get_logger().error(f"抓取过程中发生错误: {e}")
            response.success = False
            response.message = f"执行失败: {e}"

        finally:
            self._busy = False

        return response

    # ================== 夹爪控制 ==================
    def open_gripper(self):
        try:
            self.mc.set_gripper_value(80, 50)
        except Exception as e:
            self.get_logger().warn(f"打开夹爪失败：{e}")

    def close_gripper(self):
        try:
            self.mc.set_gripper_value(10, 50)
        except Exception as e:
            self.get_logger().warn(f"关闭夹爪失败：{e}")


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("节点退出，保持当前扭矩。")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
