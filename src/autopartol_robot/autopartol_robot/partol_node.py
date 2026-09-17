import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from rclpy.time import Time
from autopatol_interfaces.srv import SpeechText

# 添加服务接口
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class PartolNode(BasicNavigator):  # BasicNavigator本身继承了Node
    def __init__(self, node_name="partol_node"):
        super().__init__(node_name)
        # 声明相关参数
        self.declare_parameter("initial_point", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "target_points", [0.0, 0.0, 0.0, 1.0, 1.0, 1.57]
        )  # 每三个数据当作一个点
        self.initial_point = self.get_parameter("initial_point").value
        self.target_points = self.get_parameter("target_points").value
        # 实时位置获取 TF 相关定义
        self.buffer_ = Buffer()
        self.listener_ = TransformListener(self.buffer_, self)

        self.speech_client_ = self.create_client(SpeechText, "speech_text")

        self.declare_parameter("img_save_path", "patrol_images/")
        self.image_save_path = self.get_parameter("img_save_path").value
        self.bridge = CvBridge()
        self.latest_image = None
        self.img_sub_ = self.create_subscription(
            Image, "/camera_sensor/image_raw", self.img_callback, 10
        )

    def img_callback(self, msg):
        self.latest_image = msg

    def record_image(self):
        """
        记录图像
        """
        if self.latest_image is None:
            self.get_logger().warn("还没有接收到相机图像，跳过本次图像记录")
            return
        transform = self.get_current_pose()
        if transform is None:
            self.get_logger().error("获取当前位姿失败，跳过本次图像记录")
            return
        save_dir = self.image_save_path if self.image_save_path else "."
        os.makedirs(save_dir, exist_ok=True)
        # 相机发布的是rgb8，转成bgr8保存，否则图像会红蓝颠倒
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        image_path = os.path.join(
            save_dir,
            f"image_{transform.translation.x:3.2f}_{transform.translation.y:3.2f}.png",
        )
        if cv2.imwrite(image_path, cv_image):
            self.get_logger().info(f"图像已保存：{image_path}")
        else:
            self.get_logger().error(f"图像保存失败：{image_path}")

    def get_pose_by_xyyaw(self, x, y, yaw):
        """
        return PoseStamp对象
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        # 返回形式是xyzw
        quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        if self.initial_point is None:
            self.get_logger().error("initial_point 参数未设置，无法初始化机器人位姿")
            return
        init_pose = self.get_pose_by_xyyaw(
            self.initial_point[0], self.initial_point[1], self.initial_point[2]
        )

        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

    def get_target_points(self):
        """
        通过参数的值获取目标点的集合
        """
        points = []
        if self.target_points is None:
            self.get_logger().error("target_points 参数未设置，无法获取目标点")
            return points
        if len(self.target_points) % 3 != 0:
            self.get_logger().warn("target_points 参数长度不是3的整数倍，最后一个点可能不完整")
        for index in range(int(len(self.target_points) / 3)):
            x = self.target_points[index * 3]
            y = self.target_points[index * 3 + 1]
            yaw = self.target_points[index * 3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f"获取到目标点->{x},{y},{yaw}")
        return points

    def nav_to_pose(self, target_point):
        """
        导航到目标点，成功返回True，失败返回False
        """
        # 发送目标接收反馈结果
        self.goToPose(target_point)
        while rclpy.ok() and not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback is None:
                continue
            remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            self.get_logger().info(f"预计: {remaining} s 后到达")
            self.get_logger().info(f"剩余距离：{feedback.distance_remaining}")
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航结果：成功")
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航结果：被取消")
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航结果：失败")
            return False
        else:
            self.get_logger().error("导航结果：返回状态无效")
            return False

    def get_current_pose(self):
        """
        通过TF获取当前位姿，获取失败返回None
        """
        max_retries = 10
        for _ in range(max_retries):
            if not rclpy.ok():
                return None
            try:
                # 先spin一次，让TF监听器有机会更新缓存
                rclpy.spin_once(self, timeout_sec=0.1)
                tf = self.buffer_.lookup_transform(
                    "map",
                    "base_footprint",
                    Time(seconds=0),
                    Duration(seconds=1),
                )
            except Exception as e:
                self.get_logger().warn(f"不能够获取坐标变换，原因: {str(e)}")
                continue
            transform = tf.transform
            rotation_euler = euler_from_quaternion(
                [
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ]
            )
            self.get_logger().info(
                f"平移:{transform.translation},旋转欧拉角:{rotation_euler}"
            )
            return transform
        self.get_logger().error(f"获取坐标变换失败，超过{max_retries}次重试")
        return None

    def speech_text(self, text):
        """

        调用服务合成语音

        """
        while not self.speech_client_.wait_for_service(timeout_sec=1):
            self.get_logger().info("语音合成服务未上线，等待中.......")
        request = SpeechText.Request()
        request.text = text
        future = self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            if response.result is not None:  # type: ignore
                self.get_logger().info(f"语音合成成功{text}")
            else:
                self.get_logger().info(f"语音合成失败{text}")
        else:
            self.get_logger().info(f"语音合成失败{text}")


def main():
    rclpy.init()
    partol = PartolNode()
    try:
        partol.speech_text("正在准备初始化位置")
        # rclpy.spin(partol)只是为了生成参数文件
        partol.init_robot_pose()
        partol.speech_text("位置初始化完成")
        points = partol.get_target_points()
        while rclpy.ok():
            for point in points:
                x, y, yaw = point[0], point[1], point[2]
                target_pose = partol.get_pose_by_xyyaw(x, y, yaw)
                partol.speech_text(f"正在准备前往{x},{y}目标点")
                if not partol.nav_to_pose(target_pose):
                    partol.speech_text(f"无法到达{x},{y}目标点，跳过该点")
                    continue
                partol.speech_text(f"已经到达{x},{y}目标点，正在准备记录图像")
                partol.record_image()
                partol.speech_text("图像记录完成")
    except KeyboardInterrupt:
        pass
    finally:
        partol.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
