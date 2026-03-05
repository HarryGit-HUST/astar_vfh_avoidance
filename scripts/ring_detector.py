#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os            
import rospkg        
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO

class YoloRingDetector:
    def __init__(self):
        # 初始化节点
        rospy.init_node('yolo_ring_detector', anonymous=True)
        
        # === 核心改造：使用 rospkg 动态获取绝对路径 ===
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path('astar') 
            model_path = os.path.join(pkg_path, 'scripts', 'yolo_models', 'ring_best.pt')
        except rospkg.ResourceNotFound:
            rospy.logerr("❌ 找不到 astar 功能包！请确认是否已经 source devel/setup.bash")
            return

        rospy.loginfo(f"🚀 正在加载 YOLO 模型: {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo("✅ YOLO 模型加载成功！准备订阅图像...")

        self.bridge = CvBridge()

        # 定义输入输出话题名 (方便修改)
        self.camera_topic = "/camera_front/image_raw"
        self.result_img_topic = "/yolo/result_image"

        # 订阅与发布
        self.image_sub = rospy.Subscriber(self.camera_topic, Image, self.image_callback, queue_size=1)
        self.center_pub = rospy.Publisher("/ring_center", Point, queue_size=1)
        self.result_image_pub = rospy.Publisher(self.result_img_topic, Image, queue_size=1)

        rospy.loginfo(f"📡 正在监听相机话题: {self.camera_topic}")
        rospy.loginfo(f"📺 请在 RViz 中订阅输出话题: {self.result_img_topic}")

    def image_callback(self, data):
        # [DEBUG 1] 确认是否收到了相机图像
        rospy.loginfo_throttle(2.0, "🔄 [状态] 正在持续接收相机图像并推理...")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr_throttle(2.0, f"❌ CVBridge 转换失败: {e}")
            return

        # 推理
        results = self.model.predict(source=cv_image, conf=0.8, device='cpu', verbose=False)

        best_box = None
        max_area = 0

        # [DEBUG 2] 看看 YOLO 到底有没有查出任何框
        raw_box_count = len(results[0].boxes) if len(results) > 0 else 0
        if raw_box_count > 0:
            rospy.loginfo_throttle(1.0, f"👀 [YOLO] 原始检测到 {raw_box_count} 个目标，正在进行长宽比和面积过滤...")
        else:
            rospy.loginfo_throttle(2.0, "⚠️ [YOLO] 当前画面未检测到任何目标(或者置信度均<0.8)")

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x_c, y_c, w, h = box.xywh[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()

                area = w * h
                aspect_ratio = w / h  

                #[DEBUG 3] 打印被丢弃的原因，看是不是过滤条件太严了
                if aspect_ratio > 3.0 or aspect_ratio < 0.3:
                    rospy.logwarn_throttle(1.0, f"✂️ [过滤] 丢弃形状异常目标: 宽高比={aspect_ratio:.2f}")
                    continue  

                if confidence < 0.75:
                    rospy.logwarn_throttle(1.0, f"✂️ [过滤] 丢弃低置信度目标: conf={confidence:.2f}")
                    continue

                if area > max_area:
                    max_area = area
                    best_box = box

        if best_box is not None:
            x_c, y_c, w, h = best_box.xywh[0].cpu().numpy()
            
            center_msg = Point()
            center_msg.x = x_c
            center_msg.y = y_c
            center_msg.z = w 
            self.center_pub.publish(center_msg)
            
            # [DEBUG 4] 确认目标已锁定并画框
            rospy.loginfo_throttle(1.0, f"🎯 [锁定] 成功锁定目标！中心点:({x_c:.1f}, {y_c:.1f}), 面积:{max_area:.0f}")

            cv2.circle(cv_image, (int(x_c), int(y_c)), 5, (0, 0, 255), -1)
            cv2.rectangle(cv_image, (int(x_c - w/2), int(y_c - h/2)), (int(x_c + w/2), int(y_c + h/2)), (0, 255, 0), 2)

        # 无论有没有检测到，都把图像发出去（没检测到就发原图），证明节点还活着
        try:
            ros_result_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.result_image_pub.publish(ros_result_image)
            # [DEBUG 5] 确认图像已发布
            rospy.loginfo_throttle(2.0, f"📤 [发布] 实时图像已推送至 {self.result_img_topic}")
        except CvBridgeError as e:
            rospy.logerr_throttle(2.0, f"❌ [发布失败] {e}")

if __name__ == '__main__':
    try:
        detector = YoloRingDetector()
        rospy.spin() 
    except rospy.ROSInterruptException:
        pass