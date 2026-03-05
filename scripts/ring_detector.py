#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os            # 新增
import rospkg        # 新增
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from ultralytics import YOLO

class YoloRingDetector:
    def __init__(self):
        # 初始化节点
        rospy.init_node('yolo_ring_detector', anonymous=True)
        
        # === 核心改造：使用 rospkg 动态获取绝对路径，完美支持跨设备移植 ===
        rospack = rospkg.RosPack()
        try:
            # 自动寻址你的功能包目录
            pkg_path = rospack.get_path('astar') 
            # 拼接出模型文件的最终路径
            model_path = os.path.join(pkg_path, 'scripts', 'yolo_models', 'ring_best.pt')
        except rospkg.ResourceNotFound:
            rospy.logerr("❌ 找不到 vision_ring 功能包！请确认是否已经 source devel/setup.bash")
            return

        rospy.loginfo(f"正在加载 YOLO 模型: {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo("✅ YOLO 模型加载成功！")

        self.bridge = CvBridge()


        # 2. 核心：订阅无人机的相机画面 
        # (⚠️ 注意："/camera/image_raw" 需要替换成你仿真环境里无人机真实的相机话题名)
        self.image_sub = rospy.Subscriber("/camera_front/image_raw", Image, self.image_callback, queue_size=1)

        # 3. 核心：发布穿越门中心点坐标 (使用 Point 类型，z 轴暂时填 0)
        self.center_pub = rospy.Publisher("/ring_center", Point, queue_size=1)
        
        # (可选) 发布画了框的实时结果图像，方便你在 rviz 或 rqt_image_view 里看
        self.result_image_pub = rospy.Publisher("/yolo/result_image", Image, queue_size=1)

    def image_callback(self, data):
        try:
            # 将 ROS 的 Image 消息转换成 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 把图像送给 YOLO 推理 (conf=0.8 表示只要置信度大于 80% 的结果)
        results = self.model.predict(source=cv_image, conf=0.8,device='cpu', verbose=False)

        # ====== 解析检测结果 (加入目标过滤逻辑) ======
        best_box = None
        max_area = 0

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # 获取中心点 x, y 和 宽高 w, h
                x_c, y_c, w, h = box.xywh[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()

                # 1. 计算面积和长宽比
                area = w * h
                aspect_ratio = w / h  # 宽度除以高度

                # 2. 形状过滤：排除被压扁的地面标志(H标)或细长杂物
                # 真实的门框长宽比一般在 0.5 到 2.5 之间
                if aspect_ratio > 3.0 or aspect_ratio < 0.3:
                    continue  # 直接忽略这个框

                # 3. 置信度过滤：提高门槛
                if confidence < 0.75:
                    continue

                # 4. 距离优先法则：选择画面中面积最大的框（绝对是我们要穿的那个门，而不是远处的假目标）
                if area > max_area:
                    max_area = area
                    best_box = box

        # 如果经过层层筛选，找到了真正的门
        if best_box is not None:
            x_c, y_c, w, h = best_box.xywh[0].cpu().numpy()
            
            # 构造 ROS 消息并发布 (⚠️ 记得 z 轴传宽度 w)
            center_msg = Point()
            center_msg.x = x_c
            center_msg.y = y_c
            center_msg.z = w 
            self.center_pub.publish(center_msg)
            
            # 可视化...
            cv2.circle(cv_image, (int(x_c), int(y_c)), 5, (0, 0, 255), -1)
            cv2.rectangle(cv_image, (int(x_c - w/2), int(y_c - h/2)), (int(x_c + w/2), int(y_c + h/2)), (0, 255, 0), 2)

        
        # 把画好框的图片转换回 ROS 消息并发布出去，供人眼监视
        try:
            ros_result_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.result_image_pub.publish(ros_result_image)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        detector = YoloRingDetector()
        rospy.spin() # 保持节点运行，等待回调
    except rospy.ROSInterruptException:
        pass
