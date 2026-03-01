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
        rospy.init_node('yolo_ring_detector', anonymous=True)
        
        # === 1. 路径设置 (自动寻找当前包下的模型) ===
        # 请确保下面的 'astar' 换成你 package.xml 里写的真实包名！
        package_name = 'astar' 
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path(package_name)
            model_path = os.path.join(pkg_path, 'scripts', 'yolo_models', 'ring_best.pt')
        except Exception as e:
            rospy.logerr(f"无法找到包路径: {e}")
            return

        rospy.loginfo(f"加载模型: {model_path}")
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # === 2. 话题设置 ===
        # 注意：这里订阅的话题名必须和你 Gazebo 里的相机话题一致
        # 如果不确定，先用 rostopic list 查一下
        self.image_sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.image_callback, queue_size=1)
        
        # 发布识别结果：x, y 为像素中心，z 为像素宽度
        self.center_pub = rospy.Publisher("/ring_center", Point, queue_size=1)
        
        # 调试图像发布
        self.result_image_pub = rospy.Publisher("/yolo/debug_view", Image, queue_size=1)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 推理
        results = self.model.predict(source=cv_image, conf=0.6, device='cpu', verbose=False)

        best_box = None
        max_area = 0

        for r in results:
            for box in r.boxes:
                x_c, y_c, w, h = box.xywh[0].cpu().numpy()
                area = w * h
                
                # 简单的过滤逻辑：找最大的那个
                if area > max_area:
                    max_area = area
                    best_box = box

        if best_box is not None:
            x_c, y_c, w, h = best_box.xywh[0].cpu().numpy()
            
            # 发布消息：Point.z 存储宽度 w
            center_msg = Point()
            center_msg.x = x_c
            center_msg.y = y_c
            center_msg.z = w 
            self.center_pub.publish(center_msg)
            
            # 画框用于调试
            cv2.rectangle(cv_image, (int(x_c - w/2), int(y_c - h/2)), (int(x_c + w/2), int(y_c + h/2)), (0, 255, 0), 2)

        # 发布调试图像
        try:
            self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            pass

if __name__ == '__main__':
    try:
        detector = YoloRingDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass