import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import os
import tf2_ros
import math
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
class PoseEstimator(Node):
    def __init__(self):
        super().__init__("PoseEstimator")
        self.create_subscription(Image, "/camera/image_raw",self.pose_estimation,qos_profile=10)
        # self.timer_ = self.create_timer(1, self.pose_estimation)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                  
        self.get_logger().info("Pose Estimator node has started")

    def pose_estimation(self, msg):
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)

        id_to_find = 1
        marker_size = 0.50# cm

        # realWorldEfficiency = .7
        # aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv.aruco.DetectorParameters()


        # Load the camera calibration matrices
        cameraMatrix =np.array([[381.567418, 0.000000 ,319.184172], [0.000000 ,381.616168, 239.334918], [0.0, 0.0, 1.0]])
        cameraDistortion = np.array([0.0,0.0,0.0,0.0,0.0])

        bridge = CvBridge()
        frame_converted = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # cv.imshow("Converted Frame", frame_converted)
        # cv.waitKey(1)

        frame_np= np.array(frame_converted)

        gray_img = cv.cvtColor(frame_converted, cv.COLOR_BGR2GRAY)
        ids = ''
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = cv.aruco.detectMarkers(gray_img, dictionary)
                
                 
        # if ids is not None:
        #     print("Found these IDs in the frame:")
        #     print(ids)
        if ids is not None and ids[0] == id_to_find:
            ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, cameraDistortion)
            self.rvec, self.tvec = ret[0][0, 0, :], ret[1][0, 0, :]
            rotation_matrix, _ = cv.Rodrigues(self.rvec)
            self.rvec_degrees = np.degrees(self.rvec)
            x = "{:.2f}".format(self.tvec[0])
            y = "{:.2f}".format(self.tvec[1])
            z = "{:.2f}".format(self.tvec[2])
            # print(x,y,z)
            marker_position = "MARKER POSITION: x=" + x + " y=" + y + " z=" + z
            # print(marker_position)
            # print("")

        if corners:
            self.rvec, self.tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, cameraDistortion)
            total_markers = range(0, ids.size)
            for ids, corners,i in zip(ids, corners, total_markers):
                cv.polylines(frame_converted, [corners.astype(np.int32)], True, (90,255,255), 4, cv.LINE_AA)
                corners = corners.reshape(4,2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                bottom_left = corners[1].ravel()
                bottom_left = corners[1].ravel()
                bottom_left = corners[1].ravel()
                dist=((self.tvec[i][0][0])**2+(self.tvec[i][0][1])**2+(self.tvec[i][0][2])**2)**(1/2)
                anglebtw_marker_x= math.acos(float(x)/round(dist,2))
                anglebtw_marker_y= math.acos(float(y)/round(dist,2))
                anglebtw_marker_z= math.acos(float(z)/round(dist,2))
                frame_converted = cv.drawFrameAxes(frame_converted, cameraMatrix, cameraDistortion, self.rvec[i], self.tvec[i], 3, 2)
                text_frame =cv.putText(frame_converted, f"id:{ids[0]} Dist: {round(dist,2)}",top_right ,cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA)
                text_frame = cv.resize(text_frame, (800, 450))
                cv.imshow("text",text_frame)
                cv.waitKey(1)
            # print(f"self.rvec",self.rvec)
            self.rvec1=self.rvec.ravel()
            # print(self.tvec)
            # print(f"rotation matrix", rotation_matrix)
            # print(f"self.rvec_degrees", self.rvec_degrees)
            # rotation_in_y_rad = self.rvec1[1]
            # print(f"rotation_in_Y_RAD", rotation_in_y_rad)
            # rotation_in_y_deg = self.rvec_degrees[1]
            # print(f"rotation_in_Y_DEG", rotation_in_y_deg)
            # print(anglebtw_marker_x,anglebtw_marker_y,anglebtw_marker_z)
            
            self.base_and_odom()
            self.lookup_and_publish_transform()
            
        else:
            print("ARUCO " + str(id_to_find) + " NOT FOUND IN FRAME.")
            print("")
    
    def base_and_odom(self):
        try:
                transform = self.tf_buffer.lookup_transform('odom', 'camera_link', rclpy.time.Time())
            
                tf_msg = TransformStamped()
                
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'odom'
                tf_msg.child_frame_id = 'pos'
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation=transform.transform.rotation
  
                
                self.br.sendTransform(tf_msg)

        except Exception as e:
                self.get_logger().info(f"Error looking up or publishing transform: {str(e)}")

            
    
    def lookup_and_publish_transform(self):
            try:            
                tf_msg = TransformStamped()
                print(self.rvec)
                r,p,y,z=quaternion_from_euler(self.rvec1[1],self.rvec1[2],self.rvec1[0])
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'pos'
                tf_msg.child_frame_id = 'position_car'
                
                tf_msg.transform.translation.x = self.tvec[0][0][2]
                tf_msg.transform.translation.y = -self.tvec[0][0][0]
                tf_msg.transform.translation.z = self.tvec[0][0][1]
                
                tf_msg.transform.rotation.x = r
                tf_msg.transform.rotation.y = p
                tf_msg.transform.rotation.z = y
                tf_msg.transform.rotation.w = z
                # print(tf_msg)
                self.br.sendTransform(tf_msg)

            except Exception as e:
                print("Error",e)
                return

        

def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()