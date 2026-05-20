#!/usr/bin/env python3
"""
YOLO Object Detection Node for ROS2
Provides object detection service and forwards grasp requests to GraspNet
"""

import os
import sys
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLOE
import threading
import time
import message_filters

# Import custom service messages
try:
    from graspnet_msgs.srv import ObjectDetectionRequest
    from graspnet_msgs.msg import DetectedObject, DetectedObjects
except Exception as e:
    print("[!] Missing ROS2 service types 'graspnet_msgs/ObjectDetectionRequest'.")
    print("    Please build the graspnet_msgs package before running:")
    print("    1) cd robonix/driver/graspnet && bash build.sh")
    print("    2) source install/setup.bash")
    raise e


class YOLODetectionNode(Node):
    """ROS2 node for YOLO-based object detection with GraspNet integration."""
    
    def __init__(self, model_path=None):
        super().__init__('yolo_detection_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Camera data (synchronized)
        self.latest_color_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.data_lock = threading.Lock()
        
        # Load YOLOE model
        if model_path is None:
            # Try to find custom model in current directory or vision skill directory
            script_dir = os.path.dirname(os.path.abspath(__file__))
            # Try current directory first
            custom_model_path = os.path.join(script_dir, "yoloe-11l-seg-pf.pt")
            
            if os.path.exists(custom_model_path):
                model_path = custom_model_path
                self.get_logger().info(f'Using custom YOLOE model: {model_path}')
            else:
                # Will download automatically if not present
                model_path = 'yoloe-11l-seg-pf.pt'
                self.get_logger().info('Using default YOLOE model (will download if needed)')
        
        self.get_logger().info(f'Loading YOLOE model from: {model_path}')
        try:
            self.yolo_model = YOLOE(model_path)
            self.get_logger().info('YOLOE model loaded successfully')
            self.get_logger().info('Model supports prompt-free detection with 4585 predefined classes')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLOE model: {e}')
            self.get_logger().error('Please ensure ultralytics is installed: pip install ultralytics')
            raise e
        
        # Subscribe to camera topics with message_filters for synchronization
        self.sub_color = message_filters.Subscriber(
            self,
            Image,
            '/camera/color/image_raw')
        
        self.sub_depth = message_filters.Subscriber(
            self,
            Image,
            '/camera/depth/image_raw')
        
        self.sub_camera_info = message_filters.Subscriber(
            self,
            CameraInfo,
            '/camera/color/camera_info')
        
        # Synchronize messages
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth, self.sub_camera_info],
            queue_size=10,
            slop=0.1)  # 100ms tolerance
        self.sync.registerCallback(self.camera_callback)
        
        # Create object detection service
        self.detection_srv = self.create_service(
            ObjectDetectionRequest,
            '/yolo/detect_object',
            self.handle_detection_request)
        
        # Create publisher for annotated detection image
        self.detection_image_pub = self.create_publisher(
            Image,
            '/yolo/detection_image',
            10)
        
        # Create publisher for detected objects
        self.detected_objects_pub = self.create_publisher(
            DetectedObjects,
            '/yolo/detect_objects',
            10)
        
        # Create timer for periodic detection (10Hz)
        self.detection_timer = self.create_timer(
            1,  # 10Hz = 0.1 seconds
            self.periodic_detection_callback)
        
        self.get_logger().info('[*] YOLO Detection Node started')
        self.get_logger().info('[*] Service available at: /yolo/detect_object')
        self.get_logger().info('[*] Detection image topic: /yolo/detection_image')
        self.get_logger().info('[*] Detected objects topic: /yolo/detect_objects (10Hz)')

    def check_msg(self, msg, name="image"):
        """Check ROS Image message integrity."""
        try:
            data_len = len(msg.data)
            expected = msg.step * msg.height

            print(f"\n[{name}] encoding={msg.encoding}")
            print(f"[{name}] width={msg.width}, height={msg.height}, step={msg.step}")
            print(f"[{name}] data_len={data_len}, expected={expected}")

            # Empty or invalid size
            if msg.width == 0 or msg.height == 0:
                print(f"ERROR: {name} has zero width/height!")

            # Inconsistent buffer length
            if data_len != expected:
                print(f"ERROR: {name} buffer mismatch! (data_len={data_len}, expected={expected})")

        except Exception as e:
            print(f"check_msg({name}) exception: {e}")

    
    def camera_callback(self, color_msg, depth_msg, camera_info_msg):
        try:
            # self.check_msg(color_msg, "color_msg")
            # self.check_msg(depth_msg, "depth_msg")

            # 1) DON'T ask cv_bridge to convert rgb8 -> bgr8
            color_image = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding='passthrough'
            )
            # color_image is RGB already because msg.encoding = rgb8
            # If your model wants RGB, you're done.
            # If you want BGR for OpenCV usage, do:
            # color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding='passthrough'
            )

            with self.data_lock:
                self.latest_color_image = color_image
                self.latest_depth_image = depth_image
                self.latest_camera_info = camera_info_msg

        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')


    
    def handle_detection_request(self, request, response):
        """Handle object detection service request."""
        object_name = request.object_name
        self.get_logger().info(f'[*] Received detection request for object: {object_name}')
        
        try:
            # Get current camera data (synchronized)
            with self.data_lock:
                if self.latest_color_image is None or self.latest_depth_image is None or self.latest_camera_info is None:
                    response.success = False
                    response.message = 'Camera data not available (waiting for synchronized messages)'
                    self.get_logger().error(response.message)
                    return response
                
                color_img = self.latest_color_image.copy()
                depth_img = self.latest_depth_image.copy()
                cam_info = self.latest_camera_info
            
            # Detect object using YOLO
            detection_result = self.detect_object(object_name, color_img, depth_img, cam_info)
            
            # Publish annotated image if available (always publish, even if object_name filtering fails)
            if detection_result['annotated_image'] is not None:
                try:
                    # YOLO's plot() returns BGR image, directly convert to ROS message
                    annotated_msg = self.bridge.cv2_to_imgmsg(detection_result['annotated_image'], encoding='bgr8')
                    annotated_msg.header.stamp = self.get_clock().now().to_msg()
                    annotated_msg.header.frame_id = 'camera_color_optical_frame'
                    self.detection_image_pub.publish(annotated_msg)
                    self.get_logger().info('[*] Published annotated detection image to /yolo/detection_image')
                except Exception as e:
                    self.get_logger().warning(f'Failed to publish annotated image: {e}')
            
            if not detection_result['success']:
                response.success = False
                response.message = detection_result['message']
                self.get_logger().warning(f'Detection failed: {response.message}')
                return response
            
            # Populate detection response
            response.bbox_2d = detection_result['bbox_2d']
            response.confidence = detection_result['confidence']
            
            # Calculate 3D center of the object
            object_center_3d = self.calculate_object_center_3d(
                response.bbox_2d, depth_img, cam_info)
            
            if object_center_3d is not None:
                response.object_center_3d = list(object_center_3d)
                self.get_logger().info(f'[*] Object 3D center: [{object_center_3d[0]:.3f}, {object_center_3d[1]:.3f}, {object_center_3d[2]:.3f}]m')
            else:
                response.object_center_3d = []
                self.get_logger().warning('[*] Failed to calculate object 3D center')
            
            response.success = True
            response.message = f"Object '{object_name}' detected successfully"
            
            self.get_logger().info(f'[*] Object detected: {object_name}, confidence: {response.confidence:.3f}')
            self.get_logger().info(f'[*] 2D bbox: {response.bbox_2d}')
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f'Error during detection: {str(e)}'
            self.get_logger().error(response.message)
            import traceback
            self.get_logger().error(traceback.format_exc())
            return response
    
    def detect_object(self, object_name, color_img, depth_img, cam_info):
        """Detect specific object and return its 2D bounding box."""
        result = {
            'success': False,
            'message': '',
            'bbox_2d': [],
            'confidence': 0.0,
            'annotated_image': None
        }
        
        try:
            # Run YOLOE inference (detects all predefined classes)
            self.get_logger().info('[*] Running YOLOE inference...')
            results = self.yolo_model.predict(source=color_img, device="cuda:0", verbose=False)
            detection = results[0]
            
            # Get annotated image from YOLO (with all detections drawn)
            # The plot() method returns the image with bounding boxes and labels
            annotated_img = detection.plot()  # Returns BGR image with annotations
            result['annotated_image'] = annotated_img

            save_dir = "/home/syswonder/lgw/robonix/robonix/driver/yoloe"
            filepath = os.path.join(save_dir, "saved_image.jpg")
            cv2.imwrite(filepath, annotated_img)
            self.get_logger().info(f'[✓] Annotated image saved to: {filepath}')
            
            if detection is None or len(detection.boxes) == 0:
                result['message'] = f"No objects detected in image"
                return result
            
            # Extract detection results
            boxes = detection.boxes.xyxy.cpu().numpy()
            confidences = detection.boxes.conf.cpu().numpy()
            classes = detection.boxes.cls.cpu().numpy()
            
            # First filter: confidence >= 0.3
            self.get_logger().info(f'[*] Total detections: {len(boxes)}')
            high_conf_indices = []
            high_conf_objects = []
            
            for i in range(len(boxes)):
                conf = float(confidences[i])
                if conf >= 0.2:
                    detected_name = detection.names[int(classes[i])]
                    high_conf_indices.append(i)
                    high_conf_objects.append({
                        'name': detected_name,
                        'confidence': conf,
                        'bbox': boxes[i]
                    })
            
            # Print all high-confidence objects
            self.get_logger().info(f'[*] Objects with confidence >= 0.3: {len(high_conf_objects)}')
            for idx, obj in enumerate(high_conf_objects):
                self.get_logger().info(f'    [{idx+1}] {obj["name"]} (conf: {obj["confidence"]:.3f})')
            
            if len(high_conf_indices) == 0:
                result['message'] = 'No objects detected with confidence >= 0.3'
                return result
            
            # Second filter: match object_name from high-confidence detections
            object_name_lower = object_name.lower().strip()
            matching_indices = []
            matching_confidences = []
            
            for i in high_conf_indices:
                detected_name = detection.names[int(classes[i])]
                detected_name_lower = detected_name.lower()
                conf = float(confidences[i])
                
                # Check if detected name matches requested object name
                # Support exact match, substring match, or partial word match
                if (object_name_lower == detected_name_lower or
                    object_name_lower in detected_name_lower or
                    detected_name_lower in object_name_lower):
                    matching_indices.append(i)
                    matching_confidences.append(conf)
                    self.get_logger().info(f'[*] Found match: "{detected_name}" (confidence: {conf:.3f}) for requested "{object_name}"')
            
            if len(matching_indices) == 0:
                result['message'] = f"Object '{object_name}' not found in high-confidence detections"
                return result
            
            # Get the detection with highest confidence among matches
            best_match_idx = matching_confidences.index(max(matching_confidences))
            best_idx = matching_indices[best_match_idx]
            best_conf = matching_confidences[best_match_idx]
            matched_name = detection.names[int(classes[best_idx])]
            
            self.get_logger().info(f'[*] Found {len(matching_indices)} matching instance(s) for "{object_name}", best: "{matched_name}" (confidence: {best_conf:.3f})')
            
            # Get 2D bounding box in pixel coordinates
            x1, y1, x2, y2 = boxes[best_idx]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            result['success'] = True
            result['message'] = f"Object '{object_name}' detected successfully (matched as '{matched_name}')"
            result['bbox_2d'] = [float(x1), float(y1), float(x2), float(y2)]
            result['confidence'] = float(best_conf)
            
            self.get_logger().info(f'[*] 2D bbox (pixels): x=[{x1}, {x2}], y=[{y1}, {y2}]')
            
            return result
            
        except Exception as e:
            result['message'] = f'Detection error: {str(e)}'
            return result
    
    def calculate_object_center_3d(self, bbox_2d, depth_img, cam_info):
        """Calculate 3D center position of object from 2D bbox and depth image."""
        try:
            x_min, y_min, x_max, y_max = [int(v) for v in bbox_2d]
            
            # Extract depth values in the bounding box region
            depth_roi = depth_img[y_min:y_max, x_min:x_max]
            
            # Filter out invalid depth values (0 or too far)
            valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < 3000)]  # depth in mm, max 3m
            
            if len(valid_depths) == 0:
                self.get_logger().warning(f'No valid depth values in object bounding box {x_min, y_min, x_max, y_max}')
                return None
            
            # Calculate median depth (more robust than mean)
            depth_median = np.median(valid_depths) / 1000.0  # Convert mm to meters
            
            # Calculate center pixel coordinates
            center_x = (x_min + x_max) / 2.0
            center_y = (y_min + y_max) / 2.0
            
            # Get camera intrinsics
            K = cam_info.k
            fx, fy = K[0], K[4]
            cx, cy = K[2], K[5]
            
            # Back-project 2D center to 3D using median depth
            # X = (u - cx) * Z / fx
            # Y = (v - cy) * Z / fy
            # Z = depth
            x_3d = (center_x - cx) * depth_median / fx
            y_3d = (center_y - cy) * depth_median / fy
            z_3d = depth_median
            
            return [x_3d, y_3d, z_3d]
            
        except Exception as e:
            self.get_logger().error(f'Error calculating 3D center: {e}')
            return None
    
    def periodic_detection_callback(self):
        """Periodic callback to detect all objects and publish to topic."""
        try:
            # Get current camera data (synchronized)
            with self.data_lock:
                if self.latest_color_image is None or self.latest_depth_image is None or self.latest_camera_info is None:
                    # Silently skip if camera data not available yet
                    return
                
                color_img = self.latest_color_image.copy()
                depth_img = self.latest_depth_image.copy()
                cam_info = self.latest_camera_info
            
            # Detect all objects using YOLO
            detection_result = self.detect_all_objects(color_img, depth_img, cam_info)
            
            if not detection_result['success']:
                # No objects detected, publish empty message
                msg = DetectedObjects()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_color_optical_frame'
                msg.objects = []
                self.detected_objects_pub.publish(msg)
                return
            
            # Create DetectedObjects message
            msg = DetectedObjects()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_color_optical_frame'
            
            # Populate detected objects
            for obj_info in detection_result['objects']:
                detected_obj = DetectedObject()
                detected_obj.object_name = obj_info['name']
                detected_obj.bbox_2d = obj_info['bbox_2d']
                detected_obj.confidence = obj_info['confidence']
                
                # Calculate 3D center
                object_center_3d = self.calculate_object_center_3d(
                    obj_info['bbox_2d'], depth_img, cam_info)
                
                if object_center_3d is not None:
                    detected_obj.object_center_3d = object_center_3d
                else:
                    detected_obj.object_center_3d = []
                
                msg.objects.append(detected_obj)
            
            # Publish detected objects
            self.detected_objects_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in periodic detection: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def detect_all_objects(self, color_img, depth_img, cam_info):
        """Detect all objects in the image and return their information."""
        result = {
            'success': False,
            'message': '',
            'objects': []
        }
        
        try:
            # Run YOLOE inference (detects all predefined classes)
            results = self.yolo_model.predict(source=color_img, device="cuda:0", verbose=False)
            detection = results[0]
            
            if detection is None or len(detection.boxes) == 0:
                result['message'] = "No objects detected in image"
                return result
            
            # Extract detection results
            boxes = detection.boxes.xyxy.cpu().numpy()
            confidences = detection.boxes.conf.cpu().numpy()
            classes = detection.boxes.cls.cpu().numpy()
            
            # Filter detections by confidence >= 0.2
            for i in range(len(boxes)):
                conf = float(confidences[i])
                if conf >= 0.2:
                    detected_name = detection.names[int(classes[i])]
                    x1, y1, x2, y2 = boxes[i]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    
                    obj_info = {
                        'name': detected_name,
                        'confidence': float(conf),
                        'bbox_2d': [float(x1), float(y1), float(x2), float(y2)]
                    }
                    result['objects'].append(obj_info)
            
            if len(result['objects']) > 0:
                result['success'] = True
                result['message'] = f"Detected {len(result['objects'])} objects"
            else:
                result['message'] = "No objects detected with confidence >= 0.2"
            
            return result
            
        except Exception as e:
            result['message'] = f'Detection error: {str(e)}'
            return result
    


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = YOLODetectionNode()
        node.get_logger().info('[*] YOLO Detection Node ready, spinning...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[*] Shutting down...")
    except Exception as e:
        print(f"[!] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

