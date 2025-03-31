#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class TTCAnalyzerNode(Node):
    def __init__(self):
        super().__init__('ttc_analyzer_node')
        
        # Parameters
        self.declare_parameter('display_flow', True)  # Enabled by default
        self.declare_parameter('warning_ttc', 2.0)    # seconds
        self.declare_parameter('critical_ttc', 1.0)   # seconds
        self.declare_parameter('process_every_n_frames', 1)  # Process every frame
        
        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/front_camera',  # Update with your camera topic
            self.image_callback,
            10)
        
        # Publisher for TTC results
        self.ttc_pub = self.create_publisher(
            Float32,
            '/ttc_estimation',
            10)
        
        # Visualization publisher
        self.flow_pub = self.create_publisher(
            Image,
            '/optical_flow_visualization',
            1)  # Small queue size for visualization
        
        self.bridge = CvBridge()
        self.prev_frame = None
        self.prev_time = None
        self.frame_count = 0
        
        self.get_logger().info("TTC Analyzer Node initialized with visualization")

    def image_callback(self, msg):
        try:
            # Process every nth frame if needed (for performance)
            self.frame_count += 1
            if (self.frame_count % self.get_parameter('process_every_n_frames').value) != 0:
                return
            
            current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            if self.prev_frame is not None:
                # Calculate time difference
                delta_time = current_time - self.prev_time
                if delta_time < 0.001:  # Avoid division by zero
                    return
                
                # Calculate optical flow
                flow = self.calculate_optical_flow(self.prev_frame, current_frame)
                
                # Calculate TTC
                ttc = self.calculate_ttc(flow, delta_time)
                
                # Publish TTC
                ttc_msg = Float32()
                ttc_msg.data = float(ttc)
                self.ttc_pub.publish(ttc_msg)
                
                # Log warnings if TTC is low
                if ttc < self.get_parameter('critical_ttc').value:
                    self.get_logger().error(f"CRITICAL TTC: {ttc:.2f}s")
                elif ttc < self.get_parameter('warning_ttc').value:
                    self.get_logger().warning(f"Warning TTC: {ttc:.2f}s")
                
                # Visualize if enabled
                if self.get_parameter('display_flow').value:
                    try:
                        flow_img = self.visualize_optical_flow(current_frame, flow)
                        flow_msg = self.bridge.cv2_to_imgmsg(flow_img, encoding="bgr8")
                        flow_msg.header = msg.header  # Maintain original header
                        self.flow_pub.publish(flow_msg)
                    except Exception as e:
                        self.get_logger().error(f"Visualization error: {str(e)}", throttle_duration_sec=1)
            
            self.prev_frame = current_frame
            self.prev_time = current_time
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def calculate_optical_flow(self, prev_frame, curr_frame):
        # Convert to grayscale
        prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)
        curr_gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        prev_gray = cv2.GaussianBlur(prev_gray, (15, 15), 0)
        curr_gray = cv2.GaussianBlur(curr_gray, (15, 15), 0)
        
        # Calculate dense optical flow
        flow = cv2.calcOpticalFlowFarneback(
            prev_gray, curr_gray,
            None,
            pyr_scale=0.5,
            levels=3,
            winsize=25,  # Increased window size for more stable flow
            iterations=3,
            poly_n=5,
            poly_sigma=1.2,
            flags=cv2.OPTFLOW_FARNEBACK_GAUSSIAN
        )
        return flow

    def calculate_ttc(self, flow, delta_time):
        # Focus on center region where collision likely occurs
        h, w = flow.shape[:2]
        roi = flow[h//4:3*h//4, w//4:3*w//4]  # Center 50% of image
        
        # Calculate flow magnitudes
        fx, fy = roi[..., 0], roi[..., 1]
        magnitude = np.sqrt(fx**2 + fy**2)
        
        # Filter out small movements (noise)
        magnitude = magnitude[magnitude > 0.5]
        
        if len(magnitude) == 0:
            return float('inf')
        
        # Calculate median flow (more robust than mean)
        median_magnitude = np.median(magnitude)
        
        # Avoid division by zero
        if median_magnitude < 1e-6:
            return float('inf')
        
        # TTC calculation
        return 1.0 / (median_magnitude * delta_time)

    def visualize_optical_flow(self, frame, flow):
        h, w = frame.shape[:2]
        
        # Create HSV representation
        fx, fy = flow[..., 0], flow[..., 1]
        magnitude, angle = cv2.cartToPolar(fx, fy, angleInDegrees=True)
        
        # Normalize magnitude for better visualization
        magnitude = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX)
        
        # Create HSV image
        hsv = np.zeros((h, w, 3), dtype=np.uint8)
        hsv[..., 0] = angle / 2  # Hue (0-180)
        hsv[..., 1] = 255        # Saturation
        hsv[..., 2] = np.uint8(magnitude)  # Value
        
        # Convert to BGR
        flow_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
        
        # Create output image (50% original + 50% flow)
        output = cv2.addWeighted(frame, 0.5, flow_bgr, 0.5, 0)
        
        # Draw some flow vectors for clarity
        step = 20
        for y in range(0, h, step):
            for x in range(0, w, step):
                dx, dy = flow[y, x]
                if abs(dx) > 1 or abs(dy) > 1:  # Only draw significant flow
                    cv2.arrowedLine(output, (x, y), 
                                   (int(x + dx*2), int(y + dy*2)), 
                                   (0, 255, 255), 1, tipLength=0.3)
        
        # Add info text
        avg_magnitude = np.mean(magnitude)
        ttc = 1.0 / (avg_magnitude * (1.0/30)) if avg_magnitude > 0 else float('inf')
        cv2.putText(output, f"TTC: {ttc:.1f}s", (20, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(output, "Optical Flow Visualization", (20, h-20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return output

def main(args=None):
    rclpy.init(args=args)
    ttc_analyzer = TTCAnalyzerNode()
    
    try:
        rclpy.spin(ttc_analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        ttc_analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()