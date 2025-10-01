#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import time

class RetargetingDebugger(Node):
    def __init__(self):
        super().__init__('retargeting_debugger')
        
        # Subscribe to all the topics the retargeting system uses
        self.pose_subscriptions = {}
        self.joy_subscriptions = {}
        
        # Pose topics
        pose_topics = [
            '/hrc/poses/waist',
            '/hrc/poses/left_elbow', 
            '/hrc/poses/left_wrist',
            '/hrc/poses/right_elbow',
            '/hrc/poses/right_wrist'
        ]
        
        # Joy topics
        joy_topics = [
            '/hrc/joys/left_wrist',
            '/hrc/joys/right_wrist'
        ]
        
        # Subscribe to pose topics
        for topic in pose_topics:
            self.pose_subscriptions[topic] = self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, t=topic: self.pose_callback(msg, t),
                10
            )
        
        # Subscribe to joy topics
        for topic in joy_topics:
            self.joy_subscriptions[topic] = self.create_subscription(
                Joy,
                topic,
                lambda msg, t=topic: self.joy_callback(msg, t),
                10
            )
        
        # Track last received messages
        self.last_pose_times = {}
        self.last_joy_times = {}
        self.last_joy_messages = {}
        
        self.get_logger().info("Retargeting Debugger Started!")
        self.get_logger().info("Monitoring topics:")
        for topic in pose_topics + joy_topics:
            self.get_logger().info(f"  - {topic}")
        
        # Create timer for status updates
        self.timer = self.create_timer(1.0, self.print_status)
    
    def pose_callback(self, msg, topic):
        self.last_pose_times[topic] = time.time()
        self.get_logger().info(f"📡 {topic}: pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
    
    def joy_callback(self, msg, topic):
        self.last_joy_times[topic] = time.time()
        self.last_joy_messages[topic] = msg
        
        # Check for button presses
        if len(msg.buttons) > 0:
            if msg.buttons[0]:  # Application Menu button
                self.get_logger().info(f"🎮 {topic}: Application Menu button PRESSED!")
            if len(msg.buttons) > 1 and msg.buttons[1]:  # Grip button
                self.get_logger().info(f"🎮 {topic}: Grip button PRESSED!")
            if len(msg.buttons) > 2 and msg.buttons[2]:  # Touchpad
                self.get_logger().info(f"🎮 {topic}: Touchpad PRESSED!")
        
        # Check trigger
        if len(msg.axes) > 2 and msg.axes[2] > 0.1:
            self.get_logger().info(f"🎮 {topic}: Trigger value = {msg.axes[2]:.2f}")
    
    def print_status(self):
        current_time = time.time()
        
        print("\n" + "="*60)
        print("RETARGETING SYSTEM STATUS")
        print("="*60)
        
        # Check pose topics
        print("\n📡 POSE TOPICS:")
        pose_topics = [
            '/hrc/poses/waist',
            '/hrc/poses/left_elbow', 
            '/hrc/poses/left_wrist',
            '/hrc/poses/right_elbow',
            '/hrc/poses/right_wrist'
        ]
        
        for topic in pose_topics:
            if topic in self.last_pose_times:
                age = current_time - self.last_pose_times[topic]
                status = "✅ ACTIVE" if age < 2.0 else "⚠️  STALE"
                print(f"  {topic}: {status} (last: {age:.1f}s ago)")
            else:
                print(f"  {topic}: ❌ NO DATA")
        
        # Check joy topics
        print("\n🎮 JOY TOPICS:")
        joy_topics = ['/hrc/joys/left_wrist', '/hrc/joys/right_wrist']
        
        for topic in joy_topics:
            if topic in self.last_joy_times:
                age = current_time - self.last_joy_times[topic]
                status = "✅ ACTIVE" if age < 2.0 else "⚠️  STALE"
                print(f"  {topic}: {status} (last: {age:.1f}s ago)")
                
                # Show button states
                if topic in self.last_joy_messages:
                    msg = self.last_joy_messages[topic]
                    if len(msg.buttons) > 0:
                        print(f"    Buttons: [0]={msg.buttons[0]} [1]={msg.buttons[1] if len(msg.buttons) > 1 else 'N/A'}")
                    if len(msg.axes) > 2:
                        print(f"    Trigger: {msg.axes[2]:.2f}")
            else:
                print(f"  {topic}: ❌ NO DATA")
        
        # System readiness check
        print("\n🔍 SYSTEM READINESS:")
        required_poses = ['/hrc/poses/waist', '/hrc/poses/left_elbow', '/hrc/poses/left_wrist', 
                         '/hrc/poses/right_elbow', '/hrc/poses/right_wrist']
        required_joys = ['/hrc/joys/left_wrist', '/hrc/joys/right_wrist']
        
        poses_ready = all(topic in self.last_pose_times and (current_time - self.last_pose_times[topic]) < 2.0 
                         for topic in required_poses)
        joys_ready = all(topic in self.last_joy_times and (current_time - self.last_joy_times[topic]) < 2.0 
                        for topic in required_joys)
        
        print(f"  Poses: {'✅ READY' if poses_ready else '❌ NOT READY'}")
        print(f"  Joys:  {'✅ READY' if joys_ready else '❌ NOT READY'}")
        print(f"  Overall: {'✅ READY FOR RETARGETING' if poses_ready and joys_ready else '❌ NOT READY'}")
        
        print("\n💡 TROUBLESHOOTING:")
        if not poses_ready:
            print("  - Start PublishPoseViveTracker.py")
            print("  - Check Vive trackers are connected and tracked")
        if not joys_ready:
            print("  - Check Vive controllers are connected")
            print("  - Verify controller mapping in PublishPoseViveTracker.py")
        if poses_ready and joys_ready:
            print("  - Press Button B on RIGHT controller to ENABLE retargeting")
            print("  - Press Button B on LEFT controller to DISABLE retargeting")
            print("  - Check mc_mujoco HRC tab for status")

def main(args=None):
    rclpy.init(args=args)
    node = RetargetingDebugger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


