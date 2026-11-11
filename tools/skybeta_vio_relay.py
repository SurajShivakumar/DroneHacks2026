"""Sim-only adapter: our estimator outputs -> the organizer's cuVSLAM topic contract.

On the real Sky Beta stack, isaac_ros_visual_slam publishes
  /visual_slam/tracking/odometry  nav_msgs/Odometry, frame 'odom' (ENU, origin and
                                  heading = rig pose at startup), child 'base_link'
                                  (FLU at the Pixhawk IMU), twist in body FLU
  /visual_slam/status             VisualSlamStatus, vo_state 1 tracking / 2 lost
and the organizer's px4_odom_bridge forwards that to PX4.

In our Gazebo sim the estimator (OpenVINS stand-in, or tools/cuvslam_vo_node.py)
publishes /vio_shadow/odomimu with the same semantics: gravity-up world with an
arbitrary heading fixed at startup, child = IMU frame (FLU, at the body origin in
the sim), twist in IMU-local FLU. This node relabels frames and adds status so the
organizer bridge and our competitor node see exactly the real contract.
It never publishes to /fmu/in/*.
"""
import argparse


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', default='/vio_shadow/odomimu')
    p.add_argument('--stale-s', type=float, default=.25)
    a = p.parse_args()

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

    rclpy.init()
    node = rclpy.create_node('skybeta_vio_relay')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    reliable = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10)
    pub = node.create_publisher(Odometry, '/visual_slam/tracking/odometry', reliable)
    pub_status = node.create_publisher(VisualSlamStatus, '/visual_slam/status', reliable)
    last = {'t': None}

    def on_odom(msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id, out.child_frame_id = 'odom', 'base_link'
        out.pose, out.twist = msg.pose, msg.twist
        pub.publish(out)
        last['t'] = node.get_clock().now().nanoseconds*1e-9

    def tick():
        now = node.get_clock().now().nanoseconds*1e-9
        s = VisualSlamStatus()
        s.vo_state = 1 if last['t'] is not None and now-last['t'] <= a.stale_s else (0 if last['t'] is None else 2)
        pub_status.publish(s)

    node.create_subscription(Odometry, a.input, on_odom, qos_profile_sensor_data)
    node.create_timer(.1, tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
