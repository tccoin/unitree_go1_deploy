#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import roslibpy
import lcm

# ---------------------------------------------------------------------------
from rc_command_lcmt import rc_command_lcmt        # <- LCM message type
# ---------------------------------------------------------------------------

LCM_URL = "udpm://239.255.76.67:7667?ttl=255"
# LCM_CHANNEL = "rc_command"

class VelLCMBridge(Node):
    def __init__(self):
        super().__init__('vel_lcm_bridge')

        # ----------------- LCM --------------------------------------------
        self.lc = lcm.LCM(LCM_URL)

        # ----------------- rosbridge websocket ----------------------------
        self.ros_client = roslibpy.Ros(host='141.212.194.240', port=9090)
        try:
            self.ros_client.run()
        except Exception as e:
            self.get_logger().error(f'Unable to connect to rosbridge: {e}')
            raise

        # ----------------- ROS topic subscription -------------------------
        self.topic_vel = roslibpy.Topic(
            self.ros_client,
            '/cmd_vel',
            'geometry_msgs/Twist',
            queue_length=1,
            throttle_rate=0
        )
        self.topic_vel.subscribe(self.cb_vel)

        self.get_logger().info(
            'VelLCMBridge initialised - forwarding /cmd_vel → LCM [%s]' 
        )

    # ---------------------------------------------------------------------
    # rosbridge callback
    # ---------------------------------------------------------------------
    def cb_vel(self, msg):
        """
        msg is a plain Python dict from roslibpy, shaped as:
        {
            'linear':  {'x': float, 'y': float, 'z': float},
            'angular': {'x': float, 'y': float, 'z': float}
        }
        """
        lcm_msg = rc_command_lcmt()

        # Fill mandatory fields
        lcm_msg.mode = 0
        lcm_msg.left_stick  = [0.0, 0.0]
        lcm_msg.right_stick = [0.0, 0.0]
        lcm_msg.knobs       = [0.0, 0.0]

        lcm_msg.left_upper_switch        = 0
        lcm_msg.left_lower_left_switch   = 0
        lcm_msg.left_lower_right_switch  = 0
        lcm_msg.right_upper_switch       = 0
        lcm_msg.right_lower_left_switch  = 0
        lcm_msg.right_lower_right_switch = 0

        # ---------------- velocity mapping -------------------------------
        # left stick 1  → linear.x (forward)
        # left stick 0  → linear.y (lateral)
        # right stick 0 → angular.z (yaw)
        lcm_msg.left_stick[1]  = float(msg['linear']['x'])
        lcm_msg.left_stick[0]  = float(msg['linear']['y'])
        lcm_msg.right_stick[0] = float(msg['angular']['z'])

        # -----------------------------------------------------------------
        print(lcm_msg.left_stick)
        self.lc.publish("rc_command_relay", lcm_msg.encode())

        # Debug print (optional; comment out if spammy)
        self.get_logger().debug(
            f"LCM tx – lin:({lcm_msg.left_stick[1]:.3f},{lcm_msg.left_stick[0]:.3f})  "
            f"yaw:{lcm_msg.right_stick[0]:.3f}"
        )

    # ---------------------------------------------------------------------
    # clean shutdown
    # ---------------------------------------------------------------------
    def destroy_node(self):
        self.topic_vel.unsubscribe()
        self.ros_client.terminate()
        super().destroy_node()

# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VelLCMBridge()
    try:
        rclpy.spin(node)          # Spin *this* rclpy node (keeps logger alive)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
