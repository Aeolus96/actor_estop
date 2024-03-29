#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Header, Empty
from dbw_polaris_msgs.msg import BrakeCmd


def send_heartbeat():
    msg = Header()
    msg.stamp = rospy.Time.now()
    heartbeat_pub.publish(msg)


def send_brakes():
    msg = BrakeCmd()
    msg.enable = True
    msg.pedal_cmd = 0.0
    msg.pedal_cmd_type = BrakeCmd.CMD_PERCENT

    first_time = rospy.Time.now()
    brake_timeout = rospy.Duration(5)  # seconds
    rate = rospy.Rate(50)  # Hz

    rospy.loginfo("E-Stop: Sending brakes")
    while (rospy.Time.now() - first_time) < brake_timeout:
        brakes_pub.publish(msg)
        msg.pedal_cmd += 0.005  # Increase pedal value
        msg.pedal_cmd = min(msg.pedal_cmd, 0.4)  # 0.4 is maximum brake value. Prevents motor stall
        rospy.loginfo(f"E-Stop: Sending brakes {msg.pedal_cmd:8.2f}")
        rate.sleep()

    msg.pedal_cmd = 0.2  # Release some brake pressure
    brakes_pub.publish(msg)
    disable_pub.publish(Empty())  # Disable vehicle control using ROS messages


def check_heartbeat():
    global time_last_heartbeat, heartbeat_timeout
    # Check if received heartbeat is within timeout
    if rospy.Time.now() - time_last_heartbeat > heartbeat_timeout:
        # Edge node is not running as expected. Let other nodes know that E-Stop is triggered
        if not estop_is_activated:  # Avoid activating if already activated
            msg = Bool()
            msg.data = True
            estop_state_pub.publish(msg)
            software_state_pub.publish(msg)
            rospy.loginfo("E-Stop: Triggered due to heartbeat timeout")
            send_brakes()


def heartbeat_callback(msg):
    global time_last_heartbeat
    time_last_heartbeat = msg.stamp


def estop_state_callback(msg):
    global estop_is_activated
    estop_is_activated = msg.data


if __name__ == "__main__":
    try:
        rospy.init_node("core_estop_manager")

        # Variables -----------------------------------------------------------
        estop_is_activated = False

        time_last_heartbeat = rospy.Time(0)
        heartbeat_timeout = rospy.Duration(1 / 10)  # 10Hz
        heartbeat_rate = rospy.Duration(1 / 100)  # 100Hz

        # Define publishers ---------------------------------------------------
        estop_state_pub = rospy.Publisher("actor/e_stop/state", Bool, queue_size=1)
        software_state_pub = rospy.Publisher("actor/e_stop/software_button", Bool, queue_size=1)
        # Heartbeat - send heartbeat so that edge computer can verify connection with main computer
        heartbeat_pub = rospy.Publisher("actor/e_stop/heartbeat_core", Header, queue_size=1)
        rospy.Timer(heartbeat_rate, send_heartbeat)
        # Software Stopping via Brakes
        brakes_pub = rospy.Publisher("vehicle/brake_cmd", BrakeCmd, queue_size=1)
        disable_pub = rospy.Publisher("vehicle/disable", Empty, queue_size=1)

        # Define subscribers --------------------------------------------------
        # Heartbeat - verify connection with edge computer
        rospy.Subscriber("actor/estop/heartbeat_edge", Header, heartbeat_callback)
        rospy.Timer(heartbeat_timeout, check_heartbeat)
        # State
        rospy.Subscriber("actor/estop/state", Bool, estop_state_callback)

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
