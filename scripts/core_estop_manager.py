#!/usr/bin/env python3

import rospy
from dbw_polaris_msgs.msg import BrakeCmd
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, Header


def activate_estop():
    global estop_is_activated
    # Avoid activating if already activated
    if not estop_is_activated:
        estop_state_pub.publish(Bool(data=True))
        software_state_pub.publish(Bool(data=True))
        if is_simulated:
            disable_pub.publish(Empty())  # Disable vehicle control using ROS messages
            twist_pub.publish(twist_msg)  # Publish zero velocity for simulation
        else:
            send_brakes()  # Send brakes for real vehicle

        rospy.loginfo("E-Stop: Activated")


def reset_estop():
    # NOTE: Only used when E-Stop is triggered by software and running sim (edge computer is not running)
    global estop_is_activated, software_button
    # Check software loops to make sure E-Stop is safe to reset
    estop_is_activated = False
    software_button = False
    rospy.loginfo("E-Stop: Reset")


def send_heartbeat(TimerEvent):
    if not is_simulated:  # Only send heartbeat if not in simulation
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


def check_heartbeat(TimerEvent):
    if not is_simulated:  # Only check heartbeat if not in simulation
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
                activate_estop()


def heartbeat_callback(msg):
    global time_last_heartbeat
    time_last_heartbeat = msg.stamp


def estop_state_callback(msg):
    global estop_is_activated
    estop_is_activated = msg.data


def estop_trigger_callback(msg):
    global software_button

    if is_simulated:
        software_button = True
        rospy.loginfo("E-Stop: Triggered by software button")
        activate_estop()


def reset_callback(msg):
    global software_button

    if estop_is_activated:  # Avoid resetting if already reset
        if software_button and is_simulated:  # Previously triggered by software and sim (edge computer is not running)
            software_button = False
            reset_estop()
        else:  # E-Stop is not triggered by software, so hardware reset is required
            rospy.loginfo("E-Stop (Core): Hardware e-stop reset required")


if __name__ == "__main__":
    try:
        rospy.init_node("core_estop_manager")

        # Variables -----------------------------------------------------------
        is_simulated = rospy.get_param("estop_is_simulated")

        estop_is_activated = False
        software_button = False

        time_last_heartbeat = rospy.Time(0)
        heartbeat_timeout = rospy.Duration(1 / 10)  # 10Hz
        heartbeat_rate = rospy.Duration(1 / 100)  # 100Hz

        # Define publishers ---------------------------------------------------
        estop_state_pub = rospy.Publisher("actor/estop/state", Bool, queue_size=1)
        software_state_pub = rospy.Publisher("actor/estop/software_button", Bool, queue_size=1)

        # Heartbeat - send heartbeat so that edge computer can verify connection with main computer
        heartbeat_pub = rospy.Publisher("actor/estop/heartbeat_core", Header, queue_size=1)
        rospy.Timer(heartbeat_rate, send_heartbeat)

        # Software Stopping via Brakes
        brakes_pub = rospy.Publisher("vehicle/brake_cmd", BrakeCmd, queue_size=1)
        disable_pub = rospy.Publisher("vehicle/disable", Empty, queue_size=1)
        twist_pub = rospy.Publisher("/actor/cmd_vel", Twist, queue_size=1)  # For soft estop only

        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.angular.z = 0

        # Define subscribers --------------------------------------------------
        # Heartbeat - verify connection with edge computer
        rospy.Subscriber("actor/estop/heartbeat_edge", Header, heartbeat_callback)
        rospy.Timer(heartbeat_timeout, check_heartbeat)

        # State
        rospy.Subscriber("actor/estop/state", Bool, estop_state_callback)

        # Triggers
        rospy.Subscriber("actor/estop/trigger", Empty, estop_trigger_callback)
        rospy.Subscriber("actor/estop/reset", Empty, reset_callback)

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
