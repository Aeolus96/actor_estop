#!/usr/bin/env python3
import rospy
from gpiozero import LED, Button
from std_msgs.msg import Bool, Empty, Header
from dbw_polaris_msgs.msg import BrakeCmd


# Initialize GPIO devices
PHYSICAL_BUTTON_PIN = 17
WIRELESS_BUTTON_PIN = 27
ESTOP_RELAY_PIN = 22
FLASHING_LIGHT_PIN = 23

button_loop = Button(PHYSICAL_BUTTON_PIN, pull_up=True)  # physical buttons loop
wireless_loop = Button(WIRELESS_BUTTON_PIN, pull_up=True)  # wireless buttons loop
estop_relay = LED(ESTOP_RELAY_PIN, active_high=True)  # relay board
flashing_lights_relay = LED(FLASHING_LIGHT_PIN, active_high=True)  # relay board
# NOTE: relay is meant to powered on when E-Stop is not triggered. If the system is powered off, estop is triggered


def activate_e_stop():
    global estop_is_activated
    # Avoid activating if already activated
    if not estop_is_activated:
        estop_is_activated = True
        if soft_estop_only:
            send_brakes()
        else:
            estop_relay.off()
        rospy.loginfo("E-Stop: Activated")


def reset_e_stop():
    global estop_is_activated
    # Check all loops to make sure E-Stop is safe to reset
    if not phyiscal_button and not wireless_button and not software_button:
        estop_is_activated = False
        if soft_estop_only:
            pass  # No need to reset brakes, they will timeout after the send_brakes() call
        else:
            estop_relay.on()
        rospy.loginfo("E-Stop: Reset")


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


def send_states():
    msg = Bool()
    msg.data = estop_is_activated
    estop_state_pub.publish(msg)
    msg.data = phyiscal_button
    physical_button_state_pub.publish(msg)
    msg.data = wireless_button
    wireless_state_pub.publish(msg)
    msg.data = software_button
    software_state_pub.publish(msg)


def check_heartbeat():
    global time_last_heartbeat, heartbeat_timeout, software_button
    # Check if received heartbeat is within timeout
    if rospy.Time.now() - time_last_heartbeat > heartbeat_timeout:
        if not estop_is_activated:  # Avoid activating if already activated
            software_button = True
            activate_e_stop()

        rospy.loginfo("E-Stop: Triggered due to heartbeat timeout")


def heartbeat_callback(msg):
    global time_last_heartbeat
    time_last_heartbeat = msg.stamp


def physical_button(activated):
    global phyiscal_button
    if activated:
        phyiscal_button = True
        activate_e_stop()
        rospy.loginfo("E-Stop: Triggered due to physical button press")
    else:
        phyiscal_button = False
        reset_e_stop()
        rospy.loginfo("E-Stop: Reset due to physical button release")


def wireless_button(activated):
    global wireless_button
    if activated:
        wireless_button = True
        activate_e_stop()
        rospy.loginfo("E-Stop: Triggered due to wireless button press")
    else:
        wireless_button = False
        reset_e_stop()
        rospy.loginfo("E-Stop: Reset due to wireless button release")


def trigger_callback(msg):
    global software_button
    if not estop_is_activated:  # Avoid activating if already activated
        software_button = True
        activate_e_stop()


def reset_callback(msg):
    global software_button
    if estop_is_activated:  # Avoid resetting if already reset
        if software_button:
            software_button = False
            reset_e_stop()
        else:  # E-Stop is not triggered by software, so hardware reset is required
            rospy.loginfo("E-Stop: Hardware e-stop reset required")


def dbw_state_callback(msg):
    flashing_lights_relay.on() if msg.data else flashing_lights_relay.off()


if __name__ == "__main__":
    try:
        rospy.init_node("edge_estop_manager")

        # Variables -----------------------------------------------------------
        soft_estop_only = False  # set to True if only soft estop is being used
        estop_is_activated = False
        phyiscal_button = False
        wireless_button = False
        software_button = False

        time_last_heartbeat = rospy.Time(0)
        heartbeat_timeout = rospy.Duration(1 / 10)  # 10Hz
        heartbeat_rate = rospy.Duration(1 / 100)  # 100Hz

        # Define publishers ---------------------------------------------------
        # States
        estop_state_pub = rospy.Publisher("actor/e_stop/state", Bool, queue_size=1)
        physical_button_state_pub = rospy.Publisher("actor/e_stop/physical_button", Bool, queue_size=1)
        wireless_state_pub = rospy.Publisher("actor/e_stop/wireless_button", Bool, queue_size=1)
        software_state_pub = rospy.Publisher("actor/e_stop/software_button", Bool, queue_size=1)
        rospy.Timer(heartbeat_rate, send_states)
        # Heartbeat - send heartbeat so that main computer can verify connection with edge computer
        heartbeat_pub = rospy.Publisher("actor/e_stop/heartbeat_edge", Header, queue_size=1)
        rospy.Timer(heartbeat_rate, send_heartbeat)
        # Software Stopping via Brakes
        brakes_pub = rospy.Publisher("vehicle/brake_cmd", BrakeCmd, queue_size=1)

        # Define subscribers --------------------------------------------------
        # Heartbeat - verify connection with main computer
        rospy.Subscriber("actor/estop/heartbeat_core", Header, heartbeat_callback)
        rospy.Timer(heartbeat_timeout, check_heartbeat)
        # Software Trigger and Reset using ROS
        rospy.Subscriber("actor/e_stop/trigger", Empty, trigger_callback)
        rospy.Subscriber("actor/e_stop/reset", Empty, reset_callback)
        # Assign callbacks for physical and wireless loops - press and release events
        button_loop.when_pressed = lambda: physical_button(activated=True)
        button_loop.when_released = lambda: physical_button(activated=False)
        wireless_loop.when_pressed = lambda: wireless_button(activated=True)
        wireless_loop.when_released = lambda: wireless_button(activated=False)
        # DBW active state - used for flashing lights
        rospy.Subscriber("vehicle/dbw_enabled", Bool, dbw_state_callback)

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
