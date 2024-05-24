#!/usr/bin/env python3

import rospy
from dbw_polaris_msgs.msg import BrakeCmd
from gpiozero import LED, LineSensor
from std_msgs.msg import Bool, Empty, Header

# Initialize GPIO devices
PHYSICAL_BUTTON_PIN = "BCM4"  # GPIO 4, pin 7
WIRELESS_BUTTON_PIN = "BCM17"  # GPIO 17, pin 11
ESTOP_RELAY_PIN = "BCM21"  # GPIO 21, pin 40, relay board channel 3
FLASHING_LIGHT_PIN = "BCM26"  # GPIO 26, pin 37, relay board channel 1

button_loop = LineSensor(PHYSICAL_BUTTON_PIN, pull_up=True, queue_len=100)
wireless_loop = LineSensor(WIRELESS_BUTTON_PIN, pull_up=True, queue_len=100)
estop_relay = LED(ESTOP_RELAY_PIN, active_high=True)  # relay board
flashing_lights_relay = LED(FLASHING_LIGHT_PIN, active_high=True)  # relay board


def activate_estop():
    global estop_is_activated
    # Avoid activating if already activated
    if not estop_is_activated:
        estop_is_activated = True
        if software_estop_only:
            send_brakes()
        else:  # Hardware E-Stop using relay board
            if estop_relay.is_lit:  # Check relay state before turning it off
                estop_relay.off()
            disable_pub.publish(Empty())  # Disable vehicle control using ROS messages
        rospy.loginfo("E-Stop: ACTIVATED !!")


def reset_estop():
    global estop_is_activated
    # Check all loops to make sure E-Stop is safe to reset
    if not phyiscal_button and not wireless_button and not software_button:
        estop_is_activated = False
        if software_estop_only:
            pass  # No need to reset brakes, they will timeout after the send_brakes() call
        else:  # Hardware E-Stop using relay board
            if not estop_relay.is_lit:  # Check relay state before turning it on
                estop_relay.on()
        rospy.loginfo("E-Stop: RESET !!")


def send_heartbeat(TimerEvent):
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


def send_states(TimerEvent):
    msg = Bool()
    msg.data = estop_is_activated
    estop_state_pub.publish(msg)
    msg.data = phyiscal_button
    physical_button_state_pub.publish(msg)
    msg.data = wireless_button
    wireless_state_pub.publish(msg)
    msg.data = software_button
    software_state_pub.publish(msg)


def check_heartbeat(TimerEvent):
    global time_last_heartbeat, heartbeat_timeout, software_button
    # Check if received heartbeat is within timeout
    if rospy.Time.now() - time_last_heartbeat > heartbeat_timeout:
        if not estop_is_activated:  # Avoid activating if already activated
            rospy.loginfo("E-Stop: Heartbeat timed out")
            software_button = True
            activate_estop()


def heartbeat_callback(msg):
    global time_last_heartbeat
    time_last_heartbeat = msg.stamp


def update_physical_button(activated: bool):
    global phyiscal_button
    if activated and not phyiscal_button:
        phyiscal_button = True
        rospy.loginfo("E-Stop: Physical button pressed")
        activate_estop()

    elif not activated and phyiscal_button:
        phyiscal_button = False
        rospy.loginfo("E-Stop: Physical button released")
        reset_estop()


def update_wireless_button(activated: bool):
    global wireless_button
    if activated and not wireless_button:
        wireless_button = True
        rospy.loginfo("E-Stop: Wireless button pressed")
        activate_estop()

    elif not activated and wireless_button:
        wireless_button = False
        rospy.loginfo("E-Stop: Wireless button released")
        reset_estop()


def trigger_callback(msg):
    global software_button
    if not estop_is_activated:  # Avoid activating if already activated
        software_button = True
        activate_estop()


def reset_callback(msg):
    global software_button
    if estop_is_activated:  # Avoid resetting if already reset
        if software_button:
            software_button = False
            reset_estop()
        else:  # E-Stop is not triggered by software, so hardware reset is required
            rospy.loginfo("E-Stop: Release Hardware e-stop to reset")


def dbw_state_callback(msg):
    flashing_lights_relay.on() if msg.data else flashing_lights_relay.off()


def check_gpio(TimerEvent):
    if button_loop.is_active:
        update_physical_button(activated=True)
    else:
        update_physical_button(activated=False)
    if wireless_loop.is_active:
        update_wireless_button(activated=True)
    else:
        update_wireless_button(activated=False)


if __name__ == "__main__":
    try:
        rospy.init_node("edge_estop_manager")

        # Variables -----------------------------------------------------------
        software_estop_only = False  # set to True if only soft estop is being used
        # Software estop means, sending brakes via ROS messages and NOT direct hardware relay
        estop_is_activated = False
        phyiscal_button = False
        wireless_button = False
        software_button = False

        time_last_heartbeat = rospy.Time(0)
        heartbeat_timeout = rospy.Duration(1 / 10)  # 10Hz
        heartbeat_rate = rospy.Duration(1 / 100)  # 100Hz

        # Define publishers ---------------------------------------------------
        # States
        estop_state_pub = rospy.Publisher("actor/estop/state", Bool, queue_size=1)
        physical_button_state_pub = rospy.Publisher("actor/estop/physical_button", Bool, queue_size=1)
        wireless_state_pub = rospy.Publisher("actor/estop/wireless_button", Bool, queue_size=1)
        software_state_pub = rospy.Publisher("actor/estop/software_button", Bool, queue_size=1)
        rospy.Timer(heartbeat_rate, send_states)

        # Heartbeat - send heartbeat so that main computer can verify connection with edge computer
        heartbeat_pub = rospy.Publisher("actor/estop/heartbeat_edge", Header, queue_size=1)
        rospy.Timer(heartbeat_rate, send_heartbeat)

        # Software Stopping via Brakes
        brakes_pub = rospy.Publisher("vehicle/brake_cmd", BrakeCmd, queue_size=1)
        disable_pub = rospy.Publisher("vehicle/disable", Empty, queue_size=1)

        # Define subscribers --------------------------------------------------
        # Heartbeat - verify connection with main computer
        rospy.Subscriber("actor/estop/heartbeat_core", Header, heartbeat_callback)
        rospy.Timer(heartbeat_timeout, check_heartbeat)

        # Software Trigger and Reset using ROS
        rospy.Subscriber("actor/estop/trigger", Empty, trigger_callback)
        rospy.Subscriber("actor/estop/reset", Empty, reset_callback)

        # Define GPIO and Relay Devices ---------------------------------------
        gpio_rate = rospy.Duration(1 / 100)  # 100Hz
        rospy.Timer(gpio_rate, check_gpio)

        # DBW active state - used for flashing lights on relay board
        rospy.Subscriber("vehicle/dbw_enabled", Bool, dbw_state_callback)

        # Keep the node running
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
