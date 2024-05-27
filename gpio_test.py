from gpiozero import LED, Button, LineSensor
import time

# Inputs:
button_1 = LineSensor("BCM4", pull_up=False, queue_len=50)  # GPIO 4, pin 7
button_2 = LineSensor("BCM18", pull_up=False, queue_len=50)  # GPIO 18, pin 12

# Outputs:
ch1_relay = LED("BCM26")  # GPIO 26, pin 37
ch2_relay = LED("BCM20")  # GPIO 20, pin 38
ch3_relay = LED("BCM21")  # GPIO 21, pin 40


def button_pressed(activated):
    if activated:
        print("ON")
        ch1_relay.on()
    else:
        print("OFF")
        ch1_relay.off()


# Run for 10 seconds using timer
if __name__ == "__main__":
    start_time = time.time()
    while True:
        print(button_1.value, button_2.value)

        if button_2.is_active:
            print(ch1_relay.is_lit)
            if not ch1_relay.value:
                button_pressed(True)
        else:
            print("not pressed")
            if ch1_relay.value:
                button_pressed(False)

        if time.time() > start_time + 10:
            break

        time.sleep(0.01)
