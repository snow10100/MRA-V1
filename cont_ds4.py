import os
import time
import serial
import pygame

motor_commands = {
    "x": ("ML0", 0),
    "l1": ("ML-20", 4),
    "r1": ("MR-20", 5),
    "l2": ("ML20", 6),
    "r2": ("MR20", 7),
}

arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
time.sleep(0.5)  # Wait for the connection to establish

pygame.init()
pygame.joystick.init()
print(f"number of joysticks connected: {pygame.joystick.get_count()}")
ds4 = pygame.joystick.Joystick(0)
ds4.init()

def send_command(command):
    arduino.write(f"{command}\n".encode())

def read_response():
    time.sleep(0.5)
    if arduino.in_waiting > 0:
        response = ''.join(line.decode() for line in arduino.readlines())
        return response
    return ""

try:
    while True:
        events = pygame.event.get()
        for event in events:
            # if event.type == pygame.JOYAXISMOTION:
            #     print(event.dict, event.joy, event.axis, event.value)
            # elif event.type == pygame.JOYBALLMOTION:
            #     print(event.dict, event.joy, event.ball, event.rel)
            if event.type == pygame.JOYBUTTONDOWN:
                print(event.dict, event.joy, event.button, 'pressed')
                print("Button Pressed")
                if ds4.get_button(motor_commands["x"][1]):
                    send_command("MR0")
                    send_command("ML0")
                    print(read_response())
                else:
                    if ds4.get_button(motor_commands["l2"][1]):
                        # send_command(motor_commands["l2"][0])
                        send_command("MR-20")
                        send_command("ML-20")
                        # print(read_response())
                    # elif ds4.get_button(motor_commands["l1"][1]):
                    #     send_command(motor_commands["l1"][0])
                    #     print(motor_commands["l1"][0])
                    #     print(read_response())
                    if ds4.get_button(motor_commands["r2"][1]):
                        # send_command(motor_commands["r2"][0])
                        send_command("MR20")
                        send_command("ML20")
                        # print(read_response())
                    # elif ds4.get_button(motor_commands["r1"][1]):
                    #     send_command(motor_commands["r1"][0])
                    #     print(read_response())
            # elif event.type == pygame.JOYBUTTONUP:
            #     print(event.dict, event.joy, event.button, 'released')
            # elif event.type == pygame.JOYHATMOTION:
            #     print(event.dict, event.joy, event.hat, event.value)

except KeyboardInterrupt:
    print("EXITING NOW")
    ds4.quit()
