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
    return ""
    time.sleep(0.5)
    if arduino.in_waiting > 0:
        response = ''.join(line.decode() for line in arduino.readlines())
        return response

def ddr_ik(v, omega, L=0.5, r=0.2):
    """DDR inverse kinematics: calculate wheels speeds from desired velocity."""
    v_r = (v - (L/2)*omega)/r
    v_l = (v + (L/2)*omega)/r
    if v_r > 5:
        send_command(f"MR{int(min(30, v_r))}")
        print(f"MR{int(min(30, v_r))}")
    elif v_r < -5:
        send_command(f"MR{int(max(-30, v_r))}")
        print(f"MR{int(max(30, v_r))}")
    else:
        send_command(f"MR0")
        print(f"MR0")
    if v_l > 5:
        send_command(f"ML{int(min(30, v_l))}")
        print(f"ML{int(min(30, v_l))}")
    elif v_r < -5:
        send_command(f"MR{int(max(-30, v_r))}")
        print(f"ML{int(max(30, v_l))}")
    else:
        send_command(f"ML0")
        print(f"ML0")

    read_response()


v = 0 
omega = 0 
update = False
stopped = True
forward = False

try:
    while True:
        events = pygame.event.get()
        for event in events:
            # if event.type == pygame.JOYAXISMOTION:
            #     if abs(event.value) > 0.2: 
            #         # print(event.dict, event.joy, event.axis, event.value)
            #         omega = 30 * event.value
            #         update = True
            #     else:
            #         if omega != 0:
            #             omega = 0
            #             update = True
            # elif event.type == pygame.JOYBALLMOTION:
            #     print(event.dict, event.joy, event.ball, event.rel)
            if event.type == pygame.JOYBUTTONDOWN:
                # print(event.dict, event.joy, event.button, 'pressed')
                # v = 1
                # update = True
                if ds4.get_button(motor_commands["r2"][1]):
                    forward = True
                    send_command("MR20")
                    send_command("ML20")
                if ds4.get_button(motor_commands["l2"][1]):
                    forward = False
                    send_command("MR20")
                    send_command("ML20")
                stopped = False
                # print("Button Pressed")
                # if ds4.get_button(motor_commands["x"][1]):
                #     # send_command("MR0")
                #     # send_command("ML0")
                #     # print(read_response())
                # else:
                #     if ds4.get_button(motor_commands["l2"][1]):
                #         # send_command(motor_commands["l2"][0])
                #         send_command("MR-20")
                #         send_command("ML-20")
                #         # print(read_response())
                #     # elif ds4.get_button(motor_commands["l1"][1]):
                #     #     send_command(motor_commands["l1"][0])
                #     #     print(motor_commands["l1"][0])
                #     #     print(read_response())
                #     if ds4.get_button(motor_commands["r2"][1]):
                #         # send_command(motor_commands["r2"][0])
                #         send_command("MR20")
                #         send_command("ML20")
                #         # print(read_response())
                #     # elif ds4.get_button(motor_commands["r1"][1]):
                #     #     send_command(motor_commands["r1"][0])
                #     #     print(read_response())
            elif event.type == pygame.JOYBUTTONUP:
                # print(event.dict, event.joy, event.button, 'released')
                # v = 0 
                # update = True
                stopped = True
                send_command("MR0")
                send_command("ML0")

            elif event.type == pygame.JOYHATMOTION:
                if stopped:
                    send_command("MR" + str(-20 * event.value[0]))
                    send_command("ML" + str( 20 * event.value[0]))
                elif forward:
                    send_command("MR" + str(20 - 10 * event.value[0]))
                    send_command("ML" + str(20 + 10 * event.value[0])) 
                else:
                    send_command("MR" + str(-15 + 5 * event.value[0]))
                    send_command("ML" + str(-15 - 5 * event.value[0])) 

                # print(event.dict, event.joy, event.hat, event.value)
                    # print(event.dict, event.joy, event.axis, event.value)
                # omega = 30 * event.value[0]
                # update = True
            # if update:
            #     ddr_ik(v, omega)
            #     update = False

except KeyboardInterrupt:
    print("EXITING NOW")
    ds4.quit()
