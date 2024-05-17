from textual.app import App, ComposeResult
from textual.widgets import Header, Footer, Button, Static, Input, Label, ProgressBar
from textual.containers import Horizontal, VerticalScroll, Center

# import pygame
import serial
import time
import os

# pygame.init()
# pygame.joystick.init()
# # print(f"number of joysticks connected: {pygame.joystick.get_count()}")
# joystick = pygame.joystick.Joystick(0)
# joystick.init()
#
# motor_commands = {
#     "l1": ("ML-20", 4),
#     "r1": ("MR-20", 5),
#     "l2": ("ML20", 6),
#     "r2": ("MR20", 7),
# }

# Setup Serial connection to Arduino (update port and baudrate according to your setup)
# arduino = serial.Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1)
arduino = serial.Serial(port="/dev/ttyUSB0", baudrate=9600, timeout=1)
time.sleep(0.5)  # Wait for the connection to establish

start_time = time.time()  # Record initial time when the application starts

def get_next_log_filename():
    for i in range(100):  # Limit to 100 files
        filename = f"log_{i:02}.txt"
        if not os.path.exists(filename):
            return filename
    raise Exception("Log file limit reached. Please clear some log files.")

current_log_file = None  # Global variable to keep track of the current log file

def send_command(command):
    global current_log_file
    global start_time
    if current_log_file:
        with open(current_log_file, "a") as file:
            relative_time = time.time() - start_time  # Calculate time elapsed since start_time
            start_time = time.time()
            file.write(f"{relative_time:.2f}, {command}\n")
    arduino.write(f"{command}\n".encode())

def replay_log(file_path):
    with open(file_path, 'r') as file:
        for line in file:
            duration, command = line.strip().split(",", 1)
            arduino.write(f"{command}\n".encode())
            time.sleep(float(duration)) 

def replay_log_reverse(file_path):
    with open(file_path, 'r') as file:
        commands = file.readlines()

    for line in reversed(commands):
        duration, command = line.strip().split(",", 1)
        if command[2:] == "CW":
            command = command[:2] + "CCW"
        elif command[2:] == "CCW":
            command = command[:2] + "CW"
        # implement the reverse gripper later
        # a basic idea is to store the previous gripper position
        # elif command[0] == "G":
        #     command = command.split(",")[1]
        # send_command(command)
        arduino.write(f"{command}\n".encode())
        time.sleep(float(duration))  

def read_response():
    if arduino.in_waiting:
        response = ''.join(line.decode() for line in arduino.readlines())
        return response
    return ""

class DS4Display(Static):
    """A widget to display text."""

class TextDisplay(Static):
    """A widget to display text."""

class ResponseDisplay(Static):
    """A widget to display text."""

class RightMotorInput(Input):
    """A widget to take input value for right motor input"""

class LeftMotorInput(Input):
    """A widget to take input value for left motor input"""

class GripperInput(Input):
    """A widget to take input value for gripper input"""

class GripperProgressBar(ProgressBar):
    """A widget to display gripper value"""

class RightMotorProgressBar(ProgressBar):
    """A widget to display right motor value"""

class LeftMotorProgressBar(ProgressBar):
    """A widget to display left motor value"""

class LogVerticalScroll(VerticalScroll):
    """A widget to display logs"""

class MotorControl(App):
    # CSS_PATH = "motor_control.css"
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("ctrl+c", "quit", "Quit"),
    ]

    TITLE = "Motor Control"

    def compose(self) -> ComposeResult:
        yield Header()
        yield Horizontal(
            VerticalScroll(
                Button("Motor 1 CW", id="M1_CW"),
                Button("Motor 2 CW", id="M2_CW"),
                Button("Motor 3 CW", id="M3_CW"),
                Button("Motor 4 CW", id="M4_CW"),
                Button("Motor 5 CW", id="M5_CW"),
                Button("Motor 6 CW", id="M6_CW"),
                # Button("Close gripper", id="M7_C"),
                Label("Left Motor: "),
                LeftMotorProgressBar(total=180, show_eta=False),
                LeftMotorInput(placeholder="angle"),
                Button("Move left wheels", id="left_motor"),
                Label("Gripper: "),
                GripperProgressBar(total=180, show_eta=False),
                GripperInput(placeholder="angle"),
                Button("Move gripper", id="gripper"),
            ),
            VerticalScroll(
                Button("Motor 1 CCW", id="M1_CCW"),
                Button("Motor 2 CCW", id="M2_CCW"),
                Button("Motor 3 CCW", id="M3_CCW"),
                Button("Motor 4 CCW", id="M4_CCW"),
                Button("Motor 5 CCW", id="M5_CCW"),
                Button("Motor 6 CCW", id="M6_CCW"),
                # Button("Open gripper", id="M7_O"),
                Label("Right Motor: "),
                RightMotorProgressBar(total=180, show_eta=False),
                RightMotorInput(placeholder="angle"),
                Button("Move right wheels", id="right_motor"),
            ),
            LogVerticalScroll(
                Button("Start Recording", id="start_recording"),
                Button("Stop Recording", id="stop_recording"),
                Button("Replay Log", id="replay_log"),
                Button("Replay Reverse", id="replay_reverse"),
                Button("Stop All Motors", id="stop_all", variant="error"),
                TextDisplay("Console..."),
                Label("Response from arduino"),
                ResponseDisplay("Response..."),
                DS4Display("DS4 not connected."),
                *[Button(f"Log {i:02}", id=f"log_{i:02}") for i in range(100) if os.path.exists(f"log_{i:02}.txt")],
            ),
        )
        yield Footer()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        button_id = str(event.button.id)
        text_display = self.query_one(TextDisplay)
        response_display = self.query_one(ResponseDisplay)
        global current_log_file
        global start_time

        if button_id == "stop_all":
            text_display.update("Stopping all motors")
            for i in range(1, 7):
                send_command(f"M{i},1,STOP")
            response_display.update(read_response)

        if button_id == "gripper":
            text_value = self.query_one(GripperInput).value
            try:
                value = int(text_value)
            except ValueError:
                return
            self.query_one(GripperProgressBar).update(progress=value)
            # self.query_one(VerticalScroll).mount(Label(f"Donation for ${value} received!"))
            self.query_one(GripperInput).value = ""
            send_command(f"G{text_value}")
            time.sleep(0.1)
            response_display.update(read_response())

        if button_id == "right_motor":
            text_value = self.query_one(RightMotorInput).value
            try:
                value = int(text_value)
            except ValueError:
                return
            value = (value + 100) * 180 / 200
            self.query_one(RightMotorProgressBar).update(progress=value)
            self.query_one(RightMotorInput).value = ""
            send_command(f"MR{text_value}")
            time.sleep(0.1)
            response_display.update(read_response())

        if button_id == "left_motor":
            text_value = self.query_one(LeftMotorInput).value
            try:
                value = int(text_value)
            except ValueError:
                return
            value = (value + 100) * 180 / 200
            self.query_one(LeftMotorProgressBar).update(progress=value)
            self.query_one(LeftMotorInput).value = ""
            send_command(f"ML{text_value}")
            time.sleep(0.1)
            response_display.update(read_response())

        elif button_id.startswith("log_"):
            log_number = button_id.split("_")[1]
            current_log_file = f"log_{log_number}.txt"
            text_display.update(f"Selected log file: {current_log_file}")

        elif button_id == "start_recording":
            current_log_file = get_next_log_filename()
            with open(current_log_file, 'w'):
                pass
            start_time = time.time() 
            text_display.update("Recording started")
            filename = current_log_file[:-4]
            self.mount()
            self.query_one(LogVerticalScroll).mount(Button(filename.replace('_', ' ').capitalize(), id=filename))

        elif button_id == "stop_recording":
            current_log_file = None
            text_display.update("Recording stopped")

        elif button_id == "replay_log":
            if current_log_file:
                text_display.update("Replaying log...")
                replay_log(current_log_file)
                text_display.update("Replaying log... Done!")
            else:
                text_display.update("Select log file")

        elif button_id == "replay_reverse":
            if current_log_file:
                text_display.update("Replaying log in reverse...")
                replay_log_reverse(current_log_file)
                text_display.update("Replaying log in reverse... Done!")

        elif button_id.startswith("M"):
            motor, dir = button_id.split("_")
            text_display.update(f"Rotating {motor} {dir.lower()}")
            if dir == 'CW':
                send_command(f"{motor},1,500")
                response_display.update(read_response())
            else:
                send_command(f"{motor},1,-500")
                response_display.update(read_response())


if __name__ == "__main__":
    app = MotorControl()
    app.run()
