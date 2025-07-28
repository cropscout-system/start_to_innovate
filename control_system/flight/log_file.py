import datetime


class LogFile:
    def __init__(self):
        # Get the current date and time
        now = datetime.datetime.now()
        # Format the date and time as a string to use as the filename
        self.filename = "tfl-" + now.strftime("%Y-%m-%d_%H-%M-%S") + ".txt"
        # Create the file and write an initial header
        self.create_header()

    def create_header(self):
        # Open the file in write mode and write a header
        with open(self.filename, "w") as file:
            file.write(
                "flight_phase,flight_phase_status,arm,mode,latitude,longitude,altitude_home,altitude_amsl,altitude_rangefinder,battery_current,battery_percentage,battery_voltage,home_position_latitude,home_position_longitude,home_position_altitude,sys_messages_severity,sys_messages_text\n"
            )

    def append_string(self, text):
        # Open the file in append mode and add the provided text
        with open(self.filename, "a") as file:
            file.write(text + "\n")
