import time
from datetime import datetime

class SensorData:
    def __init__(self, temperature: float, blood_pressure: tuple, heartbeat: int, misuration_ts: float = None) -> None:
        """
        Initialize SensorData object with temperature, blood pressure, and heartbeat data.
        
        Args:
        temperature (float): Body temperature in Celsius.
        blood_pressure (tuple): Blood pressure as a tuple (systolic, diastolic).
        heartbeat (int): Heartbeat in beats per minute (bpm).
        misuration_ts (float, optional): Timestamp of measurement, in Unix time. Defaults to current time.
        """
        self.temperature = temperature
        self.blood_pressure = blood_pressure
        self.heartbeat = heartbeat
        self.misuration_ts = misuration_ts if misuration_ts else time.time()  # Use current time if not provided
        self.history = []  # To log all historical measurements

        # Validate data
        self._validate_data()

        # Log initial data
        self._log_data()

    def _validate_data(self):
        """Private method to validate sensor data."""
        if not (30 <= self.temperature <= 45):  # Human body temperature range in °C
            raise ValueError(f"Temperature {self.temperature}°C is out of range.")
        
        systolic, diastolic = self.blood_pressure
        if not (60 <= systolic <= 180) or not (40 <= diastolic <= 120):  # General BP range
            raise ValueError(f"Blood pressure {self.blood_pressure} is out of range.")
        
        if not (40 <= self.heartbeat <= 200):  # General heartbeat range
            raise ValueError(f"Heartbeat {self.heartbeat} bpm is out of range.")

    def _log_data(self):
        """Private method to log sensor data."""
        self.history.append({
            'timestamp': self.misuration_ts,
            'temperature': self.temperature,
            'blood_pressure': self.blood_pressure,
            'heartbeat': self.heartbeat
        })

    def get_timestamp(self) -> str:
        """Return the formatted timestamp."""
        return datetime.fromtimestamp(self.misuration_ts).strftime('%Y-%m-%d %H:%M:%S')

    def temperature_fahrenheit(self) -> float:
        """Convert temperature from Celsius to Fahrenheit."""
        return self.temperature * 9 / 5 + 32

    def check_health_status(self) -> str:
        """Check the overall health status based on sensor data."""
        status = "Normal"

        if self.temperature < 36 or self.temperature > 37.5:
            status = "Abnormal temperature"
        
        systolic, diastolic = self.blood_pressure
        if systolic < 90 or systolic > 140 or diastolic < 60 or diastolic > 90:
            status = "Abnormal blood pressure"
        
        if self.heartbeat < 60 or self.heartbeat > 100:
            status = "Abnormal heartbeat"
        
        return status

    def log_new_reading(self, temperature: float, blood_pressure: tuple, heartbeat: int, misuration_ts: float = None) -> None:
        """
        Log a new sensor reading.
        
        Args:
        temperature (float): New temperature reading in Celsius.
        blood_pressure (tuple): New blood pressure reading (systolic, diastolic).
        heartbeat (int): New heartbeat reading in bpm.
        misuration_ts (float, optional): Timestamp for the new reading. Defaults to current time.
        """
        # Update sensor readings
        self.temperature = temperature
        self.blood_pressure = blood_pressure
        self.heartbeat = heartbeat
        self.misuration_ts = misuration_ts if misuration_ts else time.time()

        self._validate_data()

        self._log_data()

    def __repr__(self) -> str:
        """Provide a string representation of the sensor data."""
        return (f"SensorData(Temperature: {self.temperature}°C, Blood Pressure: {self.blood_pressure}, "
                f"Heartbeat: {self.heartbeat} bpm, Timestamp: {self.get_timestamp()})")

    def get_history(self):
        """Return the history of sensor readings."""
        return [
            {
                'timestamp': datetime.fromtimestamp(entry['timestamp']).strftime('%Y-%m-%d %H:%M:%S'),
                'temperature': entry['temperature'],
                'blood_pressure': entry['blood_pressure'],
                'heartbeat': entry['heartbeat']
            }
            for entry in self.history
        ]
    
    def to_dict(self):
        """Convert the SensorData object to a dictionary for serialization."""
        return {
            'temperature': self.temperature,
            'blood_pressure': {
                'systolic': self.blood_pressure[0],
                'diastolic': self.blood_pressure[1],
            },
            'heartbeat': self.heartbeat,
            'timestamp': self.misuration_ts
        }
    

