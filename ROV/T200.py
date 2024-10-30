import numpy as np

class T200:

    def __init__(self, filename):
        self.filename = filename
        # Loading data using numpy, specifying the correct delimiter (semicolon)
        self.data = np.genfromtxt(self.filename, delimiter=';', names=True)
        
        # Display the available columns to check if they are correct
        #print("Available columns: ", self.data.dtype.names)
        
        # Store the necessary columns in variables
        self.pwm_values = self.data['PWM']
        self.rpm_values = self.data['RPM']
        self.power_values = self.data['Power']
        self.force_values = self.data['Force']*9.81
        self.efficiency_values = self.data['Efficiency']

    # Function to get the force based on the PWM with linear interpolation
    def get_force(self, pwm_value):
        if pwm_value in self.pwm_values:
            # If the PWM is exactly in the table, return the associated value
            return self.force_values[np.where(self.pwm_values == pwm_value)[0][0]]
        elif pwm_value < self.pwm_values.min() or pwm_value > self.pwm_values.max():
            # If the PWM is outside the range of the table
            return "PWM out of range"
        else:
            # Otherwise, perform linear interpolation
            return np.interp(pwm_value, self.pwm_values, self.force_values)

    # Function to calculate the torque based on RPM
    def get_torque(self, rpm_value):
        # If the RPM is outside the range of the table
        if rpm_value < self.rpm_values.min() or rpm_value > self.rpm_values.max():
            return "RPM out of range"
        
        # Perform interpolation for power and efficiency
        power = np.interp(rpm_value, self.rpm_values, self.power_values)
        efficiency = np.interp(rpm_value, self.rpm_values, self.efficiency_values)
        
        # Calculate mechanical power (W) from the interpolated electrical power and efficiency
        P_m = power * (efficiency / 100)  # Mechanical power in watts
        
        # Convert RPM to rad/s
        omega = (2 * np.pi * rpm_value) / 60  # Rad/s
        
        # Calculate torque (N·m)
        torque = P_m / omega if omega != 0 else 0
        
        return torque


if __name__ == "__main__":
    T200 = T200("/home/tbelier/Documents/GIT/python-simu/ROV/thruster.csv")
    
    # Display the first few rows of the file (manually for checking purposes)
    print(T200.data[:5])  # Display the first 5 rows

    # Example usage with a given PWM
    pwm_input = 1103  # Example of an intermediate value
    force_output = T200.get_force(pwm_input)
    print(f"The force for a PWM of {pwm_input} is: {force_output} Kg f")
    
    # Example usage with a given RPM to calculate the torque
    rpm_input = 3233  # Example RPM value
    torque_output = T200.get_torque(rpm_input)
    print(f"The torque for an RPM of {rpm_input} is: {torque_output:.4f} N·m")
