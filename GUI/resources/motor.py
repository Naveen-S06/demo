import numpy as np

class MotorRegen:
    def __init__(self, max_regen_torque_Nm):
        self.max_regen_torque_Nm = max_regen_torque_Nm
        self.check_regen_capacity()

    def check_regen_capacity(self):
        if np.isnan(self.max_regen_torque_Nm).any():
            self.motor_reg_status = "No Regen Motor"
        else:
            self.motor_reg_status = "Regen Motor"
    
    def regen_torque(self, input_torque, input_speed, motor_speed_map_rpm, motor_max_torque_map_Nm_vs_speed, motor_max_regen_torque_map_Nm_vs_speed):
        motor_max_trq_avl = np.interp(input_speed, motor_speed_map_rpm, motor_max_torque_map_Nm_vs_speed)
        input_torque = np.minimum(input_torque, motor_max_trq_avl)

        if self.motor_reg_status == "Regen Motor":
            motor_regen_max_trq_avl = np.interp(input_speed, motor_speed_map_rpm, motor_max_regen_torque_map_Nm_vs_speed)
            regen_torque = np.maximum(input_torque, motor_regen_max_trq_avl)

            # Replace negative values in input_torque with corresponding positive values from regen_torque
            input_torque = np.where(input_torque < 0, regen_torque, input_torque)
        else:
            # Set negativ values in input_torque to 0
            input_torque[input_torque < 0] = 0

        return input_torque
    
    def regen_power(self, input_power):
        regen_power = np.where(input_power > 0, 0, np.abs(input_power))
        return regen_power
    