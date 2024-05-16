import resources
import numpy as np
from random import uniform as uf

def motor_specs():
    ATM_TEMPERATURE_K = 273
    FIRST_ELEMENT = 0
    MOTOR_SURFACE_TEMP_K = 303
    MAX_REG_EFF_PERCENTAGE = 10
    PRECISION = '{0:0.2f}'

    np.set_printoptions(formatter={'float': lambda x: PRECISION.format(x)})  

    input_data = resources.ExcelDataExtractor("input_motor_dyno.xlsx",sheet_number=0)
    input_data.read_excel()
    input_data.extract_data()

    motor_selection = input_data.motor_selection[FIRST_ELEMENT]

    # motor specification
    motor_data = resources.ExcelDataExtractor(file_path='data//motor_package//motor_data.xlsx', sheet_number=int(motor_selection-1))
    motor_data.read_excel()
    motor_data.extract_data()

    motor_name = motor_data.data_frame.motor_name[FIRST_ELEMENT]
    motor_manufacturer_name = motor_data.data_frame.motor_manufacturer_name[FIRST_ELEMENT]
    motor_masss_kg = motor_data.data_frame.motor_masss_kg[FIRST_ELEMENT]
    motor_surface_area_m2 = np.array(motor_data.data_frame.motor_surface_area_m2[FIRST_ELEMENT])
    motor_cp = np.array(motor_data.data_frame.motor_cp_JpkgpK[FIRST_ELEMENT])
    motor_max_torque_Nm = motor_data.data_frame.motor_max_torque_Nm[FIRST_ELEMENT]
    motor_max_regen_torque_Nm = motor_data.data_frame.motor_max_regen_torque_Nm[FIRST_ELEMENT]
    motor_min_speed_rpm = np.array(motor_data.data_frame.motor_min_speed_rpm[FIRST_ELEMENT])
    motor_max_speed_rpm = motor_data.data_frame.motor_max_speed_rpm[FIRST_ELEMENT]
    motor_min_voltage = motor_data.data_frame.motor_min_voltage[FIRST_ELEMENT]
    motor_max_current = motor_data.data_frame.motor_max_current[FIRST_ELEMENT]
    motor_cost_rs = np.array(motor_data.data_frame.motor_cost_rs[FIRST_ELEMENT])
    motor_inertia = np.array(motor_data.data_frame.motor_inertia[FIRST_ELEMENT])
    motor_speed_map_rpm = resources.clean_data(np.array(motor_data.data_frame.motor_speed_map_rpm))
    motor_torque_map_Nm = resources.clean_data(np.array(motor_data.data_frame.motor_torque_map_Nm))
    motor_max_torque_map_Nm_against_speed = resources.clean_data(np.array(motor_data.data_frame.motor_max_torque_map_Nm_against_speed))
    motor_max_regen_torque_map_Nm_against_speed = resources.clean_data(np.array(motor_data.data_frame.motor_max_regen_torque_map_Nm_against_speed))
    motor_reg_cap = resources.MotorRegen(motor_max_regen_torque_Nm)
    #print(motor_reg_cap)
    
    return motor_name, motor_manufacturer_name, motor_max_torque_Nm, motor_max_regen_torque_Nm, motor_max_speed_rpm, motor_min_voltage, motor_max_current, motor_reg_cap
#motor_specs()