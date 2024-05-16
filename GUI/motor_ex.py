import resources
import numpy as np
from random import uniform as uf
from openpyxl import load_workbook

def motor_program(speed, load, voltage, coolant_flow):
    ATM_TEMPERATURE_K = 273
    FIRST_ELEMENT = 0
    MOTOR_SURFACE_TEMP_K = 303
    MAX_REG_EFF_PERCENTAGE = 10
    PRECISION = '{0:0.2f}'
    input_speed_rpm = speed
    input_load_Nm = load
    input_voltage_V = voltage
    input_coolant_flow_mps = coolant_flow

    # Function

    def display_motor_specification():
        print('Motor Specification')
        print('*******************')
        print(f'Motor Name : {motor_name}')
        print(f'Manufature : {motor_manufacturer_name}')
        print(f'Min Voltage (V) : {motor_min_voltage:}')
        print(f'Max Current (A) : {motor_max_current:}')
        print(f'Rated Speed (RPM) : {(motor_max_speed_rpm*0.8):}')
        print(f'Rated Torque (Nm) : {(motor_max_torque_Nm*0.8):}')
        print(f'Max Speed (RPM) : {motor_max_speed_rpm:}')
        print(f'Max Torque (Nm) : {motor_max_torque_Nm:}')
        print(f'Regen Limit (Nm) : {motor_max_regen_torque_Nm:}')
        print(f'Motor Reg Capacity : {motor_reg_cap.motor_reg_status}')
        print("\n")

    def display_motor_dyno_input():
        print("\n") 
        print('Motor Dyno Input')
        print('*****************')
        print(f'Input Speed (RPM) : {speed}')
        print(f'Input Load (Nm) : {load}')
        print(f'Input Voltage (V) : {voltage}')
        #print(f'Input Motor Surface Temperature (K) : {input_motor_surface_temperature_K}')
        print(f'Input Coolant Flow (m/s) : {coolant_flow}')
        print("\n")  

    def display_motor_dyno_output():
        print('Motor Dyno Output')
        print('******************')
        print(f'Torque Available (Nm) : {motor_torque_out_Nm}')
        print(f'Voltage (V) : {motor_voltage_V}')
        print(f'Current (A) : {motor_current_A}')
        print(f'Power Input (W) : {motor_input_power_W}')
        print(f'Power Output (W) : {motor_output_power_W}')
        print(f'Regen Power Output (W) : {motor_regen_power_W}')
        print(f'Efficiency (%) : {motor_eff_percentage}')
        print(f'Power Loss (W) : {motor_power_loss_W}')
        print(f'Motor Surface Change in Temperature (K) : {motor_surface_change_in_temp_K}')
        print(f'Motor Surface Temperature (K) : {motor_surface_temperature_K}')

    np.set_printoptions(formatter={'float': lambda x: PRECISION.format(x)})  
    
    # inputs

    input_data = resources.ExcelDataExtractor("input_motor_dyno.xlsx",sheet_number=0)
    input_data.read_excel()
    input_data.extract_data()

    # motor sellection

    motor_selection = input_data.motor_selection[FIRST_ELEMENT]
    #input_speed_rpm = np.array(input_data.speed_rpm)
    #input_load_Nm = np.array(input_data.load_Nm)
    #input_voltage_V = np.array(input_data.voltage_V)
    #input_motor_surface_temperature_K = np.array(input_data.motor_surface_temperature_K)
    #input_motor_surface_temperature_K = MOTOR_SURFACE_TEMP_K
    #input_coolant_flow_mps = np.array(input_data.coolant_flow_mps)

    temperature_data = resources.ExcelDataExtractor("temperature.xlsx",sheet_number=0)
    temperature_data.read_excel()
    temperature_data.extract_data()
    temperature_K = temperature_data.data_frame.temperature.iloc[-1]


    # motor specification

    motor_data = resources.ExcelDataExtractor(file_path='data\\motor_package\\motor_data.xlsx', sheet_number=int(motor_selection-1))
    motor_data.read_excel()
    motor_data.extract_data()
    
    # motor data

    motor_name = motor_data.data_frame.motor_name[FIRST_ELEMENT]
    motor_manufacturer_name = motor_data.data_frame.motor_manufacturer_name[FIRST_ELEMENT]
    motor_masss_kg = motor_data.data_frame.motor_masss_kg[FIRST_ELEMENT]
    motor_surface_area_m2 = motor_data.data_frame.motor_surface_area_m2[FIRST_ELEMENT]
    motor_cp = motor_data.data_frame.motor_cp_JpkgpK[FIRST_ELEMENT]
    motor_max_torque_Nm = motor_data.data_frame.motor_max_torque_Nm[FIRST_ELEMENT]
    motor_max_regen_torque_Nm = motor_data.data_frame.motor_max_regen_torque_Nm[FIRST_ELEMENT]
    motor_min_speed_rpm = motor_data.data_frame.motor_min_speed_rpm[FIRST_ELEMENT]
    motor_max_speed_rpm = motor_data.data_frame.motor_max_speed_rpm[FIRST_ELEMENT]
    motor_min_voltage = motor_data.data_frame.motor_min_voltage[FIRST_ELEMENT]
    motor_max_current = motor_data.data_frame.motor_max_current[FIRST_ELEMENT]
    motor_cost_rs = motor_data.data_frame.motor_cost_rs[FIRST_ELEMENT]
    motor_inertia = motor_data.data_frame.motor_inertia[FIRST_ELEMENT]
    motor_speed_map_rpm = resources.clean_data(motor_data.data_frame.motor_speed_map_rpm)
    motor_torque_map_Nm = resources.clean_data(motor_data.data_frame.motor_torque_map_Nm)
    motor_max_torque_map_Nm_against_speed = resources.clean_data(motor_data.data_frame.motor_max_torque_map_Nm_against_speed)
    motor_max_regen_torque_map_Nm_against_speed = resources.clean_data(motor_data.data_frame.motor_max_regen_torque_map_Nm_against_speed)

    # motor input power map

    inputpwr_map = resources.ExcelDataExtractor("data\\motor_package\\motor_input_power_data.xlsx",int(motor_selection-1))
    inputpwr_map.read_excel()
    motor_input_power_map = np.array(inputpwr_map.data_frame)

    # motor efficiency map

    eff_map = resources.ExcelDataExtractor("data\\motor_package\\motor_effieciency_data.xlsx",int(motor_selection-1))
    eff_map.read_excel()
    motor_eff_frac_map = np.array(eff_map.data_frame)

    # motor map

    motor_max_trq_avl = np.interp(speed,motor_speed_map_rpm,motor_max_torque_map_Nm_against_speed)

    # regen capacity check

    motor_reg_cap = resources.MotorRegen(motor_max_regen_torque_Nm)

    # torque limiter

    torque_in = motor_reg_cap.regen_torque(load, speed, motor_speed_map_rpm, motor_max_torque_map_Nm_against_speed, motor_max_regen_torque_map_Nm_against_speed)
    
    # temperature map

    temperature_map =np.linspace(300,350,10)

    # efficeincy map vs temp

    eff_frac_map = np.array([0,0.012,0.15,0.02,0.023,0.025,0.3,0.04,0.055,0.08])

    # coolant flow map

    coolant_flow_map = np.linspace(0,1,10)

    # initialization

    motor_current_A = np.array([])
    motor_voltage_V = np.array([])
    motor_input_power_W =np.array([])
    motor_eff_percentage = np.array([])
    motor_output_power_W = np.array([])
    motor_power_loss_W = np.array([])
    motor_torque_out_Nm = np.array([])
    motor_regen_power_W = np.array([])
    motor_surface_temperature_K = np.array([])
    motor_surface_change_in_temp_K = np.array([])
    motor_coolant_flow_mps = np.array([])

    # solver selection

    solver = resources.Solver

    #speed_mps = solver.kmph2mps(speed = input_speed_rpm))

    # simulation

    #for i in range(0,input_speed_rpm.size):

    # temperature input selection
        
    #motor_surface_temperature = np.array(input_motor_surface_temperature_K)
    motor_surface_temperature = np.array(temperature_K)
        
        
    # motor min voltage noise

    noise = uf(0, 0.01)
    motor_actual_voltage = np.array([int(voltage + voltage*noise)])

    # regen efficiency noise

    reg_eff = (MAX_REG_EFF_PERCENTAGE/100) + noise*0.1

    # coolant and temperature corelation and interpolation

    #motor_eff_temp = np.interp(input_motor_surface_temperature_K[i],temperature_map,eff_frac_map)
    motor_eff_coolant = np.interp(coolant_flow/10,coolant_flow_map,eff_frac_map)
                        
    # motor input power interpolation

    motor_input_pwr_req = solver.intplt2d(speed,torque_in,motor_speed_map_rpm,motor_torque_map_Nm,motor_input_power_map)
    
    # motor efficiency interpolation

    motor_eff_frac_req = solver.intplt2d(speed,torque_in,motor_speed_map_rpm,motor_torque_map_Nm,motor_eff_frac_map)
    motor_eff_frac_req = motor_eff_frac_req + motor_eff_coolant

    # without current limit

    motor_output_pwr = solver.output_pwr(motor_input_pwr_req,motor_eff_frac_req)
    motor_current_req = solver.current(motor_input_pwr_req,motor_actual_voltage)
    motor_torque_out = motor_output_pwr/(solver.rpm2radps(speed))
    motor_power_loss = solver.power_loss(motor_input_pwr_req,motor_output_pwr)
    motor_regen_power = motor_reg_cap.regen_power(motor_output_pwr)

    # current limit

    motor_corrected_current_req = np.minimum(motor_current_req,motor_max_current)
    motor_corrected_input_pwr = solver.input_pwr(motor_actual_voltage,motor_corrected_current_req)
    motor_corrected_output_pwr = solver.output_pwr(motor_corrected_input_pwr,motor_eff_frac_req)
    motor_corrected_torque_out = motor_corrected_output_pwr/(solver.rpm2radps(speed))
    motor_corrected_power_loss = solver.power_loss(motor_input_pwr_req, motor_corrected_output_pwr)
    motor_correected_regen_power = motor_reg_cap.regen_power(reg_eff*motor_corrected_output_pwr)
    motor_surface_delta_T_K = solver.delta_T_K(abs(motor_corrected_power_loss),motor_masss_kg,motor_cp)
    motor_surface_temperature = motor_surface_temperature + motor_surface_delta_T_K - (0.025*coolant_flow)
    motor_coolant_flow = np.interp(motor_surface_temperature,temperature_map,coolant_flow_map)

    #torque limit at low speed

    motor_corrected_torque_out = np.where(motor_corrected_torque_out > motor_max_torque_Nm, motor_max_torque_Nm, motor_corrected_torque_out)

    # temp limit
    motor_surface_temperature = np.where(motor_surface_temperature < MOTOR_SURFACE_TEMP_K, MOTOR_SURFACE_TEMP_K, motor_surface_temperature)
        

    # Array Outputs

    motor_voltage_V = np.append(motor_voltage_V,motor_actual_voltage,axis = 0)
    motor_current_A = np.append(motor_current_A,motor_corrected_current_req,axis = 0)
    motor_input_power_W = np.append(motor_input_power_W,motor_corrected_input_pwr,axis = 0)
    motor_eff_percentage = np.append(motor_eff_percentage,motor_eff_frac_req*100,axis = 0)
    motor_output_power_W = np.append(motor_output_power_W,motor_output_pwr,axis = 0)
    motor_power_loss_W = np.append(motor_power_loss_W,motor_corrected_power_loss,axis = 0)
    motor_torque_out_Nm = np.append(motor_torque_out_Nm,motor_corrected_torque_out,axis = 0)
    motor_regen_power_W  = np.append(motor_regen_power_W ,motor_correected_regen_power,axis = 0)
    motor_surface_change_in_temp_K = np.append(motor_surface_change_in_temp_K,motor_surface_delta_T_K,axis = 0)
    motor_surface_temperature_K = np.append(motor_surface_temperature_K,motor_surface_temperature,axis = 0)
    motor_coolant_flow_mps = np.append(motor_coolant_flow_mps, motor_coolant_flow, axis = 0)
    
    # Update 'temperature.xlsx'
    file_path = 'temperature.xlsx'
    workbook = load_workbook(file_path)
    sheet = workbook['Sheet1']
    sheet['A2'] = motor_surface_temperature[-1]
    workbook.save(file_path)

    # print function

    #display_motor_dyno_input()
    #display_motor_specification()
    #display_motor_dyno_output()
    return motor_name, motor_manufacturer_name, motor_min_voltage, motor_max_current, motor_max_speed_rpm, motor_max_torque_Nm, motor_max_regen_torque_Nm, motor_torque_out_Nm, motor_voltage_V, motor_current_A, motor_eff_percentage, motor_surface_temperature_K, motor_output_power_W, motor_regen_power_W, speed, motor_current_A, motor_voltage_V, motor_input_power_W, motor_eff_percentage, motor_output_power_W, motor_power_loss_W, motor_torque_out_Nm, motor_regen_power_W, motor_surface_temperature_K, motor_surface_change_in_temp_K, motor_coolant_flow_mps, load, voltage, coolant_flow
    
#motor_program(5500, 70, 40, 0.5)
