import resources
import numpy as np
from random import uniform as uf
from vehicle_speed import eudc_speed, ind_hwy_speed, ind_urb_speed, udds_speed

def kmph_to_rpm(speed_kmph, rolling_radius_m):
    speed_mps = 0.277778 * speed_kmph
    speed_radps = speed_mps / rolling_radius_m
    rpm = 9.5493 * speed_radps
    
    return rpm

def motor_program(rpm, load, voltage, coolant_flow):
    ATM_TEMPERATURE_K = 273
    FIRST_ELEMENT = 0
    MOTOR_SURFACE_TEMP_K = 303
    MAX_REG_EFF_PERCENTAGE = 10
    PRECISION = '{0:0.2f}'
    input_speed_rpm = rpm
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
        #print('Motor Dyno Input')
        #print('*****************')
        print(f'Input Speed (RPM) : {rpm}')
        #print(f'Input Load (Nm) : {load}')
        #print(f'Input Voltage (V) : {voltage}')
        #print(f'Input Motor Surface Temperature (K) : {input_motor_surface_temperature_K}')
        #print(f'Input Coolant Flow (m/s) : {coolant_flow}')
        print("\n")  

    def display_motor_dyno_output():
        print('Motor Dyno Output')
        print('******************')
        print(f'Torque Available (Nm) : {motor_torque_out_Nm}')
        print(f'Vehicle Speed (kmph) : {vehicle_speed_kmph}')
        print(f'Voltage (V) : {motor_voltage_V}')
        print(f'Current (A) : {motor_current_A}')
        print(f'Power Input (W) : {motor_input_power_W}')
        print(f'Power Output (W) : {motor_output_power_W}')
        print(f'Regen Output (W) : {motor_regen_power_W}')
        print(f'Efficiency (%) : {motor_eff_percentage}')
        print(f'Power Loss (W) : {motor_power_loss_W}')
        print(f'Motor Surface Change in Temperature (K) : {motor_surface_change_in_temp_K}')
        print(f'Motor Surface Temperature (K) : {motor_surface_temperature_K}')

    np.set_printoptions(formatter={'float': lambda x: PRECISION.format(x)})  
    
    # inputs

    #input_data = resources.ExcelDataExtractor("input_motor_dyno.xlsx",sheet_number=0)
    #input_data.read_excel()
    #input_data.extract_data()

    # motor sellection

    #motor_selection = input_data.motor_selection[FIRST_ELEMENT]
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


    motor_data = {
    'motor_name': '62 kW (continuous) AC Induction Motor',
    'motor_manufacturer_name': 'Dresden University of Technology (Germany)',
    'motor_masss_kg': 70,
    'motor_surface_area_m2': 0.332889153429086,
    'motor_cp_JpkgpK': 430,
    'motor_max_torque_Nm': 198,
    'motor_max_regen_torque_Nm': -198,
    'motor_min_speed_rpm': 0,
    'motor_max_speed_rpm': 8000,
    'motor_min_voltage': 120,
    'motor_max_current': 480,
    'motor_cost_rs': 30000,
    'motor_inertia': 0.1,
    'motor_speed_map_rpm': [0, 500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000],
    'motor_torque_map_Nm': [-200, -175, -150, -125, -100, -75, -50, -25, 0, 25, 50, 75, 100, 125, 150, 175, 200],
    'motor_max_torque_map_Nm_against_speed': [198, 198, 198, 198, 197, 195, 183, 165, 147, 127, 119, 105, 98, 90, 83, 76, 73],
    'motor_max_regen_torque_map_Nm_against_speed': [-198, -198, -198, -198, -197, -195, -183, -165, -147, -127, -119, -105, -98, -90, -83, -76, -73]
}

    # Now you can access the motor data directly from the dictionary
    motor_name = motor_data['motor_name']
    motor_manufacturer_name = motor_data['motor_manufacturer_name']
    motor_mass_kg = motor_data['motor_masss_kg']
    motor_surface_area_m2 = motor_data['motor_surface_area_m2']
    motor_cp = motor_data['motor_cp_JpkgpK']
    motor_max_torque_Nm = motor_data['motor_max_torque_Nm']
    motor_max_regen_torque_Nm = motor_data['motor_max_regen_torque_Nm']
    motor_min_speed_rpm = motor_data['motor_min_speed_rpm']
    motor_max_speed_rpm = motor_data['motor_max_speed_rpm']
    motor_min_voltage = motor_data['motor_min_voltage']
    motor_max_current = motor_data['motor_max_current']
    motor_cost_rs = motor_data['motor_cost_rs']
    motor_inertia = motor_data['motor_inertia']
    motor_speed_map_rpm = motor_data['motor_speed_map_rpm']
    motor_torque_map_Nm = motor_data['motor_torque_map_Nm']
    motor_max_torque_map_Nm_against_speed = motor_data['motor_max_torque_map_Nm_against_speed']
    motor_max_regen_torque_map_Nm_against_speed = motor_data['motor_max_regen_torque_map_Nm_against_speed']
    # motor input power map

    motor_input_power_map = np.array([
    [6981.32, 5858.30, 4813.73, 4011.44, 3347.60, 2406.87, 1673.80, 947.89, 947.89, 947.89, 1673.80, 2406.87, 3347.60, 4011.44, 4813.73, 5858.30, 6981.32],
    [-3490.66, -3304.68, -3040.25, -2533.54, -1888.39, -1520.13, -944.19, -361.10, 947.89, 2256.89, 4291.79, 6333.86, 8583.59, 10556.43, 12667.71, 15021.28, 17453.29],
    [-13962.63, -12379.12, -10747.55, -9179.98, -7255.09, -5507.99, -3582.52, -1548.67, 1069.32, 3687.32, 6889.46, 10199.98, 13688.86, 16999.96, 20668.37, 24272.79, 27925.27],
    [-24426.07, -21331.68, -18389.81, -15498.78, -12306.48, -9333.72, -5999.70, -2720.66, 1206.33, 5133.32, 9708.26, 14228.23, 19109.44, 23771.13, 28734.08, 33646.19, 38405.78],
    [-34262.92, -30082.23, -25871.94, -21775.84, -17334.62, -13044.04, -8417.66, -3844.14, 1391.84, 6627.83, 12526.29, 18371.89, 24553.28, 30584.04, 36959.91, 43221.60, 49512.89],
    [-44258.00, -38908.39, -33401.99, -27964.25, -22302.54, -16752.76, -10816.12, -5009.74, 1535.24, 8080.23, 15363.81, 22517.15, 30057.33, 37485.59, 45137.83, 52721.40, 60461.75],
    [-54344.92, -47622.56, -40879.84, -34167.04, -27333.63, -20500.22, -13360.80, -6153.24, 1700.74, 9554.72, 18055.13, 26623.67, 35498.22, 44372.78, 53367.94, 62333.19, 71318.79],
    [-64058.30, -56132.26, -48182.85, -40210.15, -32214.24, -24195.19, -15826.96, -7312.76, 1850.22, 11013.20, 20824.95, 30782.68, 41089.59, 51419.63, 61772.89, 72149.44, 82549.36],
    [-73842.66, -64612.32, -55381.99, -46151.66, -37078.03, -27925.27, -18302.37, -8477.31, 1994.66, 12466.64, 23585.53, 34906.59, 46697.77, 58568.10, 70281.71, 81995.33, 93708.95],
    [-83190.78, -72791.93, -62393.08, -51994.24, -41712.79, -31415.93, -20768.25, -9701.98, 2079.00, 13859.97, 26355.64, 39269.91, 52534.99, 65815.49, 78978.59, 92141.68, 105304.78],
    [-92825.08, -81221.95, -69618.81, -58015.68, -46412.54, -34906.59, -23271.06, -10959.04, 2130.93, 15220.89, 29088.82, 43633.23, 58307.21, 72884.02, 87460.82, 102037.62, 116614.43],
    [-101964.59, -89219.01, -76473.44, -63727.87, -50982.29, -38397.24, -25598.16, -12247.40, 2151.57, 16550.54, 31997.70, 47996.55, 64209.44, 80261.80, 96314.16, 112366.52, 128418.87],
    [-110132.24, -96365.71, -82599.18, -68832.65, -55066.12, -41829.66, -27925.27, -13545.66, 2162.30, 17870.27, 34906.59, 52418.12, 70597.59, 88246.98, 105896.38, 123545.78, 141195.18],
    [-117571.73, -102875.26, -88178.79, -73482.33, -58785.86, -44933.92, -30252.37, -14740.31, 2276.65, 19293.61, 37815.47, 57167.84, 77349.82, 96687.27, 116024.73, 135362.18, 154699.64],
    [-122741.29, -107398.63, -92055.97, -76713.31, -61370.65, -47622.56, -32397.67, -15944.62, 2381.34, 20707.30, 40906.15, 62333.19, 85237.01, 106546.26, 127855.51, 149164.77, 170474.02],
    [-133607.96, -116906.97, -100205.97, -83504.98, -66803.98, -50102.99, -34416.32, -17058.42, 2576.53, 22211.49, 44123.49, 67706.74, 90275.65, 112844.56, 135413.48, 157982.39, 180551.30],
    [-140275.76, -122741.29, -105206.82, -87672.35, -70137.88, -52603.41, -36067.49, -18087.96, 2855.99, 23799.94, 47708.32, 73060.29, 97413.73, 121767.16, 146120.59, 170474.02, 194827.45]
    ])
    #print("Motor IP",motor_input_power_map)
    # motor efficiency map

    motor_eff_frac_map = np.array([
    [0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20],
    [0.60, 0.61, 0.62, 0.62, 0.61, 0.62, 0.61, 0.58, 0.20, 0.58, 0.61, 0.62, 0.61, 0.62, 0.62, 0.61, 0.60],
    [0.75, 0.76, 0.76, 0.77, 0.77, 0.77, 0.76, 0.71, 0.20, 0.71, 0.76, 0.77, 0.77, 0.77, 0.76, 0.76, 0.75],
    [0.82, 0.82, 0.82, 0.83, 0.82, 0.83, 0.81, 0.77, 0.20, 0.77, 0.81, 0.83, 0.82, 0.83, 0.82, 0.82, 0.82],
    [0.85, 0.85, 0.85, 0.86, 0.85, 0.85, 0.84, 0.79, 0.20, 0.79, 0.84, 0.85, 0.85, 0.86, 0.85, 0.85, 0.85],
    [0.87, 0.87, 0.87, 0.87, 0.87, 0.87, 0.85, 0.81, 0.20, 0.81, 0.85, 0.87, 0.87, 0.87, 0.87, 0.87, 0.87],
    [0.88, 0.88, 0.88, 0.89, 0.89, 0.89, 0.87, 0.82, 0.20, 0.82, 0.87, 0.89, 0.89, 0.89, 0.88, 0.88, 0.88],
    [0.89, 0.89, 0.89, 0.89, 0.89, 0.89, 0.88, 0.83, 0.20, 0.83, 0.88, 0.89, 0.89, 0.89, 0.89, 0.89, 0.89],
    [0.89, 0.89, 0.89, 0.89, 0.90, 0.90, 0.89, 0.84, 0.20, 0.84, 0.89, 0.90, 0.90, 0.89, 0.89, 0.89, 0.89],
    [0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.89, 0.85, 0.20, 0.85, 0.89, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90],
    [0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.86, 0.20, 0.86, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90],
    [0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.87, 0.20, 0.87, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90, 0.90],
    [0.89, 0.89, 0.89, 0.89, 0.89, 0.90, 0.90, 0.88, 0.20, 0.88, 0.90, 0.90, 0.89, 0.89, 0.89, 0.89, 0.89],
    [0.88, 0.88, 0.88, 0.88, 0.88, 0.89, 0.90, 0.88, 0.20, 0.88, 0.90, 0.89, 0.88, 0.88, 0.88, 0.88, 0.88],
    [0.86, 0.86, 0.86, 0.86, 0.86, 0.88, 0.90, 0.89, 0.20, 0.89, 0.90, 0.88, 0.86, 0.86, 0.86, 0.86, 0.86],
    [0.87, 0.87, 0.87, 0.87, 0.87, 0.87, 0.89, 0.88, 0.20, 0.88, 0.89, 0.87, 0.87, 0.87, 0.87, 0.87, 0.87],
    [0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.88, 0.88, 0.20, 0.88, 0.88, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86]
    ])
    #print("Motor Eff",motor_eff_frac_map)
    # motor map

    motor_max_trq_avl = np.interp(rpm,motor_speed_map_rpm,motor_max_torque_map_Nm_against_speed)

    # regen capacity check

    motor_reg_cap = resources.MotorRegen(motor_max_regen_torque_Nm)

    # torque limiter

    torque_in = motor_reg_cap.regen_torque(load, rpm, motor_speed_map_rpm, motor_max_torque_map_Nm_against_speed, motor_max_regen_torque_map_Nm_against_speed)
    
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

    motor_input_pwr_req = solver.intplt2d(rpm,torque_in,motor_speed_map_rpm,motor_torque_map_Nm,motor_input_power_map)
    
    # motor efficiency interpolation

    motor_eff_frac_req = solver.intplt2d(rpm,torque_in,motor_speed_map_rpm,motor_torque_map_Nm,motor_eff_frac_map)
    motor_eff_frac_req = motor_eff_frac_req + motor_eff_coolant

    # without current limit

    motor_output_pwr = solver.output_pwr(motor_input_pwr_req,motor_eff_frac_req)
    motor_current_req = solver.current(motor_input_pwr_req,motor_actual_voltage)
    motor_torque_out = motor_output_pwr/(solver.rpm2radps(rpm))
    motor_power_loss = solver.power_loss(motor_input_pwr_req,motor_output_pwr)
    motor_regen_power = motor_reg_cap.regen_power(motor_output_pwr)

    # current limit

    motor_corrected_current_req = np.minimum(motor_current_req,motor_max_current)
    motor_corrected_input_pwr = solver.input_pwr(motor_actual_voltage,motor_corrected_current_req)
    motor_corrected_output_pwr = solver.output_pwr(motor_corrected_input_pwr,motor_eff_frac_req)
    motor_corrected_torque_out = motor_corrected_output_pwr/(solver.rpm2radps(rpm))
    motor_corrected_power_loss = solver.power_loss(motor_input_pwr_req, motor_corrected_output_pwr)
    motor_correected_regen_power = motor_reg_cap.regen_power(reg_eff*motor_corrected_output_pwr)
    motor_surface_delta_T_K = solver.delta_T_K(abs(motor_corrected_power_loss),motor_mass_kg,motor_cp)
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
    speed_rdps = motor_output_power_W/motor_torque_out_Nm
    mps_speed = speed_rdps * rolling_radius_m
    vehicle_speed_kmph = mps_speed/0.277778

    motor_surface_temperature = 303
    '''# Update 'temperature.xlsx'
    file_path = 'temperature.xlsx'
    workbook = load_workbook(file_path)
    sheet = workbook['Sheet1']
    sheet['A2'] = motor_surface_temperature[-1]
    workbook.save(file_path)'''

    # print function

    display_motor_dyno_input()
    #display_motor_specification()
    display_motor_dyno_output()
    return motor_name, motor_manufacturer_name, motor_min_voltage, motor_max_current, motor_max_speed_rpm, motor_max_torque_Nm, motor_max_regen_torque_Nm, motor_torque_out_Nm, motor_voltage_V, motor_current_A, motor_eff_percentage, motor_surface_temperature_K, motor_output_power_W, motor_regen_power_W, rpm, motor_current_A, motor_voltage_V, motor_input_power_W, motor_eff_percentage, motor_output_power_W, motor_power_loss_W, motor_torque_out_Nm, motor_regen_power_W, motor_surface_temperature_K, motor_surface_change_in_temp_K, motor_coolant_flow_mps, load, voltage, coolant_flow
if __name__ == "__main__":
    drive_cycle = 1
    rolling_radius_m = float(sys.argv[5]) if len(sys.argv) > 5 else 0.1  
    if drive_cycle == 1:
        speed_array = eudc_speed
    elif drive_cycle == 2:
        speed_array = ind_hwy_speed
    elif drive_cycle == 3:
        speed_array = ind_urb_speed
    elif drive_cycle == 4:
        speed_array = udds_speed
    else:
        raise ValueError("Invalid drive cycle selected")

    


    for speed_kmph in speed_array:
        rpm_value = kmph_to_rpm(speed_kmph, rolling_radius_m)
        motor_program(rpm_value, 198, 200, 1)
