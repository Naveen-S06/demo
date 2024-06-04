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
        print(f'Regen Power Output (W) : {motor_regen_power_W}')
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
    'motor_name': 'Lynx Motion 15-kW',
    'motor_manufacturer_name': 'Unique Mobility',
    'motor_masss_kg': 38.1,
    'motor_surface_area_m2': 0.2175,
    'motor_cp_JpkgpK': 430,
    'motor_max_torque_Nm': 59,
    'motor_max_regen_torque_Nm': -59,
    'motor_min_speed_rpm': 0,
    'motor_max_speed_rpm': 7000,
    'motor_min_voltage': 60,
    'motor_max_current': 300,
    'motor_cost_rs': 25000,
    'motor_inertia': 0.2175,
    'motor_speed_map_rpm': [0, 500, 1000, 1500, 2000, 2500, 3000, 4000, 5000, 6000, 7000],
    'motor_torque_map_Nm': [-59, -54, -45, -36, -27, -23, -18, -14, -9, -5, 0, 5, 9, 14, 18, 23, 27, 36, 45, 54, 59],
    'motor_max_torque_map_Nm_against_speed': [57.84, 57.40, 56.96, 56.52, 56.07, 55.63, 55.19, 54.31, 53.41, 52.53, 0.00],
    'motor_max_regen_torque_map_Nm_against_speed': [-57.84, -57.40, -56.96, -56.52, -56.07, -55.63, -55.19, -54.31, -53.41, -52.53, 0.00]
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
    [4077.42, 3763.77, 2891.92, 1747.29, 946.45, 788.71, 713.99, 655.23, 492.54, 386.05, 386.05, 386.05, 492.54, 655.23, 713.99, 788.71, 946.45, 1747.29, 2891.92, 3763.77, 4077.42],
    [1001.47, 924.44, 525.80, -145.61, -473.22, -394.35, -232.46, -54.60, 19.32, 149.44, 386.05, 622.66, 965.76, 1365.07, 1660.43, 1971.76, 2366.12, 3640.18, 5258.03, 6603.11, 7153.37],
    [-3515.37, -3244.96, -2799.35, -2313.54, -1789.17, -1468.62, -1138.38, -720.43, -366.37, 0.00, 473.22, 946.45, 1526.53, 2118.91, 2647.40, 3263.61, 3889.50, 5258.03, 6665.11, 8112.40, 8788.43],
    [-6424.60, -5930.40, -5037.54, -4030.03, -2986.84, -2458.90, -1841.73, -1259.71, -655.23, -54.60, 655.23, 1365.07, 2184.11, 2999.30, 3836.94, 4639.44, 5531.18, 7327.33, 9159.16, 11105.63, 12031.10],
    [-9380.12, -8658.58, -7171.83, -5678.68, -4077.00, -3436.14, -2654.97, -1789.17, -960.57, -140.21, 806.23, 1752.68, 2825.21, 3889.50, 4916.60, 6028.32, 7280.35, 9464.46, 11757.09, 14056.13, 15227.48],
    [-12229.68, -11288.94, -9492.80, -7525.96, -5540.17, -4527.75, -3436.14, -2428.38, -1352.07, -326.36, 856.70, 2039.75, 3380.16, 4669.96, 6028.32, 7302.83, 8656.52, 11402.97, 14168.36, 17104.45, 18529.82],
    [-15096.15, -13934.91, -11632.24, -9274.05, -6834.81, -5592.64, -4259.01, -2986.84, -1735.15, -433.12, 986.55, 2406.22, 3943.53, 5531.18, 7098.35, 8604.06, 10201.22, 13440.66, 16761.15, 20137.15, 21815.25],
    [-20865.43, -19260.40, -15999.97, -12779.76, -9415.70, -7728.38, -6020.77, -4303.10, -2557.28, -630.96, 1261.93, 3154.82, 5014.28, 7054.26, 9122.37, 11200.55, 13299.01, 17506.52, 21857.88, 26169.02, 28349.77],
    [-26840.87, -24776.19, -20676.86, -16420.79, -12018.88, -9904.67, -7661.71, -5487.07, -3318.71, -788.71, 1577.41, 3943.53, 6145.75, 8709.63, 11267.22, 13756.48, 16374.50, 21437.06, 26645.44, 32010.58, 34678.13],
    [-33260.82, -30702.30, -25412.86, -20106.44, -14800.53, -12075.35, -9508.48, -6773.36, -4030.03, -516.24, 2323.10, 5162.43, 7327.33, 10262.67, 13206.23, 16318.04, 19271.53, 25322.97, 31373.91, 37441.83, 40561.98],
    [-39318.66, -36294.15, -30048.37, -23718.67, -17493.89, -14410.58, -11270.32, -8044.79, -4532.98, 0.00, 3312.56, 6625.12, 8717.27, 11830.58, 15230.17, 18715.04, 22256.85, 29282.31, 36202.86, 43207.32, 46807.93]
    ])
    #print("Motor IP",motor_input_power_map)
    # motor efficiency map

    motor_eff_frac_map = np.array([
    [0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20],
    [0.43, 0.43, 0.45, 0.52, 0.60, 0.60, 0.57, 0.52, 0.49, 0.38, 0.20, 0.38, 0.49, 0.52, 0.57, 0.60, 0.60, 0.52, 0.45, 0.43, 0.43],
    [0.70, 0.70, 0.71, 0.72, 0.73, 0.72, 0.71, 0.67, 0.62, 0.50, 0.20, 0.50, 0.62, 0.67, 0.71, 0.72, 0.73, 0.72, 0.71, 0.70, 0.70],
    [0.77, 0.77, 0.78, 0.78, 0.77, 0.77, 0.74, 0.71, 0.65, 0.52, 0.20, 0.52, 0.65, 0.71, 0.74, 0.77, 0.77, 0.78, 0.78, 0.77, 0.77],
    [0.81, 0.81, 0.81, 0.80, 0.78, 0.79, 0.77, 0.73, 0.67, 0.54, 0.20, 0.54, 0.67, 0.73, 0.77, 0.79, 0.78, 0.80, 0.81, 0.81, 0.81],
    [0.83, 0.83, 0.83, 0.83, 0.82, 0.81, 0.79, 0.76, 0.70, 0.58, 0.20, 0.58, 0.70, 0.76, 0.79, 0.81, 0.82, 0.83, 0.83, 0.83, 0.83],
    [0.85, 0.85, 0.85, 0.84, 0.83, 0.82, 0.80, 0.77, 0.72, 0.59, 0.20, 0.59, 0.72, 0.77, 0.80, 0.82, 0.83, 0.84, 0.85, 0.85, 0.85],
    [0.87, 0.87, 0.87, 0.86, 0.85, 0.84, 0.83, 0.81, 0.76, 0.60, 0.20, 0.60, 0.76, 0.81, 0.83, 0.84, 0.85, 0.86, 0.87, 0.87, 0.87],
    [0.89, 0.89, 0.89, 0.88, 0.87, 0.86, 0.84, 0.81, 0.77, 0.60, 0.20, 0.60, 0.77, 0.81, 0.84, 0.86, 0.87, 0.88, 0.89, 0.89, 0.89],
    [0.91, 0.91, 0.91, 0.90, 0.88, 0.87, 0.86, 0.83, 0.78, 0.55, 0.20, 0.55, 0.78, 0.83, 0.86, 0.87, 0.88, 0.90, 0.91, 0.91, 0.91],
    [0.92, 0.92, 0.92, 0.91, 0.89, 0.89, 0.87, 0.84, 0.76, 0.50, 0.20, 0.50, 0.76, 0.84, 0.87, 0.89, 0.89, 0.91, 0.92, 0.92, 0.92]
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
drive_cycle = 4  
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

rolling_radius_m = 0.1


for speed_kmph in speed_array:
    rpm_value = kmph_to_rpm(speed_kmph, rolling_radius_m)
    motor_program(rpm_value, 100, 200, 1)
