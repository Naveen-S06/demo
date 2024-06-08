import resources
import numpy as np
from random import uniform as uf
from vehicle_speed import eudc_speed, ind_hwy_speed, ind_urb_speed, udds_speed

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error
        control_signal = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return control_signal

loads=[]
torque=[]
eff=[]
current=[]
veh=[]
acc=[]
inp_speed=[]

def kmph_to_rpm(speed_kmph, rolling_radius_m):
    speed_mps = 0.277778 * speed_kmph
    speed_radps = speed_mps / rolling_radius_m
    rpm = 9.5493 * speed_radps
    return rpm

def control_strategy(desired_speed, current_speed):
    current_speed_array = np.array(current_speed)
    speed_difference = desired_speed - current_speed_array
    return speed_difference

def motor_program(rpm, load, voltage, coolant_flow, rolling_radius_m, gear_ratio):
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
        print(f'Required Torque (Nm) : {load}')
        print(f'Torque Available (Nm) : {motor_torque_out_Nm}')
        #print(f'Vehicle Speed (kmph) : {veh_speed_kmph}')
        #print(f'Voltage (V) : {motor_voltage_V}')
        print(f'Current (A) : {motor_current_A}')
        #print(f'Power Input (W) : {motor_input_power_W}')
        #print(f'Power Output (W) : {motor_output_power_W}')
        #print(f'Regen Output (W) : {motor_regen_power_W}')
        print(f'Efficiency (%) : {motor_eff_percentage}')
        #print(f'Power Loss (W) : {motor_power_loss_W}')
        #print(f'Motor Surface Change in Temperature (K) : {motor_surface_change_in_temp_K}')
        #print(f'Motor Surface Temperature (K) : {motor_surface_temperature_K}')

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
    'motor_name': ' BLDC Motor 10kw',
    'motor_manufacturer_name': 'Golden Motor Technology Co Ltd',
    'motor_masss_kg': 70,
    'motor_surface_area_m2': 0.332889153429086,
    'motor_cp_JpkgpK': 430,
    'motor_max_torque_Nm': 33,
    'motor_max_regen_torque_Nm': 0,
    'motor_min_speed_rpm': 0,
    'motor_max_speed_rpm': 4500,
    'motor_min_voltage': 96,
    'motor_max_current': 161,
    'motor_cost_rs': 30000,
    'motor_inertia': 0.1,
    'motor_speed_map_rpm': [0, 943, 1879, 3868 ,3916, 3979, 4112, 4309, 4606, 4750, 4762],
    'motor_torque_map_Nm': [0, 1, 2, 4, 8, 10, 13, 20, 29, 32, 33],
    'motor_max_torque_map_Nm_against_speed': [0, 18, 25, 31, 33, 32, 28, 20, 18, 15, 12],
    'motor_max_regen_torque_map_Nm_against_speed': [-0, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0]
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
    [0.00, 2495.59, 3113.34, 3731.08, 4966.57, 5584.32, 6202.06, 6202.06, 13832.26, 14902.84, 15377.01],
    [650.53, 2559.21, 3160.56, 3744.30, 4923.78, 5569.11, 5920.26, 8502.75, 12994.64, 14333.23, 14702.54],
    [649.48, 2625.03, 3183.67, 3790.52, 4976.34, 5430.62, 6131.44, 7934.56, 12366.61, 13781.22, 14111.16],
    [648.43, 2660.35, 3239.20, 3770.84, 4886.52, 5573.97, 5991.91, 8379.48, 11694.58, 13286.06, 13583.47],
    [643.17, 2741.72, 3255.27, 3816.50, 4911.42, 5404.47, 6021.12, 7581.31, 10045.41, 11414.51, 11704.11],
    [638.96, 2681.28, 3214.12, 3716.55, 4760.32, 5302.11, 5802.43, 7333.12, 9473.71, 10786.69, 11055.31],
    [635.80, 2644.24, 3103.49, 3673.33, 4719.93, 5161.36, 5771.72, 7216.64, 9479.01, 10762.99, 11045.69],
    [632.65, 2470.56, 3046.09, 3449.27, 4486.24, 5193.73, 5576.02, 7427.76, 9691.29, 11180.95, 11451.08],
    [628.44, 2103.63, 2651.88, 2981.26, 4029.89, 4982.42, 5283.39, 7831.64, 10686.69, 12375.73, 12660.68],
    [625.28, 1536.57, 1887.99, 2316.58, 3604.67, 3771.12, 5379.65, 7599.80, 12329.91, 13760.70, 14098.73],
    [623.18, 642.36, 1589.73, 1589.73, 1589.73, 1589.73, 6202.06, 6202.06, 13832.26, 14902.84, 15377.01]
    ])
    #print("Motor IP",motor_input_power_map)
    # motor efficiency map

    motor_eff_frac_map = np.array([[0.00, 0.87, 0.87, 0.87, 0.88, 0.88, 0.88, 0.91, 0.91, 0.37, 0.57],
    [0.45, 0.86, 0.86, 0.87, 0.87, 0.88, 0.88, 0.90, 0.88, 0.52, 0.57],
    [0.59, 0.83, 0.86, 0.86, 0.87, 0.88, 0.88, 0.89, 0.86, 0.67, 0.59],
    [0.86, 0.85, 0.84, 0.86, 0.87, 0.87, 0.88, 0.88, 0.85, 0.70, 0.86],
    [0.87, 0.86, 0.87, 0.86, 0.86, 0.87, 0.86, 0.87, 0.85, 0.85, 0.87],
    [0.89, 0.87, 0.87, 0.87, 0.87, 0.87, 0.86, 0.86, 0.86, 0.88, 0.89],
    [0.92, 0.88, 0.87, 0.87, 0.87, 0.86, 0.86, 0.86, 0.86, 0.88, 0.92],
    [0.92, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.86, 0.87, 0.92],
    [0.80, 0.81, 0.82, 0.84, 0.85, 0.86, 0.86, 0.87, 0.85, 0.79, 0.80],
    [0.12, 0.80, 0.83, 0.84, 0.86, 0.87, 0.87, 0.89, 0.87, 0.69, 0.57],
    [0.57, 0.87, 0.87, 0.87, 0.88, 0.88, 0.88, 0.91, 0.91, 0.57, 0.57]
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
    motor_corrected_torque_out = np.where(np.isnan(motor_corrected_torque_out), 0.0, motor_corrected_torque_out)
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
    vehicle_torque = motor_torque_out_Nm * gear_ratio
    speed_rdps = motor_output_power_W/vehicle_torque
    mps_speed = speed_rdps * rolling_radius_m
    veh_speed_kmph = mps_speed/0.277778
    if np.isnan(veh_speed_kmph):
        veh_speed_kmph = [0.00]
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
    loads.append(load)
    torque.append(motor_torque_out_Nm.tolist())
    eff.append(motor_eff_percentage.tolist())
    current.append(motor_current_A.tolist())
    return veh_speed_kmph

pid = PIDController(Kp=0.5, Ki=0.1, Kd=0.2)
def cycle(drive_cycle, rolling_radius_m, gear_ratio): 
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
    previous_c = None

    for speed_kmph in speed_array:
        print("Input Speed",speed_kmph)
        rpm_value = kmph_to_rpm(speed_kmph, rolling_radius_m)
        vehicle_speed_kmph = motor_program(rpm_value, 5, 200, 1, rolling_radius_m, gear_ratio)
        adjusted_speed_kmph = control_strategy(speed_kmph, vehicle_speed_kmph)
        #motor_program(kmph_to_rpm(adjusted_speed_kmph, rolling_radius_m), 100, 200, 1)
        for _ in range(100):
            control_signal = pid.update(adjusted_speed_kmph)
            vehicle_speed_kmph += control_signal
            adjusted_speed_kmph = control_strategy(speed_kmph, vehicle_speed_kmph)
        veh.append(vehicle_speed_kmph.tolist())
        inp_speed.append(speed_kmph)
        print("Vehicle Speed:", vehicle_speed_kmph)

        if previous_c is not None: 
            result = vehicle_speed_kmph - previous_c  
            print("Acceleration kmph/s:",result)
            acc.append(result.tolist())  
        previous_c = vehicle_speed_kmph

    current_data = [item for sublist in current for item in sublist]
    torque_data = [item for sublist in torque for item in sublist]
    eff_data = [item for sublist in eff for item in sublist]
    veh_speed_data = [item for sublist in veh for item in sublist]
    veh_acc_data = [item for sublist in acc for item in sublist]

    with open('values.py', 'w') as f:
        f.write('req_torque = {}\n'.format(loads))
        f.write('avail_torque = {}\n'.format(torque_data))   
        f.write('eff = {}\n'.format(eff_data))   
        f.write('current = {}\n'.format(current_data))
        f.write('veh_speed = {}\n'.format(veh_speed_data))
        f.write('inp_speed = {}\n'.format(inp_speed))    
        f.write('veh_acc = {}\n'.format(veh_acc_data)) 

cycle(2,0.1,6)