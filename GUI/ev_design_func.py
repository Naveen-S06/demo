import numpy as np
from scipy.interpolate import RegularGridInterpolator as RGI
from math import isnan

def kmph2mps(speed):
    speed_mps = 0.277778*speed
    return speed_mps

def mph2mps(speed):
    speed_mps = 0.44704*speed
    return speed_mps

def mps2radps(speed_mps,rolling_radius_m):
    speed_radps = speed_mps/rolling_radius_m
    return speed_radps

def radps2rpm(speed):
    speed_rpm = 9.5493*speed
    return(speed_rpm)

def rpm2radps(speed):
    speed_radps = 0.10472*speed
    return(speed_radps)


# rolling resistance calculation

def friction_force(mnu,mass_kg):
    force = mnu*mass_kg*9.81
    return force

def aero_force(cd,density_of_air,frontal_area,speed):
    force = 0.5*cd*density_of_air*frontal_area*speed**2
    return force

def grade_force(mass_kg,grade_degree):
    grade = 0.0174533*grade_degree
    grade = np.asfarray(grade)
    sin_grade = np.sin(grade)
    force = mass_kg*9.81*sin_grade
    return force

def rr_force(f_friction,f_aero,f_grade):
    force = f_friction + f_aero + f_grade
    return force

# torque calculation

def req_torque(force,rolling_radius,gear_ratio,motor_gear_ratio):
    torque = force*rolling_radius/(gear_ratio*motor_gear_ratio)
    return torque

# data cleaning

def clean_data(input_data):
    data = [item for item in input_data if not(isnan(item)) == True]
    cleaned_data = np.array(data)
    return cleaned_data


# motor_calculation

def output_pwr(input_power, efficiency_fraction):
    output = input_power*efficiency_fraction
    return output

def power_loss(input_power,output_power):
    pwr_loss = input_power - output_power
    return pwr_loss

def current(input_power,voltage):
    curnt = input_power/voltage
    return curnt

def pwr2radps(power,torque):
    spd_raps = power/torque
    return spd_raps

# cooling calculation

def delta_T_K(power_loss,mass_kg, cp):
    temp_K = power_loss/(mass_kg*cp)
    return temp_K


def temperature_K(power_loss,mass_kg, cp, atm_temp_K):
    temp_K = power_loss/(mass_kg*cp) + atm_temp_K
    return temp_K

# Interpolation

def intplt2d(x_value,y_value,x_map,y_map,z_map):
    int_func = RGI((x_map,y_map), z_map)
    z_value = int_func([x_value,y_value])
    return z_value

