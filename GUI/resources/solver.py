from numpy import sin 
from scipy.interpolate import RegularGridInterpolator as RGI
from math import isnan

class Solver():

    KMPH2MPS = 0.277778
    MPH2MPS = 0.44704
    RADPS2RPM = 9.5493
    RPM2RADPS = 0.10472
    RAD2DEG = 0.0174533
    G = 9.81

    def __init__(self):
        pass

    def kmph2mps(speed):
        return Solver.KMPH2MPS*speed

    def mph2mps(speed):
        return Solver.MPH2MPS*speed

    def mps2radps(speed_mps,rolling_radius_m):
        return speed_mps/rolling_radius_m

    def radps2rpm(speed):
        return Solver.RADPS2RPM*speed

    def rpm2radps(speed):
        return Solver.RPM2RADPS*speed

    # rolling resistance calculation

    def friction_force(mnu,mass_kg):
        return Solver.G*mnu*mass_kg

    def aero_force(cd,density_of_air,frontal_area,speed):
        force = 0.5*cd*density_of_air*frontal_area*speed**2
        return force

    def grade_force(mass_kg,grade_degree):
        grade = int(Solver.RAD2DEG*grade_degree)
        sin_grade = sin(grade)
        force = Solver.G*mass_kg*sin_grade
        return force

    def rolling_resistance_force(f_friction,f_aero,f_grade):
        return f_friction + f_aero + f_grade

    # torque calculation

    def torque(force,rolling_radius,gear_ratio,motor_gear_ratio):
        return force*rolling_radius/(gear_ratio*motor_gear_ratio)

    # motor_calculation

    def input_pwr(voltage,current):
        return voltage*current
     

    def output_pwr(input_power, efficiency_fraction):
        return input_power*efficiency_fraction
    
    def power_loss(input_power,output_power):
        return input_power - output_power

    def current(input_power,voltage):
        return input_power/voltage

    def pwr2radps(power,torque):
        return power/torque

    #cooling calculation

    def delta_T_K(power_loss,mass_kg, cp):
        return power_loss/(mass_kg*cp)


    def temperature_K(power_loss,mass_kg, cp, atm_temp_K):
        return power_loss/(mass_kg*cp) + atm_temp_K

    # Interpolation

    def intplt2d(x_value,y_value,x_map,y_map,z_map):
        int_func = RGI((x_map,y_map), z_map)
        return int_func([x_value,y_value])

