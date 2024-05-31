import pandas as pd


df = pd.read_excel("drive_cycle.xlsx", sheet_name='CYC_EUDC')

s = 3.51 
r = 0.25   

def calculate_rpm(vehicle_speed):
    return (vehicle_speed * s * 9.55) / (3.6 * r)

df['RPM'] = df['speed_kmph'].apply(calculate_rpm)

with open('eudc_rpm.py', 'w') as f:
    f.write("rpm_values = [")
    f.write(", ".join(str(rpm) for rpm in df['RPM']))
    f.write("]")
