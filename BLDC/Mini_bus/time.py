import pandas as pd

# Function to load time values from a specific sheet in the Excel file and export them to a Python file
def export_time_values_to_python(file_path, sheet_name, output_file):
    # Read the Excel file
    df = pd.read_excel(file_path, sheet_name=sheet_name)
    
    # Extract the values from the "time_s" column
    time_values = df['time_s'].values.tolist()
    
    # Write the time values to a Python file
    with open(output_file, "w") as py_file:
        py_file.write("time_values = " + str(time_values))
    
    print("Time values from sheet '{}' exported to {}".format(sheet_name, output_file))

# Provide the file paths and sheet name
excel_file_path = "drive_cycle.xlsx"
sheet_name = "CYC_UDDS"
python_output_file = "udds_time_values.py"

# Call the function to export time values to the Python file
export_time_values_to_python(excel_file_path, sheet_name, python_output_file)
