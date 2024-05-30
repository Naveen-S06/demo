import pandas as pd

# Define the path to your Excel file
excel_file_path = 'drive_cycle.xlsx'

# Load the Excel file
excel_data = pd.ExcelFile(excel_file_path)

# List all sheet names
print(excel_data.sheet_names)

# Define the sheet name and column name you want to export
sheet_name = 'CYC_INDIA_URBAN_SAMPLE'  # Change to your sheet name
column_name = 'speed_kmph'  # Change to your column name

# Load the specific sheet
sheet_data = pd.read_excel(excel_file_path, sheet_name=sheet_name)

# Extract the column values as a list
column_values = sheet_data[column_name].tolist()

# Save the list to a Python file
output_file_path = 'speedurb.py'
with open(output_file_path, 'w') as file:
    file.write(f"{column_name}_values = {column_values}")

print(f"Column values exported to {output_file_path}")
