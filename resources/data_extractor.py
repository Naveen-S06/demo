import pandas as pd
import numpy as np
from math import isnan

class ExcelDataExtractor:
    def __init__(self, file_path, sheet_number):
        self.file_path = file_path
        self.sheet_number = sheet_number
        self.data_frame = None

    def read_excel(self):
        try:
            self.data_frame = pd.read_excel(self.file_path, sheet_name=self.sheet_number)
        except Exception as e:
            print(f"Error reading Excel file: {e}")

    def extract_data(self):
        if self.data_frame is not None:
            extracted_data = {}
            for column_name in self.data_frame.columns:
                setattr(self, column_name, self.data_frame[column_name].tolist())
                #print(f"Extracted data for variable '{column_name}'")
            self.extracted_data_frame = pd.DataFrame(extracted_data)
        else:
            print("No data available. Call 'read_excel' method first.")

    def display_data(self):
        if self.data_frame is not None:
            print(f"Data in DataFrame (Sheet Index: {self.sheet_number}):")
            print(self.data_frame)
        else:
            print("No data available. Call 'read_excel' method first.")

def clean_data(input_data):
    data = [item for item in input_data if not(isnan(item)) == True]
    cleaned_data = np.array(data)
    return cleaned_data

