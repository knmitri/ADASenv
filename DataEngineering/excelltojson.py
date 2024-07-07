import pandas as pd
import json

def create_sample_excel(file_path):
    # Sample data for the Excel file
    data = {
        'BrandA': pd.DataFrame({
            'Variables': ['Var1', 'Var2'],
            'Car1': ['ValueA1', 'ValueA2'],
            'Car2': ['ValueB1', 'ValueB2'],
        }),
        'BrandB': pd.DataFrame({
            'Variables': ['Var3', 'Var4'],
            'Car3': ['ValueC1', 'ValueC2'],
            'Car4': ['ValueD1', 'ValueD2'],
        })
    }

    # Write the data to an Excel file
    with pd.ExcelWriter(file_path, engine='openpyxl') as writer:
        for sheet_name, df in data.items():
            df.to_excel(writer, sheet_name=sheet_name, index=False)

    print(f"Excel file '{file_path}' created successfully.")

def read_excel_configuration(file_path):
    # Load the entire Excel file
    excel_file = pd.ExcelFile(file_path)
    
    # Dictionary to store the configuration data
    config_data = {}

    # Iterate through each sheet (each brand)
    for sheet_name in excel_file.sheet_names:
        # Read the sheet into a DataFrame
        df = pd.read_excel(file_path, sheet_name=sheet_name, engine='openpyxl')
        
        # Dictionary to store the configurations for the current brand
        brand_config = {}
        
        # Iterate through each column (each car), skipping the first column
        for car_name in df.columns[1:]:
            # Dictionary to store the variables for the current car
            car_config = {}
            
            # Iterate through each row (each variable)
            for idx, row in df.iterrows():
                variable_name = row.iloc[0]
                variable_value = row[car_name]
                car_config[variable_name] = variable_value
            
            # Add the car's configuration to the brand's configuration
            brand_config[car_name] = car_config
        
        # Add the brand's configuration to the main config data
        config_data[sheet_name] = brand_config
    
    return config_data

def write_config_to_json(config_data, output_json_path):
    # Write the configuration data to a JSON file
    with open(output_json_path, 'w') as json_file:
        json.dump(config_data, json_file, indent=4)
    
    print(f"Configuration data has been written to '{output_json_path}'")

def compare_json_excel(json_path, excel_path):
    # Read the configuration from the Excel file
    excel_config = read_excel_configuration(excel_path)
    
    # Read the configuration from the JSON file
    with open(json_path, 'r') as json_file:
        json_config = json.load(json_file)
    
    # Compare the configurations
    if excel_config == json_config:
        print("The configurations in the JSON file and the Excel sheet match.")
    else:
        print("The configurations in the JSON file and the Excel sheet do not match.")


#
#
#

def test_read_excel_configuration(self):
    # Read the configuration from the in-memory Excel file
    config = read_excel_configuration(self.excel_data)
    
    expected_config = {
        'BrandA': {
            'Car1': {'Var1': 'ValueA1', 'Var2': 'ValueA2'},
            'Car2': {'Var1': 'ValueB1', 'Var2': 'ValueB2'},
        },
        'BrandB': {
            'Car3': {'Var3': 'ValueC1', 'Var4': 'ValueC2'},
            'Car4': {'Var3': 'ValueD1', 'Var4': 'ValueD2'},
        }
    }
    
    self.assertEqual(config, expected_config)
#
#
#

# Create the sample Excel file
excel_file_path = 'sample_test_data.xlsx'
json_output_path = 'configurations.json'

# create_sample_excel(excel_file_path)

# Read the Excel file 
#config_data = read_excel_configuration(excel_file_path)

# write the configuration to a JSON file
#write_config_to_json(config_data, json_output_path)

# Compare the JSON file and the Excel sheet
compare_json_excel(json_output_path, excel_file_path)
