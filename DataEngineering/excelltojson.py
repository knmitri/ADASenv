import pandas as pd

def read_excel_configuration(file_path):
    # Load the entire Excel file
    excel_file = pd.ExcelFile(file_path)
    
    # Dictionary to store the configuration data
    config_data = {}

    # Iterate through each sheet (each brand)
    for sheet_name in excel_file.sheet_names:
        # Read the sheet into a DataFrame
        df = pd.read_excel(file_path, sheet_name=sheet_name, usecols=, engine='openpyxl')
        
        # Dictionary to store the configurations for the current brand
        brand_config = {}
        
        # Iterate through each column (each car)
        for car_name in df.columns:
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

# Example usage
file_path = 'sample_test_data.xlsx'
configurations = read_excel_configuration(file_path)
print(configurations)





