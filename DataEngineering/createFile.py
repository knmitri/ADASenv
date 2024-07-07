import pandas as pd

# Sample data for the Excel file
data = {
    'BrandA': pd.DataFrame({
        'Car1': ['ValueA1', 'ValueA2'],
        'Car2': ['ValueB1', 'ValueB2'],
    }, index=['Var1', 'Var2']),
    'BrandB': pd.DataFrame({
        'Car3': ['ValueC1', 'ValueC2'],
        'Car4': ['ValueD1', 'ValueD2'],
    }, index=['Var3', 'Var4'])
}

# Write the data to an Excel file
file_path = 'sample_test_data.xlsx'
with pd.ExcelWriter(file_path, engine='openpyxl') as writer:
    for sheet_name, df in data.items():
        df.to_excel(writer, sheet_name=sheet_name)

print(f"Excel file '{file_path}' created successfully.")
