import pandas as pd
import unittest
from io import BytesIO
from excelltojson import read_excel_configuration

class TestReadExcelConfiguration(unittest.TestCase):
    def setUp(self):
        # Create a sample Excel file in memory
        self.excel_data = BytesIO()
        
        # Sample data
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
        
        with pd.ExcelWriter(self.excel_data, engine='openpyxl') as writer:
            for sheet_name, df in data.items():
                df.to_excel(writer, sheet_name=sheet_name)
        
        self.excel_data.seek(0)

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

if __name__ == '__main__':
    unittest.main()
