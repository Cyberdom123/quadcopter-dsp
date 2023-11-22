import unittest
from logger import DataLogger

class TestLogger(unittest.TestCase):
    def __init__(self, methodName: str = "runTest") -> None:
        self.dataLogger = DataLogger('my-filename.txt')
        super().__init__(methodName)
    
    def test_log_data(self):
        for i in range(10):
            self.dataLogger.log_data(123.12, 123.22, 1222.1, 1.1, 2.22)
        print(self.dataLogger.dataFrame)
        

if __name__ == '__main__':
    unittest.main()