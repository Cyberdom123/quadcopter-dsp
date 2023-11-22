import pandas as pd

class DataLogger:
    def __init__(self, fileName) -> None:
        self.dataFrame = pd.DataFrame()
        self.fileName = fileName

    def log_data(self, *args):
        record = []
        for data in args:
             record.append(data)
        self.dataFrame = pd.concat([self.dataFrame, pd.DataFrame(record)], axis=1,
                                   ignore_index=True)
        
    def save_data(self):
        self.dataFrame = self.dataFrame.transpose()
        self.dataFrame.to_csv(self.fileName)

    def get_data(self):
        return self.dataFrame