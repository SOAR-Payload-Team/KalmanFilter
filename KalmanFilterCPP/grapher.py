import matplotlib.pyplot as plt
import numpy as np


def main():
    # Read in raw data
    rawFile = open('fixedAltitudeData.txt', "r")
    lines = rawFile.readlines()
    rawData = []
    for line in lines:
        rawData.append(float(line))
    rawFile.close()

    # Read in filtered data
    filteredFile = open('KalmanFilterOuput.txt', "r")
    lines = filteredFile.readlines()
    filteredData = []
    for line in lines:
        filteredData.append(float(line))
    filteredFile.close()

    plt.plot(filteredData, label="Filtered Data")
    plt.plot(rawData, label="Raw data")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
