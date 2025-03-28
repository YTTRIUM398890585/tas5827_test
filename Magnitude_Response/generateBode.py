import numpy as np
import pandas as pd
from scipy.fft import fft, fftfreq
import sys
import os
import matplotlib.pyplot as plt
import math

# Check if the correct number of arguments are provided
if len(sys.argv) != 3:
    print("Usage: python generateBode.py <input_csv_file> <output_directory>")
    sys.exit(1)

# Get the input CSV file and output directory from the arguments
input_csv_file = sys.argv[1]
output_directory = sys.argv[2]

# Check if the input CSV file exists
if not os.path.isfile(input_csv_file):
    print(f"Error: The file {input_csv_file} does not exist.")
    sys.exit(1)

# Check if the output directory exists, if not create it
if not os.path.isdir(output_directory):
    print(f"Error: The directory {output_directory} does not exist.")
    sys.exit(1)

# Read CSV file
data = pd.read_csv(input_csv_file)

# Extract time and voltage
# Skip the first rows and read the CSV file
data = pd.read_csv(input_csv_file, skiprows=1)

# Extract time and voltage
freq = data.iloc[:, 0].values
voltage = data.iloc[:, 1].values

print("freq: ", freq)
print("Voltage: ", voltage)

# change to floats
freq = [float(i) for i in freq]
voltage = [float(i) for i in voltage]

# Convert Vrms values to dBVrms (referencing 1Vrms)
voltages_dbvrms = [20 * math.log10(vrms / 1.0) for vrms in voltage]

# Plot FFT
plt.figure()
plt.plot(freq, voltages_dbvrms, marker='o')
plt.xscale('log')
# plt.ylim(bottom=0)
plt.grid(which='both', linestyle='--', linewidth=0.5)
plt.grid(which='major', linestyle='-', linewidth=1)
plt.xlabel('Frequency (Hz)')
plt.ylabel('Vpbtl (dBVrms)')
plt.title('Vpbtl vs Frequency')
plt.grid(True, which='both')
plt.savefig(output_directory + '/vdbvrms_vs_frequency_cleaned.png')
plt.show()
