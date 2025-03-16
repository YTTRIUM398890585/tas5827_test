import numpy as np
import pandas as pd
from scipy.fft import fft, fftfreq
import sys
import os
import matplotlib.pyplot as plt

# Check if the correct number of arguments are provided
if len(sys.argv) != 3:
    print("Usage: python generateFFT.py <input_csv_file> <output_directory>")
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
# Skip the first two rows and read the CSV file
data = pd.read_csv(input_csv_file, skiprows=2)

# Extract time and voltage
time = data.iloc[:, 0].values
voltage = data.iloc[:, 3].values

print("Time: ", time)
print("Voltage: ", voltage)

# change to floats
time = [float(i) for i in time]
voltage = [float(i) for i in voltage]

# Compute FFT
N = len(time)
T = time[1] - time[0]  # Sample spacing
yf = fft(voltage)
xf = fftfreq(N, T)[:N//2]

# Plot FFT
plt.figure(figsize=(10, 6))
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.xscale('log')
plt.grid(which='both', linestyle='--', linewidth=0.5)
plt.grid(which='major', linestyle='-', linewidth=1)
plt.title('FFT of Voltage Signal')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Amplitude')
plt.savefig(output_directory + '/fft_plot.png')
plt.show()
