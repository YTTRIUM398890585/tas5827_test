# NOTE: the default pyvisa import works well for Python 3.6+
# if you are working with python version lower than 3.6, use 'import visa' instead of import pyvisa as visa

import matplotlib.pyplot as plt
import pyvisa as visa
import csv
import math
import paramiko
import time

# CONSTANTS
# Couldnt figure out how to set static IP for the scope
# DEVICE = 'TCPIP0::169.254.212.129::INSTR'
DEVICE = 'TCPIP0::169.254.201.215::INSTR'
MEASUREMENT_CHANNEL = 'MATH1'
DELAY = 10  # Delay in seconds to wait for the signal to stabilize


def getVrms():
    # Get Vrms value from scope

    # Query the oscilloscope for the Vrms value
    # CYCle to only take integer number of cycles on the screen
    # AC to take the AC component of the signal
    # MEASUREMENT_CHANNEL to specify the channel to measure
    temp_values = MSO_X_3024T.query_ascii_values(
        ':MEASure:VRMS? %s,%s,%s' % ('CYCLe', 'AC', MEASUREMENT_CHANNEL))

    # temp_values = MSO_X_3024T.query_ascii_values(
    #     ':MEASure:VPP? %s' % (MEASUREMENT_CHANNEL))

    print("getVrms = " + str(temp_values[0]))
    # Return the Vrms value
    return temp_values[0]


def getTHD(freq):
    # temp_values = MSO_X_3024T.query_ascii_values(
    #     ':MEASure:FFT:THD? %s,%G,%s' % ('AUTO', freq, 'FFT'))
    # print("getTHD @ " + str(freq) + " Hz = " + str(temp_values[0]))
    temp_values = MSO_X_3024T.query_ascii_values(
        ':MEASure:FFT:THD? %s,%s' % ('AUTO', 'FFT'))
    print("getTHD @ " + str(freq) + " Hz = " + str(temp_values[0]))
    return temp_values[0]


def setFreq(freq):
    # # Set RPi to generate signal and play through I2S
    # command = 'python play_sine.py -d 0 ' + str(freq)

    # # ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command('touch test.txt')
    # ssh_stdin, ssh_stdout, ssh_stderr = ssh.exec_command(command)
    # print("command = " + str(command))
    # print("ssh_stdout = " + str(ssh_stdout.read().decode("ascii")))
    # print("ssh_stderr = " + str(ssh_stderr.read().decode("ascii")))
    # print("\n")

    # Set RPi to generate signal and play through I2S
    command = 'python play_sine_modified.py -d 0 ' + str(freq)
    channel.send(command + '\n')
    time.sleep(1)  # Wait for the command to execute
    output = channel.recv(1024).decode('ascii')
    print("command = " + str(command))
    print("output = " + output)
    print("\n")


def stopSine():
    # Send CTRL+C to stop the sine wave generation
    channel.send(chr(3))


# Define the set of frequencies to loop through
# Every integer multiple of 1000 Hz from 100 Hz to 80 kHz
frequencies = [1000 * i for i in range(1, 81)]
# Short one for sanity check
# frequencies = [1000, 10000, 50000, 60000, 70000, 80000]

# Create a list to store the results
voltages = []
thds = []

# Open the resource manager
rm = visa.ResourceManager()

# Open the oscilloscope
MSO_X_3024T = rm.open_resource(DEVICE)

# SSH into the Raspberry Pi
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.1.11', username='nav', password='bb')  # lol

# Open a channel
channel = ssh.invoke_shell()

# setup rpi
commands = [
    'cd pythonTestEnv',
    'source testEnv/bin/activate',
    'cd python-sounddevice/examples/'
]

for command in commands:
    channel.send(command + '\n')
    time.sleep(1)  # Wait for the command to execute
    output = channel.recv(1024).decode('ascii')
    print("command = " + str(command))
    print("output = " + output)
    print("\n")

# estimate time to run
print("Estimated time to run: " +
      str(DELAY * len(frequencies)) + " seconds")

# Write the results to a CSV file
with open('results.csv', 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    csvwriter.writerow(['Frequency (Hz)', 'Vpbtl (Vrms)', 'THD (%)'])

    # Loop through the set of frequencies
    for freq in frequencies:
        # Set the frequency
        setFreq(freq)

        # Wait for the signal to stabilize
        time.sleep(DELAY)

        # Get the Vrms value
        vrms = getVrms()

        # Get the THD value
        thd = getTHD(freq)

        # Stop the sine wave generation
        stopSine()

        # Store the frequency and Vpp and THD value in the results list
        voltages.append(vrms)
        thds.append(thd)

        csvwriter.writerow([freq, vrms, thd])

# Close the oscilloscope
MSO_X_3024T.close()

# Close the resource manager
rm.close()

# Convert Vrms values to dBVrms (referencing 1Vrms)
voltages_dbvrms = [20 * math.log10(vrms / 1.0) for vrms in voltages]

# Plot the results using matplotlib in dBV scale and log frequency scale
plt.figure()
plt.plot(frequencies, voltages_dbvrms, marker='o')
plt.xscale('log')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Vpbtl (dBVrms)')
plt.title('Vpbtl vs Frequency')
plt.grid(True, which='both')
plt.savefig('vdbvrms_vs_frequency.png')
plt.show()

# Plot THD vs Frequency
plt.figure()
plt.plot(frequencies, thds, marker='o')
plt.xscale('log')
plt.xlabel('Frequency (Hz)')
plt.ylabel('THD (%)')
plt.title('THD vs Frequency')
plt.grid(True, which='both')
plt.savefig('thd_vs_frequency.png')
plt.show()
