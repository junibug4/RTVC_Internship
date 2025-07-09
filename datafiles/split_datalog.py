#%%
import numpy as np
import matplotlib.pyplot as plt
#%%
import os
# os.chdir('../')  # Change to the directory where the data file is located
#%% ===----------- Load data from file ----------=== %%#

filename = 'datafiles/223_multiple_runs.TXT'

data = []
pitch_array = []
with open(filename, 'r') as file:
    for line in file:
        if "Pitch" in line:
            try:
                # Extract pitch value
                pitch_part = line.split('Pitch = ')[1].split(' Fan angle = ')[0]
                pitch = float(pitch_part)
                pitch_array.append(pitch)

                # Extract fan angle value
                fan_angle_part = line.split('Fan angle = ')[1].split(' P = ')[0]
                fan_angle = float(fan_angle_part)                
                
                # Extract PID parameters
                parts = line.split('P = ')[1].split(' I = ')
                p = float(parts[0])
                
                parts = line.split('I = ')[1].split(' D = ')
                i = float(parts[0])
                
                parts = line.split('D = ')[1].split(" ")
                d = float(parts[0])
                
                data_array = [pitch, p, i, d, fan_angle]

                data.append(data_array)
            except (IndexError, ValueError) as e:
                print(f"Error parsing line: {line.strip()}. Error: {e}")

print(np.shape(data))

print(data[:3])

plt.plot(pitch_array)

#%% ----------- splitting data by PID parameters ----------- %%#

pid_213 = []
pid_413 = []
pid_233 = []
pid_215 = []

for i in data:
    if i[1] == .2 and i[2] == .1 and i[3] == .3:
        pid_213.append(i)
    elif i[1] == .4 and i[2] == .1 and i[3] == .3:
        pid_413.append(i)
    elif i[1] == .2 and i[2] == .3 and i[3] == .3:
        pid_233.append(i)
    elif i[1] == .2 and i[2] == .1 and i[3] == .5:
        pid_215.append(i)
    else:
        print(f"Unrecognized PID parameters: P={i[1]}, I={i[2]}, D={i[3]}")

print(f"PID 213: {len(pid_213)} entries")
print(f"PID 413: {len(pid_413)} entries")     
print(f"PID 233: {len(pid_233)} entries")
print(f"PID 215: {len(pid_215)} entries")



#%% ------------- splitting data by time ------------------- %%#

splits = [3.05,7.5,10.9,14.3]

pid_233_2 , pid_233_3 , pid_233_4 , pid_233_5 = [] , [] , [] , []

timeAxis = np.linspace(0,(len(pid_233)*0.008),len(pid_233))
plt.plot(pid_233,timeAxis)

for i in range (len(pid_233)):
    if (timeAxis[i] > splits[0] and timeAxis[i] < splits[1]):
        pid_233_2.append(pid_233[i][0])
    elif (timeAxis[i] > splits[1] and timeAxis[i] < splits[2]):
        pid_233_3.append(pid_233[i][0])
    elif (timeAxis[i] > splits[2] and timeAxis[i] < splits[3]):
        pid_233_4.append(pid_233[i][0])
    elif (timeAxis[i] > splits[3]):
        pid_233_5.append(pid_233[i][0])

print(f"PID 223_2: {len(pid_233_2)} entries")
print(f"PID 223_3: {len(pid_233_3)} entries")
print(f"PID 223_4: {len(pid_233_4)} entries")
print(f"PID 223_5: {len(pid_233_5)} entries")



timeAxis = timeAxis[:len(pid_233_2)]  # Adjust time axis to match the length of pid_233

plt.plot(pid_233_2, label = '2')
plt.plot(pid_233_3, label = '3')
plt.plot(pid_233_4, label = '4')
plt.plot(pid_233_5, label = '5')
plt.legend()
plt.plot()


#%% ---- Write data to files ---- %%#

def write_data_to_file(data, filename):
    with open(filename, 'w') as file:
        for entry in data:
            file.write(str(entry)+'\n')

#%%

print(type(pid_233_2[0]))

# write_data_to_file(pid_233, 'datafiles/pid_233_multi.txt')

write_data_to_file(pid_233_2, 'datafiles/pid_233_2.txt')
write_data_to_file(pid_233_3, 'datafiles/pid_233_3.txt')
write_data_to_file(pid_233_4, 'datafiles/pid_233_4.txt')
write_data_to_file(pid_233_5, 'datafiles/pid_233_5.txt')

# %%
