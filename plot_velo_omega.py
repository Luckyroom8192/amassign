import matplotlib.pyplot as plt

# Initialize lists to store time and omega values
times = []
omegas = []

# Read the output file
with open('output_final.txt', 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        if parts[0] == 'T':
            times.append(float(parts[1]))    # Time is the 2nd field (index 1)
            omegas.append(float(parts[7]))   # Omega is the 8th field (index 7)

# Create the plot
plt.plot(times, omegas, label='Omega (rad/s)', color='blue')
plt.xlabel('Time (s)')
plt.ylabel('Omega (rad/s)')
plt.title('Angular Velocity over Time')
plt.grid(True)
plt.legend()
plt.savefig('omega_plot.png')  # Optional: Save the plot as an image
plt.show()