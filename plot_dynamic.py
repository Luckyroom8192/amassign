import matplotlib.pyplot as plt

# Read the output file
waypoints = []
trajectory = []
obstacle_positions = []

with open('output_final.txt', 'r') as f:
    for line in f:
        parts = line.strip().split(',')
        if parts[0] == 'W':
            waypoints.append((float(parts[2]), float(parts[3])))  # (x, y)
        elif parts[0].startswith('O'):
            obstacle_positions.append((float(parts[2]), float(parts[3])))  # (x, y)
        elif parts[0] == 'T':
            trajectory.append((float(parts[2]), float(parts[3])))  # (x, y)

# Separate x, y for waypoints and trajectory
wp_x, wp_y = zip(*waypoints)
traj_x, traj_y = zip(*trajectory)

# Plot
plt.figure(figsize=(10, 10))
plt.plot(wp_x, wp_y, 'b-', label='Path')  # Blue line for waypoints
plt.plot(traj_x, traj_y, 'r-', label='Vehicle Trajectory')  # Red line for vehicle
for i, pos in enumerate(obstacle_positions):
    plt.plot(pos[0], pos[1], 'ko', label=f'Obstacle {i+1}' if i == 0 else "")  # Black dots for obstacles with numbering
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Vehicle Trajectory with Receding Horizon Control')
plt.legend()
plt.axis('equal')  # Equal aspect ratio
plt.grid(True)
plt.show()