import matplotlib.pyplot as plt
import json

# Function to draw a square with a centered number
def draw_square(ax, x, y, text):
    size = 0.08  # Box side length in meters (8 cm)
    square = plt.Rectangle((x - size/2, y - size/2), size, size, fill=False, color='white', linewidth=2)
    ax.add_patch(square)
    ax.text(x, y, text, ha='center', va='center', color='white', fontsize=12)

# Create a 3x3 meter arena
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.set_facecolor('darkgrey')

# Draw the arena outline
arena_outline = plt.Rectangle((-1.5, -1.5), 3, 3, fill=False, color='white', linewidth=2)
ax.add_patch(arena_outline)

# Load box positions from the "TrueMap.txt" file
with open("TrueMap.txt", "r") as file:
    box_data = json.load(file)

# Draw the boxes
for i in range(1, 11):
    box_name = f"aruco{i}_0"
    if box_name in box_data:
        x = box_data[box_name]["x"]
        y = box_data[box_name]["y"]
        draw_square(ax, x, y, text=str(i))

# Show the plot
plt.gca().invert_yaxis()
plt.gca().set_aspect('equal', adjustable='box')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Arena with Boxes (Box Side Length: 8 cm)')
plt.grid(True)
plt.show()
