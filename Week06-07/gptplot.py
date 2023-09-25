import matplotlib.pyplot as plt

data = {
    'Orange_0': {'y': -0.3465274530751806, 'x': 0.44086890522452105, 'cluster': 7},
    'Lime_0': {'y': -0.5497856029774804, 'x': 0.8185241136969592, 'cluster': 2},
    'potato_0': {'y': -0.6116480160129116, 'x': -0.7683758335802652, 'cluster': 6},
    'Capsicum_0': {'y': 0.054469405059353916, 'x': -0.5917561968347066, 'cluster': 1},
    'Tomato_0': {'y': 0.9730163534587588, 'x': -0.5639423977578565, 'cluster': 5},
    'potato_1': {'y': 1.767615165774274, 'x': -1.3338511361251175, 'cluster': 3},
    'Orange_1': {'y': 0.8174120559532374, 'x': 0.023335248660684704, 'cluster': 9},
    'Garlic_0': {'y': 0.7459377109096812, 'x': 1.2279058421747164, 'cluster': 4},
    'Pumpkin_0': {'y': 1.1532420527254796, 'x': 0.4268824434102599, 'cluster': 0},
    'potato_2': {'y': 1.1177477111858307, 'x': -0.5559445047130249, 'cluster': 5},
    'Lemon_0': {'y': 0.5377345449098834, 'x': 0.9180674548513488, 'cluster': 8},
    'Garlic_1': {'y': 0.8111317172054333, 'x': 1.3247450086571457, 'cluster': 4},
    'Garlic_2': {'y': 0.9106733936376867, 'x': 1.304737425230115, 'cluster': 4},
    'Orange_2': {'y': -0.24376780187840869, 'x': 0.5199768578905656, 'cluster': 7},
    'Lime_1': {'y': -0.4928440282999468, 'x': 0.8967740588408023, 'cluster': 2}
}
# Extract x, y, and cluster data
x_values = [point['x'] for point in data.values()]
y_values = [point['y'] for point in data.values()]
clusters = [point['cluster'] for point in data.values()]

# Create a list of unique clusters for coloring
unique_clusters = list(set(clusters))

# Define colors for clusters
colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'purple', 'orange', 'pink']

# Create a scatter plot
plt.figure(figsize=(10, 6))
for i, cluster in enumerate(unique_clusters):
    x_cluster = [x_values[j] for j, c in enumerate(clusters) if c == cluster]
    y_cluster = [y_values[j] for j, c in enumerate(clusters) if c == cluster]
    plt.scatter(x_cluster, y_cluster, label=f'Cluster {cluster}', color=colors[i])

# Set labels and legend
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Data Clustering')
legend = plt.legend(loc=0, title='Clusters', bbox_to_anchor=(1.05, 1), borderaxespad=0.)
plt.gca().set_xlim([-1.5, 1.5])
plt.gca().set_ylim([-1.5, 1.5])

# Show the plot
plt.grid(True)
plt.show()