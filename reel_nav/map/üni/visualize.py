import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml

# ==== 1. Haritayı yükle ====
pgm_file = "day1_map.pgm"
yaml_file = "day1_map.yaml"
g2o_file = "pose_graph.g2o"

# PGM dosyasını oku
img = Image.open(pgm_file)
map_data = np.array(img)

# YAML dosyasından çözünürlük ve origin al
with open(yaml_file, "r") as f:
    map_meta = yaml.safe_load(f)

res = map_meta["resolution"]
origin_x, origin_y, _ = map_meta["origin"]

# ==== 2. g2o dosyasını oku ====
poses = {}
edges = []

with open(g2o_file, "r") as f:
    for line in f:
        parts = line.strip().split()

        if parts[0] == "VERTEX_SE2":
            node_id = int(parts[1])
            x = float(parts[2])
            y = float(parts[3])
            poses[node_id] = (x, y)

        elif parts[0] == "EDGE_SE2":
            id1 = int(parts[1])
            id2 = int(parts[2])
            edges.append((id1, id2))

# ==== 3. Koordinatları piksel konumuna dönüştür ====
def world_to_pixel(x, y):
    px = (x - origin_x) / res
    py = map_data.shape[0] - ((y - origin_y) / res)
    return px, py

pose_pixels = {nid: world_to_pixel(x, y) for nid, (x, y) in poses.items()}

# ==== 4. Harita ve pose graph çiz ====
plt.figure(figsize=(10, 10))
plt.imshow(map_data, cmap='gray', origin='upper')

# Kenarları çiz (mavi çizgiler)
for id1, id2 in edges:
    if id1 in pose_pixels and id2 in pose_pixels:
        x1, y1 = pose_pixels[id1]
        x2, y2 = pose_pixels[id2]
        plt.plot([x1, x2], [y1, y2], 'b-', linewidth=0.5)

# Düğümleri çiz (kırmızı noktalar)
px = [p[0] for p in pose_pixels.values()]
py = [p[1] for p in pose_pixels.values()]
plt.scatter(px, py, c='red', s=10, label="Pose Graph Nodes")

plt.legend()
plt.title("Pose Graph on Map")
plt.show()

