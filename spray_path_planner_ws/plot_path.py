import csv
import matplotlib.pyplot as plt
x_list = []
y_list = []
index_list = []

with open("/home/hjh/spray_path_planner_ws/path_saved.csv", "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        index_list.append(int(row["index"]))
        x_list.append(float(row["x"]))
        y_list.append(float(row["y"]))

plt.figure(figsize=(8, 8))
plt.plot(x_list, y_list, marker="o")

for i, (x, y) in enumerate(zip(x_list, y_list)):
    plt.text(x, y, str(index_list[i]))

plt.title("Sprayer Path Visualization (No pandas)")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis("equal")
plt.show()