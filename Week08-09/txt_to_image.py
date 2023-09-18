import numpy as np
import matplotlib.pyplot as plt
import json

# ArUco MArkers are blue squares
# Tomato is red
# Lime is Green
# Orange is Orange
# Lemon is Yellow
# Garlic is white
# Pumpkin is Purple
# Potato is blue circle
# Capsicum is pink

f = open("TrueMap.txt", "r")
txt = f.readline()

ReferenceMap = json.loads(txt)

#print(ReferenceMap)
ReferenceObjects_x = []
ReferenceObjects_y = []
ReferenceArUcos_x = []
ReferenceArUcos_y = []
Objects_names = []
Objects_colours = [] 
j = 1

for Key in ReferenceMap:
    if j <= 10 :
        #Position = np.array([ReferenceMap[Key]["x"], ReferenceMap[Key]["y"]])
        ReferenceArUcos_x.append(-ReferenceMap[Key]["x"])
        ReferenceArUcos_y.append(-ReferenceMap[Key]["y"])
        j += 1

    elif j > 10 : 
        ReferenceObjects_x.append(-ReferenceMap[Key]["x"])
        ReferenceObjects_y.append(-ReferenceMap[Key]["y"])
        Objects_names.append(Key)
        j += 1

for object in Objects_names : 
    if (object == "Tomato_0") or (object == "Tomato_1") :
        Objects_colours.append("red")
    elif (object == "potato_0") or (object == "potato_1") :             # MUST CHANGE BEFORE DEMO 
        Objects_colours.append("skyblue")
    elif (object == "Orange_0") or (object == "Orange_1") : 
        Objects_colours.append("orange")
    elif (object == "Capsicum_0") or (object == "Capsicum_1") : 
        Objects_colours.append("pink")
    elif (object == "Lime_0") or (object == "Lime_1") : 
        Objects_colours.append("green")
    elif (object == "Lemon_0") or (object == "Lemon_1") : 
        Objects_colours.append("yellow")
    elif (object == "Garlic_0") or (object == "Garlic_1") : 
        Objects_colours.append("white")
    elif (object == "Pumpkin_0") or (object == "Pumpkin_1") : 
        Objects_colours.append("purple")
    


fig,ax = plt.subplots(figsize = (5, 5))
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])
ax.set_facecolor('xkcd:black')
ax.scatter(ReferenceArUcos_x, ReferenceArUcos_y, color = "blue", marker = "s")
ax.scatter(ReferenceObjects_x, ReferenceObjects_y, c = Objects_colours, marker = "o")
fig.savefig("map_image.png")

print(Objects_colours)
