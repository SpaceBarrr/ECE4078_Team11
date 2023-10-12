import matplotlib.pyplot as plt
import json

def visualise_map(map):
    '''
    ArUco MArkers are blue squares
    Tomato is red
    Lime is Green
    Orange is Orange
    Lemon is Yellow
    Garlic is white
    Pumpkin is Purple
    Potato is blue circle
    Capsicum is pink
    '''
    
    f = open(map, "r")
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
            ReferenceArUcos_x.append(ReferenceMap[Key]["x"])
            ReferenceArUcos_y.append(ReferenceMap[Key]["y"])
            j += 1

        elif j > 10 : 
            ReferenceObjects_x.append(ReferenceMap[Key]["x"])
            ReferenceObjects_y.append(ReferenceMap[Key]["y"])
            Objects_names.append(Key)
            if "tomato" in Key:
                Objects_colours.append("red")
            elif "potato" in Key: # TODO if we change this, we have to be consistent
                Objects_colours.append("skyblue")
            elif "orange" in Key: 
                Objects_colours.append("orange")
            elif "capsicum" in Key: 
                Objects_colours.append("pink")
            elif "lime" in Key: 
                Objects_colours.append("green")
            elif "lemon" in Key: 
                Objects_colours.append("yellow")
            elif "garlic" in Key: 
                Objects_colours.append("white")
            elif "pumpkin" in Key: 
                Objects_colours.append("purple")

            j += 1

    
    fig,ax = plt.subplots(figsize = (5, 5))
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_facecolor('xkcd:black')
    ax.scatter(ReferenceArUcos_x, ReferenceArUcos_y, color = "blue", marker = "s")
    ax.scatter(ReferenceObjects_x, ReferenceObjects_y, c = Objects_colours, marker = "o")
    ax.scatter(0,0, color = "red", marker = "s")
    fig.savefig("map_image.png")

visualise_map('TrueMap.txt')