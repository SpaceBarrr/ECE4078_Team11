import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import json
import numpy as np

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
    
    ReferenceObjects_x = []
    ReferenceObjects_y = []
    ReferenceArUcos_x = []
    ReferenceArUcos_y = []
    Objects_names = []
    j = 1

    for Key in ReferenceMap:
        if j <= 10 :
            Position = np.array([ReferenceMap[Key]["x"], ReferenceMap[Key]["y"]])
            ReferenceArUcos_x.append(ReferenceMap[Key]["x"])
            ReferenceArUcos_y.append(ReferenceMap[Key]["y"])

            if "10" in Key:
                ReferenceMap[Key]['number'] = 10
            elif "9" in Key:
                ReferenceMap[Key]['number'] = 9
            elif "8" in Key:
                ReferenceMap[Key]['number'] = 8
            elif "7" in Key:
                ReferenceMap[Key]['number'] = 7
            elif "6" in Key:
                ReferenceMap[Key]['number'] = 6
            elif "5" in Key:
                ReferenceMap[Key]['number'] = 5
            elif "4" in Key:
                ReferenceMap[Key]['number'] = 4
            elif "3" in Key:
                ReferenceMap[Key]['number'] = 3
            elif "2" in Key:
                ReferenceMap[Key]['number'] = 2
            elif "1" in Key:
                ReferenceMap[Key]['number'] = 1
            j += 1

        elif j > 10 : 
            ReferenceObjects_x.append(ReferenceMap[Key]["x"])
            ReferenceObjects_y.append(ReferenceMap[Key]["y"])
            Objects_names.append(Key)
            if "tomato" in Key:
                image_path = "guipngs/tomato.png"
                ReferenceMap[Key]['image'] = image_path
            elif "potato" in Key: # TODO if we change this, we have to be consistent
                image_path = "guipngs/potato.png"
                ReferenceMap[Key]['image'] = image_path
            elif "orange" in Key: 
                image_path = "guipngs/orange.png"
                ReferenceMap[Key]['image'] = image_path
            elif "capsicum" in Key: 
                image_path = "guipngs/capsicum.png"
                ReferenceMap[Key]['image'] = image_path
            elif "lime" in Key: 
                image_path = "guipngs/lime.png"
                ReferenceMap[Key]['image'] = image_path
            elif "lemon" in Key: 
                image_path = "guipngs/lemon.png"
                ReferenceMap[Key]['image'] = image_path
            elif "garlic" in Key: 
                image_path = "guipngs/garlic.png"
                ReferenceMap[Key]['image'] = image_path
            elif "pumpkin" in Key: 
                image_path = "guipngs/pumpkin.png"
                ReferenceMap[Key]['image'] = image_path
                

            j += 1

    
    fig,ax = plt.subplots(figsize = (5, 5))
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_facecolor('grey')
    for Key, values in ReferenceMap.items():
        x = values['x']
        y = values['y']
    
        # Check if the 'image' Key exists in the dictionary
        if 'image' in values:
            image_path = values['image']
            obj_image = mpimg.imread(image_path)
            
            # Specify the size of the displayed image (adjust these values)
            image_width = 0.15 
            image_height = 0.15  # 0.2 for both for s=1300, 0.15 for s=130
            
            # Calculate the extent for the image
            extent = [x - image_width / 2, x + image_width / 2, y - image_height / 2, y + image_height / 2]
            
            # Display the image as an annotation at the specified coordinates
            plt.imshow(obj_image, extent=extent, alpha=1.0, zorder=2)
        if 'number' in values:
            number = values['number']
            plt.text(x, y, str(number), fontsize=10, ha='center', va='center', color='white', fontweight='bold') #12 for s=1300, 10 for s=130
        
    ax.scatter(ReferenceArUcos_x, ReferenceArUcos_y, color = "blue", marker = "s", s=130)
    ax.scatter(ReferenceObjects_x, ReferenceObjects_y, c = "green", marker = "s", s =130) #s=1300 is for obstacle size, 130 closer to real size (slightly bigger by a couple cm)
    ax.scatter(0,0, color = "black", marker = "x")
    plt.grid(True)
    #plt.show()
    fig.savefig("map_imagev2.png")
