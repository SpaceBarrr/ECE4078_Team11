import ast
import json
import argparse

def parse_slam_map(fname : str) -> dict:
    with open(fname, 'r') as f:
        usr_dict = ast.literal_eval(f.read())
        aruco_dict = {}
        for (i, tag) in enumerate(usr_dict["taglist"]):
            if tag > 0 and tag < 11:
                aruco_dict[f"aruco_{tag}"] = {
                    "x": usr_dict["map"][0][i],
                    "y": usr_dict["map"][1][i]
                }
    return aruco_dict

def parse_fruit_map(fname: str) -> dict:
    with open(fname, 'r') as f:
        return json.load(f) 

if __name__ == '__main__':
    parser = argparse.ArgumentParser("Process M2 and M3 for M4")
    parser.add_argument("--slam", type=str, default="slam_1.txt")
    parser.add_argument("--fruits", type=str, default="targets_1.txt")
    parser.add_argument("--map_name", type=str, default="TrueMap.txt")
    args = parser.parse_args()

    slam_dict = parse_slam_map(args.slam)
    fruits_dict = parse_fruit_map(args.fruits)
    
    slam_dict.update(fruits_dict)
    with open(args.map_name, "w") as outfile: 
        json.dump(slam_dict, outfile)
    print(slam_dict)