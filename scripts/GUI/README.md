# How to use & setup GUI.py

# 1) Setup

## Step 1

Open your terminal in your folder that have GUI.py,
Then run this code 

```
pip install tk
pip3 install customtkinter
```

## Step 2

Be sure that in your crazyswarm.launch all three nodes are active

## Step 3

In your crazyflies.yaml file set your first three flies are ready and last crazyflie has 195 id number.
This unefficient tecnique will be changed later.

## Step 4

Since GUI.py use mission_planner.py, you should clear missions or determine the line from which missions begin and change GUI.py line 28.
Please read all commits in GUI.py

## Step 5

in 3d_gui.sh check whether cd path is matches your path. If not changed it. 

# 2) How To Use

![Screenshot from 2023-02-27 23-24-45](https://user-images.githubusercontent.com/96688864/221841423-7a598a67-22a5-4568-b6e4-c24a79c67e71.png)

## Chose which mission you want to do first (3B/Yangın/Engel)

### Mission 3B:

Select the type of the shape before give inputs (Prism/Pyramid/Cylinder)

enter inputs
Note: while entering number of edges, be aware that you are not allowed to enter total fly nums, just type num of edges of 3D shape. 

Press to "Add to list" button to make ready our flies (in this case, we edit crazyflies.yaml file such as adding flies with unique ids or remove flies.)

Finally, press "Run" button and enjoy.

### Mission Yangın

You can just press the "Run" button and start mission, "OpenWebCam" button might help you to see what is happening.

### Mission Engel

Soon





