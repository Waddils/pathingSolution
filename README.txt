I ran this code using...
 - Python 3.11.0
 - Visual Studio Code [downloaded the Python Extension Pack and Python Debugger and Python Image Preview]

        Name: Python Extension Pack
    Id: donjayamanne.python-extension-pack
    Description: Popular Visual Studio Code extensions for Python
    Version: 1.7.0
    Publisher: Don Jayamanne
    VS Marketplace Link: https://marketplace.visualstudio.com/items?itemName=donjayamanne.python-extension-pack

        Name: Python Debugger
    Id: ms-python.debugpy
    Description: Python Debugger extension using `debugpy`.
    Version: 2024.0.0
    Publisher: Microsoft
    VS Marketplace Link: https://marketplace.visualstudio.com/items?itemName=ms-python.debugpy

        Name: Python Image Preview
    Id: 076923.python-image-preview
    Description: Numpy, Pillow, OpenCV, Matplotlib, Plotly, ImageIO, Scikit Image, Tensorflow, Pytorch Image Preview
    Version: 0.1.2
    Publisher: 윤대희
    VS Marketplace Link: https://marketplace.visualstudio.com/items?itemName=076923.python-image-preview





Now you have 3 main files:
 - grid.py
 - search.py
 - utils.py

Running the program on VSCode...
1. Download the ZIP folder and drag to desktop or some place
2. Create a folder called "Search" or named something else on your desktop
3. Drag the Search.zip file into the folder
4. Right click on the zip and choose "Extract Here"
    a. All of the files from the zip should go into that folder
5. Open Visual Studio Code and click "File" --> "Open Folder" and select the folder you dragged the zip file in
6. Once you have all of the files listed above, click on search.py
7. Press F5 (Run and Debug hotkey) and something should pop up on the top of the screen called "Select Debugger"
8. Select "Python Debugger", then click "Python File"
9. The program should run successfully

I will comment out the path visuals but I will let the program show the path cost and nodes expanded results




Another thing is that if you want to test out the custom grid I made, change the name values of these variables to the ones below

epolygons = gen_polygons('TestingGrid/world1_enclosures.txt')
tpolygons = gen_polygons('TestingGrid/world1_turfs.txt')

TestingGrid/world2_enclosures.txt
TestingGrid/world2_turfs.txt