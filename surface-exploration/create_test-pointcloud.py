import numpy as np

data = []
for x in range(10):
    for y in range(5):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)
for x in range(10):
     for z in range(10):
         temp = np.array([x/1000., 0/1000., z/1000.])
         data.append(temp)
#         temp = np.array([x/1000., 19/1000., z/1000.])
#         data.append(temp)

for y in range(5):
    for z in range(10):
        temp = np.array([0/1000., y/1000., z/1000.])
        data.append(temp)
        # temp = np.array([49/1000., y/1000., z/1000.])
        # data.append(temp)
textout = ""
for i in range(len(data)):
    temp = data[i]
    separator = ""
    for r in range(3):
        textout += separator
        textout += str(temp[r])
        separator = " "
    textout +="\n"

with open("test_pointcloud.pcd", "wt") as file:
    file.write(textout)
