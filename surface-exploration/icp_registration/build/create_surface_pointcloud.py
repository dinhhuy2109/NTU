import numpy as np

data = []
for y in range(200):
    for x in range(500):
        temp = np.array([x/10000., y/10000., 0])
        data.append(temp)

for y in range(200):
    for x in range(500):
        temp = np.array([x/10000., y/10000., 0])
        data.append(temp)

for x in range(500):
    for z in range(80):
        temp = np.array([x/10000., 0/10000., -z/10000.])
        data.append(temp)
        # temp = np.array([x/10000., 190/10000., -z/10000.])
        # data.append(temp)

for y in range(200):
    for z in range(100):
        temp = np.array([0/10000., y/10000., -z/10000.])
        data.append(temp)
        # temp = np.array([490/10000., y/10000., -z/10000.])
        # data.append(temp)


textout ="# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
textout += str(len(data))
textout +="\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS "
textout += str(len(data))
textout += "\nDATA ascii\n"

for i in range(len(data)):
    temp = data[i]
    separator = ""
    for r in range(3):
        textout += separator
        textout += str(temp[r])
        separator = " "
    textout +="\n"

with open("raw_data.pcd", "wt") as file:
    file.write(textout)
