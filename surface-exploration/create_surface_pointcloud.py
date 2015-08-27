import numpy as np

data = []
for x in range(50):
    for y in range(20):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)
for x in range(50):
    for z in range(10):
        temp = np.array([x/1000., 0/1000., z/1000.])
        data.append(temp)
        temp = np.array([x/1000., 19/1000., z/1000.])
        data.append(temp)

for y in range(20):
    for z in range(10):
        temp = np.array([0/1000., y/1000., z/1000.])
        data.append(temp)
        # temp = np.array([49/1000., y/1000., z/1000.])
        # data.append(temp)


textout ="# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH "
textout += str(50*20)
textout +="\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINT "
textout += str(50*20)
textout += "\nDATA ascii\n"

for i in range(len(data)):
    temp = data[i]
    separator = ""
    for r in range(3):
        textout += separator
        textout += str(temp[r])
        separator = " "
    textout +="\n"

with open("raw_rectangle.pcd", "wt") as file:
    file.write(textout)
