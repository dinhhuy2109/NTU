import numpy as np

data = []
for x in range(1,3):
    for y in range(2,5):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)
for x in range(1,3):
    for y in range(16,19):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)
for x in range(49,51):
    for y in range(2,5):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)
for x in range(46,50):
    for y in range(15,18):
        temp = np.array([x/1000., y/1000., 0])
        data.append(temp)


for x in range(20,30,2):
    for z in range(5,9,2):
        temp = np.array([x/1000., 0/1000., -z/1000.])
        data.append(temp)

for y in range(13,18,2):
    for z in range(4,7,2):
        temp = np.array([0/1000., 0/1000., -z/1000.])

# for x in range():
#     for z in range(10):
#         temp = np.array([x/1000., 0/1000., z/1000.])
#         data.append(temp)
#         temp = np.array([x/1000., 19/1000., z/1000.])
#         data.append(temp)

# for y in range(20):
#     for z in range(10):
#         temp = np.array([0/1000., y/1000., z/1000.])
#         data.append(temp)
        # temp = np.array([49/1000., y/1000., z/1000.])
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

with open("test_data.pcd", "wt") as file:
    file.write(textout)
