import numpy as np
import cv2
import math
import utilities as util
import open3d as o3d # only needed for visualization, can be left out
import time

def findSegments(pcd, xRes, yRes, zRes, xResMin, yResMin, zResMin, xMin, yMin, zMin):
    #xMin, _, yMin, _, zMin, _ = util.pcdBoundingBox(pcd)
    #xSegMin, xSegMax, ySegMin, ySegMax, zSegMin, zSegMax = 0
    #print()
    #print(xMin, yMin, zMin)
    #print(pcd.shape)

    for x in range(2):

        xSegMin = (x*xRes) + xMin
        xSegMax = ((x+1)*xRes) + xMin
        for y in range(2):

            ySegMin = (y*yRes) + yMin
            ySegMax = ((y+1)*yRes) + yMin
            for z in range(2):

                zSegMin = (z*zRes) + zMin
                zSegMax = ((z+1)*zRes) + zMin
                #print(x, y, z, " \t:", xSegMin, ":", xSegMax, " , ", ySegMin, ":", ySegMax, " , ", zSegMin, ":", zSegMax)
                #print(xSegMin, ":", xSegMax, " , ", ySegMin, ":", ySegMax, " , ", zSegMin, ":", zSegMax,)
                pcdSegArr = pcd[(pcd[:,0] >= xSegMin) & (pcd[:,0] <= xSegMax) & (pcd[:,1] >= ySegMin) & (pcd[:,1] <= ySegMax) & (pcd[:,2] >= zSegMin) & (pcd[:,2] <= zSegMax)]

                #print(xRes, yRes, zRes, xResMin, yResMin, zResMin)
                #print(pcdSegArr.shape)


                if pcdSegArr.shape[0] > 2:
                    #print(pcdSegArr)
                    #print(x, y, z)
                    #print(xRes, yRes, zRes)
                    #print(xResMin, yResMin, zResMin)
                    #print(pcdSegArr.shape)
                    #util.visualizePCD(pcdSegArr)
                    if xRes > xResMin or yRes > yResMin or zRes > zResMin:
                        #print("True")
                        xResNew = 0.5*xRes
                        yResNew = 0.5*yRes
                        zResNew = 0.5*zRes

                        if xRes < xResMin:
                            xResNew = xResMin
                        if yRes < yResMin:
                            yResNew = yResMin
                        if zRes < zResMin:
                            zResNew = zResMin
                        #print("new: ", xRes, yRes, zRes)
                        #print(xSegMin, ":", xSegMax, " , ", ySegMin, ":", ySegMax, " , ", zSegMin, ":", zSegMax,)
                        #util.visualizePCD(pcdSegArr)


                        #findSegments(pcdSegArr, xRes, yRes, zRes, xResMin, yResMin, zResMin, xMin, yMin, zMin)
                        findSegments(pcdSegArr, xResNew, yResNew, zResNew, xResMin, yResMin, zResMin, xSegMin, ySegMin, zSegMin)




fx = 5.7592685448804468e+02
fy = 5.7640791601093247e+02
cx = 3.1515026356388171e+02
cy = 2.3058580662101753e+02

# Create PCDMap for faster pointcloud backprojection
depth = cv2.imread("1.png", cv2.IMREAD_ANYDEPTH)
depth_w = depth.shape[1]
depth_h = depth.shape[0]

pcdmap = util.depthToPCDMap(depth_w, depth_h, fx, fy, cx, cy)
pcd = util.depthToPCD(depth, pcdmap)
pcd = np.reshape(pcd, (-1,3))

# Seperate into 3D grid

## All units are cm
xmin, xmax, ymin, ymax, zmin, zmax = util.pcdBoundingBox(pcd)
xDist = xmax - xmin
yDist = ymax - ymin
zDist = zmax - zmin
print(xmin, xmax, ymin, ymax, zmin, zmax)

xRes = 200
yRes = 200
zRes = 200

xLen = int(xDist / xRes) + 1
yLen = int(yDist / yRes) + 1
zLen = int(zDist / zRes) + 1

segmentMap = np.zeros((xLen, yLen, zLen)) # For storing point cloud segments

print("Now")
t1 = int(round(time.time() * 1000))
"""
for x in range(xLen):

    xSegMin = x*xRes
    xSegMax = (x+1)*xRes

    for y in range(yLen):

        ySegMin = y*yRes
        ySegMax = (y+1)*yRes

        for z in range(zLen):

            zSegMin = z*zRes
            zSegMax = (z+1)*zRes
            print(x,y,z)
            pcdSegArr = pcd[(pcd[:,0] >= xSegMin) & (pcd[:,0] <= xSegMax) & (pcd[:,1] >= ySegMin) & (pcd[:,1] <= ySegMax) & (pcd[:,2] >= zSegMin) & (pcd[:,2] <= zSegMax)]
            #if pcdSegArr.shape[0] != 0:
            #    print(pcdSegArr)
            #if pcdSegArr.

            #print(pcd[:,:,0] >= xSegMin and pcd[:,:,0] <= xSegMax)
"""
findSegments(pcd, 0.5*xDist, 0.5*yDist, 0.5*zDist, xRes, yRes, zRes, xmin, ymin, zmin)
t2 = int(round(time.time() * 1000))
print(t2-t1)
print("Then")

t1 = int(round(time.time() * 1000))
#pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(2000, 3)))
#o3d.visualization.draw_geometries([pcd])

#print('voxelization')
pcdo3d = o3d.geometry.PointCloud()
pcdo3d.points = o3d.utility.Vector3dVector(pcd)
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcdo3d,
                                                            voxel_size=100)
#o3d.visualization.draw_geometries([voxel_grid])
t2 = int(round(time.time() * 1000))
print(t2-t1)
