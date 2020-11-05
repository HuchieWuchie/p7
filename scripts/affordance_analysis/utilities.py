import numpy as np
import open3d as o3d

def depthToPCDMap(w, h, fx, fy, cx, cy):

    map = np.zeros((h,w,2))
    map.fill(1.0)
    for i in range(w):
        for j in range(h):
            map[j][i][0] = 1 * (i - cx) / fx
            map[j][i][1] = 1 * (j - cy) / fy

    return(map)

def visualizePCD(npArr):
    npArr = np.reshape(npArr, (-1,3))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(npArr)
    o3d.visualization.draw_geometries([pcd])

def depthToPCD(npArr, pcdMap):

    pcd = np.zeros((npArr.shape[0],npArr.shape[1],3))
    z = npArr
    x = np.multiply(pcdMap[:,:,0], npArr)
    y = np.multiply(pcdMap[:,:,1], npArr)
    pcd[:,:,0] = x
    pcd[:,:,1] = y
    pcd[:,:,2] = z

    return pcd

def pcdBoundingBox(pcd):

    xmin = np.min(pcd[:,0])
    xmax = np.max(pcd[:,0])
    ymin = np.min(pcd[:,1])
    ymax = np.max(pcd[:,1])
    zmin = np.min(pcd[:,2])
    zmax = np.max(pcd[:,2])

    return xmin, xmax, ymin, ymax, zmin, zmax
