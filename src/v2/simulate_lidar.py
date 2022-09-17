#! /usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os,sys
import rospy
from core import osm_v2
import osmnx as ox

log_file_dir = '/home/mkz/catkin_ws/src/osm-localization/test_cases'
log_file_name = 'u_calib_local_maplite.csv'

logs = pd.read_csv(os.path.join(log_file_dir,log_file_name))
topX = 2000

osm_loc = osm_v2(load_saved_df=True)
steps = logs.shape[0]
logs['converged'] = pd.Series([0.0]*steps)

# Iterate over all the data
for step in range(steps):
    print('\nProcessing Step: {}'.format(step+1))
    # Generate Simulated lidar images the given coordinates
    e,n = logs.gps_e[step], logs.gps_n[step]
    # Find best lidar image
    idx,idy = osm_loc.coord2idx(e,n)
    img1 = osm_loc.SliceAtAngle(idx,idy,-2.2,osm_loc.map_bev)
    # img2 = osm_loc.SliceAtAngle(idx,idy,1.4,osm_loc.map_bev)
    # plt.imshow(img1)
    # plt.show()
    # imgs = [img1,img2]
    # img_ = [np.sum(i) for i in imgs]
    # idx_max = img_.index(max(img_))
    # img = imgs[idx_max]
    img = img1
    q_desc = osm_loc.findRoadDescriptor(img,osm_loc.ranges)
    poses2 = osm_loc.findGolbaltopX_descriptor(img,q_desc,2000,(e,n,2000))
    # poses2 = osm_loc.find_descriptor_Pose(poses,q_desc,10)
    poses2 = poses2.iloc[:topX,:]
    poses2['diff'] = np.sqrt((poses2.e-e)**2+(poses2.n-n)**2)
    if poses2['diff'].min()<5.0:
        logs.converged[step] = 1.0
    print('\n Step {} processed'.format(step+1))
    print('Min distance to  GT:',poses2['diff'].min())



g = ox.graph_from_point((30.720717, -96.468491),dist=1300,dist_type="bbox")
g = ox.project_graph(g)
fig,ax=ox.plot.plot_graph(g,bgcolor='white',figsize=(24,16),edge_color='black',edge_linewidth=2,show=False, close=False)

for step in range(steps):
    if logs.converged[step] == 1.0:
        ax.scatter(logs.gps_e.iloc[step],logs.gps_n.iloc[step],c='red',marker='.',linewidths=0.5)

# fig.savefig('./desc_search.png',dpi=150) 
plt.show()


