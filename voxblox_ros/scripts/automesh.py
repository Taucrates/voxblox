#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import time

import yaml
import subprocess
import rosbag
# from rosbag.bag import Bag
import pymeshlab as pml
import signal


def rosBagPlay(rate=1.0, start=0.0, duration=1000.0, cont_folder="", paused=True):

    if(paused):
        comando_bash = f"rosbag play --clock --pause --rate {rate} --start {start} --duration {duration} {cont_folder}/*/m*.bag {cont_folder}/*/p*.bag"
    else:
        comando_bash = f"rosbag play --clock --rate {rate} --start {start} --duration {duration} {cont_folder}/*/m*.bag {cont_folder}/*/p*.bag"

    # Ejecutar el comando de bash
    try:
        
        subprocess.check_call(comando_bash, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing the bash command: {e}")

def saveMesh(namespace=''):

    # Comando de bash con las variables reemplazadas
    comando_bash = f"rosservice call {namespace}/voxblox_node/generate_mesh"

    # Ejecutar el comando de bash
    try:
        print("Saving mesh")
        subprocess.check_call(comando_bash, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing the bash command: {e}")

def getFilesWithExtension(path, extension):
    contained_files = []
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith(extension) and file.startswith("mussol"):
                contained_files.append(os.path.join(root, file))
    return contained_files


def handler(signum, frame):
    exit(1)


def automesh():
    rospy.init_node('automesh', anonymous=True)

    bagfiles_path = rospy.get_param('~path','/home/tonitauler/Desktop/Bagflies')
    bagfiles_path = bagfiles_path
    resolution = rospy.get_param('~resolution', 0.1)
    voxblox_mode = rospy.get_param('~vb_mode','simple')
    rate = rospy.get_param('~rate', 1.0)
    namespace = rospy.get_param('~namespace', '')
    auto_save = rospy.get_param('~auto_save', True)
    paused = rospy.get_param('~paused', True)
    ht = rospy.get_param('~height_threshold', 1.0)

    # print(bagfiles_path)

    files = getFilesWithExtension(bagfiles_path + "/mussol", ".bag")

    # for archivo in files:
    #     print(archivo)

    first_pos = True
    start_time = 0.0
    end_time = 1000.0

    with rosbag.Bag(files[0], 'r') as bag:
        # Topic name
        if namespace == '/':
            topic_name = "/liodom/odom"
        else:
            topic_name = "/" + namespace + "/liodom/odom"

        # Start time
        for topic, msg, timestamp in bag.read_messages(topics=[topic_name]):
            if first_pos:
                first_pos = False
                init_stamp = timestamp
            
            if msg.pose.pose.position.z >= ht: #1.0m
                start_time = (timestamp - init_stamp) * 1e-9
                break
        

        # End time
        heights = []
        times = []
        for _, msg, timestamp in bag.read_messages(topics=[topic_name]):
            heights.append(msg.pose.pose.position.z)
            times.append(timestamp)

        for height, timestamp in zip(reversed(heights), reversed(times)):
            if height >= ht: #1.5m
                end_time = ((timestamp - init_stamp) * 1e-9)
                break

        
    duration = end_time - start_time
    print("\n\nStart at " + str(start_time) + "s and end at " + str(end_time) + "s")
    rosBagPlay(rate, start_time, duration, bagfiles_path, paused)

    if(auto_save):
        ms = pml.MeshSet()
        time.sleep(10)
        ## Save mesh from voxblox
        if namespace == '/':
            saveMesh(namespace= '')
        else:
            saveMesh(namespace= '/' + namespace)

        mesh_file_name = "mesh_" + str(resolution) + "_" + voxblox_mode

        ## Refining mesh
        ms.load_new_mesh(bagfiles_path + '/' + mesh_file_name + '.ply')
        # Merging vertices (needed in next steps)
        ms.meshing_merge_close_vertices()
        # Adding ambient occlusion 
        ms.compute_scalar_ambient_occlusion(usegpu=True)
        # Delete Isolated Faces
        ms.meshing_remove_connected_component_by_face_number(mincomponentsize=200)
        # Closing small holes
        ms.meshing_repair_non_manifold_edges()
        ms.meshing_close_holes(maxholesize=100)
        
        # Smoothing mesh
        # ms.apply_coord_laplacian_smoothing_surface_preserving()
        # ms.apply_coord_laplacian_smoothing()
        ms.apply_coord_hc_laplacian_smoothing()
        
        ## Export to different formats (.off, .dae)
        ms.save_current_mesh(bagfiles_path + '/' + mesh_file_name + '.ply')
        ms.save_current_mesh(bagfiles_path + '/' + mesh_file_name + '.dae')
        ms.save_current_mesh(bagfiles_path + '/' + mesh_file_name + '.off')

        print("Files saved")

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    automesh()
    