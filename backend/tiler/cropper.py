import open3d as o3d
import numpy as np
import os
import subprocess
from datetime import datetime


# This file crops an obj file based on a bounding box, and outputs
# the cropped mesh as well as the texture that pertains to the remaining
# part of the mesh. This code calls blender_code.py, which relies on Blender.
#
# Make sure to set the `blender_exec_dir` variable to point to the location
# of your Blender installation. Tested with Blender 3.2.1
#
# Tuneable parameters: texture output resolution
# 
# Version 8/12/22
# Author: Nitzan Orr, University of Wisconsin--Madison
# Email: nitzan@cs.wisc.edu
# Please reach out with any questions
#
# Version 6/6/22
# Author: Khaled Sharif, NASA

blender_exec_dir = '/home/nitz/software/blender-3.2.1-linux-x64'


def run(command):
    try:
        subprocess.run(command, shell=True, check=True, capture_output=True)
    except subprocess.CalledProcessError as e:
        print(e.returncode)
        print(e.output)





# Get the 3 vertex indices of a face from a line defining a face
def get_f_line_vertex_indices(line_elements):
    '''
    Examples of lines defining faces. 
    In each line, vertex indices are 10, 40, and 70
    f 10 40 70
    f 10/10 40/20 70/30
    f 10/30/40 40/50/60 70/80/90
    f 10//30 40//60 70//90
    '''
    output = []
    for l in line_elements:
        output.append(l.split('/')) # split each result by '/'
    return output[1][0], output[2][0], output[3][0] 





# Main cropping function: given a bounding box, if the centroid of a face
# falls within the bounding box, then include the whole face, otherwise discard.
# Also crop the texture image to include only textures requested by the bounding box
def make_tile_4(input_file,output_file, minX, minY, minZ, maxX, maxY, maxZ, resolution=512):

    print('\nCropping Mesh:', input_file)

    vertex_idx = [-1]

    obj_file = open(input_file, 'r')
    new_obj_file = open(output_file, 'w')

    # Retrieve index of each vertex by adding all vertices to list
    # And write vertices, vt, vn, etc. to new_obj_file to maintain 
    # their index after cropping
    for line in obj_file:
        line_elements = line.split()
        if line_elements:
            if line_elements[0] in ['v', 'vt', 'vn', 'vp', 'l', 'usemtl', 'mtllib', 'o', 'g']:
                new_obj_file.write(line)
                if line_elements[0] == 'v':
                    vX = float(line_elements[1])
                    vY = float(line_elements[2])
                    vZ = float(line_elements[3])
                    vertex_idx.append((vX, vY, vZ))
            if line_elements[0] == 'f':
                # get vertex indices from line defining a face
                vertex_idxs = get_f_line_vertex_indices(line_elements)
                v1_idx = int(vertex_idxs[0])
                v2_idx = int(vertex_idxs[1])
                v3_idx = int(vertex_idxs[2])
                
                # get x y z coords of the 3 vertices making a face
                v1 = np.array(vertex_idx[v1_idx])
                v2 = np.array(vertex_idx[v2_idx])
                v3 = np.array(vertex_idx[v3_idx])

                # calculate centroid of face
                # centroid = (v1 + v2 + v3) / 3
                centroid = (v1 + v2 + v3) / 3.0
                bbox_min = np.array((minX, minY, minZ))
                bbox_max = np.array((maxX, maxY, maxZ))

                # check if triangle's centroid falls inside bbox
                inside_bbox_min = (centroid >= bbox_min).all()
                inside_bbox_max = (centroid < bbox_max).all()
                inside_bbox = inside_bbox_min and inside_bbox_max

                if inside_bbox:
                    new_obj_file.write(line)

            
    # close both opened files
    obj_file.close()
    new_obj_file.close()

    # Save only the textures pertaining to the newly cropped mesh
    # Assumes blender_code.py is in same directory at this file
    this_dir = os.path.dirname(os.path.realpath(__file__))
    print("PATH:", this_dir)
    blender_py_code_path = os.path.join(this_dir, 'blender_code.py')

    # Command: cd [path_to_blender_executable] && ./blender --background --python blender_code.py arg1 arg2 arg3)
    command ='cd ' + blender_exec_dir 
    command += ' && ./blender --background --python '+blender_py_code_path+' -- '+input_file+' '+output_file+' '+str(resolution)
    print('COMMAND:', command)
    run(command)

    print('Wrote cropped mesh to:', output_file)
    print()

'''
def make_tile_2(input_file,output_file, minX, minY, minZ, maxX,maxY,maxZ):

    print('Cropping To:')
    print('X: {} to {}'.format(minX, maxX))
    print('Y: {} to {}'.format(minY, maxY))
    print('Z: {} to {}'.format(minZ, maxZ))


    
    obj_path = input_file

    # Explanation for why enable_pose_processing=True in below function:
    #
    # I found a forum post explaining that enable_post_processing at read time 
    # is designed to eliminate a lot of that duplication. In practice, I found 
    # it eliminated most (not all) of the duplicated v and vn entries, 
    # but had no effect on the vt entries. It also seemed to make the OBJ writing 
    # work better on my machine. Specifically, without enable_post_processing it 
    # failed to write the PNG texture on a large model (perhaps ran out of memory?) 
    # but with it, it worked.
    print("\nCropping: {}".format(obj_path))
    mesh = o3d.io.read_triangle_mesh(obj_path, enable_post_processing=True)
    print('\nMesh:', mesh)
    print('Mesh Center', mesh.get_center())
    print('Mesh Max', mesh.get_max_bound())
    print('Mesh Min', mesh.get_min_bound())


    bbox = o3d.geometry.AxisAlignedBoundingBox([minX, minY, minZ],[maxX, maxY, maxZ])
    print('\nCreated axis aligned bounding box for cropping:')
    print(bbox)

    cropped_mesh = o3d.geometry.TriangleMesh.crop(mesh, bbox)
    print('\nCropped Mesh:')
    print(cropped_mesh)
    o3d.io.write_triangle_mesh(output_file, cropped_mesh)
    print('\nWrote cropped mesh to:', output_file)

    return cropped_mesh
'''

# Deprecated. Replaced by make_tile_4
# def make_tile_3(input_file,output_file, minX, minY, minZ, maxX, maxY, maxZ):

#     vertex_idx = [-1]

#     obj_file = open(input_file, 'r')
#     new_obj_file = open(output_file, 'w')

#     # Retrieve index of each vertex by adding all vertices to list
#     # And write vertices, vt, vn, etc. to new_obj_file to maintain 
#     # their index after cropping
#     for line in obj_file:
#         line_elements = line.split()
#         if line_elements:
#             if line_elements[0] in ['v', 'vt', 'vn', 'vp', 'l', 'usemtl', 'mtllib', 'o', 'g']:
#                 new_obj_file.write(line)
#                 if line_elements[0] == 'v':
#                     vX = float(line_elements[1])
#                     vY = float(line_elements[2])
#                     vZ = float(line_elements[3])
#                     vertex_idx.append((vX, vY, vZ))


#     # copy over the faces that fall within the bounding box
#     obj_file = open(input_file, 'r')
#     for line in obj_file:
#         line_elements = line.split()
#         if line_elements:
#             if line_elements[0] == 'f':
#                 '''
#                 Examples of lines defining faces
#                 f 1 2 3
#                 f 3/1 4/2 5/3
#                 f 6/4/1 3/5/3 7/6/5
#                 f 7//1 8//2 9//3
#                 '''

#                 # BUGGY: Replace with fet_f_line_vertex_idxs func
#                 # get vertex indices (first number in each of group) of the face
#                 v1_idx = int(line_elements[1][0])
#                 v2_idx = int(line_elements[2][0])
#                 v3_idx = int(line_elements[3][0])
                
#                 # get x y z coords of the 3 vertices making a face
#                 v1 = np.array(vertex_idx[v1_idx])
#                 v2 = np.array(vertex_idx[v2_idx])
#                 v3 = np.array(vertex_idx[v3_idx])

#                 # calculate centroid of face
#                 # centroid = (v1 + v2 + v3) / 3
#                 centroid = (v1 + v2 + v3) / 3.0
#                 bbox_min = np.array((minX, minY, minZ))
#                 bbox_max = np.array((maxX, maxY, maxZ))


#                 inside_bbox_min = (centroid >= bbox_min).all()
#                 inside_bbox_max = (centroid < bbox_max).all()
#                 inside_bbox = inside_bbox_min and inside_bbox_max

#                 if inside_bbox:
#                     new_obj_file.write(line)

#     # close both opened files
#     obj_file.close()
#     new_obj_file.close()

#     print('\nCropped Mesh:', input_file)
#     print('Wrote cropped mesh to:', output_file)
#     print()








'''
# BUG: This function has possibly incorrect order of arguments
def make_tile(input_file,output_file,maxX,maxY,maxZ,minX,minY,minZ):

    # bounding_box = [  11.8,-7.1,4.01,  9.8, -9.8,3.9    ]

    # maxX = bounding_box[0]
    # maxY = bounding_box[1]
    # maxZ = bounding_box[2]
    # minX = bounding_box[3]
    # minY = bounding_box[4]
    # minZ = bounding_box[5]

    v_keepers = dict()  # keeps track of which vertices are within the bounding box

    kept_vertices = 0
    discarded_vertices = 0

    kept_faces = 0
    discarded_faces = 0

    discarded_lines = 0
    kept_lines = 0

    obj_file = open(input_file, 'r')
    new_obj_file = open(output_file, 'w')

    # the number of the next "v" vertex lines to process.
    original_v_number = 1  # the number of the next "v" vertex lines to process.
    # the new ordinal position of this vertex if out of bounds vertices were discarded.
    new_v_number = 1

    for line in obj_file:
        line_elements = line.split()

        # Python doesn't have a SWITCH statement, but we only have three cases, so we'll just use cascading if stmts
        # if it isn't an "f" type line (face definition)
        if line_elements[0] != "f":

            # and it isn't an "v" type line either (vertex definition)
            if line_elements[0] != "v":
                # ************************ PROCESS ALL NON V AND NON F LINE TYPES ******************
                # then we just copy it unchanged from the input OBJ to the output OBJ
                new_obj_file.write(line)
                kept_lines = kept_lines + 1

            else:  # then line_elements[0] == "v":
                # ************************ PROCESS VERTICES ****************************************
                #  a "v" line looks like this:
                #  f x y z ...
                x = float(line_elements[1])
                y = float(line_elements[2])
                z = float(line_elements[3])

                if minX < x < maxX and minY < y < maxY and minZ < z < maxZ:
                    # if vertex is within  the bounding box, we include it in the new OBJ file
                    new_obj_file.write(line)
                    v_keepers[str(original_v_number)] = str(new_v_number)
                    new_v_number = new_v_number + 1
                    kept_vertices = kept_vertices + 1
                    kept_lines = kept_lines + 1
                else:     # if vertex is NOT in the bounding box
                    new_obj_file.write(line)
                    discarded_vertices = discarded_vertices + 1
                    discarded_lines = discarded_lines + 1
                original_v_number = original_v_number + 1

        else:  # line_elements[0] == "f":
            # ************************ PROCESS FACES ****************************************
            #  a "f" line looks like this:
            #  f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ...

            #  We need to delete any face lines where ANY of the 3 vertices v1, v2 or v3 are NOT in v_keepers.

            v = ["", "", ""]
            # Note that v1, v2 and v3 are the first "/" separated elements within each line element.
            for i in range(0, 3):
                v[i] = line_elements[i+1].split('/')[0]

            # now we can check if EACH of these 3 vertices are  in v_keepers.
            # for each f line, we need to determine if all 3 vertices are in the v_keepers list
            if v[0] in v_keepers and v[1] in v_keepers and v[2] in v_keepers:
                new_obj_file.write(line)
                kept_lines = kept_lines + 1
                kept_faces = kept_faces + 1
            else:  # at least one of the vertices in this face has been deleted, so we need to delete the face too.
                discarded_lines = discarded_lines + 1
                discarded_faces = discarded_faces + 1
                new_obj_file.write("# CROPPED "+line)

    # end of line processing loop
    obj_file.close()
    new_obj_file.close()

'''

# For testing cropper.py independently
# `python3 cropper.py`
input_file = '/home/nitz/Downloads/6_23_22/run.obj'
time_str = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
output_file = '/home/nitz/Downloads/6_23_22/output_2/output_new_'+time_str+'.obj'
# make_tile_4(input_file, output_file, 11, -9, 3, 13, -8, 6) # no crop
# make_tile_4(input_file, output_file, 11, -9, 3, 13, -8.5, 6) # crops top
make_tile_4(input_file, output_file, 11, -8.5, 3, 13, -8, 6) # crops bottom

