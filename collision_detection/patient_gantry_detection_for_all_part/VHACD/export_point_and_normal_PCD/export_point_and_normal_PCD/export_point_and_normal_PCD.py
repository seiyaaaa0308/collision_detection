import numpy as np
import open3d as o3d

def Define_Robotpart():
    part_names=[]

    part_names.append('needle');
    part_names.append('needle_gripper');
    part_names.append('air_parts');
    part_names.append('force_sensor_cover');
    part_names.append('upper_part(V30L)');
    part_names.append('bottom_part(V30L)');
    part_names.append('front_part(V30L)');
    part_names.append('back_part(V30L)');
    part_names.append('upper_cover(V30L)');
    part_names.append('bottom_cover(V30L)')
    part_names.append('b_axis_cover(V30L)');
    return part_names

def read_and_export_PCD(pn, devide_number):
    mesh_vertices_center = []
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/4_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/8_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/16_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/32_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/64_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    file_path = "/Users/Seiya/Collision_Detect/VHACD/128_convexhull/PCD_triangle_mesh/" + str(pn) + "/"
    for i in range(devide_number):
        filename = file_path + str(pn) + "_" + str(i) + ".pcd"
        mesh_pcd=o3d.io.read_point_cloud(filename)
        mesh_vertices = np.asarray(mesh_pcd.points)
        mesh_vertices_center = np.average(np.unique(mesh_vertices,axis=0), axis=0)
        triangle_mesh_vertices = np.reshape(mesh_vertices, (-1, 3, 3))

        normal_list = Calc_normal(triangle_mesh_vertices)
        plane_ele = Normal_direction(normal_list, triangle_mesh_vertices,  mesh_vertices_center)
        point_of_plane = plane_ele[0]
        normal_of_plane = plane_ele[1]
        #point_filepath = "/Users/Seiya/Collision_Detect/VHACD/4_convexhull/PCD_point/"
        #point_filepath = "/Users/Seiya/Collision_Detect/VHACD/8_convexhull/PCD_point/"
        #point_filepath = "/Users/Seiya/Collision_Detect/VHACD/16_convexhull/PCD_point/"
        #point_filepath = "/Users/Seiya/Collision_Detect/VHACD/32_convexhull/PCD_point/"
        #point_filepath = "/Users/Seiya/Collision_Detect/VHACD/64_convexhull/PCD_point/"
        point_filepath = "/Users/Seiya/Collision_Detect/VHACD/128_convexhull/PCD_point/"

        #normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/4_convexhull/PCD_normal/"
        #normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/8_convexhull/PCD_normal/"
        #normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/16_convexhull/PCD_normal/"
        #normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/32_convexhull/PCD_normal/"
        #normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/64_convexhull/PCD_normal/"
        normal_filepath = "/Users/Seiya/Collision_Detect/VHACD/128_convexhull/PCD_normal/"

        point_filename = point_filepath + "/" + pn + "/" + pn + "_point_" + str(i) + ".pcd"
        normal_filename = normal_filepath + "/" + pn + "/" + pn + "_normal_" + str(i) + ".pcd"

        export_PCD(point_of_plane, point_filename)
        export_PCD(normal_of_plane, normal_filename)

def Calc_normal(triangle_mesh_vertices):
    normal_ele_list = []
    for triangle in triangle_mesh_vertices:
        vec1=[triangle[1,0]-triangle[0,0], triangle[1,1]-triangle[0,1], triangle[1,2]-triangle[0,2]]
        vec2=[triangle[2,0]-triangle[0,0], triangle[2,1]-triangle[0,1], triangle[2,2]-triangle[0,2]]
        a=vec1[1]*vec2[2]-vec2[1]*vec1[2]
        b=vec1[2]*vec2[0]-vec2[2]*vec1[0]
        c=vec1[0]*vec2[1]-vec2[0]*vec1[1]
        normal_ele_list.append(a)
        normal_ele_list.append(b)
        normal_ele_list.append(c)
    normal_list = np.asarray(normal_ele_list)
    normal_list = np.reshape(normal_list,(-1,3))
    return normal_list

def Normal_direction(normal_list, triangle_mesh_vertices, mesh_vertices_center):
    normal_list_trans = []
    point_list = []
    normals_direction = 0
    for i in range(len(normal_list)):
       triangle = triangle_mesh_vertices[i]
       point_list.append(triangle[0])
       normal = normal_list[i]
       mesh_vertices_center_vec = mesh_vertices_center-triangle[0]
       inner = np.inner(normal, mesh_vertices_center_vec)
       if inner>=0:
           normals_direction += 1
           normal_list_trans.append(normal)
       else:
           normal_vertices_trans = normal * -1
           inner = np.inner(normal_vertices_trans, mesh_vertices_center_vec)
           if inner>=0:
               normals_direction += 1
               normal_list_trans.append(normal_vertices_trans)
    normal_list_trans = np.asarray(normal_list_trans)
    point_list = np.asarray(point_list)
    return point_list, normal_list_trans

def export_PCD(plane_ele_arr, file_name):
    plane_ele_export = o3d.geometry.PointCloud()
    plane_ele_export.points = o3d.utility.Vector3dVector(plane_ele_arr)
    o3d.io.write_point_cloud(file_name, plane_ele_export, write_ascii=True)

if __name__=="__main__":
    pns = Define_Robotpart()
    for pn in pns:
        read_and_export_PCD(pn, 128)
    


