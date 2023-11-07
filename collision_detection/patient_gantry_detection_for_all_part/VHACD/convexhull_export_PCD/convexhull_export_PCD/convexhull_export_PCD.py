import numpy as np
import open3d as o3d

def Define_Robotpart():
    part_names=[]
    dict_devide_number = {}

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

def load_obj(file_path):
    data = {'vertices': [], 'faces': {}}
    current_material = None
    
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split()
            if not parts:
                continue
            if parts[0] == 'v':
                # 頂点座標を読み込む
                vertex = [float(parts[1]), float(parts[2]), float(parts[3])]
                data['vertices'].append(vertex)
            elif parts[0] == 'f':
                # 面（三角形）を読み込む
                face = [int(part.split('/')[0]) - 1 for part in parts[1:]]
                if current_material is not None:
                    if current_material not in data['faces']:
                        data['faces'][current_material] = []
                    data['faces'][current_material].append(face)
            elif parts[0] == 'usemtl':
                # マテリアルを読み込む
                current_material = parts[1]
                if current_material not in data['faces']:
                    data['faces'][current_material] = []
    
    return data

def export_PCD(mesh_arr, file_name):
    mesh_array_export = o3d.geometry.PointCloud()
    mesh_array_export.points = o3d.utility.Vector3dVector(mesh_arr)
    o3d.io.write_point_cloud(file_name, mesh_array_export, write_ascii=True)
    
if __name__=="__main__":
    pns = Define_Robotpart()
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/4_Convex-hull/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/8_Convex-hull/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/16_Convex-hull/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/32_Convex-hull/"
    #file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/64_Convex-hull/"
    file_path = "/Users/Seiya/Collision_Detect/VHACD/VHACD_result/128_Convex-hull/"
    dict_vertices_arr_all = {}
    for pn in pns:
        file_name_count = 0
        dict_vertices_arr_part = {}
        file_name = file_path  + pn + ".obj"
        obj_data = load_obj(file_name)
        for material, material_faces in obj_data['faces'].items():
            vertices_arr = []
            for face_indices in material_faces:
                vertices_of_face = [obj_data['vertices'][i] for i in face_indices]
                for vertex in vertices_of_face:
                    vertices_arr.append(vertex)
                #print(f"Part: {pn}")                   
                #print(f"Material: {material}")
                #print("-----------------------------------------------")
                #print(np.reshape(vertices_arr, (-1, 3, 3)))
                #print("-----------------------------------------------")
            #export_filepath = "/Users/Seiya/Collision_Detect/VHACD/4_convexhull/PCD_triangle_mesh/"
            #export_filepath = "/Users/Seiya/Collision_Detect/VHACD/8_convexhull/PCD_triangle_mesh/"
            #export_filepath = "/Users/Seiya/Collision_Detect/VHACD/16_convexhull/PCD_triangle_mesh/"
            #export_filepath = "/Users/Seiya/Collision_Detect/VHACD/32_convexhull/PCD_triangle_mesh/"
            #export_filepath = "/Users/Seiya/Collision_Detect/VHACD/64_convexhull/PCD_triangle_mesh/"
            export_filepath = "/Users/Seiya/Collision_Detect/VHACD/128_convexhull/PCD_triangle_mesh/"
            export_filename = export_filepath + "/" + pn + "/" + pn + "_" + str(file_name_count) + ".pcd"
            export_PCD(vertices_arr, export_filename)
            file_name_count += 1
            dict_vertices_arr_part[material] =np.reshape(vertices_arr, (-1, 3, 3))
        dict_vertices_arr_all[pn] = dict_vertices_arr_part            
                


