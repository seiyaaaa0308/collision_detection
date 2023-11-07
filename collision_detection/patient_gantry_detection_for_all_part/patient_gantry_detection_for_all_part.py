import open3d as o3d
import numpy as np
import copy
import time
import datetime
import os


class ClassZerobot3D():
    part_names=[];
    dict_T_CT_H={};
    vertices_disp={}
    hull={}
    hull_ls={}
    devide_number = 8

    def __init__(self):
        self.dict_mesh={}
        self.dict_point_pcd={}
        self.dict_normal_pcd={}
        self.dict_vertices_pcd={}
        self.dict_convex_hull_vertices_disp={}
        self.disp_collision_point_arr = []
        self.part_names.append('needle');
        self.part_names.append('needle_gripper');
        self.part_names.append('air_parts');
        self.part_names.append('force_sensor_cover');
        self.part_names.append('upper_part(V30L)');
        self.part_names.append('bottom_part(V30L)');
        self.part_names.append('front_part(V30L)');
        self.part_names.append('back_part(V30L)');
        self.part_names.append('upper_cover(V30L)');
        self.part_names.append('bottom_cover(V30L)')
        self.part_names.append('b_axis_cover(V30L)');
        pns =  self.part_names; 
        
        self.devide_number = []
        for i in range(64):
            self.devide_number.append(str(i))

    def read_PCD(self):
        for pn in pns:
            filename_stl = "./STL/STL_Custom(100)/" + pn + ".stl";      
            self.dict_mesh[pn] = o3d.io.read_triangle_mesh(filename_stl);
            
            #file_path = "/Users/Seiya/Collision_Detect/VHACD/4_convexhull/"
            #file_path = "/Users/Seiya/Collision_Detect/VHACD/8_convexhull/"
            #file_path = "/Users/Seiya/Collision_Detect/VHACD/16_convexhull/"
            #file_path = "/Users/Seiya/Collision_Detect/VHACD/32_convexhull/"
            file_path = "/Users/Seiya/Collision_Detect/VHACD/64_convexhull/"

            self.point_pcd={}
            self.normal_pcd={}
            self.vertices_pcd={}
            for number in range(len(self.devide_number)):
                filename_point_pcd = file_path + "PCD_point/" + pn + "/" + pn + "_point_" + str(number) + ".pcd"
                filename_normal_pcd = file_path + "PCD_normal/" + pn + "/" + pn + "_normal_" + str(number) + ".pcd"
                filename_vertices_pcd = file_path + "PCD_triangle_mesh/" + pn + "/" + pn + "_" + str(number) + ".pcd"
                self.normal_pcd[number]=o3d.io.read_point_cloud(filename_normal_pcd)             
                self.point_pcd[number]=o3d.io.read_point_cloud(filename_point_pcd) 
                self.vertices_pcd[number]=o3d.io.read_point_cloud(filename_vertices_pcd)
            self.dict_point_pcd[pn]=self.point_pcd
            self.dict_normal_pcd[pn]=self.normal_pcd
            self.dict_vertices_pcd[pn]=self.vertices_pcd
            self.dict_convex_hull_vertices_disp[pn] = copy.deepcopy(self.dict_vertices_pcd[pn])

    def calcTranspose(self):
        theta_A = self.theta_A_rad;
        theta_B = self.theta_B_rad;
        XYZ_CT_RC = self.XYZ_CT_RC;
        air_cylinder_len_m = self.air_cylinder_len_m;
        spacer_len_m = self.spacer_len_m;
        p_axis_m = self.p_axis_m;
        needle_length_m = self.needle_length_m;

        T_0_A = np.asarray([[1, 0, 0, 0],
            [0, np.cos(theta_A), -np.sin(theta_A), 0],
            [0, np.sin(theta_A), np.cos(theta_A), 0],
            [0, 0, 0, 1]])
        R_0_A = T_0_A[:3,:3];

        T_0_B = np.asarray([[np.cos(theta_B), 0, np.sin(theta_B), 0],
            [np.sin(theta_A)*np.sin(theta_B), np.cos(theta_A), -np.sin(theta_A)*np.cos(theta_B), 0],
            [-np.cos(theta_A)*np.sin(theta_B), np.sin(theta_A), np.cos(theta_A)*np.cos(theta_B), 0],
            [0, 0, 0, 1]])
        R_0_B = T_0_B[:3,:3];

        T_CT_RC = copy.deepcopy(T_0_B)
        for i in range(3):
            T_CT_RC[i,3] = XYZ_CT_RC[i];
        self.dict_T_CT_H['needle'] = T_CT_RC; 

        XYZ_RC_D1 = np.dot(R_0_B,[0, 0, needle_length_m])
        T_CT_D1 = copy.deepcopy(T_0_B)
        for i in range(3):
            T_CT_D1[i,3] = XYZ_CT_RC[i] + XYZ_RC_D1[i];
        self.dict_T_CT_H['needle_gripper'] = T_CT_D1; 

        T_CT_D1C = copy.deepcopy(T_0_B)
        x_D1_D1C = -75.0/1000.0;#[m]
        z_D1_D1C = 24.5/1000.0 + air_cylinder_len_m;#
        XYZ_D1_D1C = np.dot(R_0_B,[x_D1_D1C, 0, z_D1_D1C])
        for i in range(3):
            T_CT_D1C[i,3] = XYZ_CT_RC[i] + XYZ_RC_D1[i] + XYZ_D1_D1C[i] 
        self.dict_T_CT_H['air_parts'] = T_CT_D1C; 

        T_CT_D1D = copy.deepcopy(T_0_B)
        z_D1C_D1D = 49.0/1000.0 + spacer_len_m;#
        XYZ_D1C_D1D = np.dot(R_0_B,[0, 0, z_D1C_D1D])
        for i in range(3):
            T_CT_D1D[i,3] = T_CT_D1C[i,3] + XYZ_D1C_D1D[i]; 
        self.dict_T_CT_H['force_sensor_cover'] = T_CT_D1D; 

        x_D1D_D2 = -31.5/1000.0;#[m]
        z_D1D_D2 = -9.5/1000.0;#
        XYZ_D1D_D2 = np.dot(R_0_B,[x_D1D_D2, 0, z_D1D_D2])
        T_CT_D2 = copy.deepcopy(T_0_A)
        for i in range(3):
            T_CT_D2[i,3] = T_CT_D1D[i,3] + XYZ_D1D_D2[i]; 
        self.dict_T_CT_H['upper_part(V30L)'] = T_CT_D2; 

        z_D2_D3 = - 52.0/1000.0;#
        XYZ_D2_D3 = np.dot(R_0_B,[0, 0, z_D2_D3])
        T_CT_D3 = copy.deepcopy(T_0_A)
        for i in range(3):
            T_CT_D3[i,3] = T_CT_D2[i,3] + XYZ_D2_D3[i]; 
        self.dict_T_CT_H['bottom_part(V30L)'] = T_CT_D3; 

        x_D2_D4 = -479.951/1000.0;#[m]
        z_D2_D4 =  85.017/1000.0 ;#[m]
        XYZ_D2_D4_A = np.dot(R_0_A,[x_D2_D4, 0, z_D2_D4])
        XYZ_D2_D4_B = np.dot(R_0_B,[0, 0, p_axis_m + 18.0/1000.0])
        T_CT_D4 = copy.deepcopy(T_0_B)
        for i in range(3):
            T_CT_D4[i,3] = T_CT_D2[i,3] + XYZ_D2_D4_A[i] + XYZ_D2_D4_B[i]; 
        self.dict_T_CT_H['front_part(V30L)'] = T_CT_D4; 

        x_D4_D5 = -100.084/1000.0;#[m]
        XYZ_D4_D5 = np.dot(R_0_A,[x_D4_D5, 0, 0])
        T_CT_D5 = copy.deepcopy(T_0_B)
        for i in range(3):
            T_CT_D5[i,3] = T_CT_D4[i,3] + XYZ_D4_D5[i]; 
        self.dict_T_CT_H['back_part(V30L)'] = T_CT_D5; 

        z_D4_D6 = 65.0/1000.0;
        XYZ_D4_D6 = np.dot(R_0_B, [0, 0, z_D4_D6 ])
        T_CT_D6 = copy.deepcopy(T_0_A)
        for i in range(3):
            T_CT_D6[i,3] = T_CT_D4[i,3] + XYZ_D4_D6[i]; 
        self.dict_T_CT_H['upper_cover(V30L)'] = T_CT_D6; 

        z_D4_D7 = -239.0/1000.0;
        XYZ_D4_D7 = np.dot(R_0_B,[0, 0, z_D4_D7 ])
        T_CT_D7 = copy.deepcopy(T_0_A)
        for i in range(3):
            T_CT_D7[i,3] = T_CT_D4[i,3] + XYZ_D4_D7[i]; 
        self.dict_T_CT_H['bottom_cover(V30L)'] = T_CT_D7;

        z_D4_D8 =  -142.0/1000.0;#[m]
        XYZ_D4_D8 = np.dot(R_0_B,[0, 0, z_D4_D8 ])
        T_CT_D8 = copy.deepcopy(T_0_A)
        for i in range(3):
            T_CT_D8[i,3] = T_CT_D4[i,3] + XYZ_D4_D8[i]; 
        self.dict_T_CT_H['b_axis_cover(V30L)'] = T_CT_D8; 

    def mesh_norm_recalc(self, point_arr, normal_arr ,T):
        R = T[:3, :3].T
        point_arr.transform(T) 
        point_arr = np.asarray(point_arr.points) 
        normal_arr= np.asarray(normal_arr.points)     
        normal_arr = np.dot(normal_arr, R)
        return(point_arr, normal_arr)

    def create_plane(self, point_arr, normal_arr):
        plane_list = []
        for i in range(len(normal_arr)):
            point = point_arr[i]
            normal = normal_arr[i]
            d=-1*(normal[0]*point[0] + normal[1]*point[1] + normal[2]*point[2])
            plane_list.append(normal[0])
            plane_list.append(normal[1])
            plane_list.append(normal[2])
            plane_list.append(d)
        plane_list_np = np.asarray(plane_list)
        plane_list_np = np.reshape(plane_list_np,(-1,4))
        return plane_list_np 

    def tip_calc(self,tip_start,tip_move):
        tip_pos=np.asarray([0.0, 0.0 ,0.0])
        self.XYZ_CT_RC[0]=tip_start[0]-tip_move*np.sin(self.phi_B)
        self.XYZ_CT_RC[1]=tip_start[1]+tip_move*np.sin(self.theta_A_rad)
        self.XYZ_CT_RC[2]=tip_start[2]-tip_move*np.cos(self.theta_A_rad)
        self.p_axis_m+=tip_move
        for i in range(3):
            tip_pos[i]=self.XYZ_CT_RC[i]
        return tip_pos

    def box_bounding(self, vertices_pcd, target_arr):
        vertices_pcd = np.unique(np.asarray(vertices_pcd.points), axis=0)
        x_max, y_max, z_max = np.amax(vertices_pcd, axis=0)
        x_min, y_min, z_min = np.amin(vertices_pcd, axis=0)

        # Filter target_arr based on the bounding box
        mask_x = (target_arr[:, 0] >= x_min) & (target_arr[:, 0] <= x_max)
        mask_y = (target_arr[:, 1] >= y_min) & (target_arr[:, 1] <= y_max)
        mask_z = (target_arr[:, 2] >= z_min) & (target_arr[:, 2] <= z_max)

        target_arr = target_arr[mask_x & mask_y & mask_z]

        return target_arr
    
    def create_plane_for_calc(self, pns, dict_T_CT_H, point_pcd, normal_pcd, vertices_pcd):
        plane_list_part = {}
        vertices_list_part = {}
        for part in pns:
            point_part = point_pcd[part]
            normal_part = normal_pcd[part]
            vertices_part = vertices_pcd[part]
            T = dict_T_CT_H[part]
            plane_list_each_part = {}
            vertices_list_each_part = {}
            for number in range(len(self.devide_number)):
                vertices_part[number].transform(T)
                transform_result = self.mesh_norm_recalc(point_part[number], normal_part[number], T)
                plane_list = self.create_plane(transform_result[0], transform_result[1])
                plane_list_each_part[number] = plane_list
                vertices_list_each_part[number] = vertices_part[number]
            plane_list_part[part] = plane_list_each_part
            vertices_list_part[part] = vertices_list_each_part
        return plane_list_part, vertices_list_part


    def process_part(self, part, plane_list, vertices_part, target_arr):
        break_frag = False
        non_count = 0
        disp_collision_point_arr = []
        plane_list_part = plane_list[part]
        vertices_pcd = vertices_part[part]
        for number in range(len(self.devide_number)):
            target_arr_calc = self.box_bounding(vertices_pcd[number], target_arr)
            if len(target_arr_calc) == 0:
                non_count += 1
            target_arr_reduce = copy.deepcopy(target_arr_calc)
            target_arr_reduce_copy = copy.deepcopy(target_arr_calc)
            for plane in plane_list_part[number]:
                non_collision_count = 0
                for i in range(len(target_arr_reduce_copy)):
                    target_point = target_arr_reduce_copy[i]
                    point_result = plane[0] * target_point[0] + plane[1] * target_point[1] + plane[2] * target_point[2] + plane[3]
                    if point_result < 0:
                        target_arr_reduce = np.delete(target_arr_reduce, i - non_collision_count, axis=0)
                        non_collision_count += 1
                    if i == len(target_arr_reduce_copy) - 1:
                        target_arr_reduce_copy = copy.deepcopy(target_arr_reduce)
                    if len(target_arr_reduce) == 0:
                        non_count += 1
                        break
            if len(target_arr_reduce) != 0:
                collision_point_arr = target_arr_reduce
                for collision_point in collision_point_arr:
                    disp_collision_point_arr.append(collision_point)
        if non_count != len(self.devide_number):
            break_frag = True
        return part, disp_collision_point_arr, "collision" if break_frag else "non_collision"
    
    def collision_detection(self, plane_list, vertices_part, dict_target_arr):
        all_collision_point_arr = []
        for key, target_arr in dict_target_arr.items():
            non_count_part = 0
            for part in pns:
                part, collision_point_arr, part_result = self.process_part(part, plane_list, vertices_part, target_arr)
                print(f"target={key}")
                print(f"{part}={part_result}")
                print(f"number_of_collision_point={len(collision_point_arr)}")
                print('----------------')
                if part_result == "non_collision":
                    non_count_part += 1
                for collision_point in collision_point_arr:
                    all_collision_point_arr.append(collision_point)

            if non_count_part == len(pns):
                print("non_collision")
                print('----------------')
            else:
                print("collision")
                print('----------------')
        return all_collision_point_arr
                
    def mesh_norm_recalc_for_vis(self, mesh_in, Rin, Tin):
        mesh_in.transform(Tin)      
        mesh_in.compute_vertex_normals()        
        nml = np.asarray(mesh_in.vertex_normals);      
        nmlv = np.dot(nml, Rin)
        mesh_in.vertex_normals = o3d.utility.Vector3dVector(nmlv)
                
    def visualize(self, patient_point, gantry_point, disp_collision_point_arr):
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1080, height=720)       
      
        self.disp_patient_point=o3d.geometry.PointCloud()       
        self.disp_patient_point.points = o3d.utility.Vector3dVector(patient_point)    
        self.disp_patient_point.paint_uniform_color([0.8, 0.8, 0.8])           
        vis.add_geometry(self.disp_patient_point)

        self.disp_gantry_point=o3d.geometry.PointCloud()        
        self.disp_gantry_point.points= o3d.utility.Vector3dVector(gantry_point)        
        self.disp_gantry_point.paint_uniform_color([0.5, 0.5, 0.5])       
        vis.add_geometry(self.disp_gantry_point)
        
        self.disp_collision_point_arr = np.asarray(disp_collision_point_arr)
        self.disp_collision_point_arr = np.reshape(self.disp_collision_point_arr,(-1,3))
        self.disp_collision_point_arr = np.unique(self.disp_collision_point_arr, axis=0)
        self.disp_collision_point=o3d.geometry.PointCloud()        
        self.disp_collision_point.points= o3d.utility.Vector3dVector(self.disp_collision_point_arr)        
        self.disp_collision_point.paint_uniform_color([1, 0, 0])       
        vis.add_geometry(self.disp_collision_point)

        for pn in pns:
            T = self.dict_T_CT_H[pn]  
            R = T[:3,:3]     
            self.mesh_norm_recalc_for_vis(self.dict_mesh[pn], R, T)
            self.dict_mesh[pn].paint_uniform_color([0.5, 0.5, 0.5])
            vis.add_geometry(self.dict_mesh[pn])
            self.pcd_mesh = self.dict_convex_hull_vertices_disp[pn]
            for number in range(len(self.devide_number)):
                self.pcd_mesh[number].transform(T)
                self.vertices_disp[number]=o3d.geometry.PointCloud()
                self.vertices_disp[number].points=o3d.utility.Vector3dVector(np.asarray(self.pcd_mesh[number].points))
                self.hull[number], _  = self.vertices_disp[number].compute_convex_hull() 
                self.hull_ls[number]=o3d.geometry.LineSet.create_from_triangle_mesh(self.hull[number])
                self.hull_ls[number].paint_uniform_color([1.0, 0.0, 0.0])
                vis.add_geometry(self.hull_ls[number])
        vis.run()       
        vis.destroy_window()
            
def read_PCD(pcd_file):      
    pcd_arr=o3d.io.read_point_cloud(pcd_file)
    pcd_point_arr = np.asarray(pcd_arr.points) 
    return pcd_point_arr
    
def read_STL(stl_file):
    stl_arr = o3d.io.read_triangle_mesh(stl_file);
    stl_arr = stl_arr.sample_points_poisson_disk(10000)
    stl_point_arr = np.asarray(stl_arr.points)
    return stl_point_arr

if __name__=="__main__":
    dict_target_arr = {}
    patient_point_arr = read_PCD("./patient.pcd")
    gantry_point_arr = read_STL("./CTGuntry2020.stl")
    dict_target_arr["patient"] = patient_point_arr
    #dict_target_arr["gantry"] = gantry_point_arr

    CZSD=ClassZerobot3D()

    CZSD.XYZ_CT_RC=[-0.05014, 0.01600, -0.10932]

    angle_A = 0.0
    angle_A_rad=angle_A * 3.14159/180.0
    CZSD.theta_A_rad = angle_A_rad 
    angle_B = 0.0
    angle_B_rad=angle_B * 3.14159/180.0
    CZSD.theta_B_rad = angle_B_rad;

    CZSD.phi_B=np.arcsin(np.cos(angle_A_rad)*np.sin(angle_B_rad))

    CZSD.p_axis_m=0.0
    air_cylinder_len_m = 0.002;#off
    CZSD.air_cylinder_len_m = air_cylinder_len_m
    #air_cylinder_len_m = 0.015;
    spacer_len_m = 0.042;#[m]
    CZSD.spacer_len_m = spacer_len_m
    #needle_length_m = 0.114;
    needle_length_m = 0.110;
    CZSD.needle_length_m = needle_length_m

    tip_pos=np.array([0.0, 0.0, 0.0])
    for i in range(3):
            tip_pos[i]=CZSD.XYZ_CT_RC[i]

    tip_move = 0.012  #[m]

    tip_pos=CZSD.tip_calc(tip_pos,tip_move)

    pns = CZSD.part_names

    CZSD.read_PCD()
    CZSD.calcTranspose()

    start_time=time.time()
    
    plane_list, vertices_part = CZSD.create_plane_for_calc(pns, CZSD.dict_T_CT_H, CZSD.dict_point_pcd, CZSD.dict_normal_pcd, CZSD.dict_vertices_pcd)
        
    disp_collision_point_arr = CZSD.collision_detection(plane_list, vertices_part, dict_target_arr)
    print(f"Total_number_of_collision_point={len(disp_collision_point_arr)}")

    print("judge_time:",time.time()-start_time)
    print("-----------------------")

    CZSD.visualize(patient_point_arr, gantry_point_arr, disp_collision_point_arr)
    

