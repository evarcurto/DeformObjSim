import pybullet as p
import numpy as np
import time
import os
from pathlib import Path
from PIL import Image
import matplotlib.pyplot as plt
import math
import sympy
from sympy import Symbol, Matrix
import csv


class DefEnv:
   
    def __init__(self, bullet_client, gui = True) -> None:
        self.bullet_client = bullet_client
        self.gui = gui
        if self.gui == True:
            # connect bullet
            self.bullet_client.connect(self.bullet_client.GUI) #or p.GUI (for test) or p.DIRECT (for train) for non-graphical version
        else:
            self.bullet_client.connect(self.bullet_client.DIRECT) 

        #p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.bullet_client.setAdditionalSearchPath(os.path.dirname(__file__) + '/objects') #só posso usar uma vez este comando senão o segundo sobrepom-se ao primeiro

        #directory to save RGBD captures
        self.directory_gt = Path(r'/media/eva-isr/SSD/Simulator_Datasets/dataset3/gt')
        self.directory_gt.mkdir(parents=True, exist_ok=True)
        self.directory_color_frames = Path(r'/media/eva-isr/SSD/Simulator_Datasets/dataset3/frames/color/')
        self.directory_depth_frames = Path(r'/media/eva-isr/SSD/Simulator_Datasets/dataset3/frames/depth/')
        self.directory_depth_maps = Path(r'/media/eva-isr/SSD/Simulator_Datasets/dataset3/depth_maps')
      

        #self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        #self.flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES

        self.counter = 0

    def initial_reset(self):
        # reset pybullet to deformable object
        self.bullet_client.resetSimulation(self.bullet_client.RESET_USE_DEFORMABLE_WORLD)
        
        self.bullet_client.setGravity(0, 0, -9.81)
        self.load_plane()

        #Load deformable object - chose object to deform
        self.load_deformable_object()
        #self.load_deformable_object_anchor()
        #self.load_deformable_ball()

        p.stepSimulation()
        #p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
        self.camera_system(4,True)
        
        
    def load_plane(self):
        self.planeId = self.bullet_client.loadURDF("plane/plane.urdf")
        tex_plane = p.loadTexture("checker_blue.png")
        p.changeVisualShape(self.planeId, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex_plane, flags=0)
	
    def load_deformable_object(self):
        self.torusId = p.loadSoftBody("torus/torus_textured.obj", basePosition = [0,0,1], simFileName="torus.vtk", mass = 3, 
                                 useNeoHookean = 1, NeoHookeanMu = 180, NeoHookeanLambda = 600, NeoHookeanDamping = 0.01, 
                                 collisionMargin = 0.006, useSelfCollision = 1, frictionCoeff = 0.5, repulsionStiffness = 800)
        #bunny2 = p.loadURDF("torus_deform.urdf", [0,0,1.5], flags=p.URDF_USE_SELF_COLLISION)
        #tex = p.loadTexture("uvmap.png")
        #p.changeVisualShape(bunny2, -1, rgbaColor=[1,1,1,1], textureUniqueId=tex, flags=0)

    def load_deformable_object_anchor(self):
        self.torusId2 = p.loadSoftBody("torus/torus_textured.obj", basePosition = [0,0,1], mass = 3, useNeoHookean = 0, useBendingSprings=1,
                                 useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, 
                                 useSelfCollision = 0, frictionCoeff = .5, useFaceContact=1)
        p.createSoftBodyAnchor(self.torusId2  ,235,-1,-1)
        p.createSoftBodyAnchor(self.torusId2  ,239,-1,-1)
        p.createSoftBodyAnchor(self.torusId2  ,240,-1,-1)

    def load_deformable_ball(self):
        self.ballId = p.loadSoftBody("ball.obj", simFileName = "ball.vtk", basePosition = [0,0,1.5], scale = 0.5, mass = 4, 
                                useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.001, 
                                useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)

    def get_points3D(self, Id):
        vertices, coords = p.getMeshData(Id, -1, flags=p.MESH_DATA_SIMULATION_MESH)
        file_name_gt = f'gt{self.counter}.csv'
        file_path_gt = os.path.join(self.directory_gt, file_name_gt)
        with open(file_path_gt, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerows(coords)  # Write data rows

 
    def deforming(self):
        self.camera_system(4,False)
        #self.get_points3D(self.torusId2)
        self.RGBDcapture(4, False, self.counter)
        

    def convert_hz_intrinsic_to_opengl_projection(self, K, x0, y0, width, height, znear, zfar, window_coords=None):
        znear = float(znear)
        zfar = float(zfar)
        depth = zfar - znear
        q = -(zfar + znear) / depth
        qn = -2 * (zfar * znear) / depth

        if window_coords=='y up':
            proj = np.array([[ 2*K[0,0]/width, -2*K[0,1]/width, (-2*K[0,2]+width+2*x0)/width, 0 ],
                            [  0,             -2*K[1,1]/height,(-2*K[1,2]+height+2*y0)/height, 0],
                            [0,0,q,qn],  # This row is standard glPerspective and sets near and far planes.
                            [0,0,-1,0]]) # This row is also standard glPerspective.
        else:
            assert window_coords=='y down'
            proj = np.array([[ 2*K[0,0]/width, -2*K[0,1]/width, (-2*K[0,2]+width+2*x0)/width, 0 ],
                            [  0,              2*K[1,1]/height,( 2*K[1,2]-height+2*y0)/height, 0],
                            [0,0,q,qn],  # This row is standard glPerspective and sets near and far planes.
                            [0,0,-1,0]]) # This row is also standard glPerspective.
        return proj

    def camera_system(self, num_cameras: int, show_views: bool):
        self.width = 960
        self.height = 540
        
        aspect = self.width / self.height
      
        cameraTargetPosition=[0.0,0.0,0.5]
        distance=6.5
        yaw=[0+30, 90+30, 180+30, 270+30]
        pitch=-50
        roll=0
        upAxisIndex=2

        #calibration matrix
        K00 = 1050.0
        K01 = 0.0
        K02 = 480.0
        K11 = 1050.0
        K12 = 270.0

        K = Matrix([[K00, K01, K02],
                    [  0, K11, K12],
                    [  0,   0,   1]])
        x0= 0.0
        y0= 0.0
        znear, zfar = .01, 10.
     

        self.rgba_array = []
        self.depth_opengl_array =[]
        for i in range(num_cameras):
            view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition,distance,yaw[i],pitch,roll,upAxisIndex)
            #print(view_matrix)
            #project = self.convert_hz_intrinsic_to_opengl_projection(K,x0,y0,self.width,self.height,znear,zfar, window_coords='y down') 
            #project_opengl_format =tuple(np.array(project).T.reshape(16))
            #print("Projection Matrix OpenGl:", project_opengl_format)
            focal_len = K00
            fovh = (self.height/ 2) / focal_len
            fovh = 180 * np.arctan(fovh) * 2 / np.pi
            projection_matrix = p.computeProjectionMatrixFOV(fovh, aspect, znear, zfar)
            #print(projection_matrix)

            width, height, rgbImg, depthImg, segImg = self.camera_rend(view_matrix,projection_matrix)  
            camera_params = p.getDebugVisualizerCamera()
            
            

            #camera_position = camera_params[11]  # Index 10 corresponds to camera position
            #camera_orientation = camera_params[9:10]  # Index 11 corresponds to camera orientation
            # Extract position and orientation
            #position = camera_position
            #orientation = camera_orientation
            #print('orientation')
            #print(orientation)
            #print('position')
            #print(position)
            # Compute the extrinsic camera matrix
            #extrinsic_matrix = p.getMatrixFromQuaternion(orientation)
            #extrinsic_matrix[:3, 3] = position

            #print('--------------------------------------------------------------')
            #print(i)
            #print(view_matrix)
            # Print the extrinsic matrix
            #print("Extrinsic Matrix:")
            #print(np.array2string(extrinsic_matrix, separator=', '))
            #print('--------------------------------------------------------------')

            rgbBuffer = np.reshape(rgbImg, (height, width, 4))
            rgbArr = np.asarray(rgbBuffer[:,:,:-1])   #guardar só RGB e não RGBA!!!
            self.rgba_array.append(rgbArr)

            depth_buffer_opengl = np.reshape(depthImg, [height, width])
            depth_opengl = zfar * znear / (zfar - (zfar - znear) * depth_buffer_opengl)
            self.depth_opengl_array.append(depth_opengl)

            if show_views==True:
                plt.subplot(2,2,i+1)
                plt.imshow(depth_opengl, cmap='gray', vmin=0, vmax=10)
                plt.title('Depth cam %i' %(i+1))
        plt.show()
                   
        p.stepSimulation()
        
    def camera_rend(self, view_matrix, projection_matrix):

        width, height, rgbImg, depthImg, segImg  = p.getCameraImage(self.width, self.height, view_matrix, projection_matrix)
        return width, height, rgbImg, depthImg, segImg

    def RGBDcapture(self,num_cameras, capture: bool, i: float):
        self.counter += 1
        print("counter", self.counter)
        if capture == True:
            for c in range(num_cameras):

                #Save RGB frame
                self.rgba_array[c] = self.rgba_array[c].astype(np.uint8)
                file_name_color_frame = f'cam{c+1}/rgb_cam{c+1}_im{self.counter}.png'
                file_path_color_frame = os.path.join(self.directory_color_frames, file_name_color_frame)
                dir_split = os.path.split(file_path_color_frame)
                path_without_file = Path(dir_split[0])
                path_without_file.mkdir(parents=True, exist_ok=True)
                Image.fromarray(self.rgba_array[c]).save(file_path_color_frame)

                # Scale the depth values to the range [0, 255]
                depth_scaled = (self.depth_opengl_array[c] - np.min(self.depth_opengl_array[c])) / (np.max(self.depth_opengl_array[c]) - np.min(self.depth_opengl_array[c]))
                depth_scaled *= 255

                # Convert the depth data to uint8
                depth_uint8 = depth_scaled.astype(np.uint8)

                #Save Depth frame
                new_p1 = Image.fromarray(depth_uint8)
                file_name_depth_frame = f'cam{c+1}/depth_cam{c+1}_im{self.counter}.png'
                file_path_depth_frame = os.path.join(self.directory_depth_frames, file_name_depth_frame)
                dir_split = os.path.split(file_path_depth_frame)
                path_without_file = Path(dir_split[0])
                path_without_file.mkdir(parents=True, exist_ok=True)
                new_p1.save(file_path_depth_frame)

                #Print depth data to .csv files
                file_name_depth_map = f'cam{c+1}/depth_map_cam{c+1}_im{self.counter}.csv'
                file_path_depth_maps = os.path.join(self.directory_depth_maps, file_name_depth_map)
                dir_split = os.path.split(file_path_depth_maps)
                path_without_file = Path(dir_split[0])
                path_without_file.mkdir(parents=True, exist_ok=True)
                np.savetxt(file_path_depth_maps, self.depth_opengl_array[c], delimiter=",", fmt='%1.4f')


class SimAuto(DefEnv):
  def __init__(self, bullet_client):
    DefEnv.__init__(self, bullet_client)
