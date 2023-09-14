import pybullet as p
import time
from deform_env import DefEnv
from deform_env import SimAuto


def main():

    DeformObj = SimAuto(p)
    #timeStep=1./240.
    timeStep=1./120.
    #timeStep= 0.001
    p.setTimeStep(timeStep)
    #DeformObj.control_dt = timeStep
    DeformObj.initial_reset()
    #p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)
    #p.setRealTimeSimulation(0)

    dataset_size = 0
    #for i in range (15000000):
    while (dataset_size <= 150):
        DeformObj.deforming()
        p.stepSimulation()
        dataset_size = DeformObj.counter
        #time.sleep(timeStep)

   
if __name__ == '__main__':
    main()

