import Sofa
import Sofa.Core
import sys
import os
path = os.path.dirname(os.path.abspath(__file__)) +'/mesh/'
resultPath = os.path.dirname(os.path.abspath(__file__))
import matplotlib.pyplot as plt
import numpy as np
from sys import argv

# Choose in your script to activate or not the GUI
USE_GUI = False

print("CURRENT PATH IS: "+os.getcwd())


class Controller(Sofa.Core.Controller):   
    
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        print(" Python::__init__::" + str(self.name.value))
        
        self.RootNode = kwargs['RootNode']        
        self.TubeMO = self.RootNode.Shape.tetras
        self.ROI1 = self.RootNode.Shape.roi1
        self.ROI2 = self.RootNode.Shape.roi2
        self.ROI3 = self.RootNode.Shape.roi3
        self.ROI4 = self.RootNode.Shape.roi4
        self.ReferenceMO = self.RootNode.ExternalMONode.ExternalMO
        
        self.NIterations = 200
        self.Iterator = 0
        self.ForcesArray = np.empty((0,1))
        self.DiffsArray = np.empty((0,1))
        self.DiffsArray2_X = np.empty((0,1))
        self.DiffsArray2_Y = np.empty((0,1))
        self.Increment = 0.1

    def onAnimateBeginEvent(self, eventType):
        
        SelectedIndices1 = np.array(self.ROI1.indices.value) # indices for monitoring buckling force
        SelectedIndices2 = np.array(self.ROI2.indices.value) # indices for monitoring buckling force
        SelectedIndices3 = np.array(self.ROI3.indices.value) # indices for monitoring displacement
        SelectedIndices4 = np.array(self.ROI4.indices.value) # indices for monitoring displacement of buckling
        
        DisIndices = SelectedIndices3[0]
        ForceIndices = SelectedIndices1[0]
        #DisIndices2 = SelectedIndices4[0]

        #print("SelectedIndics", DisIndices)
        #print("SelectedIndics", SelectedIndices4)
        #print("lengthIndics", len(SelectedIndices4))
        ForcesDis = np.array(self.TubeMO.position.value)
        ForcesRest = np.array(self.TubeMO.rest_position.value)
        #AllForcesDis = np.array(self.TubeMO.force.value)
        AllForcesDis = ForcesDis - ForcesRest
        #print("AllForcesDis", AllForcesDis)
        #self.SelectedForcesDis = AllForcesDis[ForceIndices,2] #From the forces on all the nodes, select the ones inside roi1, which is a cross-section
        self.SelectedForcesDis = AllForcesDis[SelectedIndices1,:]
        #print("SelectedForcesDis", AllForcesDis[ForceIndices,2]) 
        
        AllDisplacements = np.array(self.TubeMO.position.value)
        AllRestPositions = np.array(self.TubeMO.rest_position.value)
        #print("AllDisplacements", AllDisplacements)
        AllDiffs = AllDisplacements-AllRestPositions
        self.SelectedDiffs = AllDiffs[DisIndices,2]  # measure displacement in z-axis
        
        AllDisplacements2 = np.array(self.TubeMO.position.value)
        AllRestPositions2 = np.array(self.TubeMO.rest_position.value)
        #print("AllDisplacements2", AllDisplacements2)
        AllDiffs2 = AllDisplacements2-AllRestPositions2
        #self.SelectedDiffs2_X = AllDiffs2[DisIndices2,0]
        #self.SelectedDiffs2_Y = AllDiffs2[DisIndices2,1] 
        self.SelectedDiffs2_X = AllDiffs2[SelectedIndices4,0]
        self.SelectedDiffs2_Y = AllDiffs2[SelectedIndices4,1]
        
        if self.Iterator <= self.NIterations:
            CurrentPositions = np.array(self.ReferenceMO.position.value) # get current positions
            CurrentPositions[:,2] = CurrentPositions[:,2] - self.Increment # apply increment to positions
            self.ReferenceMO.position.value = CurrentPositions.tolist() # Move the reference MO, so that the end of the object will move accordingly           
            self.ReferenceMO.reinit()
            
            TotalForceZ = np.sum(self.SelectedForcesDis[:,2])*1e13  # integrate over all the forces of the cross-section. Only do so along the Z-axis, because an uni-axial sensor would not be able to detect the forces in the other directions
            #TotalForceZ = self.SelectedForcesDis*1e13 # Multiply the distance by the spring stiffness to find force
            print("TotalForceZ", TotalForceZ)
            TotalDiffZ = self.SelectedDiffs
            #TotalDiff2_X = np.sum(self.SelectedDiffs2_X)/len(SelectedIndices4) # Buclking displacement in x axis
            #TotalDiff2_Y = np.sum(self.SelectedDiffs2_Y)/len(SelectedIndices4) # Buclking displacement in y axis
            TotalDiff2_X = self.SelectedDiffs2_X # Buclking displacement in x axis
            TotalDiff2_Y = self.SelectedDiffs2_Y # Buclking displacement in y axis
            
            #print("Total force in Z: ", TotalForceZ)
            #print("Total displacement in Z: ", TotalDiffZ)
            #print("Total displacement in X: ", TotalDiff2_X)
            self.ForcesArray = np.append(self.ForcesArray, TotalForceZ) 
            self.DiffsArray = np.append(self.DiffsArray, TotalDiffZ)
            self.DiffsArray2_X = np.append(self.DiffsArray2_X, TotalDiff2_X)
            self.DiffsArray2_Y = np.append(self.DiffsArray2_Y, TotalDiff2_Y)
            #self.DiffsArray2 = np.array(self.DiffsArray2_X,self.DiffsArray2_Y)
            #print(self.DiffsArray2)
            
                
            
        elif self.Iterator == self.NIterations + 1:
            plt.figure(1)
            plt.subplot(211) 
            plt.plot(-self.ForcesArray) # Plot the force series
            plt.subplot(212) 
            plt.plot(-self.DiffsArray) # Plot the displacement series
            plt.figure(2)
            plt.subplot(211) 
            plt.plot(self.DiffsArray2_X) # Plot the buckling displacement series X
            plt.subplot(212) 
            plt.plot(self.DiffsArray2_Y) # Plot the buckling displacement series Y
            plt.figure(3)
            plt.plot(self.DiffsArray2_X,self.DiffsArray2_Y) 
            plt.show()          
            
        self.Iterator += 1
        
        pass
  
        
    def onKeypressedEvent(self, c):
        key = c['key']
        print("key",key)
    
        CurrentPositions = np.array(self.ReferenceMO.position.value)
        Increment = 0.25
        print("CurrentPositions", CurrentPositions)

        if (key == "+"):     
            print('awa')
            CurrentPositions[:,2] = CurrentPositions[:,2] + Increment
            self.ReferenceMO.position.value = CurrentPositions.tolist()            
            self.ReferenceMO.reinit()
            
        if (key == "-"):            
            print('awa2')
            CurrentPositions[:,2] = CurrentPositions[:,2] - Increment
            self.ReferenceMO.position.value = CurrentPositions.tolist()            
            self.ReferenceMO.reinit()
            
        
        print("SelectedForces", self.SelectedForces)
        print("Total force in Z: ", np.sum(self.SelectedForces[:,2]))

     
def createScene(rootNode):
    print("This is the axial load Circle cylinder simulation")
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SofaPython3 SofaSparseSolver SofaOpenglVisual SofaExporter SofaConstraint SofaDeformable SofaEngine SofaImplicitOdeSolver SofaLoader SofaSimpleFem SofaBoundaryCondition SofaGeneralLoader SofaValidation')
    rootNode.addObject('VisualStyle', displayFlags='hideVisualModels showBehaviorModels showForceFields showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    # Add a QPInverseProblemSolver to the scene if you need to solve inverse problem like the one that involved
    # when manipulating the robots by specifying their effector's position instead of by direct control
    # of the actuator's parameters.
    #rootNode.addObject('QPInverseProblemSolver', printLog=False)
    # Otherwise use a GenericConstraintSolver
 
    rootNode.addObject('GenericConstraintSolver', tolerance=1e-6, maxIterations=200)

    rootNode.gravity = [0.0, 0.0, 0.0] #[0.0, -9810, 0.0] # Applied gravitational force in y axis, Applied 40N axial load (z axis) 
    #totalForce = [Fx, Fy, Fz, Tx, Ty, Tz] force template
    #
    rootNode.dt=0.1 # Timestep 100 ms - make sure the gravity force is also compatible 
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    # RGB Red => First (x axis), Green => Second (y axis), Blue => Third (z axis)
    rootNode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    
    
    s = rootNode.addChild("Shape")
    
    # Time integration scheme
    s.addObject('EulerImplicitSolver', name='odesolver', firstOrder=True, rayleighMass=0.1, rayleighStiffness=0.1)
    
    # Solving method
    s.addObject('SparseLDLSolver', name='preconditioner')
    
    # Add a componant to load a VTK tetrahedral mesh and expose the resulting topology in the scene .
    #s.addObject('MeshVTKLoader', name='loader', filename=path+'Circle_cylinder.vtk') # the mesh unit is mm
    s.addObject('MeshVTKLoader', name='loader', filename=path+'n5_a2.5.vtk') # the mesh unit is mm
    s.addObject('MeshTopology', src='@loader', name='container')
    s.addObject('TetrahedronSetTopologyModifier')

    # Create a mechanical object component to stores the DoFs of the model
    s.addObject('MechanicalObject', name='tetras', template='Vec3', showIndices=False, showIndicesScale=4e-5) # Choose template='Vec3' for beam simulation
    s.addObject('VolumeFromTetrahedrons')


    # Gives a mass to the model
    s.addObject('UniformMass', totalMass=0.0033) # change mass accordingly 3.3g

    # Add a TetrahedronFEMForceField componant which implement an elastic material model solved using the Finite Element Method on
    # tetrahedrons.
    #s.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.48,  youngModulus=12, computeVonMisesStress=2) # Choose template='Vec3' for beam simulation, 3d printer ninjaflex material
    s.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.48,  youngModulus=12e3) # Choose template='Vec3' for beam simulation, 3d printer ninjaflex material.

    # To facilitate the selection of DoFs, SOFA has a concept called ROI (Region of Interest).
    # The idea is that ROI component "select" all DoFS that are enclosed by their "region".
    # We use ROI here to select a group of finger's DoFs that will be constrained to stay
    # at a fixed position.
    # You can either use 'BoxROI'...
    s.addObject('BoxROI', name='roi1', box=[-10, -10, 9, 10, 10, 11], drawBoxes=True)
    s.addObject('BoxROI', name='roi2', box=[-10, -10, 85, 10, 10, 79], drawBoxes=True)
    s.addObject('BoxROI', name='roi3', box=[-10, -10, 77.6, 10, 10, 76.3], drawBoxes=True)
    s.addObject('BoxROI', name='roi4', box=[-10, -10, 41, 10, 10, 40], drawBoxes=True)
    # Or 'SphereROI'...
    #finger.addObject('SphereROI', name='roi', centers=[0, 0, 0], radii=5)

    # RestShapeSpringsForceField is one way in Sofa to implement fixed point constraint.
    # Here the constraints are applied to the DoFs selected by the previously defined BoxROI
    s.addObject('RestShapeSpringsForceField', points=s.roi1.indices.getLinkPath(), stiffness=1e13)
    s.addObject('PartialFixedConstraint', indices= '@roi2.indices', fixedDirections = [1, 1, 0])


    s.addObject('RestShapeSpringsForceField', name='fixed1', points="@roi2.indices", external_rest_shape="@../ExternalMONode/ExternalMO", stiffness=1e13)
    s.addObject('ConstantForceField', name = 'CFF', indices = '@roi4.indices', totalForce = [100.0, 0.0, 0.0], showArrowSize = '0.5') # Here apply 100mN
    '''forcePointer = s.CFF.totalForce
    forcePointer.value = [float(argv[1]), float(argv[2]), float(argv[3])] # Casting string arguments to float
    print("x,y,z forces:", argv[1], argv[2], argv[3])'''
    # It is also possible to simply set by hand the indices of the points you want to fix.
    #finger.addObject('RestShapeSpringsForceField', points=[0, 1, 2, 11, 55], stiffness=1e12)

    s.addObject('GenericConstraintCorrection')
    
    # Collision model for morphels
    # 

    ##########################################
    # Visualization                          #
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    # Create an empty child node to store this rendering model.
    circleVisual = s.addChild('visual')

    # Add to this empty node a rendering model made of triangles and loaded from an stl file.
    #circleVisual.addObject('MeshSTLLoader', filename=path+"Circle_cylinder_tas.stl", name="loader")
    circleVisual.addObject('MeshSTLLoader', filename=path+"n5_a2.5.stl", name="loader")
    circleVisual.addObject('OglModel', src="@loader", color=[1,0.0,0.0])             #[0.0, 0.7, 0.7, 1])

    # Add a BarycentricMapping to deform the rendering model in a way that follow the ones of the parent mechanical model.
    circleVisual.addObject('BarycentricMapping') # use when its a deformable model
    
    ##########################################
    # Simulation results                     #
    ##########################################
    #s.addObject('ExtraMonitor', template = 'Vec3', name = 'Simulation_results_circle_axial_load', listening = '1', indices = '@roi3.indices', showPositions = '0', PositionsColor = [1, 1, 0, 1], showTrajectories = 'true', TrajectoriesColor = [1, 0, 1, 1], showForces = 'false', ForcesColor =[1, 0, 0, 1], sizeFactor = '0.5', ExportForces = 'true',resultantF = 'true')
    #s.addObject('Monitor', template = 'Vec3', name = 'Simulation_results_circle_axial_load', listening = '1', indices = '@roi3.indices', showPositions = '0', PositionsColor = [1, 1, 0, 1], showTrajectories = 'true', TrajectoriesColor = [1, 0, 1, 1], showForces = 'false', ForcesColor =[1, 0, 0, 1], sizeFactor = '0.5', ExportPositions = 'true')
    
    # can also use VTKExporter to extract data.
    #s.addObject('VTKExporter', name = 'Simulation_data', filename = 'Ellipse_axial_load_data', position = '@MechanicalObject.position', pointsDataFields = '@MechanicalObject.velocity', exportAtEnd ='1')
    ExternalMONode = rootNode.addChild('ExternalMONode')
    ExternalMONode.addObject('MechanicalObject', name='ExternalMO', template='Vec3', showObject=True ,showObjectScale= 15, showIndices=False, showIndicesScale=4e-5, position='@../Shape/roi2.pointsInROI');
    
    rootNode.addObject(Controller(name="DeformationController", RootNode=rootNode))
    
    return rootNode
    
    ###########################################
    # Plotting data                           #
    ###########################################
def my_plotter(): 
       
        
    fig1 = plt.figure(1)
    plt.subplot(211) 
    #x = data[:,0]
    y = -data1/1000  # the force is in mN so divided by 1000 to convert to N    
    plt.plot(y,'r--')
    plt.ylabel('Force(N)')
    
    plt.subplot(212) 
    #x2 = data2[:,0]
    y2 = -data2
    plt.plot(y2,'g--')
    plt.xlabel('Steps')
    plt.ylabel('Displacement z-axis (mm)')
    
       
    fig2 = plt.figure(2)
     
    x3 = data3
    y3 = data4
    plt.plot(x3, y3,'g--')
    plt.xlabel('Displacement x-axis (mm)')
    plt.ylabel('Displacement y-axis (mm)')
        
    
    #plt.show()
    #print('max force',max(y))
    
def main():
    import Sofa.Gui
    import SofaRuntime
    print("Number of arguments:", len(sys.argv), "arguments.")
    print("Argument List:", str(sys.argv))
    
    SofaRuntime.importPlugin("SoftRobots")
    root=Sofa.Core.Node("root")
    createScene(root)
    Sofa.Simulation.init(root)

    # Run the simulation for 200 steps    
    if not USE_GUI:
        for iteration in range(10):
            print(f'Iteration #{iteration}')
            Sofa.Simulation.animate(root, root.dt.value)           
    else:
        Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
        Sofa.Gui.GUIManager.createGUI(root, __file__)
        Sofa.Gui.GUIManager.SetDimension(1080, 1080)
        Sofa.Gui.GUIManager.MainLoop(root)
        Sofa.Gui.GUIManager.closeGUI()
        
    print("End of simulation.")
    my_plotter()

# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
    
    
    
    
    
    
