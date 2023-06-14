import os
import mmap
import sys
import math
import struct
import argparse
from omni.isaac.kit import SimulationApp
import xml.etree.ElementTree as ET 

def get_args():
    parser = argparse.ArgumentParser(description='This is sample argparse script')
    parser.add_argument('-p', '--path', required=True, type=str, help='URDF Path.')

    return parser.parse_args()

FLOAT_SIZE = 4  #1FLOAT = 4byte
MAP_SIZE = 4 * FLOAT_SIZE * 256 # command position velocity force
MAP_FILE_NAME = "/dev/shm/isaac_ros2_control_data"

class classMMap:
    def __init__(self) -> None:
        self._mm = None
        ''' map file check '''
        if os.path.exists(MAP_FILE_NAME):
            ''' Read memory map '''
            self._readMMapFile()
        else:
            ''' create and initialize '''
            self._createMMapFile()

            ''' Read memory map '''
            self._readMMapFile()

        print(" __init__ comp.")


    def _createMMapFile(self):        
        with open(MAP_FILE_NAME, mode="wb") as file:
            initStr = '00' * MAP_SIZE
            initByte = bytes.fromhex(initStr)
            file.write(initByte)

        print("_createMMapFile fin.")

    def _readMMapFile(self):
        with open(MAP_FILE_NAME, mode="r+b") as file:
            self._mm = mmap.mmap(file.fileno(), 0)
            self._mm.seek(0)

        print("_readMMapFile fin.")

    def ReadFloat(self, adr:int):
        try:
            self._mm.seek(adr*FLOAT_SIZE)
            bytes = self._mm.read(FLOAT_SIZE)
            val = struct.unpack('<f', bytes)[0]
            self._mm.seek(0)
            return val
        except Exception as e:
            print("ReadShort except" + str(e).replace('\n',''))
            return

    def WriteFloat(self, adr:int, data:float) -> None:
        try:
            bytes = struct.pack('<f', data)

            for i in range(FLOAT_SIZE):
                self._mm[adr*FLOAT_SIZE + i] = bytes[i]
        except Exception as e:
            print("WriteShort except" + str(e).replace('\n',''))

    def __del__(self):
        self._mm.close()
        
def search_prim_path(target_dict:dict, path:str, target_name:str):
    if target_dict["A_joint"] == target_name:
        path = path + target_name
        return path
    path = path + target_dict["B_link"] + "/"
    for child in target_dict["B_node"]:
        ret = search_prim_path(child, path, target_name)
        if not ret == None:
            return ret
    return None

def main():
    args = get_args()
    
    clsMMap = classMMap()

    # URDF import, configuration and simulation sample
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})
    import omni.kit.commands
    from omni.isaac.urdf import _urdf
    urdf_interface = _urdf.acquire_urdf_interface()
    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Sdf, Gf, UsdPhysics, UsdLux, PhysxSchema

    # Setting up import configuration:
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.fix_base = False
    import_config.distance_scale = 100

    # Import URDF, stage_path contains the path the path to the usd prim in the stage.
    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=args.path,
        import_config=import_config,
    )
    # Import URDF, stage_path contains the path the path to the usd prim in the stage.
    status, urdf = omni.kit.commands.execute(
        "URDFParseFile",
        urdf_path=args.path,
        import_config=import_config,
    )
    kinematics_chain = urdf_interface.get_kinematic_chain(urdf)
    # Get stage handle
    stage = omni.usd.get_context().get_stage()

    # Enable physics
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
    # Set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    # Set solver settings
    PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")

    # Add ground plane
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage,
        planePath="/groundPlane",
        axis="Z",
        size=1500.0,
        position=Gf.Vec3f(0, 0, -20),
        color=Gf.Vec3f(0.5),
    )

    # Add lighting
    distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

    urdf_root = ET.parse(args.path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break
    urdf_joints = []
    for child in urdf_root.findall('.//ros2_control/joint'):
        print(child.attrib["name"])
        urdf_joints.append(child)

    joint_name = []
    for joint in urdf_joints:
        joint_name.append(joint.attrib["name"])

    joint_type = []
    for joint in urdf_joints:
        for child in urdf_root.findall('./joint'):
            if child.attrib["name"] == joint.attrib["name"]:
                if child.attrib["type"] == "continuous":
                    joint_type.append("angular")
                else:
                    joint_type.append(child.attrib["type"])
                break

    joints_prim_paths = []
    for joint in urdf_joints:
        joints_prim_paths.append(search_prim_path(kinematics_chain, "/" + robot_name + "/", joint.attrib["name"]))
        

    drive = []
    for index in range(len(joints_prim_paths)):
        drive.append(UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath(joints_prim_paths[index]), joint_type[index]))

    # dynamic control can also be used to interact with the imported urdf.
    dc = _dynamic_control.acquire_dynamic_control_interface()

    # Start simulation
    omni.timeline.get_timeline_interface().play()
    # perform one simulation step so physics is loaded and dynamic control works.
    kit.update()
    art = dc.get_articulation(stage_path)

    if art == _dynamic_control.INVALID_HANDLE:
        print(f"{stage_path} is not an articulation")
    else:
        print(f"Got articulation {stage_path} with handle {art}")

    try:
        while True:
            for index in range(len(joints_prim_paths)):
                radps = clsMMap.ReadFloat(4*index);
                # Set the velocity drive target in degrees/second
                print(radps * 180 / math.pi)
                drive[index].GetTargetVelocityAttr().Set(radps * 180 / math.pi)

                # Set the drive damping, which controls the strength of the velocity drive
                drive[index].GetDampingAttr().Set(15000)

                # Set the drive stiffness, which controls the strength of the position drive
                # In this case because we want to do velocity control this should be set to zero
                drive[index].GetStiffnessAttr().Set(0)
                
                dof_ptr = dc.find_articulation_dof(art, joint_name[index])
                # read position
                clsMMap.WriteFloat(4*index+1, dc.get_dof_position(dof_ptr))
                # read velocity
                clsMMap.WriteFloat(4*index+2, dc.get_dof_velocity(dof_ptr))
                # read effort
                clsMMap.WriteFloat(4*index+3, dc.get_dof_effort(dof_ptr))

            kit.update()
    except:
        # Shutdown and exit
        omni.timeline.get_timeline_interface().stop()
        kit.close()
    
if __name__ == '__main__':
    main()
