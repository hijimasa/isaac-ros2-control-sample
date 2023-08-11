import os
import mmap
import sys
import math
import struct
import argparse
from omni.isaac.kit import SimulationApp
import xml.etree.ElementTree as ET 
import inspect

Frame_per_Second = 60.0

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
        
def search_joint_prim_path(target_dict:dict, path:str, target_name:str):
    if target_dict["A_joint"] == target_name:
        path = path + target_name
        return path
    path = path + target_dict["B_link"] + "/"
    for child in target_dict["B_node"]:
        ret = search_joint_prim_path(child, path, target_name)
        if not ret == None:
            return ret
    return None

def search_link_prim_path(target_dict:dict, path:str, target_name:str):
    if target_dict["B_link"] == target_name:
        path = path + target_name
        return path
    if "A_link" in target_dict:
        path = path + target_dict["A_link"] + "/"
    for child in target_dict["B_node"]:
        ret = search_link_prim_path(child, path, target_name)
        if not ret == None:
            return ret
    return None

def main():
    args = get_args()
    
    clsMMap = classMMap()

    
    # URDF import, configuration and simulation sample
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

    import omni
    from omni.isaac.core.utils.extensions import enable_extension, disable_extension
    from omni.isaac.core import SimulationContext, World
    from omni.isaac.core.utils import stage, extensions, nucleus
    from omni.isaac.core.utils.render_product import create_hydra_texture
    import omni.kit.viewport.utility
    import omni.replicator.core as rep
    #from omni.isaac.sensor import LidarRtx, Camera
    #import omni.isaac.core.utils.numpy.rotations as rot_utils
    #import numpy as np
    from pxr import Gf, UsdGeom, Usd
    from omni.kit.viewport.utility import get_active_viewport, get_viewport_from_window_name
    import omni.graph.core as og
    from omni.isaac.core.utils.prims import set_targets

    disable_extension("omni.isaac.ros_bridge")
    kit.update()
    disable_extension("omni.isaac.ros2_bridge")
    kit.update()
    enable_extension("omni.isaac.ros2_bridge-humble")    
    kit.update()
    
    my_world = World(stage_units_in_meters=1.0)

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
    import_config.self_collision = False
    import_config.fix_base = False
    import_config.distance_scale = 1

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
    stage_handle = omni.usd.get_context().get_stage()

    # Enable physics
    scene = UsdPhysics.Scene.Define(stage_handle, Sdf.Path("/physicsScene"))
    # Set gravity
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    # Set solver settings
    PhysxSchema.PhysxSceneAPI.Apply(stage_handle.GetPrimAtPath("/physicsScene"))
    physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage_handle, "/physicsScene")
    physxSceneAPI.CreateEnableCCDAttr(True)
    physxSceneAPI.CreateEnableStabilizationAttr(True)
    physxSceneAPI.CreateEnableGPUDynamicsAttr(False)
    physxSceneAPI.CreateBroadphaseTypeAttr("MBP")
    physxSceneAPI.CreateSolverTypeAttr("TGS")

    # Add ground plane
    omni.kit.commands.execute(
        "AddGroundPlaneCommand",
        stage=stage_handle,
        planePath="/groundPlane",
        axis="Z",
        size=1500.0,
        position=Gf.Vec3f(0, 0, -0.1),
        color=Gf.Vec3f(0.5),
    )

    # Add lighting
    distantLight = UsdLux.DistantLight.Define(stage_handle, Sdf.Path("/DistantLight"))
    distantLight.CreateIntensityAttr(500)

    urdf_root = ET.parse(args.path).getroot()
    robot_name = None
    for child in urdf_root.iter("robot"):
        robot_name = child.attrib["name"]
        break
    urdf_joints = []
    for child in urdf_root.findall('.//ros2_control/joint'):
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
        joints_prim_paths.append(search_joint_prim_path(kinematics_chain, "/" + robot_name + "/", joint.attrib["name"]))
        
    viewportId = 1
    for child in urdf_root.findall('.//isaac/sensor'):        
        if child.attrib["type"] == "lidar":
            prim_path = search_link_prim_path(kinematics_chain, "/" + robot_name + "/", child.attrib["name"])

            _, my_lidar = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/Lidar",
                parent=prim_path,
                config=child.find("config").text,
                translation=(0.0, 0, 0.0),
                orientation=Gf.Quatd(0.5, 0.5, -0.5, -0.5),
            )
            # RTX sensors are cameras and must be assigned to their own render product
            _, render_product_path = create_hydra_texture([1, 1], my_lidar.GetPath().pathString)

            # For Debug
            #writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
            #writer.attach([render_product_path])

            if int(child.find("sensor_dimension_num").text) == 2:
                # Create LaserScan publisher pipeline in the post process graph
                writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
                writer.initialize(topicName=prim_path + "/" + child.find("topic").text, frameId=child.attrib["name"])
                writer.attach([render_product_path])
            else:
                # Create Point cloud publisher pipeline in the post process graph
                writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
                writer.initialize(topicName=prim_path + "/" + child.find("topic").text, frameId=child.attrib["name"])
                writer.attach([render_product_path])
                
            kit.update()

        if child.attrib["type"] == "camera":
            image_height = int(child.find("image/height").text)
            image_width = int(child.find("image/width").text)
            aspect_ratio = float(image_width) / float(image_height)
            horizontal_fov_rad = float(child.find("horizontal_fov_rad").text)
            horizontal_focal_length = float(child.find("horizontal_focal_length").text)
            vertical_focal_length = float(child.find("vertical_focal_length").text)
            focus_distance = float(child.find("focus_distance").text)
            
            horizontal_aperture = math.tan(horizontal_fov_rad / 2.0) * 2.0 * horizontal_focal_length
            vertical_aperture = math.tan(horizontal_fov_rad / aspect_ratio / 2.0) * 2.0 * vertical_focal_length
            
            prim_path = search_link_prim_path(kinematics_chain, "/" + robot_name + "/", child.attrib["name"])

            # Creating a Camera prim
            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(prim_path + "/Camera", "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
            xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetProjectionAttr().Set(child.find("projection").text)
            camera_prim.GetFocalLengthAttr().Set(horizontal_focal_length)
            camera_prim.GetFocusDistanceAttr().Set(focus_distance)
            camera_prim.GetClippingRangeAttr().Set((float(child.find("clip/near").text), float(child.find("clip/far").text)))

            kit.update()

            # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
            keys = og.Controller.Keys
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Camera_Graph",
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                        ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                        ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                        ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                        ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                        ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                        ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                        ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createViewport.inputs:viewportId", viewportId),
                        ("createViewport.inputs:name", prim_path + "/Viewport"),
                        ("cameraHelperRgb.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperRgb.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperRgb.inputs:type", "rgb"),
                        ("cameraHelperInfo.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperInfo.inputs:topicName", prim_path + "/camera_info"),
                        ("cameraHelperInfo.inputs:type", "camera_info"),
                    ],
                },
            )
            
            viewportId += 1

            set_targets(
                prim=stage.get_current_stage().GetPrimAtPath(prim_path + "/Camera_Graph" + "/setCamera"),
                attribute="inputs:cameraPrim",
                target_prim_paths=[prim_path + "/Camera"],
            )

            # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
            og.Controller.evaluate_sync(ros_camera_graph)

            kit.update()

            # Inside the SDGPipeline graph, Isaac Simulation Gate nodes are added to control the execution rate of each of the ROS image and camera info publishers.
            # By default the step input of each Isaac Simulation Gate node is set to a value of 1 to execute every frame.
            # We can change this value to N for each Isaac Simulation Gate node individually to publish every N number of frames.
            viewport_api = get_viewport_from_window_name(prim_path + "/Viewport")
            viewport_api.set_texture_resolution((image_width, image_height))

            if viewport_api is not None:
                import omni.syntheticdata._syntheticdata as sd

                # Get name of rendervar for RGB sensor type
                rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

                # Get path to IsaacSimulationGate node in RGB pipeline
                rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path()
                )

                # Get path to IsaacSimulationGate node in CameraInfo pipeline
                camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
                )

                # Set Rgb execution step to 5 frames
                rgb_step_size = int(Frame_per_Second / float(child.find("update_rate").text))
            
                # Set Camera info execution step to every frame
                info_step_size = 1

                # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
                og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(rgb_step_size)
                og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

            kit.update()

        if child.attrib["type"] == "depth_camera":
            image_height = int(child.find("image/height").text)
            image_width = int(child.find("image/width").text)
            aspect_ratio = float(image_width) / float(image_height)
            horizontal_fov_rad = float(child.find("horizontal_fov_rad").text)
            horizontal_focal_length = float(child.find("horizontal_focal_length").text)
            vertical_focal_length = float(child.find("vertical_focal_length").text)
            focus_distance = float(child.find("focus_distance").text)

            horizontal_aperture = math.tan(horizontal_fov_rad / 2.0) * 2.0 * horizontal_focal_length
            vertical_aperture = math.tan(horizontal_fov_rad / aspect_ratio / 2.0) * 2.0 * vertical_focal_length

            prim_path = search_link_prim_path(kinematics_chain, "/" + robot_name + "/", child.attrib["name"])

            # Creating a Camera prim
            camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(prim_path + "/Camera", "Camera"))
            xform_api = UsdGeom.XformCommonAPI(camera_prim)
            xform_api.SetTranslate(Gf.Vec3d(0, 0, 0))
            xform_api.SetRotate((90, 0, -90), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
            camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
            camera_prim.GetProjectionAttr().Set(child.find("projection").text)
            camera_prim.GetFocalLengthAttr().Set(horizontal_focal_length)
            camera_prim.GetFocusDistanceAttr().Set(focus_distance)
            camera_prim.GetClippingRangeAttr().Set((float(child.find("clip/near").text), float(child.find("clip/far").text)))

            kit.update()

            # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
            keys = og.Controller.Keys
            (ros_camera_graph, _, _, _) = og.Controller.edit(
                {
                    "graph_path": prim_path + "/Camera_Graph",
                    "evaluator_name": "push",
                    "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
                },
                {
                    keys.CREATE_NODES: [
                        ("OnTick", "omni.graph.action.OnTick"),
                        ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                        ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                        ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                        ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                        ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ],
                    keys.CONNECT: [
                        ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                        ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                        ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                        ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                        ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                        ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                        ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                    ],
                    keys.SET_VALUES: [
                        ("createViewport.inputs:viewportId", viewportId),
                        ("createViewport.inputs:name", prim_path + "/Viewport"),
                        ("cameraHelperInfo.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperInfo.inputs:topicName", prim_path + "/camera_info"),
                        ("cameraHelperInfo.inputs:type", "camera_info"),
                        ("cameraHelperDepth.inputs:frameId", child.attrib["name"]),
                        ("cameraHelperDepth.inputs:topicName", prim_path + "/" + child.find("topic").text),
                        ("cameraHelperDepth.inputs:type", "depth"),
                    ],
                },
            )
            
            viewportId += 1

            set_targets(
                prim=stage.get_current_stage().GetPrimAtPath(prim_path + "/Camera_Graph" + "/setCamera"),
                attribute="inputs:cameraPrim",
                target_prim_paths=[prim_path + "/Camera"],
            )

            # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
            og.Controller.evaluate_sync(ros_camera_graph)

            kit.update()

            # Inside the SDGPipeline graph, Isaac Simulation Gate nodes are added to control the execution rate of each of the ROS image and camera info publishers.
            # By default the step input of each Isaac Simulation Gate node is set to a value of 1 to execute every frame.
            # We can change this value to N for each Isaac Simulation Gate node individually to publish every N number of frames.
            viewport_api = get_viewport_from_window_name(prim_path + "/Viewport")
            viewport_api.set_texture_resolution((image_width, image_height))

            if viewport_api is not None:
                import omni.syntheticdata._syntheticdata as sd

                # Get name of rendervar for DistanceToImagePlane sensor type
                rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                    sd.SensorType.DistanceToImagePlane.name
                )

                # Get path to IsaacSimulationGate node in Depth pipeline
                depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path()
                )

                # Get path to IsaacSimulationGate node in CameraInfo pipeline
                camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                    "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
                )
            
                # Set Depth execution step to 60 frames
                depth_step_size = int(Frame_per_Second / float(child.find("update_rate").text))

                # Set Camera info execution step to every frame
                info_step_size = 1

                # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
                og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(depth_step_size)
                og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

            kit.update()

    drive = []
    for index in range(len(joints_prim_paths)):
        drive.append(UsdPhysics.DriveAPI.Get(stage_handle.GetPrimAtPath(joints_prim_paths[index]), joint_type[index]))

    # dynamic control can also be used to interact with the imported urdf.
    dc = _dynamic_control.acquire_dynamic_control_interface()

    simulation_context = SimulationContext(physics_dt=1.0 / Frame_per_Second, rendering_dt=1.0 / Frame_per_Second, stage_units_in_meters=1.0)

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
