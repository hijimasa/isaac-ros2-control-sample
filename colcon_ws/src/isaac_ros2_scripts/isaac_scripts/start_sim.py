from omni.isaac.kit import SimulationApp

Frame_per_Second = 60.0

def main():
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

    enable_extension("omni.isaac.ros2_bridge")    
    kit.update()
    enable_extension("omni.isaac.repl")
    
    my_world = World(stage_units_in_meters=1.0)

    import omni.kit.commands
    from omni.isaac.dynamic_control import _dynamic_control
    from pxr import Sdf, Gf, UsdPhysics, UsdLux, PhysxSchema
    
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

    simulation_context = SimulationContext(physics_dt=1.0 / Frame_per_Second, rendering_dt=1.0 / Frame_per_Second, stage_units_in_meters=1.0)

    # Start simulation
    omni.timeline.get_timeline_interface().play()

    try:
        while True:
            kit.update()
    except KeyboardInterrupt:
        # Shutdown and exit
        omni.timeline.get_timeline_interface().stop()
        kit.close()
    
if __name__ == '__main__':
    main()
