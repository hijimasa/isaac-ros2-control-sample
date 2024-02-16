import sys
from omni.isaac.kit import SimulationApp

Frame_per_Second = 60.0

def main():
    args = sys.argv
    usd_path = args[1]
    
    # URDF import, configuration and simulation sample
    kit = SimulationApp({"renderer": "RayTracedLighting", "headless": False, "open_usd": usd_path})

    import omni
    from omni.isaac.core.utils.extensions import enable_extension, disable_extension
    from omni.isaac.core import SimulationContext, World

    enable_extension("omni.isaac.ros2_bridge")    
    kit.update()
    enable_extension("omni.isaac.repl")
    
    my_world = World(stage_units_in_meters=1.0)

    import omni.kit.commands
    from pxr import Sdf, Gf, UsdPhysics, PhysxSchema
    
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
