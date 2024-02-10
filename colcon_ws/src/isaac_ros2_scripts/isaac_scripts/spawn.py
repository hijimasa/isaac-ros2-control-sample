import omni
import omni.kit.commands
from omni.isaac.urdf import _urdf

def main(urdf_path:str):
    urdf_interface = _urdf.acquire_urdf_interface()
    
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = False
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.fix_base = False
    import_config.distance_scale = 1
    
    status, stage_path = omni.kit.commands.execute("URDFParseAndImportFile", urdf_path=urdf_path, import_config=import_config,)
