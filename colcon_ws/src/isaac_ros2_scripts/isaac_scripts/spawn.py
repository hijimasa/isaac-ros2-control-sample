import math
import omni
import omni.kit.commands
import omni.usd
from pxr import UsdGeom, Gf

def main(urdf_path:str, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=90, fixed=False):
    status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.self_collision = False
    import_config.fix_base = fixed
    import_config.default_drive_strength = 100000.0
    import_config.default_position_drive_damping = 100000.0
    import_config.distance_scale = 1

    status, stage_path = omni.kit.commands.execute(
        "URDFParseAndImportFile", 
        urdf_path=urdf_path, 
        import_config=import_config,
        get_articulation_root=True,
        )

    # ステージを取得
    stage = omni.usd.get_context().get_stage()

    # 指定したパスのオブジェクトにアクセス
    obj = stage.GetPrimAtPath(stage_path)
    if obj.IsValid():
        # オブジェクトの位置を取得
        obj_xform = UsdGeom.Xformable(obj)

        # オブジェクトの現在の回転をクリア
        obj_xform.ClearXformOpOrder()

        # 回転のXformOpを追加
        rotate_op = obj_xform.AddRotateXYZOp()

        # 回転を設定（度数法で指定）
        rotate_op.Set((roll*180.0/math.pi, pitch*180.0/math.pi, yaw*180.0/math.pi))

        # 回転のXformOpを追加
        translate_op = obj_xform.AddTranslateOp()
        translate_op.Set((x, y, z))

    return stage_path
