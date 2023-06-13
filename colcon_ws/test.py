import xml.etree.ElementTree as ET
tree = ET.parse("./install/diffbot_description/share/diffbot_description/robots/diffbot.urdf")
root = tree.getroot()

print(root.iter('robot').attrib["name"])
for child in root.iter('robot'):
    print(child.attrib["name"])
    print("get")
for child in root.findall('.//ros2_control/joint'):
    print(child.attrib["name"])

from omni.isaac.kit import SimulationApp
import omni.kit.commands

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False

urdf = omni.kit.commands.execute("URDFParseFile", 
        urdf_path="./install/diffbot_description/share/diffbot_description/robots/diffbot.urdf",
        import_config=import_config,
        )
