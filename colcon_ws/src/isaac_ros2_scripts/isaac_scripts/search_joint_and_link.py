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