import bpy
from bpy.props import StringProperty, BoolProperty
from bpy.types import Operator
from bpy_extras.io_utils import ExportHelper

import os

import mathutils
from mathutils import Matrix, Vector

bl_info = {
    "name": "MJCF Exporter",
    "author": "Balint Hodossy",
    "version": (0, 1),
    "blender": (4, 2, 5),
    "location": "File > Import-Export",
    "description": "Exporter for creating an MJCF file from parented meshes and empty objects",
    "warning": "Under development. Using exporter will change the scene, by switching rotation modes and parent matrices",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Import-Export"
}


class MuJoCoExportOperator(Operator, ExportHelper):
    """
    Builds an MJCF file from the scene hierarchy.
    """
    bl_idname = "export_scene.mjcf_export"  # ID name of the operator
    bl_label = "Export MJCF"  # Label for export button
    bl_options = {'PRESET', 'UNDO'}
    filename_ext = ".xml"

    filter_glob = StringProperty(
        default="*.xml",
        options={'HIDDEN'},
    )

    only_selected = BoolProperty(
        name="Selection Only",
        description="Export selected objects only",
        default=False
    )

    def execute(self, context):
        dir_path = os.path.dirname(self.filepath)
        file_name = os.path.basename(self.filepath)

        if self.only_selected:
            selected_objects = bpy.context.selected_objects
            if not selected_objects:
                # If no objects are selected, show an error message and return
                self.report({'ERROR'}, "No objects selected!")
                return {'CANCELLED'}
            # Export only selected objects
            objects_to_export = selected_objects
        else:
            # Export all objects
            objects_to_export = bpy.context.scene.objects

        fix_scene()

        write_mjcf(dir_path, file_name, objects_to_export, export=True)

        return {'FINISHED'}


def write_mjcf(dir_path, model_file_name, selected_objects, export=False):
    # Open the file for writing
    with open(os.path.join(dir_path, model_file_name), "w") as file:

        # Write the MJCF header
        file.write('<mujoco>\n')
        file.write('  <compiler angle="radian" meshdir="assets" autolimits="true" />\n')

        file.write('  <visual>\n')
        file.write('    <headlight diffuse="0.5 0.5 0.5" ambient="0.6 0.6 0.6" specular="0 0 0" />\n')
        file.write('    <rgba haze="0.15 0.25 0.35 1" />\n')
        file.write('    <global azimuth="120" elevation="-20" />\n')
        file.write('  </visual>\n')

        file.write('  <default>\n')
        file.write('    <default class="visual">\n')
        file.write('      <geom contype="0" conaffinity="0" group="2" />\n')
        file.write('    </default>\n')
        file.write('    <default class="collision">\n')
        file.write('      <geom group="3" />\n')
        file.write('      </default>\n')
        file.write('  </default>\n')

        file.write('  <asset>\n')
        file.write('    <texture type="skybox" file="assets/space.png" gridsize="3 4" gridlayout=".U..LFRB.D.."/>\n')
        file.write('    <texture type="skybox" file="assets/space.png" gridsize="3 4" gridlayout=".U..LFRB.D.."/>\n')
        file.write('  </asset>\n')

        file.write('  <worldbody>\n')

        materials = []
        materials_map = {}
        closed_list = set()
        for obj in selected_objects:
            if obj.active_material:
                mat = obj.active_material
                if mat.name in closed_list:
                    materials_map[obj.name] = mat.name
                    continue

                if mat and "Principled BSDF" in mat.node_tree.nodes:
                    closed_list.add(mat.name)

                    node = mat.node_tree.nodes["Principled BSDF"]
                    texture = None
                    base_color = None
                    metallic = None
                    roughness = None
                    alpha = 1.0
                    for ind in range(len(node.inputs)):
                        input = node.inputs[ind]
                        if input.name == "Base Color":
                            base_color = input.default_value
                        elif input.name == "Metallic":
                            metallic = input.default_value
                        elif input.name == "Roughness":
                            roughness = input.default_value
                        elif input.name == "Alpha":
                            alpha = input.default_value

                    if "Image Texture" in mat.node_tree.nodes:
                        # new_mat = f'<material name="{mat.name}" rgba="{base_color[0]} {base_color[1]} {base_color[2]} 1.0" reflectance="{1.0-roughness}" metallic="{metallic}" />'
                        new_texture = f'<texture type="2d" name="{mat.name}" file="assets/{mat.name}.png" mark="edge" width="2048" height="2048" />'
                        new_mat = f'<material name="{mat.name}" texture="{mat.name}" texuniform="true" texrepeat="1 1" reflectance="{1.0 - roughness}" metallic="{metallic}" />'
                        materials.append(new_texture)
                        materials.append(new_mat)
                    else:
                        new_mat = f'<material name="{mat.name}" rgba="{base_color[0]} {base_color[1]} {base_color[2]} {alpha}" reflectance="{1.0 - roughness}" metallic="{metallic}" />'
                        materials.append(new_mat)

                    materials_map[obj.name] = mat.name
                else:
                    print(f'Skipping material: {obj.active_material.name} since it is not Principled BSDF')

        mesh_file_names = []
        # Write the bodies recursively
        for obj in selected_objects:
            if obj.parent is None:
                mesh_file_names.extend(write_body(obj, file, 2, dir_path, materials_map, export))

        # Write the closing tags for the XML
        file.write('  </worldbody>\n')

        mesh_elements = [f'\n    <mesh name="{os.path.splitext(filename)[0]}" file="{filename}"/>' for filename in
                         mesh_file_names]
        mesh_string = "".join(mesh_elements)
        mesh_string += "\n".join([mat for mat in materials])
        asset_string = f"<asset>{mesh_string}\n  </asset>"
        file.write(asset_string)
        file.write('</mujoco>\n')


def write_body(obj, file, level, dir_path, materials_map, export=False):
    # Set the indentation for this level
    indent = "  " * level
    filepaths = []

    # Get the local position and rotation of the object, need to switch to quaternion mode to calculate elements
    pos = obj.location
    obj.rotation_mode = 'QUATERNION'

    # Blender uses local reference frames, but there's some trickery with parent inverses. This is why we called
    rot = obj.rotation_quaternion

    if site_criteria(obj):
        # Write the site element with the local position
        file.write(
            f'{indent}<site name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}" size="{obj.empty_display_size}" />\n')
    elif hinge_joint_criteria(obj):
        axis = rot @ Vector((0, 1, 0))
        file.write(
            f'{indent}<joint type="hinge" name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}" axis="{axis[0]} {axis[1]} {axis[2]}" />\n')
    elif slide_joint_criteria(obj):
        axis = rot @ Vector((0, 0, 1))
        file.write(
            f'{indent}<joint type="slide" name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}" axis="{axis[0]} {axis[1]} {axis[2]}" />\n')
    elif free_joint_criteria(obj):
        file.write(f'{indent}<joint type="free" name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}"  />\n')
    elif ball_joint_criteria(obj):
        file.write(f'{indent}<joint type="ball" name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}"  />\n')

    elif obj.type == "MESH":
        obj.data.update()
        # Write the body element with the local position and rotation
        file.write(
            f'{indent}<body name="{obj.name}" pos="{pos[0]} {pos[1]} {pos[2]}" quat="{rot[0]} {rot[1]} {rot[2]} {rot[3]}">\n')
        mesh_dir = os.path.join(dir_path, 'mesh')
        os.makedirs(mesh_dir, exist_ok=True)
        filepath = os.path.join(mesh_dir, f'{obj.name}.obj')

        if export:
            # Export the mesh to a separate STL file
            bpy.ops.object.select_all(action="DESELECT")
            obj.select_set(True)

            matrix_world = obj.matrix_world.copy()
            obj.matrix_world = mathutils.Matrix.Identity(4)
            obj.data.update()
            bpy.ops.wm.obj_export(filepath=filepath, apply_modifiers=True, export_selected_objects=True,
                                  global_scale=1.0, forward_axis='Y', up_axis='Z')
            obj.matrix_world = matrix_world
            obj.data.update()

        # Write the geom element for the object
        mat_str = f'material="{materials_map[obj.name]}"' if obj.name in materials_map else ""
        collision_str = 'collision' if 'collision' in obj.name else 'visual'
        file.write(
            f'{indent}  <geom class="{collision_str}" type="mesh" name="{obj.name} geom" mesh="{obj.name}" {mat_str} />\n')
        filepaths.append(os.path.basename(filepath))

    child_file_paths = []
    # Recursively write the children
    for child in obj.children:
        child_file_paths.extend(write_body(child, file, level + 1, dir_path, materials_map, export))

    if obj.type == "MESH":
        file.write(f'{indent}</body>\n')

    return [*filepaths, *child_file_paths]


def fix_scene():
    # Iterate through all objects in the scene
    for obj in bpy.context.scene.objects:
        # Set the rotation mode to quaternion
        obj.rotation_mode = 'QUATERNION'
        if obj.type == "EMPTY" and "site" in obj.name:
            obj.empty_display_size *= sum(obj.scale * obj.delta_scale) / 3
            obj.scale = (1, 1, 1)
            obj.delta_scale = (1, 1, 1)
        obj.matrix_local = obj.matrix_parent_inverse @ obj.matrix_local
        obj.matrix_parent_inverse = Matrix.Identity(4)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(MuJoCoExportOperator.bl_idname, text="MJCF (.xml)")


def register():
    bpy.utils.register_class(MuJoCoExportOperator)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(MuJoCoExportOperator)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_import)


def site_criteria(obj):
    return obj.type == "EMPTY" and "site" in obj.name.lower()


def hinge_joint_criteria(obj):
    return obj.type == "EMPTY" and obj.empty_display_type == "CIRCLE" and "joint" in obj.name.lower()


def free_joint_criteria(obj):
    return obj.type == "EMPTY" and obj.empty_display_type == "PLAIN_AXES" and "joint" in obj.name.lower()


def ball_joint_criteria(obj):
    return obj.type == "EMPTY" and obj.empty_display_type == "SPHERE" and "joint" in obj.name.lower()


def slide_joint_criteria(obj):
    return obj.type == "EMPTY" and obj.empty_display_type == "SINGLE_ARROW" and "joint" in obj.name.lower()


if __name__ == "__main__":
    register()
