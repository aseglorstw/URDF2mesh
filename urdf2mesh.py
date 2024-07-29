from urdfpy import URDF
import numpy as np
import trimesh
import argparse


def merge_meshes(mesh_list):

    verts_list = [mesh.vertices for mesh in mesh_list]
    faces_list = [mesh.faces for mesh in mesh_list]

    faces_offset = np.cumsum([v.shape[0] for v in verts_list], dtype=np.float32)

    faces_offset = np.insert(faces_offset, 0, 0)[:-1]

    verts = np.vstack(verts_list)
    faces = np.vstack([face + offset for face, offset in zip(faces_list, faces_offset)])

    mesh = trimesh.Trimesh(verts, faces)

    return mesh


def main(args):
    print(args)
    robot = URDF.load(args.urdf_path)

    #meshes = robot.visual_trimesh_fk()
    # output_dir = os.path.dirname(meshes_extracted.__file__)
    # mesh_list = []
    # for idx, mesh in enumerate(meshes):
    #
    #     pose = meshes[mesh]  # 4 x 4 : rotation + translation
    #
    #     translation = pose[:3, 3][:, None]
    #
    #     # Add a column of zeros to the vertices
    #     verts = np.array(mesh.vertices)
    #     zeros = np.zeros((verts.shape[0], 1))
    #     new_verts = np.hstack((verts, zeros))
    #
    #     # Apply pose to the vertices
    #     verts_pose = pose @ new_verts.transpose(1, 0)
    #     verts_pose = verts_pose[:3, :] + translation
    #     verts_pose = verts_pose.transpose(1, 0)
    #
    #     mesh_extracted = trimesh.Trimesh(verts_pose, mesh.faces)
    #
    #     mesh_list.append(mesh_extracted)
    #
    #     # Save mesh
    #     if args.multiple_obj:
    #         path = os.path.join(output_dir, f'mesh_extracted_{idx}.obj')
    #         trimesh.exchange.export.export_mesh(mesh_extracted, path, file_type='obj')
    #
    #     # Merge meshes
    # mesh_merged = merge_meshes(mesh_list)
    #
    # # Save merged mesh
    # path = os.path.join(output_dir, 'mesh_merged.obj')
    # trimesh.exchange.export.export_mesh(mesh_merged, path, file_type='obj')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf_path", default='', type=str, help="Path to the .urdf file")
    parser.add_argument("--multiple_obj", default=False, action='store_true', help="Export multiple .obj files")
    args = parser.parse_args()
    main(args)
