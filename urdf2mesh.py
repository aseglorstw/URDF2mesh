from urdfpy import URDF
import numpy as np
import trimesh
import argparse
import os
import meshes_extracted


def merge_meshes(mesh_list):
    vertices_list = [mesh.vertices for mesh in mesh_list]
    faces_list = [mesh.faces for mesh in mesh_list]
    faces_offset = np.cumsum([v.shape[0] for v in vertices_list], dtype=np.float32)
    faces_offset = np.insert(faces_offset, 0, 0)[:-1]
    vertices = np.vstack(vertices_list)
    faces = np.vstack([face + offset for face, offset in zip(faces_list, faces_offset)])
    mesh = trimesh.Trimesh(vertices, faces)
    return mesh


def main(arguments):
    print(arguments)
    robot = URDF.load(arguments.urdf_path)
    # The function visual_trimesh_fk() returns a dictionary where the key is a mesh and value is a matrix of
    # transformation of mesh relative base link frame.
    meshes = robot.visual_trimesh_fk()
    output_dir = os.path.dirname(meshes_extracted.__file__)
    mesh_list = []

    for idx, mesh in enumerate(meshes):
        matrix_of_rotation_and_translation = meshes[mesh]
        translation_vector = matrix_of_rotation_and_translation[:3, 3][:, None]
        rotation_matrix = matrix_of_rotation_and_translation[:3, :3]

        # So, we have now the mesh as "mesh" with the coordinates of vertices with respect to its frame. And we have a
        # matrix of transformation of each vertice of mesh relative to base link - "matrix_of_rotation_and_translation."

        vertices = np.array(mesh.vertices)
        vertices_base_link = (rotation_matrix @ vertices.transpose(1, 0) + translation_vector).T





        #
        # mesh_extracted = trimesh.Trimesh(verts_pose, mesh.faces)
        #
        # mesh_list.append(mesh_extracted)
        #
        # # Save mesh
        # if args.multiple_obj:
        #     path = os.path.join(output_dir, f'mesh_extracted_{idx}.obj')
        #     trimesh.exchange.export.export_mesh(mesh_extracted, path, file_type='obj')
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
