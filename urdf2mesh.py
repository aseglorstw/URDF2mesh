from urdfpy import URDF
import numpy as np
import trimesh
import argparse
import os
import meshes_extracted


def main(arguments):
    robot = URDF.load(arguments.urdf_path)

    meshes = robot.visual_trimesh_fk()

    output_dir = os.path.dirname(meshes_extracted.__file__)
    mesh_list = []

    for (mesh, matrix_of_rotation_and_translation) in meshes.items():

        translation_vector = matrix_of_rotation_and_translation[:3, 3][:, None]
        rotation_matrix = matrix_of_rotation_and_translation[:3, :3]

        vertices = np.array(mesh.vertices)
        vertices_base_link = (rotation_matrix @ vertices.T + translation_vector).T

        mesh_extracted = trimesh.Trimesh(vertices_base_link, mesh.faces)

        mesh_list.append(mesh_extracted)

    mesh_merged = trimesh.util.concatenate(mesh_list)

    path = os.path.join(output_dir, 'mesh_merged.obj')
    trimesh.exchange.export.export_mesh(mesh_merged, path, file_type='obj')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf_path", default='', type=str, help="Path to the .urdf file")
    args = parser.parse_args()
    main(args)
