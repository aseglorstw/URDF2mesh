from urdfpy import URDF
import numpy as np
import trimesh
import argparse
import os
import meshes_extracted


def main(arguments):
    robot_model = URDF.load(arguments.urdf_path)
    extracted_mesh = extract_mesh(robot_model)
    save_mesh(extracted_mesh, os.path.dirname(meshes_extracted.__file__))


def extract_mesh(robot_model):
    meshes = robot_model.visual_trimesh_fk()
    mesh_list = []
    for (mesh, matrix_of_rotation_and_translation) in meshes.items():
        translation_vector = matrix_of_rotation_and_translation[:3, 3][:, None]
        rotation_matrix = matrix_of_rotation_and_translation[:3, :3]
        vertices = np.array(mesh.vertices)
        vertices_base_link = (rotation_matrix @ vertices.T + translation_vector).T
        mesh_extracted = trimesh.Trimesh(vertices_base_link, mesh.faces)
        mesh_list.append(mesh_extracted)
    mesh_merged = trimesh.util.concatenate(mesh_list)
    return mesh_merged


def save_mesh(extracted_mesh, output_directory):
    path = os.path.join(output_directory, 'mesh_merged.obj')
    trimesh.exchange.export.export_mesh(extracted_mesh, path, file_type='obj')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf_path", default='', type=str, help="Path to the .urdf file")
    args = parser.parse_args()
    main(args)

