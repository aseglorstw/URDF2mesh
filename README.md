# URDF2mesh
The idea for the project arose to improve the model for predicting the forces 
acting on the robot from the surface of the Earth. The plan involves using a 
more accurate model of the robot, represented as a mesh in the .obj file format.

<p align="center">
  <img src="https://github.com/ctu-vras/monoforce/raw/master/monoforce/docs/imgs/monoforce_mayavi.gif">
  <br><sub>Visualization of the predictive model for forces using a simplified representation of the robot.</sub>  
</p>

Thus, this repository contains a script that **extracts meshes from the visual parts of links in a URDF file and merges
them into a single mesh .obj model**.

# URDF 
The Unified Robot Description Format (URDF) is an XML specification to describe 
a robot. URDF (Unified Robot Description Format) files are commonly used in ROS 
(Robot Operating System) to describe the structure and properties of robots. 
Here’s a simple example to illustrate how a URDF file is structured:

```chatinput
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```
Visualization of this file is following:

<p align="center">
  <img src="https://raw.githubusercontent.com/ros/urdf_tutorial/master/images/myfirst.png">
</p>

This description can also use mesh files. An example is shown below:

```
<link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>
 ```
The meshes can be imported in a number of different formats. STL is fairly 
common, but the engine also supports DAE, which can have its own color data, 
meaning you don’t have to specify the color/ material. 

The path to the mesh file can also be specified as an absolute path, in addition 
to using relative paths. In this case it will be:

```chatinput
<mesh filename="/absolute/path/to/mesh/file"/>
```

# Mesh .obj 
An .obj file is a text-based format used for storing 3D models. It contains 
information about the vertices, faces, texture coordinates, and normals of a 
model, which allows it to describe its geometry and structure. Example of an 
.obj file:
```chatinput
v 0.0 0.0 0.0  # Vertex 1
v 1.0 0.0 0.0  # Vertex 2
v 1.0 1.0 0.0  # Vertex 3
v 0.0 1.0 0.0  # Vertex 4

f 1 2 3 
f 1 3 4  
```
Here v: defines a vertex of the model. Each v line specifies the coordinates 
of a vertex in 3D space (X, Y, Z). And f: defines a face of the model. Each f line lists the indices of vertices that make up a face. In this example, f 1 2 3 creates a triangle using vertices 1, 2, and 3.

# Requirements 
To work correctly you will need:
1. ``python 3.8.+``
2. ``numpy 1.22.3``
3. ``urdfpy 0.0.22``
4. ``trimesh 3.16.4``

If you have all the required libraries with the correct versions installed, 
you can clone this repository:

``git clone https://github.com/aseglorstw/URDF2mesh.git``

# Usage
1. Prepare your URDF file. Ensure that all paths to mesh files in the URDF are specified as absolute paths.
2. Execute the script using the following command:  `` python3 urdf2mesh.py --urdf_path /path/to/urdf/file ``.
3. The resulting .obj file will be saved in the "meshes_extracted" directory.

# Notes 
During the installation and usage process, you may encounter some problems:
1. ``ImportError: cannot import name 'gcd' from 'fractions' (/home/robert/miniconda3/envs/pytorch3d/lib/python3.9/fractions.py)``

To resolve the issue, update the import statement as follows: 
``` 
from math import gcd 
```

2. ``if len(self._meshes) == 0: TypeError: object of type 'NoneType' has no len() ``

To resolve the issue, update the function 'meshes' as follows: 

```commandline
def meshes(self):
    """list of :class:`~trimesh.base.Trimesh` : The triangular meshes
    that represent this object.
    """
    if self._meshes is None:
        return []

    if len(self._meshes) == 0:
        self._meshes = [trimesh.creation.cylinder(
            radius=self.radius, height=self.length
        )]
    return self._mesh
```


# Example 
Below is the result of a script to translate the Husky robot model from URDF format to .obj format:
<p align="center">
  <img src="https://www.clearpathrobotics.com/assets/guides/kinetic/ros/_images/Huskyviz.png" width="34.4%">
  <img src="images/husky.png" width="30%">
</p>



# References 
1. https://github.com/ctu-vras/monoforce
2. https://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
3. https://github.com/maurock/urdf_to_obj
