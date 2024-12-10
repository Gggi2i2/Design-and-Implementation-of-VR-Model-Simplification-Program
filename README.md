# G22 Project Report: Design and Implementation of VR Model Simplification Program

The Half-Edge Mesh Simplifier is a powerful 3D mesh processing tool designed to perform mesh simplification using the
Quadratic Meshing (QEM) method. And optimized based on the original algorithm. This tool utilizes half-edge data
structures to efficiently manage and manipulate 3D geometry, making it ideal for applications such as graphics
processing and computational geometry.

## Features

- **OBJ File Support**: Load and save meshes in the popular OBJ format, allowing for easy integration with various 3D
  graphics software.
- **Mesh Simplification**: Reduce the complexity of meshes while preserving the overall shape and appearance using QEM.
- **Boundary Preservation**: Special handling for boundary edges and vertices ensures the integrity of the mesh during
  simplification.
- **Efficient Data Structures**: Utilizes the half-edge data structure to optimize the manipulation and querying of mesh
  topology.
- **Error Checking and Verification**: Includes comprehensive error checking to ensure the consistency and correctness
  of the mesh data throughout processing steps.

## System Requirements

- C++17 or higher
- [GLM (OpenGL Mathematics)](https://github.com/g-truc/glm) library for vector and matrix operations.

## Installation

1. **Clone the Repository**
    - Unzip the compressed file.

2. **Install GLM**
    - Ensure that the GLM library is installed and correctly linked to your project. Refer to the GLM documentation for
      installation instructions.

3. **Compile the Project**
    - Use a C++ compiler supporting C++17. For example, with g++
    - Usually there are automatic methods in the IDE to help you build the project

## Usage

### Run the Simplifier

Build this project using the build options in the IDE then cd into the build directory. Then run the command below.

    ./MeshSimplifier path_to_input.obj simplification_ratio

- `path_to_input.obj`: Path to the .obj file you want to simplify.
- `simplification_ratio`: A floating-point number between 0 and 1 specifying the percentage of the mesh to retain.

### Example

```bash
./simplify ../resources/woman.obj 0.5
```

This command will simplify the `woman.obj` mesh, aiming to retain about 50% of the original faces.

## Project Report

[Design and Implementation of VR Model Simplification Program](./Report.pdf)

## Code Structure

### Key Classes

- **Vertex**: Represents a vertex in 3D space, along with its properties like position, associated edges, and error
  metrics.
- **Face**: Represents a polygonal face in the mesh.
- **Edge**: Represents an edge in the mesh, containing pointers to its half-edges.
- **HalfEdge**: Represents half of an edge in the mesh, which is fundamental in the half-edge structure, containing
  adjacency information.
- **Mesh**: A class that manages the entire mesh, including loading, saving, and simplifying the mesh.

### Simplification Process

The simplification process is handled by the "Mesh::simplify" method, which calculates the optimal shrinkage of the
edges based on QEM, curvature calculation, bcp calculation and automatic weighting based on the model size, and then
performs the shrinkage until the required number of faces is reached.

---
