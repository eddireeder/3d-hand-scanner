# 3D Hand Scanner

<p>
The aim of this project is to generate a 3D reconstructed model of a human hand, using just a 2D video orbiting the hand
as input.
</p>

## Getting Started

### Prerequisites

<p>
You are required to have installed:
</p>

<ul>
  <li>C/C++ Compiler</li>
  <li>CMake</li>
  <li>OpenMVG</li>
  <li>OpenMVS</li>
  <li>OpenPose</li>
</ul>

### Building

<p>
At the project root, copy the following files to your OpenPose source directory:
</p>

```
cp openpose_user_code/* path/to/openpose/root/examples/user_code
```

<p>
Recompile OpenPose to build the required binary, before building the executable with:
</p>

```
mkdir build
cmake -S . -B build
cd build
make
```

### Running

<p>
Run the executable with:
</p>

```
./reconstruct /global/path/to/images /global/path/for/output
```
