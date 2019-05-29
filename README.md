# 3D Hand Scanner

<p>
The aim of this project is to generate a 3D reconstructed model of a human hand, using just an image dataset of the hand
as input. A lot of the experimenting that is referred to in the report is now commented out, so that the system can compile.
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
Recompile OpenPose to build the required binary, before returning to the project root and building the executable with:
</p>

```
mkdir build
cmake -S . -B build
cd build
make
```

### Running

<p>
If attempting to use any of the functionality from the OpenPose library, set the following environment variable with the OpenPose project root:
</p>

```
export OPENPOSE_ROOT = /path/to/openpose/root
```

<p>
Run the executable with:
</p>

```
./reconstruct --input /global/path/to/images --output /global/path/for/output
```

<p>
You can optionally specify the focal length in pixels for your dataset
</p>

```
./reconstruct --input /global/path/to/images --output /global/path/for/output --focal 1234
```