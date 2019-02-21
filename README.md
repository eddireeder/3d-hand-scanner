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
  <li>OpenCV</li>
  <li>OpenMVG</li>
  <li>OpenMVS</li>
</ul>

### Building

<p>
At the project root, build the executable with:
</p>

```
cmake .
make
```

### Running

<p>
Run the executable with:
</p>

```
./reconstruct /path/to/video
```
