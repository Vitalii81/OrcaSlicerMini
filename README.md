# OrcaSlicer CLI

This is a command-line interface (CLI) project based on the OrcaSlicer slicing engine. It performs slicing of 3D models (STL, OBJ, STEP, 3MF) into layers and exports the result in binary and visual formats (OBJ/MTL, optional PNG).

## Features

* Supports STL, OBJ, STEP, 3MF input formats
* Slices models using OrcaSlicer engine
* Exports slicing layers to:

  * Binary `.bin` files (one per layer)
  * `.obj` + `.mtl` files with color-coded layers *(for testing; disable in production)*
  * Optional: preview PNG *(for testing; currently commented out)*
* Parses and re-reads sliced binary data for verification
* Automatically aligns model to Z=0
* Simplified slicing configuration (no extrusion, no infill)

## Requirements

Ensure all required dependencies are built first:

```bash
./BuildLinux.sh -dbc   # Build deps in Debug mode
./BuildLinux.sh -dc    # Build deps in Release mode
```

On a fresh Linux installation, you may also need to install minimal system packages:
./install_min_sys_deps.sh

Then build the application:

```bash
./BuildLinux.sh -s     # Build the orca-slicer CLI
```

## Usage

Run with a model file and optional layer height:

```bash
./run.sh model.stl 0.2
```

Run built binary directly:

```bash
./bin/orca-slicer path/to/model.stl 0.2
```

Run built-in test with default cube:

```bash
./run.sh test
```

Show help:

```bash
./run.sh -h
```

## Output

* Creates a folder `<model_name>/` next to input file
* Stores:

  * Layer binary files: `layer_0.bin`, `layer_1.bin`, ...
  * Colorized OBJ + MTL: `layers.obj`, `layers.mtl` *(test feature; disable in release)*
  * (Optional) PNG preview image: `layers.png` *(test feature; commented in code)*

## Notes

* Uses `stb_image_write` for PNG export (currently commented out)
* All output directories are created or cleaned automatically
* Model is recentred vertically to rest on the build plate

## Main Entry Point

See [`OrcaSlicer_cli.cpp`](./src/OrcaSlicer_cli.cpp) for full implementation.
