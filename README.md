# PhiveConverter

A command line tool for converting between Splatoon 3 Phive collision files (`.bphsh`) and (`.obj`) meshes + (`.json`) for material info, reimplemented from ingame havok conversion functions. Extracted collision meshes can be edited in any 3D modeling software and rebuilt back into the binary format.

## Prerequisites

- A `fxconfig.json` file needs to be present in the working directory when running PhiveConverter. This file defines the material name table, material flag names, and collision disable flag names used by the game. A config for Splatoon 3 is included in the project root folder.

## Usage

PhiveConverter supports two operations: extracting a `.bphsh` to OBJ, and rebuilding an OBJ back into `.bphsh`.

### BPHSH to OBJ

Extracts the collision mesh and material info from a `.bphsh` file, producing a `.obj` and a `.json`.

```
PhiveConverter.exe Collision.bphsh
```

Outputs `Collision.obj` and `Collision.json` in the same directory as the input file.

To specify an output directory:

```
PhiveConverter.exe -p Collision.bphsh -o output_folder
```

### OBJ to BPHSH

Rebuilds a `.bphsh` from an OBJ mesh and an optional material info JSON.

With material info:

```
PhiveConverter.exe Model.obj Materials.json
```

The argument order does not matter -- PhiveConverter detects the `.obj` and `.json` by extension. Outputs `Model.bphsh` alongside the input OBJ.

Without material info (all faces get a default material):

```
PhiveConverter.exe Model.obj
```

To specify an explicit output path:

```
PhiveConverter.exe -obj Model.obj -mat Materials.json -o output/Rebuilt.bphsh
```

### Help

```
PhiveConverter.exe -h
```

## Output Files

When converting **BPHSH to OBJ**, two files are produced:

| File | Description |
|------|-------------|
| `<name>.obj` | OBJ mesh with vertices, faces, and `usemtl` groups for each material. |
| `<name>.json` | Material info JSON describing the material type, flags, and collision disable flags for each `usemtl` group. |

## Material Info JSON

Each key in the material info JSON corresponds to a `usemtl` group name in the OBJ. The value is an object with three fields:

```json
{
    "Stone00": {
        "mat_name": "Stone",
        "mat_flags": [
            "ForceColPaintPaintable"
        ],
        "col_disable_flags": [
            "NoHit",
            "Unspecified",
            "Ground"
        ]
    }
}
```

| Field | Description |
|-------|-------------|
| `mat_name` | Surface material type. Must be a name from the `mat_names` array in `fxconfig.json` (e.g. `Stone`, `Metal`, `Wood`, `Fence`, `Glass`). |
| `mat_flags` | Array of behavior flags applied to this material. Names come from `mat_flag_names` in `fxconfig.json` (e.g. `Slide`, `Fence`, `Ice`, `PlayerDead`, `ForceColPaintNotPaintable`). |
| `col_disable_flags` | Array of collision layers this material *disables*. Names come from `col_disable_flag_names` in `fxconfig.json` (e.g. `NoHit`, `SplCamera`, `SplInkBullet`). Layers not listed here remain enabled. |

All three fields are optional. Omitting `mat_name` defaults the material ID to 0 (`Undefined`). Omitting `mat_flags` or `col_disable_flags` leaves those bitmasks at their defaults.

## Config File (`fxconfig.json`)

| Key | Purpose |
|-----|---------|
| `mat_names` | Ordered list of surface material type names. Index 0 = `Undefined`, 1 = `Character`, etc. |
| `mat_flag_names` | Ordered list of material behavior flag names. Each entry corresponds to a bit position in the 64 bit flags field. |
| `col_disable_flag_names` | Ordered list of collision disable flag names. Each entry corresponds to a bit position in the 64 bit collision flags field. |

These can be extracted from the game's phive config and exefs, provided with the project is the file for Splatoon 3.