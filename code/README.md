# Recovered VEX V5 C++ project

This directory contains the original **`ragar` VEXcode project snapshot** supplied for the Team 2602E High Stakes portfolio on November 24, 2026.

- **Source:** the supplied `ragar.zip` archive (SHA-256: `41af2f4a45abd3c4576173f160045f64623e83ad537ebf39ddfb0b73e848cfc2`).
- **Preservation:** all 68 source/configuration files were copied into [`ragar/`](ragar/) with their original filenames, paths, and file contents. ZIP-generated `__MACOSX/` / AppleDouble metadata was excluded.
- **Status:** archived historical source. The code has **not been rebuilt or deployed** from this portfolio repository, and this snapshot has not been independently matched to a particular competition event's final firmware.

## Where to start

| Path | Content |
| --- | --- |
| [`ragar/ragar.v5code`](ragar/ragar.v5code) | Original VEXcode project manifest |
| [`ragar/src/main.cpp`](ragar/src/main.cpp) | Robot control, setup, competition routines, and integration |
| [`ragar/src/autos/`](ragar/src/autos/) | Autonomous routines, tests, callbacks, and point-based paths |
| [`ragar/src/DriveFuncs/`](ragar/src/DriveFuncs/) | PID, pure pursuit, and experimental PSO drive/turn tuning |
| [`ragar/src/Sensing/`](ragar/src/Sensing/) | Sensor integration and ring-color handling |
| [`ragar/src/nonDriveFuncs/`](ragar/src/nonDriveFuncs/) | Non-drive mechanism controls |
| [`ragar/include/`](ragar/include/) | Original headers, settings, and subsystem interfaces |
| [`ragar/makefile`](ragar/makefile) | Original VEXcode build entry point |

## Reproduction and provenance notes

The source references the VEX V5 C++ toolchain and hardware-specific SDK, neither of which is bundled with the ZIP. VEXcode configuration, ports, firmware/SDK versions, and robot hardware may need restoration to build or run this project. **No compilation, simulation, or physical-robot test result is claimed here.**

This is a **team engineering artifact** rather than individually attributed code. The archive contains `JAR-Template` and `lemLib`-named portions and other material that may incorporate external library or template work. Their inclusion should not be interpreted as claiming sole original authorship; verify original third-party provenance and applicable licenses before reuse.

The historical snapshot also preserves development comments, experimental routines, and inactive/commented code. Their presence does not prove they were executed on the competition robot. The [main portfolio README](../README.md) and the [engineering notebook](../docs/2602e-high-stakes-engineering-notebook.pdf) provide broader design and competition context.
