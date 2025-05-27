# PhysX 4.1 C++ Wrapper

PhysXWrapper is a C API wrapper for NVIDIA PhysX SDK 4.1, built as a shared library (DLL/SO)

## Table of Contents

* [Features](#features)
* [Limitations](#limitations)
* [Prerequisites](#prerequisites)
* [Project Structure](#project-structure)
* [Installation & Build](#installation--build)

    * [Linux](#linux)
    * [Windows](#windows)
* [License](#license)

---

## Features
* Management of Foundation, Physics, and Cooking objects
* Mesh cooking (triangle & convex) and creation of mesh objects
* Scene creation with configurable gravity and multithreading
* Scene simulation, result fetching, and query update flushing
* Creation and release of rigid static and dynamic actors
* Creation and release of basic shapes: box, sphere, capsule, triangle mesh, convex mesh
* Material creation and release
* Overlap, sweep, and raycast queries (both filtered and unfiltered)
* Penetration computation between capsules
* Retrieval and setting of actor global poses
* Setting kinematic targets on dynamic actors
* Setting rigid body flags and shape flags
* Defining layer-based collision filtering and custom filter shader
* Setting filter layer masks and per-shape filter data (sim/query)
* Registration of collision (enter/stay/exit) and trigger callbacks
* Creation and management of CPU dispatchers and simulation event callbacks
* Settings & retrieval of linear/angular velocity on dynamic actors
* Application of forces or impulses
* 
## Limitations

This wrapper does not currently support:

* Creation of joints or constraints (e.g., fixed, revolute, distance joints)
* Vehicle, cloth, or soft-body modules
* Character controllers or kinematic controllers beyond basic kinematic target setting
* Continuous collision detection (CCD) settings
* Articulation or skeleton-based rigid bodies
* Enumeration of materials beyond manual tracking

---

## Prerequisites

* **CMake** 3.15 or later
* **C++20** compatible compiler

    * Linux: Clang 18+ (GCC support untested)
    * Windows: MSVC (tested with Visual Studio 2019+ Build Tools)
* **PhysX SDK 4.1** (included in `third_party/PhysX`)

---

## Project Structure

```
├── CMakeLists.txt              # Root build configuration
├── build-linux-release.sh      # Automate Linux release builds
├── include/PhysXWrapper/       # Public headers for the wrapper
├── src/                        # Wrapper source files
├── third_party/
│   └── PhysX/                  # NVIDIA PhysX SDK 4.1
│       ├── include/            # PhysX headers
│       ├── linux/              # Pre-built .so/.a libraries
│       └── win32/              # Pre-built .dll/.lib libraries
└── README.md                   # This document
```

---

## Installation & Build

### Linux

1. **Clone the repo**:

   ```bash
   git clone https://github.com/bigduncllc/physxwrapper.git
   cd physxwrapper
   ```
2. **Run build script**:

   ```bash
   chmod +x build-linux-release.sh
   ./build-linux-release.sh
   ```

   This creates `linux-release-build/` with the `.so` and headers.
3. **Integrate in your CMake project**:

   ```cmake
   add_subdirectory(linux-release-build)
   target_link_libraries(YourApp PRIVATE PhysXWrapper)
   ```

### Windows (CLion tested)

> **Note:** These steps assume you open the project directly in CLion, which handles CMake configuration.

1. **Clone the repo**:

   ```powershell
   git clone https://github.com/bigduncllc/physxwrapper.git
   cd physxwrapper
   ```
2. **Open in CLion**:

    * Launch CLion and select the cloned `physxwrapper` directory as your project.
    * CLion will automatically detect `CMakeLists.txt` and configure a **Release** profile.
3. **Build**:

    * In CLion, switch to the **Release** build profile.
    * Click **Build > Build Project** (or press Ctrl+F9).
4. **Artifacts**:

    * The built `PhysXWrapper.dll` and `PhysXWrapper.lib` will be available under your build directory (e.g. `cmake-build-release/`).

---

## License

This project is licensed under the GNU General Public License v3.0 (GPL‑3.0).

See [LICENSE](LICENSE) for full terms.
