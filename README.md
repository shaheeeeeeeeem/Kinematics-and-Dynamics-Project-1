# Kinematics and Dynamics of Machinery Project

This project is a Windows desktop mechanism simulator built with C++, SFML, ImGui, and ImGui-SFML. It contains interactive simulations for:

- A four-bar mechanism
- A slider-crank mechanism
- A launcher window that lets the user choose which simulator to open

The repository also includes the project report PDF and a packaged Windows zip for people who only want to run the executables.

## Repository Contents

- `src/launcher_main.cpp`: Small launcher window with two buttons. It opens `fourbar.exe` or `slidercrank.exe` using `ShellExecuteA`.
- `src/fourbar_main.cpp`: Main four-bar simulator. It solves position, angular velocity, and angular acceleration for a configurable four-bar linkage.
- `src/slidercrank_main.cpp`: Main slider-crank simulator. It solves position, velocity, and acceleration for a crank-slider mechanism and supports multiple inversions.
- `external/imgui/`: Dear ImGui source files used to build the interface.
- `external/imgui-sfml/`: ImGui-SFML bridge used to render ImGui inside SFML windows.
- `build/`: Built Windows executables.
- `docs/`: Project PDF/report included in the repository.
- `release/`: Packaged Windows zip and helper files for distribution.

## What the Code Does

### Launcher

The launcher creates a simple SFML window with two clickable buttons:

- `Four Bar Mechanism`
- `Slider Crank Mechanism`

When the user clicks one of them, the launcher opens the matching executable and closes itself. This is meant to give a simple entry point for non-technical users.

### Four-Bar Simulator

The four-bar program lets the user define four link lengths and assign each link a role:

- Ground link
- Input link
- Coupler link
- Output link

It then performs:

- Position analysis for link angles
- Velocity analysis for angular velocities
- Acceleration analysis for angular accelerations
- Grashof classification
- Mechanism type classification
- Branch selection for alternate assembly configurations
- Simulation over time with reversal handling for non-Grashof and special Grashof cases
- Coupler-point path generation
- Plot windows for angle, velocity, and acceleration data
- Velocity and acceleration vector diagrams

Key features in the four-bar implementation:

- The code computes whether the mechanism satisfies Grashof conditions.
- It solves linkage geometry using trigonometry and continuity-based branch tracking.
- It collects simulation samples over time to generate plots.
- It can display a coupler curve traced by a user-defined point on the coupler.
- It opens separate windows for graphs and vector diagrams so the main mechanism view stays interactive.

### Slider-Crank Simulator

The slider-crank program models a crank-slider mechanism using a crank length `a` and connecting rod length `b`.

It performs:

- Position analysis
- Velocity analysis
- Acceleration analysis
- Real-time simulation
- Four inversion views of the same mechanism family
- Separate velocity and acceleration diagram windows

The supported inversions shown in the interface are:

- Inversion 1: Frame fixed
- Inversion 2: Crank fixed
- Inversion 3: Coupler fixed
- Inversion 4: Slider fixed

The program continuously updates mechanism geometry, calculates derived quantities, and renders both the physical mechanism and analytical results in the same session.

## Build Requirements

This project is currently set up for Windows and expects:

- MinGW g++ at `C:/msys64/mingw64/bin/g++.exe`
- SFML libraries available through the MinGW toolchain path
- Windows font file `C:/Windows/Fonts/arial.ttf`

The included VS Code tasks compile the project directly with `g++`.

## How to Build

### Option 1: VS Code Tasks

Use the tasks already defined in [tasks.json](C:\Users\mdsha\Desktop\Kinematics and Dynamics Of Machinery Project\.vscode\tasks.json):

- `Build Fourbar`
- `Build SliderCrank`
- `Build Launcher`

### Option 2: PowerShell / Terminal

Build the four-bar executable:

```powershell
C:/msys64/mingw64/bin/g++.exe src/fourbar_main.cpp `
  external/imgui/imgui.cpp `
  external/imgui/imgui_draw.cpp `
  external/imgui/imgui_tables.cpp `
  external/imgui/imgui_widgets.cpp `
  external/imgui/imgui_demo.cpp `
  external/imgui-sfml/imgui-SFML.cpp `
  -Iexternal/imgui `
  -Iexternal/imgui-sfml `
  -o build/fourbar `
  -lsfml-graphics -lsfml-window -lsfml-system -lopengl32 -mwindows
```

Build the slider-crank executable:

```powershell
C:/msys64/mingw64/bin/g++.exe src/slidercrank_main.cpp `
  external/imgui/imgui.cpp `
  external/imgui/imgui_draw.cpp `
  external/imgui/imgui_tables.cpp `
  external/imgui/imgui_widgets.cpp `
  external/imgui/imgui_demo.cpp `
  external/imgui-sfml/imgui-SFML.cpp `
  -Iexternal/imgui `
  -Iexternal/imgui-sfml `
  -o build/slidercrank `
  -lsfml-graphics -lsfml-window -lsfml-system -lopengl32 -mwindows
```

Build the launcher:

```powershell
C:/msys64/mingw64/bin/g++.exe src/launcher_main.cpp `
  -o build/launcher `
  -lsfml-graphics -lsfml-window -lsfml-system -lopengl32 -mwindows
```

## How to Run

### Source build

After building, keep these files together:

- `build/launcher.exe`
- `build/fourbar.exe`
- `build/slidercrank.exe`
- Required SFML and MinGW runtime DLL files

Then run:

```powershell
.\build\launcher.exe
```

### Prebuilt zip

If you only want to run the project:

1. Download the zip from the `release/` folder or from a GitHub Release.
2. Extract it completely.
3. Open the extracted folder.
4. Run `launcher.exe`.

The zip also includes the project PDF so users can read the report without cloning the repository.

## Included PDF

The project report is stored here:

- [Project Report PDF](C:\Users\mdsha\Desktop\Kinematics and Dynamics Of Machinery Project\docs\Kinematics___Dynamics_of_Machinery_Project.pdf)

## Creating the Distribution Zip

The repository includes a helper script:

- [scripts/package_release.ps1](C:\Users\mdsha\Desktop\Kinematics and Dynamics Of Machinery Project\scripts\package_release.ps1)

Run it from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\package_release.ps1
```

It creates:

- `release\Mechanism-Simulator-Windows.zip`

That zip contains:

- `launcher.exe`
- `fourbar.exe`
- `slidercrank.exe`
- Required DLL files
- The project PDF
- A short `README.txt` for end users

## Best Way to Add the Zip to GitHub

You have two good options:

### Option 1: Keep the zip in the repository

Place the zip inside the `release/` folder and push it like any other file.

This is simple, but the repository becomes larger over time if you keep updating binaries.

### Option 2: Upload the zip to GitHub Releases

This is the recommended option.

Use the repository for:

- Source code
- README
- PDF
- Build script

Then upload `release/Mechanism-Simulator-Windows.zip` as a release asset on GitHub. That keeps the repo cleaner while still giving users a direct download.

## Notes

- The code is currently Windows-specific because it uses `ShellExecuteA` and a hardcoded Windows font path.
- The launcher expects `fourbar.exe` and `slidercrank.exe` to be in the same extracted folder at runtime.
- If you share only the `.exe` files without the required DLLs, the application may fail to start on another machine.
