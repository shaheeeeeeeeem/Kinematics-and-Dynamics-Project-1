# Kinematics and Dynamics of Machinery Project

This project is a Windows desktop mechanism simulator built with C++, SFML, ImGui, and ImGui-SFML. It contains interactive simulations for:

- A four-bar mechanism
- A slider-crank mechanism
- A launcher window that lets the user choose which simulator to open during local/source use

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

## How to Download and Run

This project is meant to be used through the prebuilt Windows zip, not by building the code manually.

### Recommended way to use the project

1. Open the GitHub repository.
2. Go to the `Releases` section.
3. Download `Mechanism-Simulator-Windows.zip`.
4. Extract the zip completely.
5. Open the extracted folder.
6. Run `fourbar.exe` or `slidercrank.exe` directly on your laptop.

The zip already includes:

- `fourbar.exe`
- `slidercrank.exe`
- Required DLL files
- The project PDF
- A short `README.txt`

This means users do not need to download the full source code or build the project themselves.

## Included PDF

The project report is stored here:

- [`docs/Kinematics___Dynamics_of_Machinery_Project.pdf`](docs/Kinematics___Dynamics_of_Machinery_Project.pdf)

## Release Package

The repository includes a helper script used to generate the Windows package:

- [`scripts/package_release.ps1`](scripts/package_release.ps1)

It creates:

- `release\Mechanism-Simulator-Windows.zip`

That zip is intended to be uploaded as a GitHub Release asset so users can download and run the application directly.

## GitHub Release Distribution

The recommended distribution method for this project is:

- Keep the repository for source code, README, PDF, and the packaging script
- Upload `release/Mechanism-Simulator-Windows.zip` to GitHub Releases

This keeps the main repository focused on the project files while still giving users a simple direct download for the runnable application.

If the zip needs to be rebuilt locally, run the helper script from PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\package_release.ps1
```

The package contains:

- `fourbar.exe`
- `slidercrank.exe`
- Required DLL files
- The project PDF
- A short `README.txt` for end users

## Notes

- The code is currently Windows-specific because it uses `ShellExecuteA` and a hardcoded Windows font path.
- The source project still includes `launcher.exe`, but the downloadable zip excludes it because users should run the simulators directly.
- If you share only the `.exe` files without the required DLLs, the application may fail to start on another machine.
