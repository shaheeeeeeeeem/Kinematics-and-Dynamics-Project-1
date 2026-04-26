$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$releaseRoot = Join-Path $projectRoot "release"
$packageRoot = Join-Path $releaseRoot "Mechanism-Simulator-Windows"
$zipPath = Join-Path $releaseRoot "Mechanism-Simulator-Windows.zip"
$docsPdf = Join-Path $projectRoot "docs\Kinematics___Dynamics_of_Machinery_Project.pdf"

$requiredFiles = @(
    (Join-Path $projectRoot "build\launcher.exe"),
    (Join-Path $projectRoot "build\fourbar.exe"),
    (Join-Path $projectRoot "build\slidercrank.exe"),
    (Join-Path $projectRoot "libgcc_s_seh-1.dll"),
    (Join-Path $projectRoot "libsfml-graphics-3.dll"),
    (Join-Path $projectRoot "libsfml-system-3.dll"),
    (Join-Path $projectRoot "libsfml-window-3.dll"),
    (Join-Path $projectRoot "libstdc++-6.dll"),
    (Join-Path $projectRoot "libwinpthread-1.dll"),
    $docsPdf
)

foreach ($file in $requiredFiles) {
    if (-not (Test-Path $file)) {
        throw "Missing required file: $file"
    }
}

New-Item -ItemType Directory -Force -Path $releaseRoot | Out-Null
if (Test-Path $packageRoot) {
    Remove-Item -Recurse -Force -LiteralPath $packageRoot
}
New-Item -ItemType Directory -Force -Path $packageRoot | Out-Null

$copyList = @(
    (Join-Path $projectRoot "build\launcher.exe"),
    (Join-Path $projectRoot "build\fourbar.exe"),
    (Join-Path $projectRoot "build\slidercrank.exe"),
    (Join-Path $projectRoot "libgcc_s_seh-1.dll"),
    (Join-Path $projectRoot "libsfml-graphics-3.dll"),
    (Join-Path $projectRoot "libsfml-system-3.dll"),
    (Join-Path $projectRoot "libsfml-window-3.dll"),
    (Join-Path $projectRoot "libstdc++-6.dll"),
    (Join-Path $projectRoot "libwinpthread-1.dll"),
    $docsPdf
)

foreach ($file in $copyList) {
    Copy-Item -LiteralPath $file -Destination $packageRoot -Force
}

$readmePath = Join-Path $packageRoot "README.txt"
$readmeContent = @"
Mechanism Simulator - Windows Package

Files included:
- launcher.exe
- fourbar.exe
- slidercrank.exe
- required DLL files
- Kinematics___Dynamics_of_Machinery_Project.pdf

How to run:
1. Extract the zip fully.
2. Keep all files in the same folder.
3. Run launcher.exe.

If the launcher does not start one of the simulations, make sure
fourbar.exe and slidercrank.exe are still in the same folder as launcher.exe.
"@
Set-Content -LiteralPath $readmePath -Value $readmeContent -Encoding ASCII

if (Test-Path $zipPath) {
    Remove-Item -Force -LiteralPath $zipPath
}

Compress-Archive -Path (Join-Path $packageRoot "*") -DestinationPath $zipPath -Force
Write-Host "Created package: $zipPath"
