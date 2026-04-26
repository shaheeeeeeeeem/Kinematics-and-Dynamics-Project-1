$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$releaseRoot = Join-Path $projectRoot "release"
$packageRoot = Join-Path $releaseRoot "Mechanism-Simulator-Windows"
$zipPath = Join-Path $releaseRoot "Mechanism-Simulator-Windows.zip"
$docsPdf = Join-Path $projectRoot "docs\Kinematics___Dynamics_of_Machinery_Project.pdf"
$dllFiles = Get-ChildItem -LiteralPath $projectRoot -Filter *.dll -File | Sort-Object Name
$optionalFiles = @(
    (Join-Path $projectRoot "imgui.ini")
) | Where-Object { Test-Path $_ }

$requiredFiles = @(
    (Join-Path $projectRoot "build\fourbar.exe"),
    (Join-Path $projectRoot "build\slidercrank.exe"),
    $docsPdf
)

if ($dllFiles.Count -eq 0) {
    throw "No DLL files were found in the project root: $projectRoot"
}

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
    (Join-Path $projectRoot "build\fourbar.exe"),
    (Join-Path $projectRoot "build\slidercrank.exe"),
    $docsPdf
)

$copyList += $dllFiles.FullName
$copyList += $optionalFiles

foreach ($file in $copyList) {
    Copy-Item -LiteralPath $file -Destination $packageRoot -Force
}

$readmePath = Join-Path $packageRoot "README.txt"
$readmeContent = @"
Mechanism Simulator - Windows Package

Files included:
- fourbar.exe
- slidercrank.exe
- required DLL files
- Kinematics___Dynamics_of_Machinery_Project.pdf

How to run:
1. Extract the zip fully.
2. Keep all files in the same folder.
3. Run fourbar.exe or slidercrank.exe directly.
"@
Set-Content -LiteralPath $readmePath -Value $readmeContent -Encoding ASCII

if (Test-Path $zipPath) {
    Remove-Item -Force -LiteralPath $zipPath
}

Compress-Archive -Path (Join-Path $packageRoot "*") -DestinationPath $zipPath -Force
Write-Host "Created package: $zipPath"
