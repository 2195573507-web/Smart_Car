[CmdletBinding()]
param(
    [ValidateSet("StaticTf", "Rviz")]
    [string]$Mode = "StaticTf"
)

$ErrorActionPreference = "Stop"
$workspaceRoot = Split-Path -Parent $PSScriptRoot
$rvizConfigHostPath = Join-Path $workspaceRoot "config\rviz\radar_scan.rviz"
$rvizConfigContainerPath = "/ws/config/rviz/radar_scan.rviz"
$displayValue = "host.docker.internal:0.0"

$rows = docker ps --filter "status=running" --format "{{.ID}}|{{.Names}}"
$line = $rows | Where-Object { $_ -match "ros2-dev" } | Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($line)) {
    throw "No running ros2-dev container was found. Keep the bridge container running first."
}
$cid = ($line -split '\|')[0]

if ($Mode -eq "StaticTf") {
    Write-Host "Temporary TF for RViz display verification only: rviz_world -> laser_frame" -ForegroundColor Yellow
    Write-Host "This is not the production robot TF. Press Ctrl+C to stop." -ForegroundColor Yellow
    & docker exec --interactive $cid /entrypoint.sh bash -lc "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 rviz_world laser_frame"
    if ($LASTEXITCODE -ne 0) {
        throw "static_transform_publisher exited with code $LASTEXITCODE."
    }
    exit 0
}

if (-not (Test-Path -LiteralPath $rvizConfigHostPath)) {
    throw "RViz config was not found: $rvizConfigHostPath"
}

$vcxsrvPath = "C:\Program Files\VcXsrv\vcxsrv.exe"
if (-not (Test-Path -LiteralPath $vcxsrvPath)) {
    throw "VcXsrv was not found at $vcxsrvPath."
}
$xServer = Get-Process -Name vcxsrv -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $vcxsrvPath }
if (-not $xServer) {
    Start-Process -FilePath $vcxsrvPath `
        -ArgumentList ":0", "-multiwindow", "-ac", "-clipboard" `
        -WindowStyle Hidden
    Start-Sleep -Seconds 2
}

$env:DISPLAY = $displayValue
Write-Host "Starting RViz2 with config: $rvizConfigContainerPath" -ForegroundColor Cyan
Write-Host "Fixed Frame=rviz_world. Start the temporary TF in another terminal first." -ForegroundColor Yellow
& docker exec --interactive -e "DISPLAY=$displayValue" $cid /entrypoint.sh bash -lc "rviz2 -d $rvizConfigContainerPath"
if ($LASTEXITCODE -ne 0) {
    throw "RViz2 exited with code $LASTEXITCODE."
}
