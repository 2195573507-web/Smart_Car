$ErrorActionPreference = "Stop"

$composeDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$vcxsrvPath = "C:\Program Files\VcXsrv\vcxsrv.exe"

try {
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

    $env:DISPLAY = "host.docker.internal:0.0"
    Set-Location -LiteralPath $composeDir
    docker compose run --rm -e "DISPLAY=$env:DISPLAY" ros2-dev rviz2 `
        -d /ws/install/s3_ydlidar_bridge/share/s3_ydlidar_bridge/rviz/s3_ydlidar_bridge.rviz
    if ($LASTEXITCODE -ne 0) {
        throw "RViz2 exited with code $LASTEXITCODE."
    }
}
catch {
    Write-Host "Unable to start Smart Car RViz2:`n$($_.Exception.Message)" -ForegroundColor Red
    Read-Host "Press Enter to close"
    exit 1
}
