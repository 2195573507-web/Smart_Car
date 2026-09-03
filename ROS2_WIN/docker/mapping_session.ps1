[CmdletBinding()]
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet('Start', 'Save', 'Clear', 'Stop')]
  [string]$Action,
  [string]$WorkspaceRoot = '',
  [string]$ProjectName = 'srp_interleave_0831',
  [string]$MappingContainer = 'smartcar-mapping-session',
  [string]$SafeContainer = 'smartcar-mapping-safe',
  [switch]$StrictLiveGate,
  [string]$StatusFile = '',
  [string]$LogFile = ''
)

$ErrorActionPreference = 'Continue'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if ([string]::IsNullOrWhiteSpace($WorkspaceRoot)) {
  $WorkspaceRoot = Split-Path -Parent $scriptRoot
}
if ([string]::IsNullOrWhiteSpace($StatusFile)) {
  $StatusFile = Join-Path $scriptRoot 'mapping_console.status'
}
if ([string]::IsNullOrWhiteSpace($LogFile)) {
  $LogFile = Join-Path $scriptRoot 'mapping_console.log'
}
$composeFile = Join-Path $scriptRoot 'compose.yaml'
$mapsDir = Join-Path $WorkspaceRoot 'maps'

function Set-Status([string]$Message) {
  $record = "$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss') $Message"
  Set-Content -LiteralPath $StatusFile -Value $record -Encoding utf8
  Add-Content -LiteralPath $LogFile -Value $record -Encoding utf8
}

function Invoke-Docker([string[]]$Arguments) {
  $savedErrorActionPreference = $ErrorActionPreference
  $ErrorActionPreference = 'Continue'
  $output = @(& docker @Arguments 2>&1)
  $exitCode = $LASTEXITCODE
  $ErrorActionPreference = $savedErrorActionPreference
  foreach ($line in $output) {
    $text = [string]$line
    Add-Content -LiteralPath $LogFile -Value $text -Encoding ascii
  }
  if ($exitCode -ne 0) {
    throw "docker $($Arguments -join ' ') failed with exit code $exitCode"
  }
}

function Test-ContainerRunning([string]$Name) {
  # Do not inspect a named container that the recovery path may have already
  # removed. `docker ps` returns an empty successful result for that state.
  $savedErrorActionPreference = $ErrorActionPreference
  try {
    $ErrorActionPreference = 'Continue'
    $names = @(& docker ps --filter "name=^/$Name`$" --filter 'status=running' --format '{{.Names}}' 2>$null)
    return ($LASTEXITCODE -eq 0 -and ($names -contains $Name))
  } catch {
    return $false
  } finally {
    $ErrorActionPreference = $savedErrorActionPreference
  }
}

function Get-Port8765Containers {
  $ids = & docker ps --filter 'publish=8765' --format '{{.ID}}' 2>$null
  if ($LASTEXITCODE -ne 0) { throw 'Unable to inspect Docker port 8765 owners.' }
  return @($ids | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
}

function Stop-Port8765Containers {
  foreach ($id in Get-Port8765Containers) {
    Set-Status "Stopping TCP-8765 bridge $id"
    # Force-close only the current bridge owner so the S3 TCP client observes
    # a reset and can reconnect to the replacement listener promptly.
    Invoke-Docker @('kill', $id)
  }
}

function Remove-NamedContainer([string]$Name) {
  $id = [string](& docker ps -a --filter "name=^/$Name`$" --format '{{.ID}}' 2>$null)
  if (-not [string]::IsNullOrWhiteSpace($id)) {
    Invoke-Docker @('rm', '-f', $Name)
  }
}

function Ensure-Docker {
  Set-Status 'Checking Docker Desktop'
  Invoke-Docker @('version', '--format', '{{.Server.Version}} {{.Server.Os}}')
}

function Ensure-BuiltWorkspace {
  Set-Status 'Preparing the ROS mapping workspace'
  Invoke-Docker @('compose', '-p', $ProjectName, '-f', $composeFile, 'build')
  Invoke-Docker @(
    'compose', '-p', $ProjectName, '-f', $composeFile, 'run', '--rm', '--no-deps',
    'ros2-dev', 'bash', '-lc',
    'source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-force-configure --executor sequential'
  )
}

function Start-SafeBridge {
  Stop-Port8765Containers
  Remove-NamedContainer $SafeContainer
  Set-Status 'Restoring the false-gated bridge'
  $command = @'
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args --params-file /ws/src/s3_ydlidar_bridge/config/bridge.yaml -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_allowed_flags_mask:=1 -p allow_live_telemetry:=false -p enable_live_odom:=false -p publish_odom:=false -p publish_tf:=false
'@
  Invoke-Docker @(
    'compose', '-p', $ProjectName, '-f', $composeFile, 'run', '-d', '--service-ports',
    '--name', $SafeContainer, 'ros2-dev', 'bash', '-lc', $command.Trim()
  )
  Start-Sleep -Seconds 3
  if (-not (Test-ContainerRunning $SafeContainer)) {
    throw 'The safe bridge did not remain running.'
  }
}

function Start-Mapping {
  param([switch]$WorkspaceReady)

  Ensure-Docker
  if (-not $WorkspaceReady) { Ensure-BuiltWorkspace }
  try {
    Stop-Port8765Containers
    Remove-NamedContainer $MappingContainer
  Set-Status 'Starting robot description, SLAM, and RViz'
  $command = @'
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec ros2 launch smartcar_bringup p1_mapping.launch.py use_rviz:=true transport:=tcp allow_live_telemetry:=true enable_live_odom:=true publish_odom:=true publish_tf:=true telemetry_expected_source_id:=1 telemetry_expected_destination_id:=2
'@
    Invoke-Docker @(
    'compose', '-p', $ProjectName, '-f', $composeFile, 'run', '-d', '--service-ports',
    '--name', $MappingContainer, '-e', 'DISPLAY=host.docker.internal:0.0',
    'ros2-dev', 'bash', '-lc', $command.Trim()
  )

    $deadline = (Get-Date).AddSeconds(60)
    $attempt = 0
    while ((Get-Date) -lt $deadline) {
    $attempt++
    Start-Sleep -Seconds 2
    if (-not (Test-ContainerRunning $MappingContainer)) { break }
    $nodes = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 node list' 2>$null
    if ($nodes -match '/s3_ydlidar_bridge' -and
        $nodes -match '/robot_state_publisher' -and
        $nodes -match '/slam_toolbox') {
      $diag = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 3s ros2 topic echo --once /diagnostics' 2>$null
      if ($diag -match 'message:\s+connected' -and $diag -match 'stale.*false') {
        $scan = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 3s ros2 topic echo --once /scan' 2>$null
        $odom = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 3s ros2 topic echo --once /odom' 2>$null
        $tf = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 3s ros2 run tf2_ros tf2_echo odom base_link' 2>$null
        if ($scan -match 'frame_id:\s+laser_frame' -and
            $odom -match 'child_frame_id:\s+base_link' -and
            $tf -match 'At time') {
          Set-Status 'Mapping session is running with live scan, odometry, and TF'
          return
        }
      }
      Set-Status "Waiting for live S3 data (attempt $attempt; 60-second gate)"
    }
  }

    if (-not (Test-ContainerRunning $MappingContainer)) {
      throw 'The mapping container exited before the live gate completed.'
    }
    $warning = -join (0x5B9E, 0x65F6, 0x91CC, 0x7A0B, 0x8BA1, 0x2F, 0x54, 0x46, 0x672A, 0x5C31, 0x7EEA, 0xFF0C, 0x6682, 0x4E0D, 0x80FD, 0x5EFA, 0x56FE | ForEach-Object { [char]$_ })
    Set-Status $warning
    Add-Content -LiteralPath $LogFile -Value ("WARNING: $warning; RViz, SLAM, scan bridge, and robot description remain running.") -Encoding utf8
    if ($StrictLiveGate) {
      & docker logs --tail 100 $MappingContainer 2>&1 | ForEach-Object {
        Add-Content -LiteralPath $LogFile -Value ([string]$_) -Encoding utf8
        Write-Output $_
      }
      throw 'The mapping session did not receive live connected diagnostics, /scan, and /odom within 60 seconds.'
    }
    return
  }
  catch {
    try {
      Stop-Port8765Containers
      Remove-NamedContainer $MappingContainer
      Start-SafeBridge
    } catch {
      Add-Content -LiteralPath $LogFile -Value "Safe recovery failed: $($_.Exception.Message)" -Encoding ascii
    }
    throw
  }
}

function Save-Map {
  if (-not (Test-ContainerRunning $MappingContainer)) {
    throw 'Mapping is not running.'
  }
  $prefix = "/ws/maps/mapping_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
  Set-Status 'Saving the current map'
  Invoke-Docker @(
    'exec', $MappingContainer, 'bash', '-lc',
    "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/save_p1_map.sh $prefix"
  )
  Set-Status "Saved map $prefix"
}

function Move-MapArtifactsToRecycleBin {
  New-Item -ItemType Directory -Path $mapsDir -Force | Out-Null
  Add-Type -AssemblyName Microsoft.VisualBasic
  $files = Get-ChildItem -LiteralPath $mapsDir -File -Recurse |
    Where-Object { $_.Extension -in '.pgm', '.yaml', '.posegraph', '.data' }
  foreach ($file in $files) {
    Set-Status "Moving map artifact to Recycle Bin: $($file.Name)"
    [Microsoft.VisualBasic.FileIO.FileSystem]::DeleteFile(
      $file.FullName,
      [Microsoft.VisualBasic.FileIO.UIOption]::OnlyErrorDialogs,
      [Microsoft.VisualBasic.FileIO.RecycleOption]::SendToRecycleBin
    )
  }
  return $files.Count
}

try {
  New-Item -ItemType Directory -Path (Split-Path -Parent $StatusFile) -Force | Out-Null
  Set-Content -LiteralPath $LogFile -Value '' -Encoding ascii
  switch ($Action) {
    'Start' { Start-Mapping }
    'Save' { Save-Map }
    'Clear' {
      Ensure-Docker
      Stop-Port8765Containers
      Remove-NamedContainer $MappingContainer
      Start-SafeBridge
      $count = Move-MapArtifactsToRecycleBin
      Set-Status "Cleared $count map artifacts; starting a fresh SLAM session"
      Start-Mapping
    }
    'Stop' {
      Ensure-Docker
      Stop-Port8765Containers
      Remove-NamedContainer $MappingContainer
      Start-SafeBridge
      Set-Status 'Safe bridge is running with telemetry, odom, and TF disabled'
    }
  }
  exit 0
}
catch {
  Set-Status "FAILED: $($_.Exception.Message)"
  exit 1
}
