[CmdletBinding()]
param(
  [Parameter(Mandatory = $true)]
  [ValidateSet('Start', 'AutoStart', 'AutoStop', 'Save', 'Clear', 'Stop')]
  [string]$Action,
  [string]$WorkspaceRoot = '',
  [string]$ProjectName = 'docker',
  [string]$MappingContainer = 'smartcar-mapping-session',
  [string]$SafeContainer = 'smartcar-mapping-safe',
  [string]$MotionGatewayContainer = 'smartcar-motion-gateway',
  [switch]$StrictLiveGate,
  [switch]$LaserExtrinsicsMeasured,
  [string]$LaserXyz = '0.200 0.000 0.155',
  [string]$LaserRpy = '0.000 0.000 0.000',
  [string]$AutoSpeed = '0.05',
  [string]$RobotRadius = '0.0',
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

function Invoke-LoggedContainerScript([string]$Container, [string]$Script) {
  # PowerShell 5 can split multiline native-command arguments. Encode the
  # script so Docker receives one shell-safe argument without changing it.
  $payload = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Script))
  & docker exec $Container bash -lc "echo $payload | base64 -d | bash" 2>&1 |
    ForEach-Object { Add-Content -LiteralPath $LogFile -Value ([string]$_) -Encoding utf8 }
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
  if (Test-ContainerRunning $Name) {
    # Give slam_toolbox and rosbag2 time to flush their files before removal.
    Invoke-Docker @('stop', '-t', '10', $Name)
  }
  $id = [string](& docker ps -a --filter "name=^/$Name`$" --format '{{.ID}}' 2>$null)
  if (-not [string]::IsNullOrWhiteSpace($id)) {
    Invoke-Docker @('rm', $Name)
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

function Require-HealthyMotionGateway {
  $owners = @(& docker ps --filter 'publish=8766' --filter 'status=running' --format '{{.Names}}' 2>$null |
    Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
  if ($LASTEXITCODE -ne 0) {
    throw 'Unable to inspect the TCP-8766 motion gateway owner.'
  }
  if ($owners.Count -ne 1 -or $owners[0] -ne $MotionGatewayContainer) {
    $found = if ($owners.Count -eq 0) { 'none' } else { $owners -join ', ' }
    throw "The unique TCP-8766 motion gateway is unavailable (expected $MotionGatewayContainer; found $found). Start the existing gateway before mapping."
  }
  if (-not (Test-ContainerRunning $MotionGatewayContainer)) {
    throw "The unique TCP-8766 motion gateway $MotionGatewayContainer is not running. Start it before mapping."
  }

  $published = @(& docker port $MotionGatewayContainer 2>$null)
  if ($LASTEXITCODE -ne 0 -or -not ($published -match '^8766/tcp\s+->') -or ($published -match '^8765/tcp\s+->')) {
    throw "The unique TCP-8766 motion gateway $MotionGatewayContainer has an invalid host port mapping. Start it before mapping."
  }

  & docker exec $MotionGatewayContainer sh -c 'grep -q 00000000:223E /proc/net/tcp' 2>$null
  if ($LASTEXITCODE -ne 0) {
    throw "The unique TCP-8766 motion gateway $MotionGatewayContainer is not listening on its IPv4 socket. Start it before mapping."
  }
  Set-Status "Reusing the healthy unique TCP-8766 motion gateway $MotionGatewayContainer"
}

function Start-SafeBridge {
  Require-HealthyMotionGateway
  Stop-Port8765Containers
  Remove-NamedContainer $SafeContainer
  Set-Status 'Restoring the false-gated bridge'
  $command = @'
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec ros2 run s3_ydlidar_bridge s3_ydlidar_bridge_node --ros-args --params-file /ws/src/s3_ydlidar_bridge/config/bridge.yaml -p transport:=tcp -p tcp_listen_port:=8765 -p s3_expected_device_id:=1 -p s3_expected_stream_id:=1 -p s3_allowed_flags_mask:=1
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

function Assert-AutoExplorationSettings {
  if ($AutoSpeed -notmatch '^0\.0[2-8]$') {
    throw 'Automatic mapping speed must be from 0.02 through 0.08 m/s.'
  }
  $radius = 0.0
  if (-not [double]::TryParse($RobotRadius, [Globalization.NumberStyles]::Float,
      [Globalization.CultureInfo]::InvariantCulture, [ref]$radius) -or $radius -le 0.0) {
    throw 'Automatic mapping requires a positive measured robot radius in metres.'
  }
}

function Stop-AutoExploration {
  # A missing explorer must not prevent the direct unique-gateway disarm.
  if (Test-ContainerRunning $MappingContainer) {
    $stopExplorerCommand = @'
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
if timeout 5s ros2 service list | grep -Fxq /smartcar_auto_exploration/stop; then
  if ! timeout 5s ros2 service call /smartcar_auto_exploration/stop std_srvs/srv/Trigger "{}"; then
    echo "Automatic exploration stop did not complete within 5 seconds; continuing with gateway disarm."
  fi
else
  echo "Automatic exploration stop service is unavailable; continuing with gateway disarm."
fi
'@
    Invoke-LoggedContainerScript $MappingContainer $stopExplorerCommand
  }
  $disarmGatewayCommand = @'
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
if timeout 5s ros2 service list | grep -Fxq /smartcar_motion_gateway/set_motion_enabled; then
  if ! timeout 5s ros2 service call /smartcar_motion_gateway/set_motion_enabled std_srvs/srv/SetBool "{data: false}"; then
    echo "Motion gateway disarm did not complete within 5 seconds."
  fi
else
  echo "Motion gateway disarm service is unavailable."
fi
'@
  Invoke-LoggedContainerScript $MotionGatewayContainer $disarmGatewayCommand
}

function Start-AutoExploration {
  Ensure-Docker
  Require-HealthyMotionGateway
  Assert-AutoExplorationSettings
  if (-not (Test-ContainerRunning $MappingContainer)) {
    throw 'Automatic mapping requires a running mapping session with live scan, odometry, and TF.'
  }
  Set-Status 'Requesting automatic exploration preflight'
  $command = @"
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
ros2 param set /smartcar_auto_exploration max_linear_speed $AutoSpeed && \
ros2 param set /smartcar_auto_exploration robot_radius_m $RobotRadius && \
timeout 8s ros2 service call /smartcar_auto_exploration/start std_srvs/srv/Trigger "{}"
"@
  Invoke-Docker @('exec', $MappingContainer, 'bash', '-lc', $command.Trim())
  Set-Status "Automatic exploration preflight requested at $AutoSpeed m/s"
}

function Start-Mapping {
  param([switch]$WorkspaceReady)

  Ensure-Docker
  Require-HealthyMotionGateway
  if (-not $WorkspaceReady) { Ensure-BuiltWorkspace }
  try {
    Stop-AutoExploration
    Stop-Port8765Containers
    Remove-NamedContainer $MappingContainer
  Set-Status 'Starting robot description, SLAM, and RViz'
  $measured = if ($LaserExtrinsicsMeasured) { 'true' } else { 'false' }
  $bagUri = "/ws/bags/mapping_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
  $command = @"
source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && exec ros2 launch smartcar_bringup p1_mapping.launch.py use_rviz:=true transport:=tcp record_bag:=true bag_uri:='$bagUri' laser_extrinsics_measured:=$measured laser_xyz:='$LaserXyz' laser_rpy:='$LaserRpy' auto_exploration_speed:=$AutoSpeed robot_radius_m:=$RobotRadius
"@
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
      # Docker DDS discovery can take longer than the previous 3-second
      # process timeout even when the data stream is healthy.
      $gatewayDiag = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 8s ros2 topic echo --once /smartcar/motion_diagnostics' 2>$null
      $gatewayDiagText = $gatewayDiag -join "`n"
      $gatewayReady =
        $gatewayDiagText -match "key:\s+authentication_ready\s+value:\s+'true'" -and
        $gatewayDiagText -match "key:\s+protocol_ready\s+value:\s+'true'" -and
        $gatewayDiagText -match "key:\s+scan\s+value:\s+'true'" -and
        $gatewayDiagText -match "key:\s+odom\s+value:\s+'true'" -and
        $gatewayDiagText -match "key:\s+tf_odom_base_link\s+value:\s+'true'"
      if ($gatewayReady) {
        $scan = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 8s ros2 topic echo --once /scan' 2>$null
        $odom = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 8s ros2 topic echo --once /odom' 2>$null
        $tf = & docker exec $MappingContainer bash -lc 'source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && timeout 8s ros2 run tf2_ros tf2_echo odom base_link' 2>$null
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
  Set-Status 'Saving the current map and pose graph'
  Invoke-Docker @(
    'exec', $MappingContainer, 'bash', '-lc',
    "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/save_p1_map.sh $prefix"
  )
  Invoke-Docker @(
    'exec', $MappingContainer, 'bash', '-lc',
    "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/save_p1_posegraph.sh $prefix"
  )
  Set-Status "Saved map and pose graph $prefix; rosbag is finalized when mapping stops"
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
    'AutoStart' { Start-AutoExploration }
    'AutoStop' {
      Ensure-Docker
      Require-HealthyMotionGateway
      Stop-AutoExploration
      Set-Status 'Automatic exploration stopped; motion gateway disarmed'
    }
    'Save' { Save-Map }
    'Clear' {
      Ensure-Docker
      Require-HealthyMotionGateway
      Stop-AutoExploration
      Stop-Port8765Containers
      Remove-NamedContainer $MappingContainer
      Start-SafeBridge
      $count = Move-MapArtifactsToRecycleBin
      Set-Status "Cleared $count map artifacts; starting a fresh SLAM session"
      Start-Mapping
    }
    'Stop' {
      Ensure-Docker
      Require-HealthyMotionGateway
      Stop-AutoExploration
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
