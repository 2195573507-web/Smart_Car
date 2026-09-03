[CmdletBinding()]
param(
  [switch]$StrictLiveGate
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

$workspaceRoot = Split-Path -Parent $PSScriptRoot
$workerScript = Join-Path $PSScriptRoot 'mapping_session.ps1'
$statusFile = Join-Path $PSScriptRoot 'mapping_console.status'
$logFile = Join-Path $PSScriptRoot 'mapping_console.log'
$vcxsrvPath = 'C:\Program Files\VcXsrv\vcxsrv.exe'
$script:worker = $null
$script:closeAfterWorker = $false

function Get-PortOwners {
  $savedErrorActionPreference = $ErrorActionPreference
  try {
    $ErrorActionPreference = 'Continue'
    $names = @(& docker ps --filter 'publish=8765' --format '{{.Names}}' 2>$null)
    if ($LASTEXITCODE -ne 0) { return @() }
    return @($names | Where-Object { -not [string]::IsNullOrWhiteSpace($_) })
  } catch {
    return @()
  } finally {
    $ErrorActionPreference = $savedErrorActionPreference
  }
}

function Test-MappingRunning {
  $savedErrorActionPreference = $ErrorActionPreference
  try {
    # Query the running-container list instead of inspecting a name that the
    # worker deliberately removes after a failed live gate. Docker returns an
    # empty successful result when the mapping container is absent.
    $ErrorActionPreference = 'Continue'
    $names = @(& docker ps --filter 'name=^/smartcar-mapping-session$' --filter 'status=running' --format '{{.Names}}' 2>$null)
    return ($LASTEXITCODE -eq 0 -and ($names -contains 'smartcar-mapping-session'))
  } catch {
    return $false
  } finally {
    $ErrorActionPreference = $savedErrorActionPreference
  }
}

function Test-LiveMappingReady {
  if (-not (Test-MappingRunning)) { return $false }
  try {
    if (-not (Test-Path -LiteralPath $statusFile)) { return $false }
    $status = (Get-Content -LiteralPath $statusFile -Raw).Trim()
    return $status -match 'Mapping session is running with live scan, odometry, and TF$'
  } catch {
    return $false
  }
}

function Start-VcxSrv {
  if (-not (Test-Path -LiteralPath $vcxsrvPath)) {
    throw "VcXsrv was not found at $vcxsrvPath."
  }
  $server = Get-Process -Name vcxsrv -ErrorAction SilentlyContinue |
    Where-Object { $_.Path -eq $vcxsrvPath }
  if (-not $server) {
    Start-Process -FilePath $vcxsrvPath -ArgumentList ':0', '-multiwindow', '-ac', '-clipboard' -WindowStyle Hidden
    Start-Sleep -Seconds 2
  }
}

$form = New-Object System.Windows.Forms.Form
$form.Text = 'Smart Car Mapping'
$form.StartPosition = 'CenterScreen'
$form.ClientSize = New-Object System.Drawing.Size(560, 300)
$form.FormBorderStyle = 'FixedDialog'
$form.MaximizeBox = $false
$form.Font = New-Object System.Drawing.Font('Segoe UI', 10)

$title = New-Object System.Windows.Forms.Label
$title.Text = 'Smart Car Mapping'
$title.Font = New-Object System.Drawing.Font('Segoe UI Semibold', 18)
$title.Location = New-Object System.Drawing.Point(24, 20)
$title.AutoSize = $true
$form.Controls.Add($title)

$state = New-Object System.Windows.Forms.Label
$state.Text = 'Ready'
$state.Location = New-Object System.Drawing.Point(26, 58)
$state.Size = New-Object System.Drawing.Size(508, 24)
$form.Controls.Add($state)

$startButton = New-Object System.Windows.Forms.Button
$startButton.Text = 'Start Mapping'
$startButton.Location = New-Object System.Drawing.Point(24, 98)
$startButton.Size = New-Object System.Drawing.Size(244, 46)
$form.Controls.Add($startButton)

$saveButton = New-Object System.Windows.Forms.Button
$saveButton.Text = 'Save Map'
$saveButton.Location = New-Object System.Drawing.Point(292, 98)
$saveButton.Size = New-Object System.Drawing.Size(244, 46)
$form.Controls.Add($saveButton)

$clearButton = New-Object System.Windows.Forms.Button
$clearButton.Text = 'Clear Saved Maps And Reset SLAM'
$clearButton.Location = New-Object System.Drawing.Point(24, 158)
$clearButton.Size = New-Object System.Drawing.Size(328, 46)
$form.Controls.Add($clearButton)

$stopButton = New-Object System.Windows.Forms.Button
$stopButton.Text = 'Stop And Restore Safe Mode'
$stopButton.Location = New-Object System.Drawing.Point(368, 158)
$stopButton.Size = New-Object System.Drawing.Size(168, 46)
$form.Controls.Add($stopButton)

$logButton = New-Object System.Windows.Forms.Button
$logButton.Text = 'Open Session Log'
$logButton.Location = New-Object System.Drawing.Point(24, 232)
$logButton.Size = New-Object System.Drawing.Size(160, 36)
$form.Controls.Add($logButton)

function Set-Controls([bool]$Busy) {
  $startButton.Enabled = -not $Busy
  $liveMappingReady = $false
  try { $liveMappingReady = Test-LiveMappingReady } catch { $liveMappingReady = $false }
  $saveButton.Enabled = (-not $Busy) -and $liveMappingReady
  $clearButton.Enabled = -not $Busy
  $stopButton.Enabled = -not $Busy
}

function Start-Worker([string]$Action) {
  if ($script:worker -and -not $script:worker.HasExited) { return }
  Set-Controls $true
  $state.Text = "$Action in progress"
  $strictArgument = if ($StrictLiveGate) { ' -StrictLiveGate' } else { '' }
  $arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$workerScript`" -Action $Action$strictArgument -StatusFile `"$statusFile`" -LogFile `"$logFile`""
  $script:worker = Start-Process -FilePath 'powershell.exe' -ArgumentList $arguments -WindowStyle Hidden -PassThru
}

$startButton.Add_Click({
  try {
    $owners = Get-PortOwners
    if ($owners.Count -gt 0) {
      $message = "Mapping replaces the current TCP-8765 bridge: $($owners -join ', ').`r`nIt enables read-only telemetry, odometry, and TF only for this mapping session. Continue?"
      if ([System.Windows.Forms.MessageBox]::Show($message, 'Start Mapping', 'YesNo', 'Warning') -ne 'Yes') { return }
    }
    Start-VcxSrv
    Start-Worker 'Start'
  }
  catch {
    $state.Text = "FAILED: $($_.Exception.Message)"
  }
})

$saveButton.Add_Click({ Start-Worker 'Save' })

$clearButton.Add_Click({
  $message = 'This stops the current SLAM session, moves saved .pgm, .yaml, .posegraph, and .data files from ROS2_WIN\\maps to the Recycle Bin, then starts a new empty mapping session. Continue?'
  if ([System.Windows.Forms.MessageBox]::Show($message, 'Clear Maps And Reset SLAM', 'YesNo', 'Warning') -eq 'Yes') {
    Start-Worker 'Clear'
  }
})

$stopButton.Add_Click({ Start-Worker 'Stop' })

$logButton.Add_Click({
  if (-not (Test-Path -LiteralPath $logFile)) {
    New-Item -ItemType File -Path $logFile -Force | Out-Null
  }
  Start-Process notepad.exe -ArgumentList "`"$logFile`""
})

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 500
$timer.Add_Tick({
  try {
    if (Test-Path -LiteralPath $statusFile) {
      $state.Text = (Get-Content -LiteralPath $statusFile -Raw).Trim()
    }
    if ($script:worker -and $script:worker.HasExited) {
      $exitCode = $script:worker.ExitCode
      $script:worker.Dispose()
      $script:worker = $null
      if ($exitCode -eq 0) {
        Set-Controls $false
      } else {
        $state.Text = "$($state.Text) Review the session log."
        Set-Controls $false
      }
      if ($script:closeAfterWorker) { $form.Close() }
    }
  } catch {
    # A status refresh must never terminate the WinForms message loop.
    $state.Text = "FAILED: $($_.Exception.Message)"
    Set-Controls $false
  }
})
$timer.Start()

$form.Add_Shown({ Set-Controls $false })
$form.Add_FormClosing({
  param($sender, $event)
  if ($script:worker -and -not $script:worker.HasExited) {
    [System.Windows.Forms.MessageBox]::Show('An operation is still running. Wait for it to complete before closing.', 'Smart Car Mapping', 'OK', 'Information') | Out-Null
    $event.Cancel = $true
    return
  }
  $mappingRunning = $false
  try { $mappingRunning = Test-MappingRunning } catch { $mappingRunning = $false }
  if ($mappingRunning) {
    $answer = [System.Windows.Forms.MessageBox]::Show(
      'Stop mapping and restore the false-gated bridge before closing?',
      'Restore Safe Mode', 'YesNoCancel', 'Warning'
    )
    if ($answer -eq 'Cancel') { $event.Cancel = $true; return }
    if ($answer -eq 'Yes') {
      $event.Cancel = $true
      $script:closeAfterWorker = $true
      Start-Worker 'Stop'
    }
  }
})

[void]$form.ShowDialog()
