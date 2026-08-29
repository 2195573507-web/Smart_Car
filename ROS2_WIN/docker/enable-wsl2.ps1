#requires -RunAsAdministrator
$ErrorActionPreference = 'Stop'

$features = @(
  'Microsoft-Windows-Subsystem-Linux',
  'VirtualMachinePlatform'
)

foreach ($feature in $features) {
  $state = (Get-WindowsOptionalFeature -Online -FeatureName $feature).State
  if ($state -ne 'Enabled') {
    Write-Host "Enabling $feature ..."
    Enable-WindowsOptionalFeature -Online -FeatureName $feature -All -NoRestart | Out-Host
  } else {
    Write-Host "$feature already enabled"
  }
}

wsl.exe --set-default-version 2
bcdedit.exe /set hypervisorlaunchtype auto | Out-Host
Write-Host ''
Write-Host 'WSL2 prerequisites and hypervisor launch are enabled. Reboot Windows, then restart Docker Desktop.'
Write-Host 'Verify with: wsl.exe --status; docker version'
