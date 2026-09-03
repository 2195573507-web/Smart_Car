param(
  [Parameter(Mandatory = $true)]
  [string]$BagPath,
  [switch]$WithoutClock,
  [double]$Rate = 1.0,
  [switch]$Loop
)

if (-not (Test-Path -LiteralPath $BagPath -PathType Container)) {
  throw "BagPath does not exist or is not a rosbag directory: $BagPath"
}
if ($Rate -le 0) { throw "Rate must be positive" }
$topics = @('/scan', '/odom', '/tf', '/tf_static', '/diagnostics', '/map')
$rateText = $Rate.ToString([Globalization.CultureInfo]::InvariantCulture)
$rosArgs = @('bag', 'play', $BagPath, '--rate', $rateText, '--topics') + $topics
if (-not $WithoutClock) { $rosArgs += '--clock' }
if ($Loop) { $rosArgs += '--loop' }
& ros2 @rosArgs
exit $LASTEXITCODE
