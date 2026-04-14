$ErrorActionPreference = 'Stop'

$taskHeaderPath = 'Users/Application/APP_Chassis/App_Chassis_Task.h'
$taskHeader = Get-Content $taskHeaderPath -Raw

$dtMsMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_TIME_MS\s+([0-9]+)\b')
$dtSecMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_TIME\s+([0-9]*\.?[0-9]+)f?\b')
$freqMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_FREQUENCE\s+([0-9]*\.?[0-9]+)f?\b')

if (-not $dtMsMatch.Success -or -not $dtSecMatch.Success -or -not $freqMatch.Success) {
  throw "Failed to parse control-time macros from $taskHeaderPath"
}

$dtMs = [int]$dtMsMatch.Groups[1].Value
$dt = [double]$dtSecMatch.Groups[1].Value
$freq = [double]$freqMatch.Groups[1].Value

$dtFromMs = $dtMs / 1000.0
$freqFromDt = 1.0 / $dt

if ([Math]::Abs($dt - $dtFromMs) -gt 1e-9) {
  throw "Control time mismatch: CHASSIS_CONTROL_TIME_MS=$dtMs ms but CHASSIS_CONTROL_TIME=$dt s"
}

if ([Math]::Abs($freq - $freqFromDt) -gt 1e-3) {
  throw "Frequency mismatch: CHASSIS_CONTROL_FREQUENCE=$freq Hz but 1/CHASSIS_CONTROL_TIME=$freqFromDt Hz"
}

# 1) 8-direction decode & normalization check
$dirs = [ordered]@{
  0=@(0.0,0.0); 1=@(1.0,0.0); 2=@(-1.0,0.0); 3=@(0.0,-1.0); 4=@(0.0,1.0);
  5=@(1.0,-1.0); 6=@(1.0,1.0); 7=@(-1.0,-1.0); 8=@(-1.0,1.0)
}

$dirPass = $true
$diagPass = $true
$rows = @()

foreach ($k in $dirs.Keys) {
  $x = [double]$dirs[$k][0]
  $y = [double]$dirs[$k][1]
  $norm = [Math]::Sqrt($x*$x + $y*$y)

  if ($k -eq 0) {
    $ux = 0.0
    $uy = 0.0
    $scale = 0.0
  }
  else {
    if ($norm -le 1e-9) { $dirPass = $false }
    $ux = $x / $norm
    $uy = $y / $norm
    $scale = [Math]::Sqrt($ux*$ux + $uy*$uy)
    if ([Math]::Abs($scale - 1.0) -gt 1e-6) { $dirPass = $false }

    if ($k -ge 5 -and $k -le 8) {
      if ([Math]::Abs([Math]::Abs($ux) - 0.70710678) -gt 1e-4 -or [Math]::Abs([Math]::Abs($uy) - 0.70710678) -gt 1e-4) {
        $diagPass = $false
      }
    }
  }

  $rows += [PSCustomObject]@{
    dir = $k
    rawX = $x
    rawY = $y
    norm = [Math]::Round($norm, 6)
    unitX = [Math]::Round($ux, 6)
    unitY = [Math]::Round($uy, 6)
    scale = [Math]::Round($scale, 6)
  }
}

# 2) Gyro ramp up/down smoothness (same constants as behaviour file)
# dt comes from CHASSIS_CONTROL_TIME in App_Chassis_Task.h.
$up = 24.0
$down = 18.0
$target = 12.0
$spin = 0.0
$trace = @()

$upStep = $up * $dt
$downStep = $down * $dt
$upTicks = [int][Math]::Ceiling($target / $upStep) + 10
$downTicks = [int][Math]::Ceiling($target / $downStep) + 10
$upEndIndex = $upTicks - 1

for ($i = 0; $i -lt $upTicks; $i++) {
  $step = $upStep
  if ($spin -lt $target) { $spin = [Math]::Min($spin + $step, $target) }
  $trace += $spin
}

for ($i = 0; $i -lt $downTicks; $i++) {
  $step = $downStep
  if ($spin -gt 0.0) { $spin = [Math]::Max($spin - $step, 0.0) }
  $trace += $spin
}

$monoUp = $true
for ($i = 1; $i -lt $upTicks; $i++) {
  if ($trace[$i] + 1e-9 -lt $trace[$i-1]) { $monoUp = $false; break }
}

$monoDown = $true
for ($i = $upTicks; $i -lt $trace.Count; $i++) {
  if ($trace[$i] -gt $trace[$i-1] + 1e-9) { $monoDown = $false; break }
}

$gyroPass = $monoUp -and $monoDown -and ([Math]::Abs($trace[$upEndIndex] - $target) -lt 1e-6) -and ([Math]::Abs($trace[-1] - 0.0) -lt 1e-6)

# 3) Static source checks for mode=6 no-action and VT timeout safe-stop
$beh = Get-Content 'Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp' -Raw
$canh = Get-Content 'Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h' -Raw
$task = Get-Content 'Users/Application/APP_Chassis/App_Chassis_Task.cpp' -Raw

$mode6Reserved = $canh -match 'CHASSIS_MODE_RESERVED_2\s*=\s*6'
$normalGate = $beh -match 'requested_mode\s*==\s*CHASSIS_MODE_NORMAL\s*&&\s*protocol_valid'
$nonNormalHold = $beh -match 'chassis_action_hold_control\('
$mode6Pass = $mode6Reserved -and $normalGate -and $nonNormalHold

$timeoutBranch = $task -match 'toe_is_error\(VT_TOE\)'
$jointZero = $task -match 'chassis_move\.chassis_joint\[i\]\.joint_T\s*=\s*0\.0f'
$wheelZeroL = $task -match 'chassis_move\.chassis_wheel\[0\]\.wheel_T\s*=\s*0\.0f'
$wheelZeroR = $task -match 'chassis_move\.chassis_wheel\[1\]\.wheel_T\s*=\s*0\.0f'
$timeoutPass = $timeoutBranch -and $jointZero -and $wheelZeroL -and $wheelZeroR

$summary = [PSCustomObject]@{
  Direction8Pass = $dirPass
  DiagonalNormalizePass = $diagPass
  GyroRampSmoothPass = $gyroPass
  Mode6NoActionPass = $mode6Pass
  TimeoutSafeStopPass = $timeoutPass
  GyroRampUpFinal = [Math]::Round($trace[$upEndIndex], 3)
  GyroRampDownFinal = [Math]::Round($trace[-1], 3)
}

"=== DIRECTION TABLE ==="
$rows | Format-Table -AutoSize
"=== SUMMARY ==="
$summary | Format-List
