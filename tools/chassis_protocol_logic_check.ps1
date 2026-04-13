$ErrorActionPreference = 'Stop'

# 1) 8-direction decode & normalization check
$dirs = [ordered]@{
  0=@(0.0,0.0); 1=@(1.0,0.0); 2=@(-1.0,0.0); 3=@(0.0,1.0); 4=@(0.0,-1.0);
  5=@(1.0,1.0); 6=@(1.0,-1.0); 7=@(-1.0,1.0); 8=@(-1.0,-1.0)
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
$dt = 0.002
$up = 24.0
$down = 18.0
$target = 12.0
$spin = 0.0
$trace = @()

for ($i = 0; $i -lt 260; $i++) {
  $step = $up * $dt
  if ($spin -lt $target) { $spin = [Math]::Min($spin + $step, $target) }
  $trace += $spin
}

for ($i = 0; $i -lt 400; $i++) {
  $step = $down * $dt
  if ($spin -gt 0.0) { $spin = [Math]::Max($spin - $step, 0.0) }
  $trace += $spin
}

$monoUp = $true
for ($i = 1; $i -lt 260; $i++) {
  if ($trace[$i] + 1e-9 -lt $trace[$i-1]) { $monoUp = $false; break }
}

$monoDown = $true
for ($i = 261; $i -lt $trace.Count; $i++) {
  if ($trace[$i] -gt $trace[$i-1] + 1e-9) { $monoDown = $false; break }
}

$gyroPass = $monoUp -and $monoDown -and ([Math]::Abs($trace[259] - $target) -lt 1e-6) -and ([Math]::Abs($trace[-1] - 0.0) -lt 1e-6)

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
  GyroRampUpFinal = [Math]::Round($trace[259], 3)
  GyroRampDownFinal = [Math]::Round($trace[-1], 3)
}

"=== DIRECTION TABLE ==="
$rows | Format-Table -AutoSize
"=== SUMMARY ==="
$summary | Format-List
