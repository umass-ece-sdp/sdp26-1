#Requires -RunAsAdministrator
<#
.SYNOPSIS
    Sets up dual-WiFi for drone control:
      - One adapter connects to the Tello (client mode)
      - Mobile Hotspot provides AP for the ESP32

.USAGE
    1. Edit the CONFIG section below
    2. Right-click PowerShell -> Run as Administrator
    3. Run: .\setup_drone_network.ps1
    4. To tear down: .\setup_drone_network.ps1 -Teardown
#>

param(
    [switch]$Teardown
)

# =====================================================================
#  CONFIG
# =====================================================================
$TELLO_SSID       = "TELLO-FE046A"
$AP_SSID          = "jetson_nano_wifi"
$AP_PASSWORD      = "team1-falcon"
$AP_IP            = "192.168.137.1"       # Mobile Hotspot default
$SERVER_PORT      = 5000
$TELLO_SUBNET     = "192.168.10.0"
$TELLO_GATEWAY    = "192.168.10.1"
$FIREWALL_RULE    = "DroneController-ESP32"

# =====================================================================
#  Helpers
# =====================================================================
function Write-Step($msg)  { Write-Host "`n>> $msg" -ForegroundColor Cyan }
function Write-Ok($msg)    { Write-Host "   OK: $msg" -ForegroundColor Green }
function Write-Warn($msg)  { Write-Host "   WARNING: $msg" -ForegroundColor Yellow }
function Write-Err($msg)   { Write-Host "   ERROR: $msg" -ForegroundColor Red }

function Get-WlanInterfaces {
    $raw = netsh wlan show interfaces
    $interfaces = @()
    $current = @{}
    foreach ($line in $raw) {
        if ($line -match '^\s*Name\s*:\s*(.+)$') {
            if ($current.Count -gt 0) { $interfaces += [PSCustomObject]$current }
            $current = @{ Name = $Matches[1].Trim() }
        }
        elseif ($line -match '^\s*State\s*:\s*(.+)$')       { $current.State = $Matches[1].Trim() }
        elseif ($line -match '^\s*SSID\s*:\s*(.+)$')        { $current.SSID  = $Matches[1].Trim() }
        elseif ($line -match '^\s*Description\s*:\s*(.+)$')  { $current.Description = $Matches[1].Trim() }
    }
    if ($current.Count -gt 0) { $interfaces += [PSCustomObject]$current }
    return $interfaces
}

function Test-HotspotRunning {
    # Check if 192.168.137.1 is assigned to any adapter -- that means hotspot is up
    $ip = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
        Where-Object { $_.IPAddress -eq $AP_IP }
    return ($null -ne $ip)
}

function Configure-HotspotRegistry {
    # Pre-configure SSID and password via registry so when the user
    # (or the ms-settings URI) turns it on, it uses our values.
    $regPath = "HKLM:\SYSTEM\CurrentControlSet\Services\WlanSvc\Parameters\HostedNetworkSettings"
    try {
        # The Mobile Hotspot settings are stored here on Win10/11
        $settingsPath = "HKLM:\SOFTWARE\Microsoft\WlanSvc\LocalProfiles"
        # Unfortunately the hotspot SSID/password are stored in a binary
        # blob that's not trivially editable. We'll configure via the
        # Settings UI prompt instead.
        return $false
    } catch {
        return $false
    }
}

function Prompt-HotspotManual {
    Write-Host ""
    Write-Host "   ============================================" -ForegroundColor Yellow
    Write-Host "   MANUAL STEP: Turn on Mobile Hotspot" -ForegroundColor Yellow
    Write-Host "   ============================================" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "   Opening Windows Settings now..." -ForegroundColor White
    Write-Host ""
    Write-Host "   Configure these settings:" -ForegroundColor White
    Write-Host "     Network name:  $AP_SSID" -ForegroundColor Cyan
    Write-Host "     Password:      $AP_PASSWORD" -ForegroundColor Cyan
    Write-Host "     Band:          2.4 GHz  (important for ESP32!)" -ForegroundColor Cyan
    Write-Host "     Turn it:       ON" -ForegroundColor Cyan
    Write-Host ""

    # Open the Mobile Hotspot settings page directly
    Start-Process "ms-settings:network-mobilehotspot"

    Write-Host "   Waiting for hotspot to come up..."
    Write-Host "   (Checking for $AP_IP every 2 seconds)`n"

    $timeout = 120  # 2 minutes
    $elapsed = 0
    while ($elapsed -lt $timeout) {
        if (Test-HotspotRunning) {
            return $true
        }
        Start-Sleep -Seconds 2
        $elapsed += 2
        # Show a dot every 10 seconds so it doesn't look frozen
        if ($elapsed % 10 -eq 0) {
            Write-Host "   ... still waiting ($elapsed s)" -ForegroundColor Gray
        }
    }
    return $false
}

# =====================================================================
#  TEARDOWN
# =====================================================================
if ($Teardown) {
    Write-Host "`n=== TEARING DOWN DRONE NETWORK ===" -ForegroundColor Yellow

    Write-Step "Mobile Hotspot"
    if (Test-HotspotRunning) {
        Write-Host "   Opening Settings -- please turn OFF the hotspot manually."
        Start-Process "ms-settings:network-mobilehotspot"
        Write-Host "   Waiting for hotspot to stop..."
        $timeout = 60; $elapsed = 0
        while ($elapsed -lt $timeout -and (Test-HotspotRunning)) {
            Start-Sleep -Seconds 2; $elapsed += 2
        }
        if (Test-HotspotRunning) {
            Write-Warn "Hotspot still running. Turn it off manually."
        } else {
            Write-Ok "Hotspot stopped"
        }
    } else {
        Write-Ok "Hotspot was not running"
    }

    Write-Step "Removing firewall rule"
    netsh advfirewall firewall delete rule name="$FIREWALL_RULE" 2>$null | Out-Null
    Write-Ok "Done"

    Write-Step "Cleaning up routes"
    route delete $TELLO_SUBNET 2>$null | Out-Null
    route delete 192.168.137.0 2>$null | Out-Null
    Write-Ok "Done"

    Write-Host "`n=== TEARDOWN COMPLETE ===`n" -ForegroundColor Green
    exit 0
}

# =====================================================================
#  SETUP
# =====================================================================
Write-Host ""
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  DRONE DUAL-WIFI NETWORK SETUP" -ForegroundColor Cyan
Write-Host "============================================" -ForegroundColor Cyan
Write-Host "  Tello SSID:     $TELLO_SSID"
Write-Host "  AP SSID:        $AP_SSID"
Write-Host "  AP IP:          $AP_IP"
Write-Host "  Server Port:    $SERVER_PORT"
Write-Host "============================================`n" -ForegroundColor Cyan

# --- Step 1: Discover adapters ---
Write-Step "Discovering wireless adapters"
$adapters = Get-WlanInterfaces

if ($adapters.Count -lt 2) {
    Write-Err "Found $($adapters.Count) wireless adapter(s). Need at least 2."
    Write-Host "   Make sure your external USB WiFi adapter is plugged in."
    foreach ($a in $adapters) {
        Write-Host "     - $($a.Name) ($($a.Description))"
    }
    exit 1
}

Write-Host ""
Write-Host "   Found $($adapters.Count) wireless adapters:" -ForegroundColor White
for ($i = 0; $i -lt $adapters.Count; $i++) {
    $a = $adapters[$i]
    $state = if ($a.State) { $a.State } else { "unknown" }
    $ssid  = if ($a.SSID)  { " -> $($a.SSID)" } else { "" }
    Write-Host "     [$i] $($a.Name) - $($a.Description) [$state$ssid]"
}

# --- Step 2: Assign roles ---
Write-Step "Assigning adapter roles"

$clientAdapter = $null

# Auto-detect if already on Tello
foreach ($a in $adapters) {
    if ($a.SSID -eq $TELLO_SSID) {
        $clientAdapter = $a
        Write-Host "   (Auto-detected: $($a.Name) already on $TELLO_SSID)"
        break
    }
}

if (-not $clientAdapter) {
    # Default: external USB for Tello, internal for hotspot
    $usb = $adapters | Where-Object { $_.Description -match 'USB|Realtek' } | Select-Object -First 1
    $clientAdapter = if ($usb) { $usb } else { $adapters[1] }
}

$hotspotAdapter = $adapters | Where-Object { $_.Name -ne $clientAdapter.Name } | Select-Object -First 1

Write-Host ""
Write-Host "   Tello client:   $($clientAdapter.Name) ($($clientAdapter.Description))" -ForegroundColor Green
Write-Host "   Hotspot source: $($hotspotAdapter.Name) ($($hotspotAdapter.Description))" -ForegroundColor Green
Write-Host ""

$confirm = Read-Host "   Proceed? (Y/n)"
if ($confirm -eq 'n' -or $confirm -eq 'N') {
    $temp = $clientAdapter
    $clientAdapter = $hotspotAdapter
    $hotspotAdapter = $temp
    Write-Host "   Swapped:" -ForegroundColor Yellow
    Write-Host "     Tello:   $($clientAdapter.Name)" -ForegroundColor Yellow
    Write-Host "     Hotspot: $($hotspotAdapter.Name)" -ForegroundColor Yellow
}

# --- Step 3: Mobile Hotspot ---
Write-Step "Setting up Mobile Hotspot"

if (Test-HotspotRunning) {
    Write-Ok "Mobile Hotspot is already running at $AP_IP"

    # Verify SSID matches (best-effort via netsh)
    $hostedInfo = netsh wlan show hostednetwork 2>$null
    Write-Host ('   Verify the SSID is {0} in Settings if ESP32 cannot connect.' -f $AP_SSID)
} else {
    $hotspotOk = Prompt-HotspotManual

    if ($hotspotOk) {
        Write-Ok "Mobile Hotspot is running at $AP_IP"
    } else {
        Write-Err "Hotspot did not start within timeout."
        Write-Host "   Make sure you turned it on in Settings."
        Write-Host ('   Check that the SSID is {0} and band is 2.4 GHz.' -f $AP_SSID)
        $cont = Read-Host "   Continue anyway? (y/N)"
        if ($cont -ne 'y' -and $cont -ne 'Y') { exit 1 }
    }
}

# --- Step 4: Connect to Tello ---
Write-Step ("Connecting to Tello ({0}) on {1}" -f $TELLO_SSID, $clientAdapter.Name)

if ($clientAdapter.SSID -eq $TELLO_SSID) {
    Write-Ok "Already connected"
} else {
    netsh wlan disconnect interface="$($clientAdapter.Name)" 2>$null | Out-Null
    Start-Sleep -Seconds 1

    $profiles = netsh wlan show profiles 2>$null
    if ($profiles -match [regex]::Escape($TELLO_SSID)) {
        Write-Host "   Found saved profile, connecting..."
        netsh wlan connect name="$TELLO_SSID" interface="$($clientAdapter.Name)" 2>$null | Out-Null
        Start-Sleep -Seconds 4
    } else {
        Write-Warn ("No saved profile for {0}." -f $TELLO_SSID)
        Write-Host "   Connect manually:" -ForegroundColor White
        Write-Host ('     1. Open WiFi settings for {0}' -f $clientAdapter.Name)
        Write-Host ('     2. Connect to {0}' -f $TELLO_SSID)
        Write-Host ""
        Read-Host "   Press Enter once connected"
    }

    Start-Sleep -Seconds 2
    $updated = Get-WlanInterfaces | Where-Object { $_.Name -eq $clientAdapter.Name }
    if ($updated.SSID -eq $TELLO_SSID) {
        Write-Ok "Connected to $TELLO_SSID"
    } else {
        Write-Warn "May not be connected. Check manually if needed."
    }
}

# --- Step 5: Routing ---
Write-Step "Configuring routes"

# Get the interface index of the Tello client adapter so we can force
# the route through the correct adapter (without IF, Windows may pick
# a WSL virtual adapter or the wrong physical adapter)
$clientIfIndex = (Get-NetAdapter | Where-Object { $_.Name -eq $clientAdapter.Name }).ifIndex

if (-not $clientIfIndex) {
    Write-Warn "Could not determine interface index for $($clientAdapter.Name)"
    Write-Host "   Run: Get-NetAdapter | Format-Table Name, ifIndex, Status"
    $clientIfIndex = Read-Host "   Enter the ifIndex for $($clientAdapter.Name)"
}

route delete $TELLO_SUBNET      2>$null | Out-Null
route delete 192.168.137.0      2>$null | Out-Null

route add $TELLO_SUBNET mask 255.255.255.0 $TELLO_GATEWAY IF $clientIfIndex metric 10 2>$null | Out-Null
Write-Ok "Tello ($TELLO_SUBNET) -> $TELLO_GATEWAY via IF $clientIfIndex ($($clientAdapter.Name))"

route add 192.168.137.0 mask 255.255.255.0 $AP_IP metric 10 2>$null | Out-Null
Write-Ok "ESP32 (192.168.137.0/24) -> $AP_IP"

# --- Step 6: Firewall ---
Write-Step "Firewall rule for port $SERVER_PORT"
netsh advfirewall firewall delete rule name="$FIREWALL_RULE" 2>$null | Out-Null
netsh advfirewall firewall add rule name="$FIREWALL_RULE" dir=in action=allow protocol=tcp localport=$SERVER_PORT | Out-Null
Write-Ok "Inbound TCP $SERVER_PORT allowed"

# --- Done ---
Write-Host ""
Write-Host "============================================" -ForegroundColor Green
Write-Host "  SETUP COMPLETE" -ForegroundColor Green
Write-Host "============================================" -ForegroundColor Green
Write-Host ""
Write-Host ('  ESP32 --wifi--> Hotspot ({0} at {1})' -f $AP_SSID, $AP_IP)
Write-Host '                        |'
Write-Host '                   YOUR LAPTOP'
Write-Host '                        |'
Write-Host ('  Tello <--wifi-- {0} ({1})' -f $clientAdapter.Name, $TELLO_SSID)
Write-Host ""
Write-Host "  ESP32 config:" -ForegroundColor White
Write-Host ('    SSID:     {0}' -f $AP_SSID)
Write-Host ('    Password: {0}' -f $AP_PASSWORD)
Write-Host ('    Server:   {0}:{1}' -f $AP_IP, $SERVER_PORT)
Write-Host ""
Write-Host ('  Server code: HOST = {0}' -f $AP_IP) -ForegroundColor Yellow
Write-Host ""
Write-Host '  Teardown:  .\setup_drone_network.ps1 -Teardown' -ForegroundColor Yellow
Write-Host ""