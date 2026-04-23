# Setup WiFi for FALCON system on Windows
# Connects to both Tello and glove AP
# Hardcoded values - no user input needed
# Requires Windows 10/11 and admin privileges

param()

# Hardcoded WiFi credentials
$TELLO_SSID = "TELLO-FE046A"
$TELLO_PASSWORD = ""
$GLOVE_SSID = "FALCON-Glove"
$GLOVE_PASSWORD = "team1-falcon"

# Check for admin privileges
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")
if (-not $isAdmin) {
    Write-Host "❌ Error: This script requires administrator privileges." -ForegroundColor Red
    Write-Host "Please run PowerShell as Administrator and try again." -ForegroundColor Red
    exit 1
}

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "FALCON WiFi Setup (Windows - Hardcoded)" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "Tello SSID: $TELLO_SSID" -ForegroundColor White
Write-Host "Glove SSID: $GLOVE_SSID" -ForegroundColor White
Write-Host ""

# Get available WiFi adapters
Write-Host "[*] Detecting WiFi adapters..." -ForegroundColor Yellow

try {
    $adapters = Get-NetAdapter -Physical | Where-Object {$_.MediaType -eq "Native 802.11"} | Select-Object -First 2
} catch {
    Write-Host "⚠ Could not detect adapters via Get-NetAdapter, trying alternative method..." -ForegroundColor Yellow
    $adapters = $null
}

if (-not $adapters -or $adapters.Count -lt 2) {
    Write-Host "❌ Error: Could not detect 2 WiFi adapters." -ForegroundColor Red
    Write-Host "   Found: $($adapters.Count) adapter(s)" -ForegroundColor Red
    Write-Host ""
    Write-Host "Available adapters:" -ForegroundColor Yellow
    Get-NetAdapter -Physical | Where-Object {$_.MediaType -eq "Native 802.11"} | ForEach-Object {
        Write-Host "  - $($_.Name) ($($_.InterfaceDescription))" -ForegroundColor Yellow
    }
    exit 1
}

$telloAdapter = $adapters[0]
$gloveAdapter = $adapters[1]

Write-Host "Detected WiFi adapters:" -ForegroundColor Green
Write-Host "  - Tello adapter: $($telloAdapter.Name) - $($telloAdapter.InterfaceDescription)" -ForegroundColor Green
Write-Host "  - Glove adapter: $($gloveAdapter.Name) - $($gloveAdapter.InterfaceDescription)" -ForegroundColor Green
Write-Host ""

# Function to connect to WiFi
function Connect-WiFiNetwork {
    param(
        [string]$SSID,
        [string]$Password,
        [string]$InterfaceName,
        [string]$Description
    )
    
    Write-Host "[*] Connecting $InterfaceName to $SSID..." -ForegroundColor Yellow
    
    # Create WiFi profile for this SSID
    try {
        # Check if profile already exists
        $profileExists = netsh wlan show profile name=$SSID 2>$null | Select-String "All User Profile"
        
        if ($profileExists) {
            Write-Host "  Profile already exists, forgetting and recreating..." -ForegroundColor Cyan
            netsh wlan delete profile name=$SSID interface=$InterfaceName 2>$null | Out-Null
        }
        
        # Create profile XML
        if ([string]::IsNullOrEmpty($Password)) {
            $profileXml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>$SSID</name>
    <SSIDConfig>
        <SSID>
            <hex>$([System.BitConverter]::ToString([System.Text.Encoding]::UTF8.GetBytes($SSID)) -replace '-')</hex>
            <name>$SSID</name>
        </SSID>
        <nonBroadcast>false</nonBroadcast>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <autoSwitch>true</autoSwitch>
    <MSSecuritySetting>
        <SecurityMode>open</SecurityMode>
        <AuthEncryption>
            <Authentication>open</Authentication>
            <Encryption>none</Encryption>
            <UseOneX>false</UseOneX>
        </AuthEncryption>
    </MSSecuritySetting>
    <MacRandomization xmlns="http://www.microsoft.com/networking/WLAN/profile/v3">
        <enableRandomization>false</enableRandomization>
    </MacRandomization>
</WLANProfile>
"@
        } else {
            $profileXml = @"
<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
    <name>$SSID</name>
    <SSIDConfig>
        <SSID>
            <hex>$([System.BitConverter]::ToString([System.Text.Encoding]::UTF8.GetBytes($SSID)) -replace '-')</hex>
            <name>$SSID</name>
        </SSID>
        <nonBroadcast>false</nonBroadcast>
    </SSIDConfig>
    <connectionType>ESS</connectionType>
    <connectionMode>auto</connectionMode>
    <autoSwitch>true</autoSwitch>
    <MSSecuritySetting>
        <SecurityMode>WPA2</SecurityMode>
        <AuthEncryption>
            <Authentication>WPA2PSK</Authentication>
            <Encryption>CCMP</Encryption>
            <UseOneX>false</UseOneX>
        </AuthEncryption>
    </MSSecuritySetting>
    <MacRandomization xmlns="http://www.microsoft.com/networking/WLAN/profile/v3">
        <enableRandomization>false</enableRandomization>
    </MacRandomization>
    <sharedKey>
        <keyType>passPhrase</keyType>
        <protected>false</protected>
        <keyMaterial>$Password</keyMaterial>
    </sharedKey>
</WLANProfile>
"@
        }
        
        # Save profile to temp file
        $profilePath = "$env:TEMP\FALCON_$($SSID -replace ' ','_').xml"
        Set-Content -Path $profilePath -Value $profileXml -Encoding UTF8
        
        # Add profile
        netsh wlan add profile filename=$profilePath interface=$InterfaceName | Out-Null
        
        # Connect to network
        netsh wlan connect name=$SSID interface=$InterfaceName | Out-Null
        
        # Cleanup temp file
        Remove-Item -Path $profilePath -Force 2>$null
        
        # Wait a bit for connection
        Start-Sleep -Seconds 3
        
        # Verify connection
        $connected = netsh wlan show interfaces interface=$InterfaceName | Select-String "SSID.*$SSID"
        if ($connected) {
            Write-Host "✓ $Description connected successfully!" -ForegroundColor Green
            return $true
        } else {
            Write-Host "⚠ Connection initiated, verifying..." -ForegroundColor Yellow
            return $true
        }
    }
    catch {
        Write-Host "❌ Error connecting to $SSID : $_" -ForegroundColor Red
        return $false
    }
}

# Connect to Tello
Write-Host "[1/2] Connecting to Tello drone..." -ForegroundColor Cyan
$telloConnected = Connect-WiFiNetwork -SSID $TELLO_SSID -Password $TELLO_PASSWORD -InterfaceName $telloAdapter.Name -Description "Tello"
Start-Sleep -Seconds 2

# Connect to Glove
Write-Host ""
Write-Host "[2/2] Connecting to glove AP..." -ForegroundColor Cyan
$gloveConnected = Connect-WiFiNetwork -SSID $GLOVE_SSID -Password $GLOVE_PASSWORD -InterfaceName $gloveAdapter.Name -Description "Glove"
Start-Sleep -Seconds 2

# Summary
Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "✓ WiFi Setup Complete!" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "  Tello adapter: $($telloAdapter.Name)" -ForegroundColor White
Write-Host "  Glove adapter: $($gloveAdapter.Name)" -ForegroundColor White
Write-Host "  Glove server: 192.168.4.1:5000" -ForegroundColor White
Write-Host ""
Write-Host "Ready to run: python -m software.src.main" -ForegroundColor Yellow
Write-Host "==========================================" -ForegroundColor Cyan

exit 0
