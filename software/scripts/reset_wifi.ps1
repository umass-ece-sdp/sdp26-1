# Reset WiFi connections for FALCON system on Windows
# Disconnects from all networks
# Hardcoded values - no user input needed
# Requires Windows 10/11 and admin privileges

param()

# Check for admin privileges
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")
if (-not $isAdmin) {
    Write-Host "❌ Error: This script requires administrator privileges." -ForegroundColor Red
    Write-Host "Please run PowerShell as Administrator and try again." -ForegroundColor Red
    exit 1
}

Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "FALCON WiFi Reset (Windows)" -ForegroundColor Cyan
Write-Host "==========================================" -ForegroundColor Cyan

# Get all WiFi adapters
Write-Host "[*] Detecting WiFi adapters..." -ForegroundColor Yellow

try {
    $adapters = Get-NetAdapter -Physical | Where-Object {$_.MediaType -eq "Native 802.11"}
} catch {
    Write-Host "⚠ Could not detect adapters, trying alternative method..." -ForegroundColor Yellow
    $adapters = @()
}

if ($adapters.Count -eq 0) {
    Write-Host "⚠ No WiFi adapters detected." -ForegroundColor Yellow
    exit 0
}

Write-Host "Found $($adapters.Count) WiFi adapter(s):" -ForegroundColor Green

# Disconnect all adapters
foreach ($adapter in $adapters) {
    Write-Host "  Disconnecting $($adapter.Name)..." -ForegroundColor Yellow
    try {
        netsh wlan disconnect interface=$adapter.Name 2>$null | Out-Null
        Write-Host "    ✓ Disconnected" -ForegroundColor Green
    }
    catch {
        Write-Host "    ⚠ Could not disconnect: $_" -ForegroundColor Yellow
    }
}

Write-Host ""
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "✓ WiFi Cleanup Complete" -ForegroundColor Green
Write-Host "==========================================" -ForegroundColor Cyan
Write-Host "All connections have been disconnected." -ForegroundColor White
Write-Host "==========================================" -ForegroundColor Cyan

exit 0
