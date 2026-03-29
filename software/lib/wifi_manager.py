import subprocess

class WifiManager:
    def __init__(self):
        self.state = {}

    def discover_interfaces(self):
        raise NotImplementedError

    def connect(self, iface, ssid, password):
        raise NotImplementedError

    def set_ap_mode(self, iface, ssid, password):
        raise NotImplementedError

    def restore(self):
        raise NotImplementedError
    
    def run(self, cmd):
        """Run shell command and return stdout."""
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        return result.stdout.strip()

class LinuxWifiManager(WifiManager):
    def discover_interfaces(self):
        output = self.run("nmcli -t -f DEVICE,TYPE device")
        ifaces = []
        for line in output.splitlines():
            if ":" in line:
                device, dev_type = line.split(":", 1)
                if dev_type == "wifi":
                    ifaces.append(device)
        if len(ifaces) < 2:
            raise RuntimeError("Need at least 2 wifi interfaces")
        return ifaces[:2]

    def get_mode(self, iface):
        output = self.run(f"iw dev {iface} info")
        for line in output.splitlines():
            if "type" in line:
                return line.split()[-1]
        return None

    def connect(self, iface, ssid, password):
        self.state[iface] = {
            "mode": self.get_mode(iface)
        }
        self.run(f"nmcli dev wifi connect '{ssid}' password '{password}' ifname {iface}")

    def set_ap_mode(self, iface, ssid, password):
        self.state[iface] = {
            "mode": self.get_mode(iface)
        }

        self.run(f"ip link set {iface} down")
        self.run(f"iw dev {iface} set type __ap")
        self.run(f"ip link set {iface} up")

        # Minimal AP (real use should use hostapd)
        self.run(f"nmcli dev wifi hotspot ifname {iface} ssid {ssid} password {password}")

    def restore(self):
        for iface, info in self.state.items():
            self.run(f"ip link set {iface} down")
            self.run(f"iw dev {iface} set type managed")
            self.run(f"ip link set {iface} up")

class WindowsWifiManager(WifiManager):
    def discover_interfaces(self):
        output = self.run("netsh wlan show interfaces")
        ifaces = []
        for line in output.splitlines():
            if "Name" in line:
                ifaces.append(line.split(":")[1].strip())
        return ifaces[:2]

    def connect(self, iface, ssid, password):
        self.state[iface] = {}
        self.run(f'netsh wlan connect name="{ssid}" interface="{iface}"')

    def set_ap_mode(self, iface, ssid, password):
        self.run(f'netsh wlan set hostednetwork mode=allow ssid={ssid} key={password}')
        self.run("netsh wlan start hostednetwork")

    def restore(self):
        self.run("netsh wlan stop hostednetwork")

class MacWifiManager(WifiManager):
    def discover_interfaces(self):
        output = self.run("networksetup -listallhardwareports")
        ifaces = []
        is_wifi = False
        for line in output.splitlines():
            if "Hardware Port:" in line and "Wi-Fi" in line:
                is_wifi = True
            elif "Device:" in line and is_wifi:
                ifaces.append(line.split(":")[1].strip())
                is_wifi = False
        return ifaces[:2]

    def connect(self, iface, ssid, password):
        self.state[iface] = {}
        self.run(f"networksetup -setairportnetwork {iface} {ssid} {password}")

    def set_ap_mode(self, iface, ssid, password):
        # macOS does not support easy CLI AP mode
        raise NotImplementedError("AP mode not easily supported on macOS")

    def restore(self):
        pass