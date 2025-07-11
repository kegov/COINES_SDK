

# MTP switch tool  

This tool jumps to the Application mode (flash start) when an MTP device is detected.

### Expected output

```sh
>mtp_switch.exe

[INFO] Searching for connected devices...Device 0 (VID=108c and PID=ab31) is UNKNOWN in libmtp v1.1.22.
Please report this VID/PID and the device model to the libmtp development team

[INFO] Device detected:
Device 0: Vendor ID: 0x108C, Product ID: 0xAB31
[INFO] Attempting to establish connection...
[INFO] Connection successful
[INFO] MTP to APP mode switch successful
[INFO] Device disconnection successful
```

### Troubleshooting (Linux)  

If you encounter the following error:  

```
Attempting to connect device(s)  
libusb_claim_interface() reports device is busy, likely in use by GVFS or KDE MTP device handling already  
LIBMTP PANIC: Unable to initialize device  
```

This is likely caused by **GVFS** interfering with the MTP connection. To resolve it, stop the GVFS process using one of the following commands:  

```sh
pkill gvfsd-mtp
```
or  
```sh
pkill gvfs-mtp-volume-monitor
```  

### Limitation
When multiple MTP devices are connected, mtp_switch switches only the first enumerated device. It does not currently support selecting a specific device.

---