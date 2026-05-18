# Testing springcard-ccid-tcp on WSL

Quick guide to set up, build, and test the CCID driver on WSL Ubuntu/Debian.

## 1. WSL Setup

### Install WSL2 with Ubuntu/Debian (if not already done)
```bash
# From PowerShell (admin) on Windows:
wsl --install -d Ubuntu-22.04
# or: wsl --install -d Debian
```

### Update the system
```bash
sudo apt update && sudo apt upgrade -y
```

## 2. Install Build Dependencies

```bash
sudo apt install -y \
  build-essential \
  autoconf \
  automake \
  libtool \
  pkg-config \
  libpcsclite-dev \
  pcscd \
  pcsc-tools \
  flex \
  git
```

## 3. Clone and Navigate

```bash
cd /path/to/springcard-ccid-tcp
```

## 4. Build

```bash
./bootstrap
./configure
make
```

## 5. Install

```bash
sudo make install
```

This installs the driver to `/usr/local/lib/libccidtcp.so` by default.

## 6. Test with pcscd

### Terminal 1: Start pcscd in foreground
```bash
sudo pcscd --foreground --debug
```

You should see debug output showing the driver is loaded.

### Terminal 2: Test the reader connection
```bash
# List readers (should show your SpringCard network reader if configured)
pcsc_scan

# Or use any PC/SC-compatible tool
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `ifdhandler.h not found` | Install `libpcsclite-dev`: `sudo apt install libpcsclite-dev` |
| `pcscd: command not found` | Install `pcscd`: `sudo apt install pcscd` |
| `Permission denied` when running `pcscd` | Use `sudo` or adjust user group membership |
| Driver not loaded | Check `/usr/local/lib` contains `libccidtcp.so` |

## Tips

- **Rebuild after changes**: `make clean && make && sudo make install`
- **View debug logs**: Run `pcscd --foreground --debug` to see detailed output
- **Config file**: Reader configuration typically at `/etc/springcard/reader.conf`
