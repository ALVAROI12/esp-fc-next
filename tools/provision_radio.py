#!/usr/bin/env python3
"""NVS image with the C5 link key, network id and radio id (docs/protocols.md section 5).

usage: tools/provision_radio.py --id 1 [--net 0x5EEDF00D]
  id 0 is the ground station (the only sender whose CRTP commands drones accept); 255 means "all drones".

The group key lives in secrets/link_key.hex (created once, git-ignored, mode 0600) and is never printed.
Output: build/provision/radio-<id>.bin. Flash it to the default single-app table's nvs partition:
  esptool.py --chip esp32c5 write_flash 0x9000 build/provision/radio-<id>.bin
Flashing a new image resets the counter reservation, so only reprovision together with a new key.
"""
import argparse
import os
import secrets
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
KEY = ROOT / "secrets" / "link_key.hex"
NVS_SIZE = "0x6000"  # default single-app partition table
IMAGE = os.environ.get("IDF_IMAGE", "espressif/idf:release-v5.5")


def group_key() -> str:
    if not KEY.exists():
        KEY.parent.mkdir(exist_ok=True)
        fd = os.open(KEY, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as f:
            f.write(secrets.token_hex(16) + "\n")
        print(f"created new group key {KEY.relative_to(ROOT)}: every radio needs an image built with it")
    key = KEY.read_text().strip()
    if len(key) != 32 or any(c not in "0123456789abcdefABCDEF" for c in key):
        sys.exit(f"{KEY}: expected 32 hex characters")
    return key


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--id", type=int, required=True, help="radio id 0-254 (0 = ground station)")
    ap.add_argument("--net", type=lambda s: int(s, 0), default=0x5EEDF00D, help="network id (u32)")
    a = ap.parse_args()
    if not 0 <= a.id <= 254 or not 0 <= a.net <= 0xFFFFFFFF:
        sys.exit("--id must be 0-254 (255 = all drones) and --net a u32")

    out = ROOT / "build" / "provision"
    out.mkdir(parents=True, exist_ok=True)
    csv = out / f"radio-{a.id}.csv"
    fd = os.open(csv, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)  # holds the key
    with os.fdopen(fd, "w") as f:
        f.write(f"key,type,encoding,value\nfclink,namespace,,\nkey,data,hex2bin,{group_key()}\n"
                f"net,data,u32,{a.net}\nid,data,u8,{a.id}\n")
    try:
        subprocess.run(["docker", "run", "--rm", "-v", f"{out}:/out", "-e", "HOME=/tmp", "-u",
                        f"{os.getuid()}:{os.getgid()}", IMAGE, "bash", "-c",
                        "python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py "
                        f"generate /out/{csv.name} /out/radio-{a.id}.bin {NVS_SIZE}"], check=True,
                       stdout=subprocess.DEVNULL)
        (out / f"radio-{a.id}.bin").chmod(0o600)  # holds the key too
    finally:
        csv.unlink()
    print(f"{(out / f'radio-{a.id}.bin').relative_to(ROOT)}: id {a.id}, net {a.net:#010x}")


if __name__ == "__main__":
    main()
