#!/usr/bin/env python3
"""Address cache for the current motor-ctrl ELF.

The cache is rebuilt only when the ELF's modification time or size changes.
Callers read this file instead of running nm on every lookup.

  sym_cache.py --ensure path/to/motor-ctrl.elf
  sym_cache.py path/to/motor-ctrl.elf g_motor_pwm_dt_on
"""
from __future__ import annotations

import glob
import subprocess
import sys
from pathlib import Path


def find_nm() -> str:
    hits = sorted(glob.glob(
        "/Applications/ArmGNUToolchain/*/arm-none-eabi/bin/arm-none-eabi-nm"))
    return hits[-1] if hits else "arm-none-eabi-nm"


def cache_path(elf: Path) -> Path:
    return elf.parent / "symbols.cache"


def stamp(elf: Path) -> str:
    st = elf.stat()
    return f"elf {int(st.st_mtime)} {st.st_size}"


def fresh(elf: Path) -> bool:
    path = cache_path(elf)
    if not path.is_file():
        return False
    first = path.read_text(encoding="utf-8").splitlines()[:1]
    return bool(first) and first[0] == stamp(elf)


def ensure(elf: Path) -> Path:
    elf = elf.resolve()
    if not elf.is_file():
        raise SystemExit(f"找不到程序文件 {elf}")
    path = cache_path(elf)
    if fresh(elf):
        return path
    out = subprocess.run([find_nm(), str(elf)], capture_output=True, text=True,
                         check=True).stdout
    lines = [stamp(elf)]
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 3:
            continue
        addr, kind, name = parts[0], parts[1], parts[2]
        if kind in ("b", "B", "d", "D") and len(addr) == 8:
            lines.append(f"{name} {addr}")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".cache.tmp")
    tmp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    tmp.replace(path)
    return path


def load(elf: Path) -> dict[str, int]:
    text = ensure(elf).read_text(encoding="utf-8").splitlines()
    table: dict[str, int] = {}
    for line in text[1:]:
        parts = line.split()
        if len(parts) == 2:
            table[parts[0]] = int(parts[1], 16)
    return table


def get(elf: Path, name: str) -> int:
    table = load(elf)
    if name not in table:
        raise SystemExit(f"符号 {name} 不在 {elf} 里")
    return table[name]


def main() -> int:
    if len(sys.argv) == 3 and sys.argv[1] == "--ensure":
        ensure(Path(sys.argv[2]))
        return 0
    if len(sys.argv) != 3:
        sys.stderr.write(
            "usage: sym_cache.py --ensure ELF\n"
            "       sym_cache.py ELF SYMBOL\n")
        return 2
    addr = get(Path(sys.argv[1]), sys.argv[2])
    sys.stdout.write(f"0x{addr:08x}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
