#!/usr/bin/env python3
"""Add dummy texture coordinates to OBJ files that lack them.

Raylib 5.5's OBJ loader crashes in Release builds when models use the
"f v//vn" face format (no texture coordinate index). This script adds a
single "vt 0.0 0.0" line after the last vertex normal and rewrites all
faces from "f v//vn" to "f v/1/vn".

Usage:
    python3 scripts/fix_obj_texcoords.py models/my_model.obj
    python3 scripts/fix_obj_texcoords.py models/*.obj
"""
import re
import sys

def fix_obj(path):
    with open(path, 'r') as f:
        lines = f.readlines()

    has_vt = any(l.startswith('vt ') for l in lines)
    has_bare = any('//' in l for l in lines if l.startswith('f '))

    if has_vt and not has_bare:
        print(f"  SKIP {path} (already has texcoords)")
        return False

    # Find last vn line and insert vt after it
    last_vn = -1
    for i, line in enumerate(lines):
        if line.startswith('vn '):
            last_vn = i

    if last_vn == -1:
        print(f"  SKIP {path} (no vertex normals)")
        return False

    if not has_vt:
        lines.insert(last_vn + 1, 'vt 0.0 0.0\n')

    # Replace f v//vn with f v/1/vn
    fixed = 0
    for i, line in enumerate(lines):
        if line.startswith('f ') and '//' in line:
            lines[i] = re.sub(r'(\d+)//(\d+)', r'\1/1/\2', line)
            fixed += 1

    with open(path, 'w') as f:
        f.writelines(lines)

    print(f"  FIXED {path} ({fixed} faces updated)")
    return True


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    count = 0
    for path in sys.argv[1:]:
        if fix_obj(path):
            count += 1

    print(f"\n{count} file(s) fixed." if count else "\nNo files needed fixing.")
