#!/usr/bin/env python3
"""Fetch checksummed backend sources; revisions are defined only by versions.json."""
import hashlib
import io
import json
from pathlib import Path, PurePosixPath
import re
import shutil
import tarfile
import urllib.request


def main():
    base = Path(__file__).resolve().parents[1] / 'third_party/backend'
    base.mkdir(parents=True, exist_ok=True)
    (base / 'COLCON_IGNORE').touch()
    locked = json.loads((base / 'versions.json').read_text())
    for name, entry in locked.items():
        revision = entry['commit']
        if not re.fullmatch(r'[0-9a-f]{40}', revision):
            raise ValueError(f'{name}: expected a full commit hash')
        target = base / 'sources' / name
        marker = target / '.superm_revision'
        if marker.exists() and marker.read_text().strip() == revision:
            continue
        url = f"https://codeload.github.com/{entry['repository']}/tar.gz/{revision}"
        with urllib.request.urlopen(url, timeout=120) as response:
            data = response.read()
        if hashlib.sha256(data).hexdigest() != entry['archive_sha256']:
            raise RuntimeError(f'{name}: archive checksum mismatch')
        temporary = base / 'sources' / f'{name}.download'
        if temporary.exists():
            shutil.rmtree(temporary)
        temporary.mkdir(parents=True)
        with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
            for member in archive.getmembers():
                parts = PurePosixPath(member.name).parts[1:]
                if not parts:
                    continue
                if '..' in parts or member.issym() or member.islnk():
                    raise RuntimeError(f'{name}: unsafe archive member')
                member.name = '/'.join(parts)
                archive.extract(member, temporary)
        (temporary / '.superm_revision').write_text(revision + '\n')
        if target.exists():
            shutil.rmtree(target)
        temporary.rename(target)
        print(f'{name}: {revision}', flush=True)

    # ROBIN's defaults fetch xenium from a floating branch. Use the verified
    # local sources for all mandatory nested dependencies instead.
    external = base / 'sources/robin/cmake/DownloadExternal.cmake'
    code = external.read_text()
    for dependency in ('pmc', 'xenium'):
        pattern = rf'download_project\(PROJ {dependency}\s+.*?\)'
        replacement = (f'set({dependency}_SOURCE_DIR "${{SUPERM_DEP_SOURCES}}/{dependency}")\n'
                       f'  set({dependency}_BINARY_DIR "${{CMAKE_BINARY_DIR}}/{dependency}-build")')
        code = re.sub(pattern, lambda _: replacement, code, flags=re.DOTALL)
    external.write_text(code)
    licenses = base / 'licenses'
    licenses.mkdir(exist_ok=True)
    for name in locked:
        source = base / 'sources' / name
        for filename in ('LICENSE', 'LICENSE.txt', 'LICENSE.md', 'COPYING'):
            if (source / filename).is_file():
                content = (source / filename).read_text()
                (licenses / f'{name}_{filename}').write_text(
                    '\n'.join(line.rstrip() for line in content.splitlines()) + '\n')
                break


if __name__ == '__main__':
    main()
