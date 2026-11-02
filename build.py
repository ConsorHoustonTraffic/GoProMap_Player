#!/usr/bin/env python3
"""
Build a standalone GoPro Map Player that runs WITHOUT Python on the target laptop.

PyInstaller builds for the OS it runs on:
  Windows -> dist/GoProMapPlayer/GoProMapPlayer.exe   (+ dist/GoProMapPlayer-Windows-*.zip)
  macOS   -> dist/GoProMapPlayer.app                  (+ dist/GoProMapPlayer-Darwin-*.zip)
  Linux   -> dist/GoProMapPlayer/GoProMapPlayer       (+ dist/GoProMapPlayer-Linux-*.zip)

Usually run via build_windows.bat or build_mac_linux.sh, which set up the environment first.
"""
import os
import platform
import shutil
import subprocess
import sys
import urllib.request

APP = 'GoProMapPlayer'
HERE = os.path.dirname(os.path.abspath(__file__))
VENDOR = os.path.join(HERE, 'vendor')
LEAFLET = {
    'leaflet.js': 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
    'leaflet.css': 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
}

# Python modules the app never uses (keeps the bundle smaller)
EXCLUDES = ['tkinter', 'unittest', 'pydoc', 'test',
            'PySide6.Qt3DCore', 'PySide6.Qt3DRender', 'PySide6.QtCharts',
            'PySide6.QtDataVisualization', 'PySide6.QtGraphs', 'PySide6.QtQuick3D',
            'PySide6.QtBluetooth', 'PySide6.QtSensors', 'PySide6.QtSerialPort',
            'PySide6.QtSql', 'PySide6.QtTest', 'PySide6.QtDesigner', 'PySide6.QtHelp',
            'PySide6.QtTextToSpeech', 'PySide6.QtRemoteObjects', 'PySide6.QtScxml']


def fetch_leaflet():
    os.makedirs(VENDOR, exist_ok=True)
    for name, url in LEAFLET.items():
        dst = os.path.join(VENDOR, name)
        if os.path.isfile(dst) and os.path.getsize(dst) > 1000:
            continue
        try:
            print('Downloading', url)
            urllib.request.urlretrieve(url, dst)
        except Exception as e:  # noqa: BLE001
            print('  WARNING: could not download %s (%s); the app will load it from the web instead.' % (name, e))


def main():
    os.chdir(HERE)
    fetch_leaflet()
    for d in ('build', 'dist'):
        shutil.rmtree(d, ignore_errors=True)

    args = [sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean',
            '--windowed',                     # no console window; .app bundle on macOS
            '--name', APP]
    if os.path.isdir(VENDOR) and os.listdir(VENDOR):
        args += ['--add-data', 'vendor' + os.pathsep + 'vendor']
    if platform.system() == 'Darwin':
        args += ['--osx-bundle-identifier', 'local.gopromapplayer']
    for m in EXCLUDES:
        args += ['--exclude-module', m]
    args.append('gopro_map_player.py')

    print(' '.join(args))
    subprocess.check_call(args)

    # Zip the result so it can be copied/shared as one file
    system = platform.system()
    archive = os.path.join('dist', '%s-%s-%s.zip' % (APP, system, platform.machine()))
    if system == 'Darwin':
        # ditto keeps the symlinks / permissions macOS frameworks need
        subprocess.check_call(['ditto', '-c', '-k', '--sequesterRsrc', '--keepParent',
                               os.path.join('dist', APP + '.app'), archive])
    else:
        shutil.make_archive(archive[:-4], 'zip', 'dist', APP)

    print('\nDone!')
    print('  App folder :', os.path.abspath(os.path.join('dist', APP + ('.app' if system == 'Darwin' else ''))))
    print('  Zip to share:', os.path.abspath(archive))


if __name__ == '__main__':
    main()
