#!/bin/bash
rm -rf build
./cmake/arkcmake/autobuild.py 5 -c"-DBOARD=mega"
./cmake/arkcmake/autobuild.py 5 -c"-DBOARD=mega2560"
