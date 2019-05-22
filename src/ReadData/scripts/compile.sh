#!/bin/bash

set -e

cd /home/qkgautier/workspace/tango/InfiniTAM/build
make -j4
cd /home/qkgautier/workspace/tango/ReadTangoRecords_build
make -j4

