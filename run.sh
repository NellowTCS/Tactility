#!/bin/sh
if [[ -v ESP_IDF_VERSION ]]; then
  idf.py flash monitor $@
else
  cmake -S ./ -B build-sim
  cmake --build build-sim
  build-sim/app-sim/app-sim
fi

