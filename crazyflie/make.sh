#!/bin/bash

make cf2_defconfig && make links && make copilot && make -j 32
