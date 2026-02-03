#!/usr/bin/env bash

echo Start script
sleep 3
echo 3 seconds passed, start recording
rosbag record --all --output-name=test1/bag6.bag
