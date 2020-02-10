#!/usr/bin/env bash

filename=ball
i=1

while [ -f ~/Videos/$filename-$i.h264 ]; do
    i++
done

raspivid -t 5000 -w 640 -h 480 -awb sun -fps 30 -o ~/Videos/$filename-$i.h264
