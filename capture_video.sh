#!/usr/bin/env bash

FLY_MODE=air

if [ -n "$1" ]; then
	echo "Using fly mode suffix:" $1
	FLY_MODE=$1
fi

i=1

while [ -f "$HOME/Videos/$FLY_MODE-v$i.mp4" ]; do
	i=$((i + 1))
done

FILE_H264=$HOME/Videos/$FLY_MODE-v$i.h264
FILE_MP4=$HOME/Videos/$FLY_MODE-v$i.mp4

if [ -n "$AWB" ]; then
	raspivid -t 5000 -w 640 -h 480 -awb sun -fps 30 -o $FILE_H264
else
	raspivid -t 5000 -w 640 -h 480          -fps 30 -o $FILE_H264
fi

MP4Box -add $FILE_H264 -fps 30 $FILE_MP4

rm $FILE_H264
