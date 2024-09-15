#!/bin/bash
#This script will bump the priority of the running sbitx app
echo "Making sbitx app less nice"
sudo renice -n -20 $(pgrep sbitx)
