#!/bin/bash

set -x

sudo apt-get update -qq
sudo apt-get install -qq -y python-pip graphviz
sudo pip install -r requires.txt

source ./build.sh
