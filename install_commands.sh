#!/bin/bash

sudo cp juk_launch /usr/local/bin/
sudo cp juk_stop /usr/local/bin/

sudo mkdir /usr/local/bin/juk/
sudo cp -r build/bin/* /usr/local/bin/juk/