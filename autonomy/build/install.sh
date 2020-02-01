#!/bin/bash

sudo cp *.service /etc/systemd/system
sudo systemctl daemon-reload
sudo systemctl enable robot.service
sudo systemctl restart robot.service