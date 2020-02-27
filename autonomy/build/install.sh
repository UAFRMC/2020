#!/bin/bash

sudo cp *.service /etc/systemd/system

sudo systemctl daemon-reload

sudo systemctl enable robot.service andretti.service localizer.service \
vision.service pathplanner.service

sudo systemctl start robot.service

sudo systemctl status robot.service andretti.service localizer.service \
vision.service pathplanner.service