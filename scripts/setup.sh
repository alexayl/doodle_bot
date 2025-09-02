#!/bin/bash
# This script automates the setup of the VS Code workspace for this project.

west update

# Move vsc config files to workspace directory
mkdir -p ../../.vscode
shopt -s nullglob
mv .vscode/* ../../.vscode

# Update packages
cd doodle-bot
west packages pip --install