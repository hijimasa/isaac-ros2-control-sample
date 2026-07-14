#!/bin/sh
exec /usr/bin/code \
  --user-data-dir /isaac-sim/.config/vscode-data \
  --extensions-dir /isaac-sim/.vscode/extensions \
  "$@"
