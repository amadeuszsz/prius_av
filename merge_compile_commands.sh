#!/usr/bin/env bash

jq -s "map(.[])" ./build/*/compile_commands.json > compile_commands.json