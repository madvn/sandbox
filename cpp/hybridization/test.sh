#!/usr/bin/env bash

# Exit if any command fails
set -e
set -o pipefail

ctest -V

# TODO export test results to reporting server
