#!/bin/sh
export DISPLAY=:0
export BEACON=10.10.10.100
DIR="$( dirname $0)"
#DIR="$( cd "$( dirname "${BASH_SOURCE[0]}")">/dev/null 2>&1)"
"${DIR}"/backend "$@"
