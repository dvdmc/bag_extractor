#!/bin/bash

folder=
if [[ $1 = -f ]]; then
    shift
    if [[ $1 != */ ]]; then
        folder=$1/
    else
        folder=$1
    fi
    shift
else
    echo "Not folder selected, default=result"
    folder=result/
fi
### Script to create nested directories to store the output data from the bag.
while [[ ! -z $1 ]] && [[ $1 != *__* ]]; do
    mkdir -p ~/.ros/$folder$1
    echo "Created ~/.ros/$folder$1"
    shift
done

