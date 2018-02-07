#!/bin/bash

toexec=$(git commit 2>&1 >/dev/null | sed -n -e 's/^.*bad formatting, please run "make format" or "\(.*\)"$/\1/p')
echo "The following commands will be executed:"
echo $toexec
eval $toexec
