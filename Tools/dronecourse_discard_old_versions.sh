#!/bin/bash

ls dist/ | tac | uniq -s 13 -w 6 | sed 's/^\(.*\)$/ -not -name \"\1\"/' | paste -s | sed "s/\t/ -and /g;s/$/ -delete/" | xargs find dist/ -type f
