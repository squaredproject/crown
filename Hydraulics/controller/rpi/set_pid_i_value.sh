#!/bin/bash
if [ "$#" -lt 3 ]
then
  echo "must have 3 arguments"
  exit
fi
echo curl -X PUT http://10.0.0.2:5050/crown/sculpture/towers/$1/PID -d joint$2_I=$3
curl -X PUT http://10.0.0.2:5050/crown/sculpture/towers/$1/PID -d joint$2_I="$3"

