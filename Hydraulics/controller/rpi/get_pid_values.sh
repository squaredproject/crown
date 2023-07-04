#!/bin/bash
if [ "$#" -lt 1 ]
then
  echo "must have 1 argument (the tower)"
  exit
fi
echo curl http://10.0.0.2:5050/crown/sculpture/towers/$1/PID
curl http://10.0.0.2:5050/crown/sculpture/towers/$1/PID

