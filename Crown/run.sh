#!/bin/sh

cd "$( dirname "$0" )"
exec java -Xms256m -Xmx1g -cp "build-tmp:code/*" RunHeadless "${PWD}"
