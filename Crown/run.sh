#!/bin/sh

cd "$( dirname "$0" )"
echo java -Xms256m -Xmx1g -cp "build-tmp:code/*" RunHeadless "${PWD}"
exec java -Xms256m -Xmx1g -cp "build/classes/java/main:code/*" RunHeadless "${PWD}"
