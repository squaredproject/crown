#!/bin/sh

cd "$( dirname "$0" )"
mkdir -p build-tmp
javac -cp "code/*" -Xlint:unchecked -d build-tmp *.java
