#!/bin/env bash

if [ ! -d $1 ]; then
mkdir $1
fi
mv outcmaes* $1
mv variablescmaes.mat $1
