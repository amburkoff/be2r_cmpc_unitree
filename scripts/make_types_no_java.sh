#!/bin/bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"
echo "Cur dir"
pwd

cd ${SCRIPT_DIR}/../src/lcm-types
# Clean
rm */*.jar
rm */*.java
rm */*.hpp
rm */*.class
rm */*.py
rm */*.pyc

# Make
lcm-gen -xp *.lcm
mkdir -p lcm_msgs
mv *.hpp lcm_msgs
# Python off
rm *.py

#mkdir -p python
#mv *.py python

FILES=$(ls */*.class)
echo ${FILES} > file_list.txt


echo -e "${GREEN} Done with LCM type generation${NC}"
