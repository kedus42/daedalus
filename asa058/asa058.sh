#! /bin/bash
#
cp asa058.hpp /$HOME/usr/include/asa058
#
g++ -c -Wall -I /$HOME/usr/include/asa058 asa058.cpp
if [ $? -ne 0 ]; then
  echo "Compile error."
  exit
fi
#
#mv asa058.o ~/libcpp/asa058.o
#
echo "Normal end of execution."
