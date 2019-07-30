#!/bin/bash
roscore &
sleep 5

path=$1
echo "$path"

data=`rosnode list | grep JUK`

echo "Detect JUK nodes:"
num=""
num=$((num+1))
for i in $data
do
  echo "$num) $i " 
  rosnode kill $i
  num=$((num+1))
done

echo ""


C1="./$path/juk_navigation_control" 
$C1 & 

P1=$!

C2="./$path/juk-dji-core" 
$C2 & 

P2=$!

wait $P1 $P2

