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