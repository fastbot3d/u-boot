for file in `ls *.c`
#for file in `find`
do
cat $file | grep "$1"
if [ 0 -eq $? ]
then
echo $file
fi
done
