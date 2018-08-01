rm -r data.csv
pid=$(pgrep asl_msckf)
while :
do
ps -p $pid -o %cpu=,%mem= >> data.csv
sleep 5
done
