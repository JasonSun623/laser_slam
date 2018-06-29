pubname=Noah_IPC_V3.1.0.0.22
workspace=~/catkin_ws
cd $workspace
echo "----------make start---------"
catkin_make install
chmod 777 ./install/share/robot_state_keeper/shell/robot.sh

echo "------general tar start------"
tar -czf $workspace/navigation.tar.gz ./install

echo "--------general md5----------"
result=$(md5sum ./navigation.tar.gz)
touch $workspace/md5sum.txt 
cat /dev/null > $workspace/md5sum.txt
echo $result >> $workspace/md5sum.txt

echo "--------general dir----------"
if [ ! -d $workspace/$pubname ]; then
   mkdir $workspace/$pubname
fi
mv navigation.tar.gz $workspace/$pubname
mv md5sum.txt $workspace/$pubname
cp ./src/robot_state_keeper/info/version_release.txt $workspace/$pubname
echo "---------pub end-------------"
