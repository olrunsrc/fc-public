#!/usr/bin/env bash
/etc/init.d/xinetd start
ln -s /usr/lib/x86_64-linux-gnu/libcudnn.so.6 /home/docker/ws/libcudnn.so
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/nvidia/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/docker/ws:$LD_LIBRARY_PATH
cat fixbashrc >> /home/olrun/.bashrc
rm -f *.txt
#does this work every time?
source /home/docker/ws/install/setup.bash
#
(cd midi; python -u challenge.py 192.168.2.10 > ../chal.txt 2>&1) &
(cd midi; python -u flyer3.py --ip 192.168.2.10 > ../flyer3.txt 2>&1) &
(cd vision; python -u vision.py > ../vision.txt 2>&1) &
(cd posh; python -u launch.py olrun > ../posh.txt 2>&1) &
(python -m SimpleHTTPServer 8000; echo "oops, webserver died.") &
echo "Hang out for 3 hours"
sleep 3h
