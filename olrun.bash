#!/usr/bin/env bash
/etc/init.d/xinetd start
ln -s /usr/lib/x86_64-linux-gnu/libcudnn.so.6 /home/docker/ws/libcudnn.so
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/nvidia/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/docker/ws:$LD_LIBRARY_PATH
chmod 666 midi/data/chalmap
cat fixbashrc.txt >> /home/olrun/.bashrc
#
(cd midi; ./challenge.py 192.168.2.10 > ../chal.log) &
(cd midi; ./flyer2.py --ip 192.168.2.10 > ../flyer2.log) &
(cd vision; ./vision.py > ../vision.log) &
(cd posh; python launch.py olrun > ../posh.log) &
python -m SimpleHTTPServer 8000
echo "oops, webserver died.  Hang out for 3 hours"
sleep 3h
