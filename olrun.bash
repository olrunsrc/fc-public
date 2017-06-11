#!/usr/bin/env bash
/etc/init.d/xinetd start
ln -s /usr/lib/x86_64-linux-gnu/libcudnn.so.6 /home/docker/ws/libcudnn.so
export LD_LIBRARY_PATH=/usr/local/nvidia/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/nvidia/lib64:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/home/docker/ws:$LD_LIBRARY_PATH
chmod 666 /midi/data/chalmap
cat fixbashrc.txt >> /home/olrun/.bashrc
#
echo "hanging out for 3 hours"
sleep 3h
