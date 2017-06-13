#!/usr/bin/env bash
#
(cd midi; python -u challenge.py 192.168.2.10 > ../chal.txt 2>&1) &
(cd midi; python -u flyer2.py --ip 192.168.2.10 > ../flyer2.txt 2>&1) &
(cd vision; python -u vision.py > ../vision.txt 2>&1) &
(cd posh; python -u launch.py olrun > ../posh.txt 2>&1) &
(python -m SimpleHTTPServer 8000; \
echo "oops, webserver died.  Hang out for 3 hours") &
#sleep 3h
