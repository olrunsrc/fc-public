#!/bin/sh
# This script was used to create the mason.jar file in this directory
# It is based on the MASON sources of MASON release 12.
# The script assumes that the files
# /tmp/mason/mason.tar.gz
# /tmp/mason/libraries.tar.gz
# exists, as downloadable from the MASON webpage

cd /tmp/mason

# upacking the files first
tar xzf mason.tar.gz
tar xzf libraries.tar.gz

# set classpath to the libraries
export CLASSPATH=/tmp/mason/libraries/itext-1.2.jar:/tmp/mason/libraries/jcommon-1.0.0.jar:/tmp/mason/libraries/jfreechart-1.0.1.jar:/tmp/mason/libraries/jmf.jar

# we are not interested in building the samle simulations -> remove them
cd mason
rm -R sim/app

# modify the Makefile such that
# - it does not attempt to build the sample simulations
# - it does not build the 3d components when creating a jar
# - it adds a missing file (sim/portrayal/inspector/propertyinspector.classes) to the jar
cp Makefile Makefile.orig
grep -v "sim/app/" Makefile.orig > Makefile
sed -e 's/jar: 3d/jar: all/' \
    -e 's#sim/display/simulation.classes#sim/display/simulation.classes sim/portrayal/inspector/propertyinspector.classes#' \
    -i Makefile

# indicate that all sample simulations have been removed
echo "ONLY" > sim/display/simulation.classes

# make mason.jar
make jar

echo "mason.jar can now be found in /tmp/mason/mason/"
