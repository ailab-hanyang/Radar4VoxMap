#!/bin/bash
#@brife Build and install tool for g2o

# install dependency of g2o
PACKAGE_NAME="libsuitesparse-dev"
if ! dpkg -s $PACKAGE_NAME &> /dev/null; then
    echo "Install $PACKAGE_NAME"
    echo "$PACKAGE_NAME is NOT installed."
    (eval "sudo apt install libsuitesparse-dev -y")
    if [ "$?" != "0" ] ; then
        echo "g2o dependency install filed"
        echo "please update apt (apt update)"
        exit 1
    fi
    echo "$PACKAGE_NAME is installed."    
fi


# make biuld folder
(eval "mkdir build")
if [ "$?" != "0" ] ; then
    echo "g2o build incomplete in progress {mkdir build}"
    echo "g2o already builded"
    exit 0
fi

# move to build folder
cd build
if [ "$?" != "0" ] ; then
    echo "g2o build incomplete in progress {cd ./build}"
    exit 1
fi

# generate make files
(cmake -DBUILD_CSPARSE_PLUGIN=ON ..)
if [ "$?" != "0" ] ; then
    echo "g2o build incomplete in progress {cmake ..}"
    cd ..
    ./uninstall_g2o.sh
    exit 1
fi

# build g2o
(make -j8)
if [ "$?" != "0" ] ; then
    echo "g2o build incomplete in progress {make -j8}"
    cd ..
    ./uninstall_g2o.sh
    exit 1
fi

# install g2o your local
(sudo make install)
if [ "$?" != "0" ] ; then
    echo "g2o build incomplete in progress {sudo make install}"
    cd ..
    ./uninstall_g2o.sh
    exit 1
fi

# Library cache update
echo "Updating library cache..."
sudo ldconfig

# Add /usr/local/lib to LD_LIBRARY_PATH for current session
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
echo "Added /usr/local/lib to LD_LIBRARY_PATH for current session"

# Permanently set LD_LIBRARY_PATH
if ! grep -q "export LD_LIBRARY_PATH=.*:/usr/local/lib" ~/.bashrc; then
    echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib' >> ~/.bashrc
    echo "Added /usr/local/lib to LD_LIBRARY_PATH permanently in ~/.bashrc"
    echo "Please run 'source ~/.bashrc' or restart your terminal to apply changes"
fi

# Add configuration to /etc/ld.so.conf.d (system-wide)
if [ ! -f "/etc/ld.so.conf.d/g2o.conf" ]; then
    echo "Creating /etc/ld.so.conf.d/g2o.conf..."
    echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/g2o.conf > /dev/null
    sudo ldconfig
    echo "Added /usr/local/lib to system library path"
fi

echo "g2o installation completed successfully!"