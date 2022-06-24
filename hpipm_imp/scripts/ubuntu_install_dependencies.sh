sudo apt-get update && sudo apt-get -y upgrade

# essentials
sudo apt-get install -y build-essential git wget vim

# dependencies
sudo apt-get install -y libeigen3-dev
cd /home
sudo wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2-linux-x86_64.sh
sudo chmod +x cmake-3.23.2-linux-x86_64.sh
sudo mkdir /usr/bin/cmake_3_23_3
sudo ./cmake-3.23.2-linux-x86_64.sh --skip-license --prefix=/usr/bin/cmake_3_23_3
sudo rm cmake-3.23.2-linux-x86_64.sh

echo "export PATH='${PATH}:/usr/bin/cmake_3_23_3/bin'" >> ~/.bashrc

cd /home && \
        git clone https://github.com/mayataka/hpipm-cpp && \
        cd hpipm-cpp && \
        mkdir build && \
        cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make -j4 && \
        make install -j4

suoo apt install -y lsb-release gnupg2 curl && \
    echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add - && \
    sudo apt-get update && \
    sudo apt install -y robotpkg-py27-pinocchio && \
    echo 'export PATH=/opt/openrobots/bin:$PATH' >> ~/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:/usr/local/lib:/usr/local/lib/hpipm-cpp:$LD_LIBRARY_PATH' >> ~/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH' >> ~/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> ~/.bashrc
