FROM --platform=linux/x86_64 ubuntu:18.04

RUN apt-get update -qq && apt-get -y -qq upgrade

# essentials
RUN apt-get install -y -qq build-essential git wget vim

# dependencies
RUN apt-get install -y -qq libeigen3-dev
RUN cd /home && \
        wget https://github.com/Kitware/CMake/releases/download/v3.23.2/cmake-3.23.2-linux-x86_64.sh && \
        chmod +x cmake-3.23.2-linux-x86_64.sh && \
        mkdir /usr/bin/cmake && \
        ./cmake-3.23.2-linux-x86_64.sh --skip-license --prefix=/usr/bin/cmake && \
        rm cmake-3.23.2-linux-x86_64.sh

ENV PATH="/usr/bin/cmake/bin:${PATH}"

RUN cd /home && \
        git clone https://github.com/mayataka/hpipm-cpp && \
        cd hpipm-cpp && \
        mkdir build && \
        cd build && \
        cmake .. -DCMAKE_BUILD_TYPE=Release && \
        make -j4 && \
        make install -j4

RUN apt install -qqy lsb-release gnupg2 curl && \
    echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
    curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add - && \
    apt-get update && \
    apt install -qqy robotpkg-py27-pinocchio && \
    echo 'export PATH=/opt/openrobots/bin:$PATH' >> /root/.bashrc && \
    echo 'export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH' >> /root/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/opt/openrobots/lib:/usr/local/lib:/usr/local/lib/hpipm-cpp:$LD_LIBRARY_PATH' >> /root/.bashrc && \
    echo 'export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages:$PYTHONPATH' >> /root/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH' >> /root/.bashrc
