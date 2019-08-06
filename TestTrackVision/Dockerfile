FROM ubuntu:18.04

LABEL maintainer="Juan David Ríos <juanrios@kiwicampus.com>"

RUN apt-get update && apt-get install -y \
    git \
    cmake \
    tar \
    curl \
    wget \
    tar \
    bzip2 \
    build-essential && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python-dev \
    python3-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && python get-pip.py && rm -rf get-pip.py

RUN pip install Cython && python3 -m pip install Cython
RUN pip install numpy

RUN git clone --branch 3.3.0 --single-branch --depth 1 https://github.com/opencv/opencv
RUN git clone https://github.com/opencv/opencv_contrib.git 
RUN cd opencv_contrib && git checkout 3.3.0 & cd ..
RUN cd opencv && mkdir build && cd build && cmake -DWITH_CUDA=OFF -DWITH_LAPACK=ON -DBUILD_opencv_dnn=OFF -DBUILD_opencv_ml=OFF -DBUILD_opencv_objdetect=OFF -DBUILD_opencv_ts=OFF -DBUILD_PERF_TEST=OFF -DBUILD_PERF_TEST=OFF -DWITH_OPENCL=OFF -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules    .. && make -j3 && make install && rm -rf /opt/opencv
