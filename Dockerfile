FROM ubuntu:18.04

# Packages required for Installing Ceres
RUN apt-get update && apt-get install build-essential libboost-dev cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev wget -y

# Install Ceres
WORKDIR /home/ceres
# Download the stable release of ceres
RUN wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && tar -xvf ceres-solver-2.1.0.tar.gz
# Compile and install the Ceres
WORKDIR /home/ceres/ceres-solver-2.1.0
RUN mkdir build && cd build && cmake .. && make -j4 install
# Remove unwanted files
RUN cd /home/ && rm -r ceres

# Installed packages required for IMU-TK
RUN apt-get install build-essential cmake libeigen3-dev libqt4-dev libqt4-opengl-dev freeglut3-dev gnuplot -y
# A workaround to allow for successful compilation of the IMU-TK in finding Eigen
RUN ln -s /usr/include/eigen3/Eigen/ /usr/include/Eigen
# Copy the IMU-TK source files and compile it
WORKDIR /home/imu_tk
COPY . /home/imu_tk
# Compile the tool
RUN mkdir build && cd build && cmake .. && make -j4
CMD ["bash"]