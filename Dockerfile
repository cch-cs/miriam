
FROM ubuntu:xenial

# REQUIREMENTS
RUN apt-get update

# bonmin
RUN apt-get install -y libblas3 liblapack3 wget
#####

# roadmap
# RUN apt-get install -y sudo cmake libcgal-dev libcgal-qt5-dev libeigen3-dev libmgl-dev mathgl qt5-default libboost-filesystem-dev
# RUN wget http://ompl.kavrakilab.org/core/install-ompl-ubuntu.sh
# RUN sh install-ompl-ubuntu.sh
#####

# python
RUN apt-get install -y python2.7 python-pip python-tk git iputils-ping
COPY requirements.txt /
RUN mkdir /miriam/
RUN pip install -r requirements.txt
RUN python -m pip install -U pip setuptools
RUN python -m pip install matplotlib
#####

# bonmin
RUN wget https://www.coin-or.org/download/binary/CoinAll/CoinAll-1.6.0-linux-x86_64-gcc4.4.5.tgz
RUN tar -xzf CoinAll-1.6.0-linux-x86_64-gcc4.4.5.tgz
RUN ln -s /usr/lib/liblapack.so.3 /lib/liblapack.so.3gf
RUN ln -s /usr/lib/libblas.so.3 /lib/libblas.so.3gf
#####

# cobra
RUN apt-get update
RUN apt-get -y install git gcc make libboost-all-dev g++

RUN git clone https://github.com/ct2034/cobra.git
WORKDIR cobra/COBRA
RUN make all
RUN cp cobra /miriam/
RUN chmod +x /miriam/cobra
ENV COBRA_BIN='/miriam/cobra'
#####

COPY . /miriam/

# roadmap
# RUN mkdir -p /miriam/roadmaps/prm-star/prm-star/build
# WORKDIR /miriam/roadmaps/prm-star/prm-star/build
# RUN cmake ..
# RUN make all
#####

FROM ros:kinetic-ros-base

RUN rm -f /etc/apt/sources.list.d/ros-latest.list/binary-amd64/Packages
RUN apt-get update
RUN apt-get install -y ros-kinetic-navigation 
RUN apt-get install -y ros-kinetic-rviz
RUN apt-get install -y ros-kinetic-rviz-plugin-tutorials
RUN rm -rf /var/lib/apt/lists/
RUN touch ./root/bashrc && echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc


# python
WORKDIR /miriam/real_robot_controller/scripts
ENV PYTHONPATH /miriam
CMD ["echo", "Hello World...!"]
#CMD ["py.test","/miriam/.","-vs"]
#CMD ["python","eval_scenarios.py"]
CMD ["rosrun","job_publisher_1.py"]
#CMD ["roslaunch","real_robot_controller gazebo_multiplanner.launch" ]
#####

