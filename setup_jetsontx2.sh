#!/bin/bash
# copyright https://github.com/GustavZ
# usage: ./setup_jetsontx2.sh

DIR="$( cd "$( dirname "$0" )" && pwd )"
ME="$(whoami)"

echo -e "\e[32mWritten for Nvidia Jetson Tx2 running on Ubuntu 16.04 flashed with JetPack3.2 \e[0m"
read -p "> Hit any key to start..."

# set Jetson into power mode
echo -e "> Set Jetson to Power Mode"
sudo nvpmodel -m 0
sudo ~/jetson_clocks.sh

# remove apt update bug
echo -e "> Remove apt update bug"
sudo rm /etc/apt/sources.list.d/cuda-9-0-local.list
sudo rm /etc/apt/sources.list.d/nv-tensorrt-ga-cuda9.0-trt3.0.4-20180208.list

# setup realsense camera
echo -e "> Installing librealsense (this must be the first thing to do, idk why, ask intel)"
if [ ! -e /usr/local/bin/realsense-viewer ]
then
	sudo apt-get update
	cd 
	git clone https://github.com/freemanlo/librealsense
	cd librealsense
	echo -e "\e[34mATTENTION: In 'patch-utils.sh' comment line 138 'sudo rm \${tgt_ko}.bckup'"
	echo -e " Change, save and close file \e[0m"
	gedit scripts/patch-utils.sh
	read -p "> Hit any key to continue"
	sudo apt-get install libusb-1.0-0-dev pkg-config cmake git libglfw3-dev qtcreator cmake-curses-gui build-essential libgtk-3-dev libssl-dev
	./scripts/patch-realsense-ubuntu-xenial-jetson-tx2.sh
	sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	mkdir build && cd build
	cmake ../ -DBUILD_EXAMPLES=true
	sudo make uninstall && make clean && make -j4 && sudo make install
	echo -e "\e[32mok: Installed realsense sdk \e[0m"
else
	echo -e "\e[33mskip: Realsense packages are already installed \e[0m"
fi

# setup ansible
echo -e "> Installing ansible and setup folder"
if [ ! -e /usr/bin/ansible ]
then
	sudo apt-get update
	sudo apt-get install software-properties-common
	sudo apt-add-repository ppa:ansible/ansible
	sudo apt-get update
	sudo apt-get install ansible
	sudo mkdir -p /var/log/ansible/
	sudo chown -R $ME:adm /var/log/ansible/
	sudo chmod 2750 /var/log/ansible/
	echo -e "\e[32mok: Set-up ansible \e[0m"
else
	echo -e "\e[33mskip: Ansible is already installed \e[0m"
fi

# setup ssh-keys
echo -e "> Generating ssh-key and edit config file"
if [ ! -e ~/.ssh/bsh_sdd ]
then
	ssh-keygen -t rsa -b 4096
	cd ~/.ssh
	mv id_rsa.pub bsh_sdd.pub
	mv id_rsa bsh_sdd
	echo $'Host remote scr.bsh-sdd.com \nHostname scr.bsh-sdd.com \nUser git \nIdentityFile ~/.ssh/bsh_sdd \nIdentitiesOnly yes' >config
	echo -e "\e[34mATTENTION: Copy public key stored in bsh_sdd.pub to the following link \e[0m"
	echo -e "https://scr.bsh-sdd.com/plugins/servlet/ssh/account/keys/add"
	gedit bsh_sdd.pub
	read -p "> Hit any key to continue"
	echo -e "\e[32mok: successfully generated ssh-keys \e[0m"
else
	echo -e "\e[33mskip: ssh-key already exists \e[0m"
fi

# setup BSH Structure
echo -e "> Setting up Robot Base"
if [ ! -e ~/BSH/myles/setup-myles/ansible/jetson.yml ]
then
	sudo apt install curl
	curl -L https://scr.bsh-sdd.com/projects/CIVBOTBA/repos/setup-robot-base/raw/setup.sh -u civ-robotbase:'n!io=Iq^7s(BOx' | bash -s $HOME/BSH myles CIVMYLES
	cp $DIR/stuff/jetson.yml ~/BSH/myles/setup-myles/ansible/jetson.yml
	cp $DIR/stuff/device.yml ~/BSH/myles/setup-myles/config/device.yml
	cp $DIR/stuff/tf_role.yml ~/BSH/setup-robot-base/ansible/roles/tf_object_detection/tasks/main.yml
	cp $DIR/stuff/sc_role.yml ~/BSH/setup-common-lib/ansible/roles/setup_common/tasks/main.yml
	echo -e "\e[32mok: Added Robot Base config files \e[0m"
else
	echo -e "\e[33mskip: Robot Base already set-up \e[0m"
fi

# delete openCV
echo -e "> Do you want to delete openCV which is necessary if flashed with JetPack3.2 (y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
    	dpkg -l libopencv*
	sudo apt-get purge libopencv*
	echo -e "\e[32mok: Removed all openCV packages \e[0m"
else
    echo -e "\e[33mskip: Keeping openCV \e[0m"
fi

# ansible playbook
echo -e "> Do you want to run ansible playbook. This will take a while.. (y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
	cd ~/BSH/myles/setup-myles/ansible/
	ansible-playbook -i hosts -v jetson.yml -K
	sudo chown -R $ME ~/BSH/*
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	echo -e "\e[32mok: Ran playbook \e[0m"
else
    echo -e "\e[33mskip: Not running playbook \e[0m"
fi

# setup pre-build bazel
echo -e "> Adding prebuild bazel binary v0.8.1"
if [ ! -e /usr/local/bin/bazel ]
then
	wget -O /usr/local/bin/bazel "https://www.dropbox.com/s/wlsmzji2q95ojmi/bazel?dl=1?"
	sudo cp $DIR/stuff/bazel /usr/local/bin/bazel
	sudo chmod 755 /usr/local/bin/bazel
	echo -e "\e[32mok: Added bazel \e[0m"
else
	echo -e "\e[33mskip: Bazel already installed \e[0m"
fi

# install pre-build tensorflow
echo -e "> Installing pre-build tensorflow 1.4 for py2.7 with gpu support"
if [ ! -d /usr/local/lib/python2.7/dist-packages/tensorflow ]
then
	wget -O $DIR/stuff/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl "https://www.dropbox.com/s/27xpbyg5m4a6v13/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl?dl=1"
	sudo pip2 install $DIR/stuff/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl
	echo -e "\e[32mok:Successfully installed tensorflow \e[0m"
else
	echo -e "\e[33mskip: Tensorflow is already installed \e[0m"
fi

# setup object detection
echo -e "> Cloning real_time_detection repo"
if [ ! -d ~/realtime_object_detection ]
then
	cd
	git clone https://github.com/GustavZ/realtime_object_detection.git
	echo -e "\e[32mok: Set-up object detection repo \e[0m"
else
	echo -e "\e[33mskip: Object detection repo already exists \e[0m"
fi

# setup Realsense for ROS
echo -e "Setup ROS for Realsense"
if [ ! -d ~/realsense/catkin_ws ]
then
	mkdir -p ~/realsense/catkin_ws/src
	cd ~/realsense/catkin_ws/src/
	git clone https://github.com/intel-ros/realsense.git
	catkin_init_workspace 
	cd ..
	catkin_make clean
	catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
	catkin_make install
	echo "source ~/wrealsense/catkin_ws/devel/setup.bash" >> ~/.bashrc
	source ~/.bashrc
	echo -e "\e[32mok: Build Realsense catkin_ws  \e[0m"
else
	echo -e "\e[33mskip: Realsense catkin_ws already exists \e[0m"
fi

# clean jetson from unneeded packages
echo -e "Do you want to clean Jetson from unneeded packages(y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
	sudo rm $DIR/stuff/bazel
	sudo rm $DIR/stuff/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl
	sudo apt purge libreoffice
	sudo rm -rf /usr/src/public_release
	sudo autoremove
	echo -e "\e[32mok: Cleaned jetson \e[0m"
else
    echo -e "\e[33mskip: Keeping packages \e[0m"
fi

echo -e "\e[95mend: Jetson Tx2 successfully set-up \e[0m"
