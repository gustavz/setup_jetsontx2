#!/bin/bash
# copyright https://github.com/GustavZ
# usage: ./setup_jetsontx2.sh

DIR="$( cd "$( dirname "$0" )" && pwd )"
ME="$(whoami)"

echo -e "\e[32mWritten for Nvidia Jetson Tx2 running on Ubuntu 16.04 flashed with JetPack3.2 \e[0m"
read -p "> Hit any key to start..."

# make workspace directory
if [ ! -d ~/workspace ]
then
	mkdir ~/workspace
fi

# setup realsense camera
echo -e "installing librealsense"
if [ ! -e /usr/local/bin/realsense-viewer ]
then
	sudo apt-get update
	cd ~/workspace
	git clone https://github.com/freemanlo/librealsense
	cd librealsense
	echo -e "\e[34mATTENTION: In 'patch-utils.sh' comment line 138 'sudo rm \${tgt_ko}.bckup'"
	echo -e " change,save and close file to continue...\e[0m"
	gedit scripts/patch-utils.sh
	sudo apt-get install libusb-1.0-0-dev pkg-config cmake git libglfw3-dev qtcreator cmake-curses-gui build-essential libgtk-3-dev libssl-dev
	./scripts/patch-realsense-ubuntu-xenial-jetson-tx2.sh
	sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules && udevadm trigger
	mkdir build && cd build
	cmake ../ -DBUILD_EXAMPLES=true
	sudo make uninstall && make clean && make -j4 && sudo make install
	echo -e "\e[32mok: installed realsense sdk \e[0m"
else
	echo -e "\e[33mskip: realsense packages are already installed \e[0m"
fi

# setup ansible
echo -e "installing ansible and setup folder"
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
	echo -e "\e[32mok: set-up ansible \e[0m"
else
	echo -e "\e[33mskip: ansible is already installed \e[0m"
fi

# setup ssh-keys
echo -e "generating ssh-key and edit config file"
if [ ! -e ~/.ssh/bsh_sdd ]
then
	ssh-keygen -t rsa -b 4096
	cd ~/.ssh
	mv id_rsa.pub bsh_sdd.pub
	mv id_rsa bsh_sdd
	echo $'Host remote scr.bsh-sdd.com \nHostname scr.bsh-sdd.com \nUser git \nIdentityFile ~/.ssh/bsh_sdd \nIdentitiesOnly yes' >config
	echo -e "\e[34mATTENTION: copy public key stored in bsh_sdd.pub to the following link and hit Enter \e[0m"
	echo -e "https://scr.bsh-sdd.com/plugins/servlet/ssh/account/keys/add"
	gedit bsh_sdd.pub
	echo -e "\e[32mok: successfully generated ssh-keys \e[0m"
else
	echo -e "\e[33mskip: ssh-key already exists \e[0m"
fi

# setup BSH Structure
echo -e "setting up Robot Base"
if [ ! -e ~/BSH/myles/setup-myles/ansible/jetson.yml ]
then
	sudo apt install curl
	curl -L https://scr.bsh-sdd.com/projects/CIVBOTBA/repos/setup-robot-base/raw/setup.sh -u civ-robotbase:'n!io=Iq^7s(BOx' | bash -s $HOME/BSH myles CIVMYLES
	cp $DIR/stuff/jetson.yml ~/BSH/myles/setup-myles/ansible/jetson.yml
	cp $DIR/stuff/device.yml ~/BSH/myles/setup-myles/config/device.yml
	cp $DIR/stuff/tf_role.yml ~/BSH/setup-robot-base/ansible/roles/tf_object_detection/tasks/main.yml
	cp $DIR/stuff/sc_role.yml ~/BSH/setup-common-lib/ansible/roles/setup_common/tasks/main.yml
	echo -e "\e[32mok: added Robot Base config files \e[0m"
else
	echo -e "\e[33mskip: Robot Base already set-up \e[0m"
fi

# delete openCV
echo -e "Do you want to delete openCV which is necessary if flashed with JetPack3.2 (y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
    dpkg -l libopencv*
	sudo apt-get purge libopencv*
	echo -e "\e[32mok: removed all openCV packages \e[0m"
else
    echo -e "\e[33mskip: keeping openCV \e[0m"
fi

# ansible playbook
echo -e "Do you want to run ansible playbook. This will take a while.. (y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
	cd ~/BSH/myles/setup-myles/ansible/
	ansible-playbook -i hosts -v jetson.yml -K
	sudo chown -R $ME ~/BSH/*
	echo -e "\e[32mok: ran playbook \e[0m"
else
    echo -e "\e[33mskip: not running playbook \e[0m"
fi

# setup pre-build bazel
echo -e "adding prebuild bazel binary v0.8.1"
if [ ! -e /usr/local/bin/bazel ]
then
	wget -O $DIR/stuff/bazel "https://www.dropbox.com/s/wlsmzji2q95ojmi/bazel?dl=1?"
	cp $DIR/stuff/bazel /usr/local/bin/bazel
	echo -e "\e[32mok: added bazel \e[0m"
else
	echo -e "\e[33mskip: bazel already installed \e[0m"
fi

# install pre-build tensorflow
echo -e "installing pre-build tensorflow 1.4 for py2.7 with gpu support"
if [ ! -d /usr/local/lib/python2.7/dist-packages/tensorflow ]
then
	wget -O $DIR/stuff/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl "https://www.dropbox.com/s/27xpbyg5m4a6v13/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl?dl=1"
	sudo pip2 install $DIR/stuff/tensorflow-1.4.0-cp27-cp27mu-linux_aarch64.whl
	echo -e "\e[32mok:successfully installed tensorflow \e[0m"
else
	echo -e "\e[33mskip: tensorflow is already installed \e[0m"
fi

# setup object detection
echo -e "cloning real_time_detection repo"
if [ ! -d ~/workspace/realtime_object_detection ]
then
	cd ~/workspace
	git clone https://github.com/GustavZ/realtime_object_detection.git
	echo -e "\e[32mok: set-up object detection repo \e[0m"
else
	echo -e "\e[33mskip: object detection repo already exists \e[0m"
fi

# clean jetson from unneeded packages
echo -e "Do you want to clean Jetson from unneeded packages like libreoffice (y/n)? "
read answer
if echo "$answer" | grep -iq "^y"
then
	sudo apt purge libreoffice
	sudo autoremove
	echo -e "\e[32mok: cleaned jetson \e[0m"
else
    echo -e "\e[33mskip: keeping packages \e[0m"
fi

echo -e "\e[95mend: Jetson Tx2 successfully set-up \e[0m"
