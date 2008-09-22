#! /bin/bash
REPO=http://dl.allstarlink.org
echo "****** Phase 1 post install ******"
sleep 1
echo "Importing centos 5 gpg key..."
rpm --import http://mirror.centos.org/centos/RPM-GPG-KEY-CentOS-5
if [ $? -gt 0 ]
then
	echo "Failure: Unable to retrieve GPG KEY from mirror.centos.org"
	exit 255
fi
echo "Updating system..."
yum -y update 
if [ $? -gt 0 ]
then
	echo "Failure: Unable to update Centos to latest binaries"
	exit 255
fi

echo "Installing ntp..."
yum -y install ntp
if [ $? -gt 0 ]
then
        echo "Failure: Unable to install ntp"
        exit 255
fi

echo "Installing Development Tools..."
yum -y groupinstall "Development Tools"
if [ $? -gt 0 ]
then
	echo "Failure: Unable install development tools"
	exit 255
fi
echo "Installing Devel Headers for Libraries..."
yum -y install kernel-devel alsa-lib-devel ncurses-devel libusb-devel newt-devel
if [ $? -gt 0 ]
then
	echo "Failure: Unable install development library headers"
	exit 255
fi
cp -f /etc/rc.d/rc.local.orig /etc/rc.d/rc.local
cat <<EOF >>/etc/rc.d/rc.local
(cd /etc/rc.d; rm -f phase2.sh; wget -q $REPO/installcd/phase2.sh) 
if [ -e /etc/rc.d/phase2.sh ]
then
chmod 755 /etc/rc.d/phase2.sh
/etc/rc.d/phase2.sh
else
echo "Unable to download post install script phase2.sh!"
echo "Installation aborted"
sleep 3
fi
EOF
sync
reboot



