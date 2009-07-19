#! /bin/bash
HWTYPE=usbradio
#HWTYPE=pciradio
REPO=$(cat /etc/rc.d/acidrepo)
SSHDCONF=/etc/ssh/sshd_config
SSHDPORT=222
TMP=/tmp
INSTALLOG=/root/acid-install.log

function log {
        local tstamp=$(/bin/date)
        echo "$tstamp:$1" >>$INSTALLOG
}

function logecho {
        echo "$1"
        log "$1"
}

function die {
        logecho "Fatal error: $1"
        exit 255
}

function promptyn
{
        echo -n "$1 [y/N]? "
        read ANSWER
        if [ ! -z $ANSWER ]
        then
                if [ $ANSWER = Y ] || [ $ANSWER = y ]
                then
                        ANSWER=Y
                else
                        ANSWER=N
                fi
        else
                ANSWER=N
        fi
}

logecho "****** Phase 2 post install ******"
sleep 1

DESTDIR=/usr/src
cd $DESTDIR

logecho "Getting asterisk install script from $REPO..."
wget -q $REPO/installcd/astinstall.sh -O /etc/rc.d/astinstall.sh
if [ $? -gt 0 ]
then
        die "Unable to download Asterisk install script"
else
	chmod 755 /etc/rc.d/astinstall.sh
fi

logecho "Getting files.tar.gz from $REPO..."

wget -q $REPO/installcd/files.tar.gz -O $DESTDIR/files.tar.gz

if [ $? -gt 0 ]
then
	die "Unable to download files.tar.gz"
fi

if [ -e /etc/rc.d/astinstall.sh ]
then
	/etc/rc.d/astinstall.sh 
	if [ $? -gt 0 ]
	then
		die "Unable to install Asterisk!"
	fi
else
	die "exec of /etc/rc.d/astinstall.sh failed!"
fi

cd $DESTDIR

logecho "Setting up config..."
rm -rf /etc/asterisk
mkdir -p /etc/asterisk

cp configs/*.conf /etc/asterisk
if [ $? -gt 0 ]
then
	die "Unable to copy configs 1"
fi
cp configs/$HWTYPE/*.conf /etc/asterisk
if [ $? -gt 0 ]
then
	die "Unable to copy configs 2"
fi
mv /etc/asterisk/zaptel.conf /etc
if [ $? -gt 0 ]
then
	die "Unable to copy configs 3"
fi

logecho "Updating local script(s)..."
cp $DESTDIR/allstar/rc.updatenodelist /etc/rc.d/rc.updatenodelist
chmod +x /etc/rc.d/rc.updatenodelist
if [ $? -gt 0 ]
then
	die "Unable to chmod script"
fi

mv -f /etc/rc.d/rc.local.orig /etc/rc.d/rc.local; sync
egrep updatenodelist /etc/rc.d/rc.local
if [ $? -gt 0 ]
then
	echo "/etc/rc.d/rc.updatenodelist &" >> /etc/rc.d/rc.local
fi

mkdir -p /root/acid
logecho "Getting setup scripts..."
wget -q $REPO/installcd/nodesetup.sh -O /root/acid/nodesetup.sh
wget -q $REPO/installcd/astupd.sh -O /root/acid/astupd.sh
wget -q $REPO/installcd/irlpsetup.sh -O /root/acid/irlpsetup.sh
wget -q $REPO/installcd/astres.sh -O /root/acid/astres.sh
wget -q $REPO/installcd/backup.sh -O /root/acid/backup.sh
wget -q $REPO/installcd/restore.sh -O /root/acid/restore.sh
wget -q $REPO/installcd/astup.sh -O /root/acid/astup.sh
wget -q $REPO/installcd/astdn.sh -O /root/acid/astdn.sh
wget -q $REPO/installcd/acidvers -O /root/acid/acidvers
wget -q $REPO/installcd/savenode.conf -O /etc/asterisk/savenode.conf
wget -q $REPO/installcd/savenode.sh -O /root/acid/savenode.sh

chmod 770 /root/acid/*.sh

sync

logecho "Turning off unused services..."
chkconfig acpid off
chkconfig apmd off
chkconfig autofs off
chkconfig bluetooth off
chkconfig cpuspeed off
chkconfig cups off
chkconfig gpm off
chkconfig haldaemon off
chkconfig hidd off
chkconfig lvm2-monitor off
chkconfig mcstrans off
chkconfig mdmonitor off
chkconfig netfs off
chkconfig nfslock off
chkconfig pcscd off
chkconfig portmap off
chkconfig rpcgssd off
chkconfig rpcidmapd off
chkconfig avahi-daemon off
chkconfig yum-updatesd off

echo "**************************************"
echo "*   Setup for app_rpt/Centos  *"
echo "**************************************"
echo
echo "You must supply a root password..."
passwd root
logecho "Password set"
logecho "Changing SSHD port number to $SSHDPORT..."
sed "s/^#Port[ \t]*[0-9]*/Port 222/" <$SSHDCONF >$TMP/sshd_config
mv -f $TMP/sshd_config $SSHDCONF || die "mv sshd_config failed"
logecho "Enabling SSHD and Asterisk to start on next boot..."
chkconfig sshd on
chkconfig iptables off
logecho "Running nodesetup.sh..."
if [ -e /root/acid/nodesetup.sh ]
then
        /root/acid/nodesetup.sh || die "Could not modify asterisk config files!"
fi


echo "Script done. Please plug in your modified USB fob or URI now!"
sleep 10
logecho "Rebooting...."
reboot
exit

