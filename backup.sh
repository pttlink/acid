#!/bin/sh
cd /root
if [ -e /home/irlp ]
then
	echo "Initiating IRLP backup..."
	(cd /home/irlp/scripts; su - repeater -c /home/irlp/scripts/backup_for_reinstall)
fi	
echo "Backing up asterisk config files, and node name sound files..."
tar cf /root/backup.tar /etc/zaptel.conf /etc/asterisk/* /var/lib/asterisk/sounds/rpt/nodenames/*
if [ -e /home/irlp/backup/irlp_backup.tgz ]
then
	echo "Including irlp_backup.tgz in backup..."
	tar rf /root/backup.tar /home/irlp/backup/irlp_backup.tgz
fi
rm -rf backup.tgz
echo "Compressing tar archive...."
gzip -9 backup.tar
mv backup.tar.gz backup.tgz
if [ -e /home/irlp/backup/irlp_backup.tgz ]
then
	rm /home/irlp/backup/irlp_backup.tgz
fi		
echo "Done. Backup file backup.tgz is in /root" 
