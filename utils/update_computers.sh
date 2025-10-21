#
# USED TO update all computers
#
# Instrucciones:
# Ordenador profesor: >> sudo apt install sshpass
# Todos ordenadores instalar: >> sudo apt install openssh-server -y
# Final: ejecutar este script ./update_computers.sh

UPDATEFILE=~/Escritorio/ARTE/utils/update.sh
UPDATELOG=~/Escritorio/ARTE/utils/update.log
croncmd="$UPDATEFILE > $UPDATELOG"
cronjob="@reboot $croncmd"

#To add it to the crontab, with no duplication:
#( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -"
#To remove it from the crontab whatever its current schedule:
#( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -
# add to a single command
# this command executes crontab and add the desired line
commandssh="( crontab -l | grep -v -F "$croncmd" ; echo "$cronjob" ) | crontab -"
echo $commandssh

#!/bin/bash
for i in {2..100}
 do
 if ping -c1 172.16.28.$i 2>&1 2>/dev/null; then
  echo "Found up IP: 172.16.28.$i"
  sshpass -pusuario scp update.sh usuario@172.16.28.$i:UPDATEFILE
#   sshpass -pusuario ssh usuario@172.16.28.$i "echo 'usuario' | sudo -S -k mv /home/usuario/rc.local /etc/rc.local"
  sshpass -pusuario ssh usuario@172.16.28.$i "echo 'usuario' | $commandssh"
  echo "SUCCESSSSSSSSSSSSSSSSSSSS ************************************"
 else
  echo "Could not connect to 172.16.28.$i"
 fi
done

