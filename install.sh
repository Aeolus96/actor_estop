#!/bin/bash

# Install base packages
pip3 install -r $HOME/catkin_ws/src/actor_estop/requirements.txt

# Set ROS IP
echo "Setting ROS IP..."
ROS_IP=$(hostname -I | cut -d' ' -f1)
echo "ROS_IP: $ROS_IP"

# Set ROS MASTER URI
echo "Setting ROS MASTER URI..."
read -p "PLEASE ENTER ROS MASTER'S IP (0.0.0.0): " MASTER_IP
echo "MASTER'S IP: $MASTER_IP"

# Ask for confirmation
read -p "Do you want to continue? [y/n] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]
then
    exit 1
fi

# Create a systemd service
echo "Creating systemd service..."

echo "[Unit]" > $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Description=actor_estop ROS Node" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "After=network.target" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "[Service]" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "User=$USER" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Environment=\"ROS_IP=$ROS_IP\"" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Environment=\"ROS_MASTER_URI=http://$MASTER_IP:11311/\"" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "ExecStart=/usr/bin/env bash -c \"source $HOME/.bashrc && source $HOME/catkin_ws/devel/setup.bash && rosrun actor_estop edge_estop_manager.py\"" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Restart=always" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "RestartSec=5" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "[Install]" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "WantedBy=multi-user.target" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service

# Copy the systemd service to the systemd directory
echo "Copying systemd service to systemd directory..."

sudo cp $HOME/catkin_ws/src/actor_estop/temp/estop.service /etc/systemd/system/

# Enable and start the systemd service
echo "Enabling and starting systemd service..."

sudo systemctl daemon-reload
sudo systemctl enable estop.service
sudo systemctl start estop.service

echo "estop service installation complete. Please check the status with 'systemctl status estop.service'."
