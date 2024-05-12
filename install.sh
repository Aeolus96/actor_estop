#!/bin/bash

# Set ROS IP
echo "Setting ROS IP..."
ROS_IP=$(hostname -I | cut -d' ' -f1)
echo "export ROS_IP=$ROS_IP" >> $HOME/.bashrc

# Set ROS MASTER URI
echo "Setting ROS MASTER URI..."
read -p "PLEASE ENTER ROS MASTER'S IP (0.0.0.0): " MASTER_IP
echo "export ROS_MASTER_URI=http://$MASTER_IP:11311/" >> $HOME/.bashrc

# Create a shell script to run your ROS node
echo "Creating ROS node launch script..."

mkdir -p $HOME/catkin_ws/src/actor_estop/temp
echo "#!/bin/bash" > $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh
echo "source $HOME/.bashrc" >> $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh
echo "rosrun actor_estop edge_estop_manager.py" >> $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh

# Make the script executable
echo "Making script executable..."

sudo chmod +x $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh

# Create a systemd service
echo "Creating systemd service..."

echo "[Unit]" > $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Description=actor_estop ROS Node" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "After=network.target" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "[Service]" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "User=$USER" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Environment=\"ROS_IP=$ROS_IP\"" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Environment=\"ROS_MASTER_URI=$MASTER_IP\"" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "ExecStart=/bin/bash /home/ubuntu/catkin_ws/src/actor_estop/temp/rosrun_estop.sh >> /home/ubuntu/catkin_ws/src/actor_estop/temp/rosrun_estop.log 2>&1" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
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
