#!/bin/bash

# Set ROS MASTER URI
echo "Setting ROS MASTER URI..."

echo "export ROS_IP=$(hostname -I | cut -d' ' -f1)" >> $HOME/.bashrc
read -p "PLEASE ENTER ROS MASTER'S IP (0.0.0.0): " master_ip
echo "export ROS_MASTER_URI=http://$master_ip:11311/" >> $HOME/.bashrc

# Create a shell script to run your ROS node --------------
echo "Creating ROS node launch script..."

mkdir -p $HOME/catkin_ws/src/actor_estop/temp
echo "#!/bin/bash" > $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh
echo "source /opt/ros/noetic/setup.bash" >> $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh
echo "source ~/catkin_ws/devel/setup.bash" >> $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh
echo "rosrun actor_estop/temp edge_estop_manager.py" >> $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh

# Make the script executable
echo "Making script executable..."

sudo chmod +x $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh

# Create a systemd service --------------------------------
echo "Creating systemd service..."

echo "[Unit]" > $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "Description=actor_estop ROS Node" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "After=network.target" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "[Service]" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "User=$USER" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
echo "ExecStart=/bin/bash $HOME/catkin_ws/src/actor_estop/temp/rosrun_estop.sh" >> $HOME/catkin_ws/src/actor_estop/temp/estop.service
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
