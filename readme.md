robo_challenge/
├── client/                # React front-end
│   └── src/App.jsx
├── server/                # Express + Mongoose backend
│   └── index.js
├── ros2_bridge/           # ROS2 Python node
│   └── ros2_bridge.py



# 1. Source your ROS2 environment
source /opt/ros/<your-distro>/setup.bash

# 2. Run the turtlesim node
ros2 run turtlesim turtlesim_node
# 3. Start rosbridge websocket server (in a new terminal)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# 4. Run the ROS2 Python bridge (in a new terminal)
cd ros2_bridge
python3 ros2_bridge.py
# 5. Start MongoDB server (if not already running, in a new terminal)
mongod --dbpath ~/mongo-data
# 6. Start Express backend server
cd server
npm install
node index.js
# 7. Start React frontend
cd client
npm install
npm start
