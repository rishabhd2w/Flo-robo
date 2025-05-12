//src to dst shortes path 
import React, { useState, useEffect, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import * as THREE from 'three';
import ROSLIB from 'roslib';
import { Text } from '@react-three/drei';

// ROS setup
const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
ros.on('connection', () => console.log('✅ Connected to ROS'));
ros.on('error', (err) => console.error('❌ ROS error:', err));

const pathPublisher = new ROSLIB.Topic({
  ros,
  name: '/path_goal',
  messageType: 'geometry_msgs/PoseArray',
});

// Components
const Station = ({ position }) => (
  <mesh position={position}>
    <sphereGeometry args={[0.3, 32, 32]} />
    <meshStandardMaterial color="orange" />
  </mesh>
);

const BoxStation = ({ position }) => (
  <mesh position={position}>
    <boxGeometry args={[0.4, 0.4, 0.4]} />
    <meshStandardMaterial color="green" />
  </mesh>
);

const StationLabel = ({ position, name }) => (
  <Text
    position={[position[0], 0.6, position[2]]}
    fontSize={0.3}
    color="white"
    anchorX="center"
    anchorY="bottom"
  >
    {name}
  </Text>
);

const PathLine = ({ path, color = 'yellow' }) => {
  if (path.length < 2) return null;
  const points = path.map((p) => new THREE.Vector3(...p));
  const curve = new THREE.CatmullRomCurve3(points);
  const geometry = new THREE.BufferGeometry().setFromPoints(curve.getPoints(100));
  return (
    <line>
      <primitive object={geometry} attach="geometry" />
      <lineBasicMaterial attach="material" color={color} />
    </line>
  );
};

// 🟨 New: Animated Moving Box
const MovingBox = ({ start, end, startTime, duration }) => {
  const ref = useRef();

  useFrame(() => {
    if (!start || !end || !startTime) return;
    const elapsed = Date.now() - startTime;
    const t = Math.min(elapsed / duration, 1);
    const x = start[0] + (end[0] - start[0]) * t;
    const y = start[1] + (end[1] - start[1]) * t;
    const z = start[2] + (end[2] - start[2]) * t;
    ref.current.position.set(x, y, z);
  });

  if (!start || !end) return null;
  return (
    <mesh ref={ref}>
      <boxGeometry args={[0.4, 0.4, 0.4]} />
      <meshStandardMaterial color="gold" />
    </mesh>
  );
};

export default function App() {
  const [stations, setStations] = useState([]);
  const [stationName, setStationName] = useState('');
  const [livePosition, setLivePosition] = useState([0, 0, 0]);
  const [trail, setTrail] = useState([]);
  const [originOffset, setOriginOffset] = useState(null);
  const [showBox, setShowBox] = useState(false);
  const [selectingMission, setSelectingMission] = useState(false);
  const [availableStations, setAvailableStations] = useState([]);
  const [selectedStations, setSelectedStations] = useState([]);
  const [missionPath, setMissionPath] = useState([]);
  const [midpoint, setMidpoint] = useState(null);
  const [midTurtleRotation, setMidTurtleRotation] = useState(0);

  // 🟨 New animation state
  const [isMoving, setIsMoving] = useState(false);
  const [moveStart, setMoveStart] = useState(null);
  const [moveEnd, setMoveEnd] = useState(null);
  const [moveStartTime, setMoveStartTime] = useState(null);
  const moveDuration = 5000;

  useEffect(() => {
    const poseListener = new ROSLIB.Topic({
      ros,
      name: '/turtle1/pose',
      messageType: 'turtlesim/Pose',
    });

    poseListener.subscribe((msg) => {
      const rawPos = [msg.x, 0, msg.y];
      setOriginOffset((prev) => prev || rawPos);

      if (originOffset) {
        const relative = [msg.x - originOffset[0], 0, msg.y - originOffset[2]];
        setLivePosition(relative);
        setTrail((prev) => [...prev.slice(-200), relative]);
      }
    });

    return () => poseListener.unsubscribe();
  }, [originOffset]);

  const adjustPosition = (pos) => originOffset ? [pos[0] - originOffset[0], 0, pos[2] - originOffset[2]] : pos;

  const addStation = async () => {
    if (!stationName) return;
    const rawPos = originOffset
      ? [livePosition[0] + originOffset[0], 0, livePosition[2] + originOffset[2]]
      : livePosition;

    const station = { name: stationName, position: rawPos };
    setStations([...stations, station]);
    setStationName('');
    setShowBox(true);
    setTimeout(() => setShowBox(false), 2000);

    try {
      await fetch('http://localhost:4000/stations', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: stationName,
          x: rawPos[0],
          y: rawPos[2],
        }),
      });
    } catch (error) {
      console.error('❌ Error saving station to DB:', error);
    }
  };

  const clearStations = async () => {
    await fetch('http://localhost:4000/stations', { method: 'DELETE' });
    setStations([]);
  };

  const loadStations = async () => {
    const res = await fetch('http://localhost:4000/stations');
    const data = await res.json();

    const waitForOrigin = async () => {
      while (!originOffset) {
        await new Promise(res => setTimeout(res, 100));
      }
    };
    await waitForOrigin();

    const loaded = data.map(s => ({
      name: s.name,
      position: [s.x, 0, s.y],
    }));

    setStations(loaded);
  };

  const publishMove = (linear = 0, angular = 0) => {
    if (!ros.isConnected) return;

    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angular },
    });

    const cmdVel = new ROSLIB.Topic({
      ros,
      name: '/turtle1/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    cmdVel.advertise();
    cmdVel.publish(twist);
  };

  const moveTurtleToStation = async (from, to, duration = 5000) => {
    const steps = 50;
    const interval = duration / steps;
    const dx = to.x - from.x;
    const dy = to.y - from.y;
    const distance = Math.hypot(dx, dy);
    const targetAngle = Math.atan2(dy, dx);
    const linearSpeed = distance / (duration / 1000);

    const twistPub = new ROSLIB.Topic({
      ros,
      name: '/turtle1/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });

    const getCurrentPose = () => new Promise((resolve) => {
      const listener = new ROSLIB.Topic({
        ros,
        name: '/turtle1/pose',
        messageType: 'turtlesim/Pose',
      });
      listener.subscribe((msg) => {
        listener.unsubscribe();
        resolve(msg);
      });
    });

    const pose = await getCurrentPose();
    let angleDiff = Math.atan2(Math.sin(targetAngle - pose.theta), Math.cos(targetAngle - pose.theta));

    const angularSpeed = 4.0;
    const rotateTime = Math.abs(angleDiff / angularSpeed) * 1000;

    twistPub.publish(new ROSLIB.Message({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: angularSpeed * Math.sign(angleDiff) },
    }));

    await new Promise(res => setTimeout(res, rotateTime));

    for (let i = 0; i < steps; i++) {
      twistPub.publish(new ROSLIB.Message({
        linear: { x: linearSpeed, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: 0.0 },
      }));
      await new Promise(res => setTimeout(res, interval));
    }

    twistPub.publish(new ROSLIB.Message({
      linear: { x: 0.0, y: 0.0, z: 0.0 },
      angular: { x: 0.0, y: 0.0, z: 0.0 },
    }));

    console.log('✅ Turtle arrived at target station!');
  };

  const handleRunMission = async () => {
    const res = await fetch('http://localhost:4000/stations');
    const data = await res.json();
  
    if (data.length < 2) {
      alert("❗ At least 2 stations are required to run a mission.");
      return;
    }
  
    setAvailableStations(data);
    setSelectingMission(true); // Only open the modal
  };
  

  const confirmMission = async () => {
    if (selectedStations.length !== 2) {
      alert("Please select exactly two stations.");
      return;
    }
  
    const [name1, name2] = selectedStations;
    const station1 = availableStations.find(s => s.name === name1);
    const station2 = availableStations.find(s => s.name === name2);
  
    if (!station1 || !station2) {
      alert("Station names not found.");
      return;
    }
  
    // Compute midpoint & angle (optional, for display)
    const dx = station2.x - station1.x;
    const dy = station2.y - station1.y;
    const angle = Math.atan2(dy, dx);
    const midX = (station1.x + station2.x) / 2;
    const midY = (station1.y + station2.y) / 2;
    const adjustedMid = adjustPosition([midX, 0, midY]);
  
    setMidpoint(adjustedMid);
    setMidTurtleRotation(-angle);
  
    // 🟨 Start animation and turtle move
    setMoveStart([station1.x, 0, station1.y]);
    setMoveEnd([station2.x, 0, station2.y]);
    const now = Date.now();
    setMoveStartTime(now);
    setIsMoving(true);
    moveTurtleToStation(station1, station2, moveDuration);
    setTimeout(() => setIsMoving(false), moveDuration);
  
    setSelectingMission(false);
    setSelectedStations([]);
  };
  
  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      {showBox && (
        <div style={{ position: 'absolute', top: '100px', left: '50%', transform: 'translateX(-50%)', background: '#4caf50', padding: '15px 25px', borderRadius: '10px', color: 'white', fontWeight: 'bold', zIndex: 10 }}>✅ Station Added!</div>
      )}

      {selectingMission && (
        <div style={{ position: 'absolute', top: 150, left: '50%', transform: 'translateX(-50%)', background: '#222', color: 'white', padding: '20px', borderRadius: '10px', zIndex: 10, width: '300px' }}>
          <h3>Select Two Stations</h3>
          <ul style={{ listStyle: 'none', padding: 0 }}>
            {availableStations.map((s, i) => (
              <li key={i}>
                <label>
                  <input
                    type="checkbox"
                    value={s.name}
                    onChange={(e) => {
                      const checked = e.target.checked;
                      setSelectedStations((prev) => {
                        if (checked && prev.length < 2) return [...prev, s.name];
                        if (!checked) return prev.filter(name => name !== s.name);
                        return prev;
                      });
                    }}
                    disabled={!selectedStations.includes(s.name) && selectedStations.length >= 2}
                    checked={selectedStations.includes(s.name)}
                  />
                  {' '}{s.name}
                </label>
              </li>
            ))}
          </ul>
          <button onClick={confirmMission}>✅ Confirm</button>
          <button onClick={() => { setSelectingMission(false); setSelectedStations([]); }}>❌ Cancel</button>
        </div>
      )}

      {/* Control Panel */}
      <div style={{ position: 'absolute', top: 10, left: 10, zIndex: 5, background: 'rgba(0,0,0,0.7)', padding: '10px', color: 'white', borderRadius: 8 }}>
        <div>
          <input type="text" value={stationName} placeholder="Station name" onChange={(e) => setStationName(e.target.value)} />
          <button onClick={addStation}>Add Station</button>
          <button onClick={handleRunMission}>Run Mission</button>
          <button onClick={clearStations}>Clear Stations</button>
          <button onClick={loadStations}>Load Saved Stations</button>
        </div>
        <div style={{ marginTop: 10 }}>
          <strong>Manual Control:</strong><br />
          <button onClick={() => publishMove(1.5, 0)}>↑ Forward</button>
          <button onClick={() => publishMove(-1.5, 0)}>↓ Backward</button><br />
          <button onClick={() => publishMove(0, 1.5)}>↶ Left</button>
          <button onClick={() => publishMove(0, -1.5)}>↷ Right</button>
        </div>
      </div>

      <Canvas camera={{ position: [5, 10, 10], fov: 45 }}>
        <ambientLight />
        <pointLight position={[10, 10, 10]} />
        <gridHelper args={[10, 10]} />
        <axesHelper args={[5]} />

        {stations.map((s, i) => {
          const pos = adjustPosition(s.position);
          return (
            <React.Fragment key={i}>
              <Station position={pos} />
              <BoxStation position={pos} />
              <StationLabel position={pos} name={s.name} />
            </React.Fragment>
          );
        })}

     

        <PathLine path={stations.map(s => adjustPosition(s.position))} color="yellow" />
        <PathLine path={missionPath} color="red" />
        <PathLine path={trail} color="lightblue" />

        <mesh position={livePosition}>
  <boxGeometry args={[0.4, 0.4, 0.4]} />
  <meshStandardMaterial color="skyblue" />
</mesh>


        {/* 🟨 Animated box */}
        {isMoving && (
          <MovingBox
            start={adjustPosition(moveStart)}
            end={adjustPosition(moveEnd)}
            startTime={moveStartTime}
            duration={moveDuration}
          />
        )}
      </Canvas>
    </div>
  );
}
