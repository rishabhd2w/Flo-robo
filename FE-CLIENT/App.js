import React, { useState, useEffect } from 'react';
import { Canvas } from '@react-three/fiber';
import * as THREE from 'three';
import ROSLIB from 'roslib';
import { Text } from '@react-three/drei';

const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
ros.on('connection', () => console.log(' Connected to ROS'));
ros.on('error', (err) => console.error(' ROS error:', err));

const pathPublisher = new ROSLIB.Topic({
  ros,
  name: '/path_goal',
  messageType: 'geometry_msgs/PoseArray',
});

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

export default function App() {
  const [stations, setStations] = useState([]);
  const [stationName, setStationName] = useState('');
  const [livePosition, setLivePosition] = useState([5.5, 0, 5.5]);
  const [trail, setTrail] = useState([]);
  const [showBox, setShowBox] = useState(false);

  useEffect(() => {
    const poseListener = new ROSLIB.Topic({
      ros,
      name: '/turtle1/pose',
      messageType: 'turtlesim/Pose',
    });

    poseListener.subscribe((msg) => {
      const pos = [msg.x, 0, msg.y];
      setLivePosition(pos);
      setTrail((prev) => [...prev.slice(-200), pos]);
    });

    return () => poseListener.unsubscribe();
  }, []);

  const addStation = async () => {
    if (!stationName) return;

    const pos = livePosition;
    const station = { name: stationName, position: pos };
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
          x: pos[0],
          y: pos[2],
        }),
      });
    } catch (error) {
      console.error(' Error saving station to DB:', error);
    }
  };

  const runMission = () => {
    const name1 = prompt("Enter first station name:");
    const name2 = prompt("Enter second station name:");
  
    const station1 = stations.find(s => s.name === name1);
    const station2 = stations.find(s => s.name === name2);
  
    if (!station1 || !station2) {
      alert(" One or both station names not found.");
      return;
    }
  
    console.log(`🚀 Running mission between: ${name1} and ${name2}`);
  
    // Create looping pose array
    const makePose = (pos) => ({
      position: { x: pos[0], y: pos[2], z: 0 },
      orientation: { x: 0, y: 0, z: 0, w: 1 },
    });
  
    const poses = [
      makePose(station1.position),
      makePose(station2.position),
      makePose(station1.position),
      makePose(station2.position),
      makePose(station1.position),
      makePose(station2.position),
    ];
  
    const msg = new ROSLIB.Message({ poses });
    pathPublisher.publish(msg);
  };
  

  const clearStations = async () => {
    await fetch('http://localhost:4000/stations', { method: 'DELETE' });
    setStations([]);
  };

  const loadStations = async () => {
    const res = await fetch('http://localhost:4000/stations');
    const data = await res.json();

    console.log(data);
    const loaded = data.map(s => ({
      name: s.name,
      position: [s.x, 0, s.y],
    }));
    setStations(loaded);
  };

  const publishMove = (linear = 0, angular = 0) => {
    if (!ros.isConnected) {
      console.error("ROS not connected");
      return;
    }

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
    console.log("✅ Sent cmd_vel:", twist);
  };

  return (
    <div style={{ width: '100vw', height: '100vh' }}>
      {showBox && (
        <div style={{
          position: 'absolute',
          top: '100px',
          left: '50%',
          transform: 'translateX(-50%)',
          background: '#4caf50',
          padding: '15px 25px',
          borderRadius: '10px',
          color: 'white',
          fontWeight: 'bold',
          zIndex: 10
        }}>
          ✅ Station Added!
        </div>
      )}

      <div style={{
        position: 'absolute', top: 10, left: 10, zIndex: 5,
        background: 'rgba(0,0,0,0.7)', padding: '10px', color: 'white', borderRadius: 8,
      }}>
        <div>
          <input
            type="text"
            value={stationName}
            placeholder="Station name"
            onChange={(e) => setStationName(e.target.value)}
          />
          <button onClick={addStation}>Add Station</button>
          <button onClick={runMission}>Run Mission</button>
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

        {stations.map((s, i) => (
          <React.Fragment key={i}>
            <Station position={s.position} />
            <BoxStation position={s.position} />
            <StationLabel position={s.position} name={s.name} />
          </React.Fragment>
        ))}

        <PathLine path={stations.map(s => s.position)} color="yellow" />

        <mesh position={livePosition}>
          <boxGeometry args={[0.4, 0.4, 0.4]} />
          <meshStandardMaterial color="skyblue" />
        </mesh>

        <PathLine path={trail} color="lightblue" />
      </Canvas>
    </div>
  );
}
