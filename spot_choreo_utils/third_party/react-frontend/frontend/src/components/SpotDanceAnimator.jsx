import React, { useState, useEffect, useRef } from 'react';
import { Camera, Play, Pause, Save, SkipBack, SkipForward, FileText, Edit3, Scissors, Clock, Wifi, WifiOff, Activity, AlertOctagon } from 'lucide-react';
import SpotAnimatorClient from '../api/SpotAnimatorClient';

// WebSocket Status Component
const WebSocketStatus = ({ apiClient, lastUpdate }) => {
  const [connectionStatus, setConnectionStatus] = useState('disconnected');
  const [errorMessage, setErrorMessage] = useState(null);
  
  useEffect(() => {
    // Update status based on websocket state
    if (!apiClient.websocket) {
      setConnectionStatus('disconnected');
    } else {
      switch (apiClient.websocket.readyState) {
        case WebSocket.CONNECTING:
          setConnectionStatus('connecting');
          break;
        case WebSocket.OPEN:
          setConnectionStatus('connected');
          break;
        case WebSocket.CLOSING:
        case WebSocket.CLOSED:
          setConnectionStatus('disconnected');
          break;
        default:
          setConnectionStatus('disconnected');
      }
    }
  }, [apiClient.websocket]);
  
  // Manual reconnect function
  const handleReconnect = () => {
    setConnectionStatus('connecting');
    apiClient.connectWebSocket();
  };
  
  return (
    <div className="fixed bottom-4 right-4 bg-white p-2 rounded-lg shadow-md z-50 flex items-center space-x-2">
      {connectionStatus === 'connected' && (
        <>
          <Wifi size={18} className="text-green-500" />
          <span className="text-sm">Connected</span>
          {lastUpdate && (
            <span className="text-xs text-gray-500">
              Last update: {lastUpdate.toLocaleTimeString()}
            </span>
          )}
        </>
      )}
      
      {connectionStatus === 'connecting' && (
        <>
          <Activity size={18} className="text-blue-500 animate-pulse" />
          <span className="text-sm">Connecting...</span>
        </>
      )}
      
      {connectionStatus === 'disconnected' && (
        <>
          <WifiOff size={18} className="text-gray-500" />
          <span className="text-sm">Disconnected</span>
          <button 
            onClick={handleReconnect}
            className="bg-blue-500 text-white text-xs px-2 py-1 rounded hover:bg-blue-600"
          >
            Reconnect
          </button>
        </>
      )}
      
      {connectionStatus === 'error' && (
        <>
          <AlertOctagon size={18} className="text-red-500" />
          <span className="text-sm text-red-500">Error</span>
          <button 
            onClick={handleReconnect}
            className="bg-blue-500 text-white text-xs px-2 py-1 rounded hover:bg-blue-600"
          >
            Retry
          </button>
          {errorMessage && (
            <span className="text-xs text-red-500 max-w-xs truncate">{errorMessage}</span>
          )}
        </>
      )}
    </div>
  );
};

// Main component
const SpotDanceAnimator = () => {
  const apiClient = useRef(new SpotAnimatorClient('http://localhost:8000'));

  const [meshcatUrl, setMeshcatUrl] = useState('http://localhost:7000');
  const [lastWebSocketUpdate, setLastWebSocketUpdate] = useState(null);
  
  // State for animation data
  const [animation, setAnimation] = useState(null);
  const [currentKeyframeIndex, setCurrentKeyframeIndex] = useState(0);
  const [keyframeCount, setKeyframeCount] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [availableAnimations, setAvailableAnimations] = useState([]);
  const [keyframeTime, setKeyframeTime] = useState(0);
  const [keyframeJointAngles, setKeyframeJointAngles] = useState(null);
  
  // State for UI panels
  const [activePanel, setActivePanel] = useState('robot');
  const [showNewAnimationDialog, setShowNewAnimationDialog] = useState(false);
  const [newAnimationName, setNewAnimationName] = useState('');
  const [showLoadAnimationDialog, setShowLoadAnimationDialog] = useState(false);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  // State for semantic segments
  const [segments, setSegments] = useState([]);
  const [segmentName, setSegmentName] = useState('');
  const [segmentStartIdx, setSegmentStartIdx] = useState(0);
  const [segmentEndIdx, setSegmentEndIdx] = useState(0);
  
  // State for robot configuration
  const [robotConfig, setRobotConfig] = useState({
    arm: {
      shoulder_0: 0.0,
      shoulder_1: -3.12,
      elbow_0: 3.13,
      elbow_1: 1.57,
      wrist_0: -0.00001,
      wrist_1: -1.57,
    },
    gripper: -0.054,
    legs: {
      frontLeft: { hip_x: 0.12, hip_y: 0.72, knee: -1.45 },
      frontRight: { hip_x: -0.12, hip_y: 0.72, knee: -1.45 },
      rearLeft: { hip_x: 0.12, hip_y: 0.72, knee: -1.45 },
      rearRight: { hip_x: -0.12, hip_y: 0.72, knee: -1.45 }
    },
    body: {
      position: { x: 0.0, y: 0.0, z: 0.525 },
      rotation: { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }
  });

  // Initialize WebSocket connection with enhanced reconnection logic
useEffect(() => {
  // Enhanced connection with reconnection support
  const setupWebSocket = () => {
    console.log("Setting up WebSocket connection...");
    apiClient.current.connectWebSocket();
    
    // Configure WebSocket callbacks
    apiClient.current.setWebSocketCallbacks({
      onOpen: () => {
        console.log("WebSocket connection established!");
        
        // Request meshcat URL on connection
        apiClient.current.sendWebSocketMessage({
          type: 'get_meshcat_url'
        });
        
        // Fetch current animation from backend
        fetchCurrentAnimation();
        
        // Start ping interval for connection health
        startPingInterval();
      },
      
      onMessage: (message) => {
        console.log("WebSocket message received:", message);
        setLastWebSocketUpdate(new Date());
        
        if (message.type === 'robot_pose_updated') {
          // Update was successful - could update UI if needed
          console.log('Robot pose updated successfully on server', message.success);
        } else if (message.type === 'update_received') {
          // Acknowledgment from server that it received our update
          console.log('Server acknowledged update receipt');
        } else if (message.type === 'meshcat_url') {
          // Update Meshcat URL when received from server
          console.log(`Setting Meshcat URL to: ${message.url}`);
          setMeshcatUrl(message.url);
        } else if (message.type === 'play_animation') {
          setIsPlaying(true);
        } else if (message.type === 'stop_animation') {
          setIsPlaying(false);
        } else if (message.type === 'pong') {
          // Server responded to our ping
          console.log('Received pong from server');
        } else {
          console.log('Unknown message type:', message.type);
        }
      },
      
      onClose: (event) => {
        console.log(`WebSocket closed with code: ${event.code}, reason: ${event.reason}`);
        // Stop ping interval when connection closes
        stopPingInterval();
      },
      
      onError: (event) => {
        console.error('WebSocket error:', event);
        setError(`WebSocket error: ${event}`);
      }
    });
  };
  
  
  
  // Ping interval to keep connection alive
  const pingIntervalRef = { current: null };
  
  const startPingInterval = () => {
    // Clear any existing interval
    if (pingIntervalRef.current) {
      clearInterval(pingIntervalRef.current);
    }
    
    // Start a new ping interval
    pingIntervalRef.current = setInterval(() => {
      if (apiClient.current.websocket && 
          apiClient.current.websocket.readyState === WebSocket.OPEN) {
        console.log("Sending ping to server");
        apiClient.current.sendWebSocketMessage({
          type: 'ping',
          timestamp: Date.now()
        });
      }
    }, 15000); // Send ping every 15 seconds
  };
  
  const stopPingInterval = () => {
    if (pingIntervalRef.current) {
      clearInterval(pingIntervalRef.current);
      pingIntervalRef.current = null;
    }
  };
  // Function to fetch the current animation from the backend
  const fetchCurrentAnimation = async () => {
    try {
      console.log("Fetching current animation from backend...");
      const response = await apiClient.current.getCurrentAnimation();
      console.log("Current animation response:", response);
      
      if (response.name) {
        console.log(`Loading animation '${response.name}' from backend`);
        
        // Set the animation state
        setAnimation({
          name: response.name,
          keyframes: response.keyframes
        });
        
        setKeyframeCount(response.keyframes.length);
        
        // Update the segment start/end index sliders to match the animation length
        if (response.keyframes.length > 0) {
          setCurrentKeyframeIndex(0);
          
          // Update UI elements for keyframes
          const max_idx = response.keyframes.length - 1;
                    
          // Load the first keyframe
          loadKeyframe(0);
        }
      }
    } catch (error) {
      console.error('Error fetching current animation:', error);
    }
  };
  // Initial setup
  setupWebSocket();
  
  // Try to manually request the meshcat URL after a short delay
  setTimeout(() => {
    if (apiClient.current.websocket && 
        apiClient.current.websocket.readyState === WebSocket.OPEN) {
      console.log("Sending delayed meshcat URL request");
      apiClient.current.sendWebSocketMessage({
        type: "get_meshcat_url"
      });
    }
  }, 1000);
  
  // Load available animations
  fetchAnimations();
  fetchCurrentAnimation();
  
  // Cleanup function to handle component unmount
  return () => {
    // Stop the ping interval
    stopPingInterval();
    
    // Close the WebSocket connection
    if (apiClient.current.websocket) {
      console.log("Closing WebSocket connection on cleanup");
      const socket = apiClient.current.websocket;
      // Remove event handlers to prevent callbacks after unmount
      socket.onclose = null;
      socket.onerror = null;
      socket.onmessage = null;
      socket.onopen = null;
      
      // Only close if still connected
      if (socket.readyState === WebSocket.OPEN || 
          socket.readyState === WebSocket.CONNECTING) {
        socket.close();
      }
    }
  };
}, []);  // Empty dependency array means this effect runs once on mount
  
  // Fetch available animations from backend
  const fetchAnimations = async () => {
    try {
      setLoading(true);
      const response = await apiClient.current.getAnimations();
      setAvailableAnimations(response.animations || []);
      setLoading(false);
    } catch (error) {
      setError('Failed to load animations');
      setLoading(false);
      console.error('Error loading animations:', error);
    }
  };
  // Function to fetch the current animation from the backend
  const fetchCurrentAnimation = async () => {
    try {
      console.log("Fetching current animation from backend...");
      const response = await apiClient.current.getCurrentAnimation();
      console.log("Current animation response:", response);
      
      if (response.name) {
        console.log(`Loading animation '${response.name}' from backend`);
        
        // Set the animation state
        setAnimation({
          name: response.name,
          keyframes: response.keyframes
        });
        
        setKeyframeCount(response.keyframes.length);
        
        // Update the segment start/end index sliders to match the animation length
        if (response.keyframes.length > 0) {
          setCurrentKeyframeIndex(0);
          
          // Update UI elements for keyframes
          const max_idx = response.keyframes.length - 1;
                    
          // Load the first keyframe
          loadKeyframe(0);
        }
      }
    } catch (error) {
      console.error('Error fetching current animation:', error);
    }
  };
  
  // Create new animation
  const createNewAnimation = async () => {
    try {
      setLoading(true);
      const response = await apiClient.current.createAnimation(newAnimationName);
      setAnimation({
        name: response.name,
        keyframes: response.keyframes
      });
      setKeyframeCount(response.keyframes.length);
      setCurrentKeyframeIndex(0);
      setShowNewAnimationDialog(false);
      setLoading(false);
      
      // Update animations list
      fetchAnimations();
      fetchCurrentAnimation();
    } catch (error) {
      setError('Failed to create animation');
      setLoading(false);
      console.error('Error creating animation:', error);
    }
  };
  
  // Load existing animation
  const loadAnimation = async (name) => {
    try {
      setLoading(true);
      const response = await apiClient.current.loadAnimation(name);
      setAnimation({
        name: response.name,
        keyframes: response.keyframes
      });
      setSegments(response.segments || []);
      setKeyframeCount(response.keyframes.length);
      setCurrentKeyframeIndex(0);
      setShowLoadAnimationDialog(false);
      
      // Load the first keyframe
      if (response.keyframes.length > 0) {
        loadKeyframe(0);
      }
      
      setLoading(false);
    } catch (error) {
      setError('Failed to load animation');
      setLoading(false);
      console.error('Error loading animation:', error);
    }
  };
  
  // Load a specific keyframe
const loadKeyframe = async (index) => {
  if (!animation || index < 0 || index >= keyframeCount) {
    return;
  }
  
  try {
    setLoading(true);
    const response = await apiClient.current.getKeyframe(animation.name, index);
    
    // Update the current keyframe
    setCurrentKeyframeIndex(index);
    setKeyframeTime(response.time);
    setKeyframeJointAngles(response.joint_angles);
    
    // Update the robot configuration from joint angles
    updateRobotConfigFromJointAngles(response.joint_angles);
    
    setLoading(false);
    return response; // Return the response for chaining
  } catch (error) {
    setError('Failed to load keyframe');
    setLoading(false);
    console.error('Error loading keyframe:', error);
    throw error; // Rethrow for caller handling
  }
};
  
  // Update the robot configuration from joint angles
  const updateRobotConfigFromJointAngles = (jointAngles) => {
    const newConfig = {...robotConfig};
    
    // Update arm joint angles
    for (const joint of ['shoulder_0', 'shoulder_1', 'elbow_0', 'elbow_1', 'wrist_0', 'wrist_1']) {
      if (jointAngles[joint] !== undefined) {
        newConfig.arm[joint] = jointAngles[joint];
      }
    }
    
    // Update gripper
    if (jointAngles.gripper !== undefined) {
      newConfig.gripper = jointAngles.gripper;
    }
    
    // Update legs
    if (jointAngles.front_left_hip_x !== undefined) {
      newConfig.legs.frontLeft.hip_x = jointAngles.front_left_hip_x;
    }
    if (jointAngles.front_left_hip_y !== undefined) {
      newConfig.legs.frontLeft.hip_y = jointAngles.front_left_hip_y;
    }
    if (jointAngles.front_left_knee !== undefined) {
      newConfig.legs.frontLeft.knee = jointAngles.front_left_knee;
    }
    
    if (jointAngles.front_right_hip_x !== undefined) {
      newConfig.legs.frontRight.hip_x = jointAngles.front_right_hip_x;
    }
    if (jointAngles.front_right_hip_y !== undefined) {
      newConfig.legs.frontRight.hip_y = jointAngles.front_right_hip_y;
    }
    if (jointAngles.front_right_knee !== undefined) {
      newConfig.legs.frontRight.knee = jointAngles.front_right_knee;
    }
    
    if (jointAngles.rear_left_hip_x !== undefined) {
      newConfig.legs.rearLeft.hip_x = jointAngles.rear_left_hip_x;
    }
    if (jointAngles.rear_left_hip_y !== undefined) {
      newConfig.legs.rearLeft.hip_y = jointAngles.rear_left_hip_y;
    }
    if (jointAngles.rear_left_knee !== undefined) {
      newConfig.legs.rearLeft.knee = jointAngles.rear_left_knee;
    }
    
    if (jointAngles.rear_right_hip_x !== undefined) {
      newConfig.legs.rearRight.hip_x = jointAngles.rear_right_hip_x;
    }
    if (jointAngles.rear_right_hip_y !== undefined) {
      newConfig.legs.rearRight.hip_y = jointAngles.rear_right_hip_y;
    }
    if (jointAngles.rear_right_knee !== undefined) {
      newConfig.legs.rearRight.knee = jointAngles.rear_right_knee;
    }
    
    // Update body position
    if (jointAngles.body_pos_x !== undefined) {
      newConfig.body.position.x = jointAngles.body_pos_x;
    }
    if (jointAngles.body_pos_y !== undefined) {
      newConfig.body.position.y = jointAngles.body_pos_y;
    }
    if (jointAngles.body_pos_z !== undefined) {
      newConfig.body.position.z = jointAngles.body_pos_z;
    }
    
    // Update body rotation
    if (jointAngles.body_quat_w !== undefined) {
      newConfig.body.rotation.w = jointAngles.body_quat_w;
    }
    if (jointAngles.body_quat_x !== undefined) {
      newConfig.body.rotation.x = jointAngles.body_quat_x;
    }
    if (jointAngles.body_quat_y !== undefined) {
      newConfig.body.rotation.y = jointAngles.body_quat_y;
    }
    if (jointAngles.body_quat_z !== undefined) {
      newConfig.body.rotation.z = jointAngles.body_quat_z;
    }
    
    setRobotConfig(newConfig);
    
    // Update robot visualization via WebSocket
    const jointAnglesPayload = robotConfigToJointAngles(newConfig);
    console.log("Sending robot update to backend:", jointAnglesPayload);
    apiClient.current.updateRobotPose(jointAnglesPayload);
  };
  
  // Convert robot configuration to flat joint angles object
  const robotConfigToJointAngles = (config) => {
    const jointAngles = {};
    
    // Arm joints
    for (const [joint, value] of Object.entries(config.arm)) {
      jointAngles[joint] = value;
    }
    
    // Gripper
    jointAngles.gripper = config.gripper;
    
    // Legs
    jointAngles.front_left_hip_x = config.legs.frontLeft.hip_x;
    jointAngles.front_left_hip_y = config.legs.frontLeft.hip_y;
    jointAngles.front_left_knee = config.legs.frontLeft.knee;
    
    jointAngles.front_right_hip_x = config.legs.frontRight.hip_x;
    jointAngles.front_right_hip_y = config.legs.frontRight.hip_y;
    jointAngles.front_right_knee = config.legs.frontRight.knee;
    
    jointAngles.rear_left_hip_x = config.legs.rearLeft.hip_x;
    jointAngles.rear_left_hip_y = config.legs.rearLeft.hip_y;
    jointAngles.rear_left_knee = config.legs.rearLeft.knee;
    
    jointAngles.rear_right_hip_x = config.legs.rearRight.hip_x;
    jointAngles.rear_right_hip_y = config.legs.rearRight.hip_y;
    jointAngles.rear_right_knee = config.legs.rearRight.knee;
    
    // Body position
    jointAngles.body_pos_x = config.body.position.x;
    jointAngles.body_pos_y = config.body.position.y;
    jointAngles.body_pos_z = config.body.position.z;
    
    // Body rotation
    jointAngles.body_quat_w = config.body.rotation.w;
    jointAngles.body_quat_x = config.body.rotation.x;
    jointAngles.body_quat_y = config.body.rotation.y;
    jointAngles.body_quat_z = config.body.rotation.z;
    
    return jointAngles;
  };
  
 // Save current pose as a keyframe
const saveKeyframe = async () => {
  if (!animation) {
    return;
  }
  
  try {
    setLoading(true);
    const jointAngles = robotConfigToJointAngles(robotConfig);
    
    // Calculate new time for the keyframe (1 second after the last keyframe)
    const lastKeyframeTime = animation.keyframes.length > 0 
      ? animation.keyframes[animation.keyframes.length - 1].time 
      : 0;
    const newKeyframeTime = lastKeyframeTime + 1.0;
    
    const response = await apiClient.current.saveKeyframe(animation.name, jointAngles, newKeyframeTime);
    
    // Update animation data
    setAnimation({
      name: response.name,
      keyframes: response.keyframes
    });
    setKeyframeCount(response.keyframes.length);
    setCurrentKeyframeIndex(response.keyframes.length - 1);
    
    // Update the time slider to match the new keyframe time
    setKeyframeTime(newKeyframeTime);
    
    setLoading(false);
    console.log(`Added new keyframe at time ${newKeyframeTime}s`);
  } catch (error) {
    setError('Failed to save keyframe');
    setLoading(false);
    console.error('Error saving keyframe:', error);
  }
};
  
  // Update existing keyframe
  const updateKeyframe = async () => {
    if (!animation || currentKeyframeIndex < 0 || currentKeyframeIndex >= keyframeCount) {
      return;
    }
    
    try {
      setLoading(true);
      const jointAngles = robotConfigToJointAngles(robotConfig);
      await apiClient.current.updateKeyframe(animation.name, currentKeyframeIndex, jointAngles, keyframeTime);
      
      // Reload the animation to get updated data
      const response = await apiClient.current.loadAnimation(animation.name);
      setAnimation({
        name: response.name,
        keyframes: response.keyframes
      });
      setSegments(response.segments || []);
      
      setLoading(false);
    } catch (error) {
      setError('Failed to update keyframe');
      setLoading(false);
      console.error('Error updating keyframe:', error);
    }
  };
  
  // Name segment
  const nameSegment = async () => {
    if (!animation || !segmentName) {
      return;
    }
    
    try {
      setLoading(true);
      const response = await apiClient.current.nameSegment(
        animation.name, 
        segmentName, 
        segmentStartIdx, 
        segmentEndIdx
      );
      
      setSegments(response.segments || []);
      setSegmentName('');
      
      setLoading(false);
    } catch (error) {
      setError('Failed to name segment');
      setLoading(false);
      console.error('Error naming segment:', error);
    }
  };
  
  // Adjust animation timing
const adjustTiming = async () => {
  if (!animation || keyframeCount < 2) {
    return;
  }
  
  try {
    setLoading(true);
    
    // Scale the entire animation by asking for new timing
    const currentEndTime = animation.keyframes[keyframeCount - 1].time;
    const newEndTime = parseFloat(prompt(`Current animation length: ${currentEndTime.toFixed(2)}s\nEnter new length in seconds:`, currentEndTime.toString()));
    
    if (isNaN(newEndTime) || newEndTime <= 0) {
      setLoading(false);
      return;
    }
    
    const scaleFactor = newEndTime / currentEndTime;
    const keyframeIndices = [0, keyframeCount - 1];
    const newTimestamps = [0, newEndTime];
    
    // Make the API call to adjust timing on the server
    await apiClient.current.adjustTiming(
      animation.name,
      keyframeIndices,
      newTimestamps
    );
    
    const updatedKeyframes = animation.keyframes.map((keyframe, index) => {
      // Keep first keyframe at time 0, scale all others
      const newTime = index === 0 ? 0 : keyframe.time * scaleFactor;
      return { ...keyframe, time: newTime };
    });
    
    // Update animation in local state
    setAnimation({
      ...animation,
      keyframes: updatedKeyframes
    });
    
    console.log(`Adjusted animation timing - new length: ${newEndTime}s (scale factor: ${scaleFactor.toFixed(2)})`);
    setLoading(false);
  } catch (error) {
    setError('Failed to adjust timing');
    setLoading(false);
    console.error('Error adjusting timing:', error);
  }
};
  

  
// Play animation
const playAnimation = async () => {
  if (!animation || !animation.keyframes || animation.keyframes.length === 0) {
    setError('No animation to play');
    return;
  }
  
  console.log(`Starting playback of animation "${animation.name}" with ${animation.keyframes.length} keyframes`);
  
  // Create a global control reference that won't be affected by React state timing
  window.animationPlaybackActive = true;
  
  // Update the React state (for UI)
  setIsPlaying(true);
  
  // Tell the server we're playing
  await apiClient.current.playAnimation(animation.name);
  
  // DEBUGGING
  console.log("Animation playback starting...");
  console.log(`Total keyframes: ${animation.keyframes.length}`);
  
  // First display the initial keyframe
  try {
    await loadKeyframe(0);
    console.log(`Displayed initial keyframe (Time: ${animation.keyframes[0].time}s)`);
  } catch (err) {
    console.error("Error loading initial keyframe:", err);
    window.animationPlaybackActive = false;
    setIsPlaying(false);
    return;
  }
  
  // Loop through all remaining keyframes with proper timing
  for (let i = 1; i < animation.keyframes.length; i++) {
    if (!window.animationPlaybackActive) {
      console.log("Playback was stopped");
      break;
    }
    
    // Calculate delay for this keyframe
    const prevTime = animation.keyframes[i-1].time;
    const currentTime = animation.keyframes[i].time;
    const delay = Math.max(200, (currentTime - prevTime) * 1000);
    
    console.log(`Waiting ${delay}ms before displaying keyframe ${i+1}/${animation.keyframes.length}`);
    
    // Wait for the calculated delay
    await new Promise(resolve => setTimeout(resolve, delay));
    
    // Check if playback was stopped during the delay
    if (!window.animationPlaybackActive) {
      console.log("Playback was stopped during delay");
      break;
    }
    
    // Display the next keyframe
    try {
      setCurrentKeyframeIndex(i);
      await loadKeyframe(i);
      console.log(`Displayed keyframe ${i+1}/${animation.keyframes.length} (Time: ${animation.keyframes[i].time}s)`);
    } catch (err) {
      console.error(`Error loading keyframe ${i+1}:`, err);
      window.animationPlaybackActive = false;
      setIsPlaying(false);
      break;
    }
  }
  
  // Playback complete
  console.log("Animation playback complete");
  window.animationPlaybackActive = false;
  setIsPlaying(false);
};

// Stop animation
const stopAnimation = async () => {
  if (!animation) return;
  
  // Set global control flag to stop playback
  window.animationPlaybackActive = false;
  
  // Update UI state
  setIsPlaying(false);
  
  // Tell the server we're stopping
  await apiClient.current.stopAnimation(animation.name);
  
  console.log("Animation playback stopped");
};
  
  // Update robot configuration
  const updateRobotConfig = (category, joint, value) => {
    const newConfig = {...robotConfig};
    
    if (category === 'arm') {
      newConfig.arm[joint] = value;
    } else if (category === 'gripper') {
      newConfig.gripper = value;
    } else if (category === 'legs') {
      const [leg, param] = joint.split('.');
      newConfig.legs[leg][param] = value;
    } else if (category === 'body') {
      const [param, axis] = joint.split('.');
      newConfig.body[param][axis] = value;
    }
    
    setRobotConfig(newConfig);
    
    // Update robot visualization via WebSocket with enhanced logging
    const jointAngles = robotConfigToJointAngles(newConfig);
    console.log(`Sending robot update: ${category}.${joint} = ${value}`);
    console.log("Full joint angles payload:", jointAngles);
    
    // Use the enhanced WebSocket sending function
    const success = apiClient.current.updateRobotPose(jointAngles);
    if (success) {
      console.log("WebSocket message was sent successfully");
      setLastWebSocketUpdate(new Date());
    } else {
      console.error("Failed to send WebSocket message");
    }
};

  return (
    <div className="flex h-screen bg-gray-100">
      {/* Loading overlay */}
      {loading && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className="bg-white p-4 rounded-lg">
            <p className="text-lg">Loading...</p>
          </div>
        </div>
      )}
      
      {/* Error toast */}
      {error && (
        <div className="fixed top-4 right-4 bg-red-500 text-white p-3 rounded-lg shadow-lg z-50">
          <p>{error}</p>
          <button 
            className="absolute top-1 right-1 text-white"
            onClick={() => setError(null)}
          >
            ✕
          </button>
        </div>
      )}
      

         {/* Sidebar */}
         <div className="w-16 bg-gray-800 text-white flex flex-col items-center py-4">
        <button 
          className={`p-2 mb-4 rounded-lg ${activePanel === 'robot' ? 'bg-blue-600' : 'hover:bg-gray-700'}`}
          onClick={() => setActivePanel('robot')}
          title="Robot Controls"
        >
          <Camera size={24} />
        </button>
       
        <button 
          className={`p-2 mb-4 rounded-lg ${activePanel === 'segments' ? 'bg-blue-600' : 'hover:bg-gray-700'}`}
          onClick={() => setActivePanel('segments')}
          title="Semantic Segments"
        >
          <Scissors size={24} />
        </button>
        <button 
          className={`p-2 mb-4 rounded-lg ${activePanel === 'playback' ? 'bg-blue-600' : 'hover:bg-gray-700'}`}
          onClick={() => setActivePanel('playback')}
          title="Animation Playback"
        >
          <Play size={24} />
        </button>
        <div className="mt-auto">
          <button 
            className="p-2 rounded-lg hover:bg-gray-700"
            onClick={() => setShowNewAnimationDialog(true)}
            title="New Animation"
          >
            <FileText size={24} />
          </button>
        </div>
      </div>
      {/* Main content */}
      <div className="flex-1 flex flex-col">
        {/* Top bar */}
        <div className="h-12 bg-white shadow-sm flex items-center px-4">
          <h1 className="text-xl font-semibold">
            {animation ? `Spot Dance Animator - ${animation.name}` : 'Spot Dance Animator'}
          </h1>
          <div className="ml-auto flex items-center space-x-4">
            {!animation ? (
              <button 
                className="px-3 py-1 bg-blue-500 text-white rounded-md hover:bg-blue-600"
                onClick={() => setShowLoadAnimationDialog(true)}
              >
                Load Animation
              </button>
            ) : (
              <>
                <span>Keyframe {currentKeyframeIndex + 1}/{keyframeCount}</span>
                <button 
                  className="px-3 py-1 bg-green-500 text-white rounded-md hover:bg-green-600 flex items-center"
                  onClick={saveKeyframe}
                >
                  <Save size={16} className="mr-1" /> Save Pose
                </button>
              </>
            )}
          </div>
        </div>

        {/* content area: viewport + panel side by side */}
        <div className="flex flex-1 overflow-hidden">
          {/* 3D Viewport (flex-grow) */}
          <div className="flex-1 bg-gray-200 flex flex-col relative">
            {/* Meshcat header */}
            <div className="absolute top-0 left-0 right-0 bg-gray-700 text-white z-10 p-2 flex justify-between items-center">
              <span className="font-semibold">
                {animation ? `3D View: ${animation.name}` : '3D View: No Animation Loaded'}
              </span>
              <button 
                onClick={() => window.open(meshcatUrl, '_blank')}
                className="px-2 py-1 bg-blue-500 text-white text-xs rounded hover:bg-blue-600"
              >
                Open in New Window
              </button>
            </div>
            <iframe 
              src={meshcatUrl}
              className="w-full h-full border-0"
              title="Meshcat Visualization"
            />
            {/* WebSocket status */}
            <WebSocketStatus apiClient={apiClient.current} lastUpdate={lastWebSocketUpdate} />
          </div>

          {/* Control panels (fixed width) */}
          <div className="w-80 bg-white p-4 overflow-y-auto">
            {activePanel === 'robot' && (
              <div>
                <h2 className="text-lg font-semibold mb-4">Robot Controls</h2>
                    {/* Add Keyframe Navigation Controls */}
    {animation && animation.keyframes && animation.keyframes.length > 0 && (
      <div className="mb-4 bg-gray-100 p-3 rounded-lg">
        <div className="flex items-center justify-between mb-2">
          <h3 className="font-medium">Keyframe Navigation</h3>
          <div className="text-sm bg-blue-100 px-2 py-1 rounded">
            {currentKeyframeIndex + 1} / {animation.keyframes.length}
          </div>
        </div>
        
        <div className="flex items-center space-x-2 mb-2">
          <button 
            className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300 flex items-center"
            onClick={() => loadKeyframe(Math.max(0, currentKeyframeIndex - 1))}
            disabled={!animation || currentKeyframeIndex <= 0}
          >
            <SkipBack size={16} className="mr-1" /> Prev
          </button>
          
          <button 
            className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300 flex items-center"
            onClick={() => loadKeyframe(Math.min(keyframeCount - 1, currentKeyframeIndex + 1))}
            disabled={!animation || currentKeyframeIndex >= keyframeCount - 1}
          >
            Next <SkipForward size={16} className="ml-1" />
          </button>

          <button 
            className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300 ml-auto"
            onClick={() => {
              if (keyframeJointAngles) {
                console.log('Keyframe Info:', keyframeJointAngles);
                alert(JSON.stringify(keyframeJointAngles, null, 2));
              }
            }}
            disabled={!animation || !keyframeJointAngles}
          >
            Info
          </button>
        </div>

        <div className="mb-2">
          <label className="block text-sm font-medium mb-1">Keyframe Time (seconds)</label>
          <div className="flex items-center">
            <input 
              type="range" 
              min="0" 
              max="60" 
              step="0.1" 
              value={keyframeTime}
              onChange={(e) => setKeyframeTime(parseFloat(e.target.value))}
              className="flex-1 mr-2"
            />
            <input 
              type="number" 
              min="0" 
              step="0.1" 
              value={keyframeTime}
              onChange={(e) => setKeyframeTime(parseFloat(e.target.value))}
              className="w-20 p-1 border rounded"
            />
          </div>
        </div>

        <div className="flex space-x-2">
          <button 
            className="flex-1 px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 flex items-center justify-center"
            onClick={updateKeyframe}
            disabled={!animation}
          >
            <Edit3 size={16} className="mr-1" /> Update Keyframe
          </button>
          
        </div>
      </div>
    )}

                <div className="mb-6">
                  <h3 className="font-medium mb-2">Arm Joints</h3>
                  <div className="space-y-3">
                    {Object.keys(robotConfig.arm).map(joint => (
                      <div key={joint} className="flex items-center">
                        <span className="w-24 text-sm">{joint}</span>
                        <input
                          type="range"
                          min="-3.14"
                          max="3.14"
                          step="0.01"
                          value={robotConfig.arm[joint]}
                          onChange={e => updateRobotConfig('arm', joint, parseFloat(e.target.value))}
                          className="flex-1 mx-2"
                        />
                        <span className="w-14 text-right text-sm">{robotConfig.arm[joint].toFixed(2)}</span>
                      </div>
                    ))}
                  </div>
                </div>

                <div className="mb-6">
                  <h3 className="font-medium mb-2">Gripper</h3>
                  <div className="flex items-center">
                    <span className="w-24 text-sm">gripper</span>
                    <input
                      type="range"
                      min="-1"
                      max="0"
                      step="0.01"
                      value={robotConfig.gripper}
                      onChange={e => updateRobotConfig('gripper', 'gripper', parseFloat(e.target.value))}
                      className="flex-1 mx-2"
                    />
                    <span className="w-14 text-right text-sm">{robotConfig.gripper.toFixed(2)}</span>
                  </div>
                </div>

                <div className="mb-6">
                  <h3 className="font-medium mb-2">Legs</h3>

                  <div className="mb-2">
                    <h4 className="text-sm font-medium">Front Left Leg</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.legs.frontLeft).map(param => (
                        <div key={`frontLeft.${param}`} className="flex items-center">
                          <span className="w-24 text-xs">{param}</span>
                          <input
                            type="range"
                            min="-3.14"
                            max="3.14"
                            step="0.01"
                            value={robotConfig.legs.frontLeft[param]}
                            onChange={e => updateRobotConfig('legs', `frontLeft.${param}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.legs.frontLeft[param].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>

                  <div className="mb-2">
                    <h4 className="text-sm font-medium">Front Right Leg</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.legs.frontRight).map(param => (
                        <div key={`frontRight.${param}`} className="flex items-center">
                          <span className="w-24 text-xs">{param}</span>
                          <input
                            type="range"
                            min="-3.14"
                            max="3.14"
                            step="0.01"
                            value={robotConfig.legs.frontRight[param]}
                            onChange={e => updateRobotConfig('legs', `frontRight.${param}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.legs.frontRight[param].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>

                  <div className="mb-2">
                    <h4 className="text-sm font-medium">Rear Left Leg</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.legs.rearLeft).map(param => (
                        <div key={`rearLeft.${param}`} className="flex items-center">
                          <span className="w-24 text-xs">{param}</span>
                          <input
                            type="range"
                            min="-3.14"
                            max="3.14"
                            step="0.01"
                            value={robotConfig.legs.rearLeft[param]}
                            onChange={e => updateRobotConfig('legs', `rearLeft.${param}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.legs.rearLeft[param].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>

                  <div className="mb-2">
                    <h4 className="text-sm font-medium">Rear Right Leg</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.legs.rearRight).map(param => (
                        <div key={`rearRight.${param}`} className="flex items-center">
                          <span className="w-24 text-xs">{param}</span>
                          <input
                            type="range"
                            min="-3.14"
                            max="3.14"
                            step="0.01"
                            value={robotConfig.legs.rearRight[param]}
                            onChange={e => updateRobotConfig('legs', `rearRight.${param}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.legs.rearRight[param].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>
                </div>

                <div>
                  <h3 className="font-medium mb-2">Body Pose</h3>

                  <div className="mb-2">
                    <h4 className="text-sm font-medium">Position</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.body.position).map(axis => (
                        <div key={`position.${axis}`} className="flex items-center">
                          <span className="w-24 text-xs">pos_{axis}</span>
                          <input
                            type="range"
                            min="-1"
                            max="1"
                            step="0.01"
                            value={robotConfig.body.position[axis]}
                            onChange={e => updateRobotConfig('body', `position.${axis}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.body.position[axis].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>

                  <div>
                    <h4 className="text-sm font-medium">Rotation (Quaternion)</h4>
                    <div className="space-y-2">
                      {Object.keys(robotConfig.body.rotation).map(axis => (
                        <div key={`rotation.${axis}`} className="flex items-center">
                          <span className="w-24 text-xs">quat_{axis}</span>
                          <input
                            type="range"
                            min="-1"
                            max="1"
                            step="0.01"
                            value={robotConfig.body.rotation[axis]}
                            onChange={e => updateRobotConfig('body', `rotation.${axis}`, parseFloat(e.target.value))}
                            className="flex-1 mx-2"
                          />
                          <span className="w-14 text-right text-xs">{robotConfig.body.rotation[axis].toFixed(2)}</span>
                        </div>
                      ))}
                    </div>
                  </div>
                </div>
              </div>
            )}

          
            {activePanel === 'segments' && (
              <div>
                <h2 className="text-lg font-semibold mb-4">Semantic Segments</h2>
                <div className="mb-4">
                  <label className="block text-sm font-medium mb-1">Segment Name</label>
                  <input
                    type="text"
                    value={segmentName}
                    onChange={e => setSegmentName(e.target.value)}
                    className="w-full p-2 border rounded"
                    placeholder="Enter segment name..."
                    disabled={!animation}
                  />
                </div>
                <div className="mb-4">
                  <label className="block text-sm font-medium mb-1">Start Index</label>
                  <input
                    type="range"
                    min="0"
                    max={Math.max(0, keyframeCount - 1)}
                    value={segmentStartIdx}
                    onChange={e => setSegmentStartIdx(parseInt(e.target.value))}
                    className="w-full"
                    disabled={!animation}
                  />
                  <div className="text-center">{segmentStartIdx}</div>
                </div>
                <div className="mb-4">
                  <label className="block text-sm font-medium mb-1">End Index</label>
                  <input
                    type="range"
                    min="0"
                    max={Math.max(0, keyframeCount - 1)}
                    value={segmentEndIdx}
                    onChange={e => setSegmentEndIdx(parseInt(e.target.value))}
                    className="w-full"
                    disabled={!animation}
                  />
                  <div className="text-center">{segmentEndIdx}</div>
                </div>
                <button
                  className="w-full px-3 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
                  onClick={nameSegment}
                  disabled={!animation || !segmentName || segmentStartIdx > segmentEndIdx}
                >
                  Name Segment
                </button>
                {segments.length > 0 && (
                  <div className="mt-4">
                    <h3 className="font-medium mb-2">Existing Segments</h3>
                    <div className="space-y-2">
                      {segments.map((seg, idx) => (
                        <div key={idx} className="p-2 bg-gray-100 rounded">
                          <div className="font-medium">{seg.name}</div>
                          <div className="text-sm text-gray-600">
                            Keyframes {seg.start_index + 1}-{seg.end_index + 1}
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            )}

{activePanel === 'playback' && (
  <div>
    <h2 className="text-lg font-semibold mb-4">Animation Playback</h2>
    
    {/* Timing Adjustment Table */}
    <div className="mb-4 overflow-auto">
      <table className="w-full border-collapse">
        <thead>
          <tr className="bg-gray-100">
            <th className="p-2 border text-left">Keyframe</th>
            <th className="p-2 border text-left">Time</th>
            <th className="p-2 border text-left">Segment Name</th>
          </tr>
        </thead>
        <tbody>
          {animation && animation.keyframes && animation.keyframes.map((keyframe, index) => {
            // Find if this keyframe is part of a named segment
            const segmentName = segments.find(seg => 
              index >= seg.start_index && index <= seg.end_index
            )?.name || '';
            
            return (
              <tr key={index} className={index === currentKeyframeIndex ? "bg-blue-100" : ""}>
                <td className="p-2 border">{index + 1}</td>
                <td className="p-2 border">
                  <div className="flex items-center">
                    <span className="mr-2">{keyframe.time.toFixed(1)}</span>
                    <div className="flex flex-col">
                      <button 
                        className="text-gray-600 hover:text-black"
                        onClick={async () => {
                          try {
                            // Load the current keyframe
                            const keyframeData = await apiClient.current.getKeyframe(animation.name, index);
                            
                            // Calculate new time (increase by 0.1)
                            let newTime = keyframe.time + 0.1;
                            
                            // Ensure time remains less than next keyframe
                            if (index < animation.keyframes.length - 1) {
                              newTime = Math.min(newTime, animation.keyframes[index + 1].time - 0.1);
                            }
                            
                            // Update the keyframe
                            await apiClient.current.updateKeyframe(
                              animation.name,
                              index,
                              keyframeData.joint_angles,
                              newTime
                            );
                            
                            // Update the local animation data to reflect the change
                            const updatedKeyframes = [...animation.keyframes];
                            updatedKeyframes[index] = {...updatedKeyframes[index], time: newTime};
                            setAnimation({
                              ...animation,
                              keyframes: updatedKeyframes
                            });
                          } catch (err) {
                            console.error("Error updating keyframe time:", err);
                            setError("Failed to update keyframe time");
                          }
                        }}
                      >
                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-4 h-4">
                          <path d="M12 4l-8 8h6v8h4v-8h6z"></path>
                        </svg>
                      </button>
                      <button 
                        className="text-gray-600 hover:text-black"
                        onClick={async () => {
                          try {
                            // Load the current keyframe
                            const keyframeData = await apiClient.current.getKeyframe(animation.name, index);
                            
                            // Calculate new time (decrease by 0.1)
                            let newTime = keyframe.time - 0.1;
                            
                            // Ensure time remains greater than previous keyframe
                            if (index > 0) {
                              newTime = Math.max(newTime, animation.keyframes[index - 1].time + 0.1);
                            } else {
                              newTime = Math.max(0, newTime);
                            }
                            
                            // Update the keyframe
                            await apiClient.current.updateKeyframe(
                              animation.name,
                              index,
                              keyframeData.joint_angles,
                              newTime
                            );
                            
                            // Update the local animation data to reflect the change
                            const updatedKeyframes = [...animation.keyframes];
                            updatedKeyframes[index] = {...updatedKeyframes[index], time: newTime};
                            setAnimation({
                              ...animation,
                              keyframes: updatedKeyframes
                            });
                          } catch (err) {
                            console.error("Error updating keyframe time:", err);
                            setError("Failed to update keyframe time");
                          }
                        }}
                      >
                        <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-4 h-4">
                          <path d="M12 20l-8-8h6V4h4v8h6z"></path>
                        </svg>
                      </button>
                    </div>
                  </div>
                </td>
                <td className="p-2 border">{segmentName}</td>
              </tr>
            );
          })}
        </tbody>
      </table>
    </div>
    
    <div className="flex space-x-2 mb-4">
      <button 
        className={`flex-1 py-2 rounded flex items-center justify-center ${isPlaying ? 'bg-red-500 text-white hover:bg-red-600' : 'bg-green-500 text-white hover:bg-green-600'}`}
        onClick={isPlaying ? stopAnimation : playAnimation}
        disabled={!animation}
      >
        {isPlaying ? (
          <><Pause size={16} className="mr-1" /> Stop</>
        ) : (
          <><Play size={16} className="mr-1" /> Play Animation</>
        )}
      </button>
    </div>
    
    <button 
      className="w-full px-3 py-2 mb-4 bg-blue-500 text-white rounded hover:bg-blue-600 flex items-center justify-center"
      onClick={adjustTiming}
      disabled={!animation || keyframeCount < 2}
    >
      <Clock size={16} className="mr-1" /> Adjust All Timing
    </button>
  </div>
)}
          </div>
        </div>
      </div>
      
      {/* New Animation Dialog */}
      {showNewAnimationDialog && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-40">
          <div className="bg-white p-6 rounded-lg w-96">
            <h2 className="text-xl font-semibold mb-4">Create New Animation</h2>
            <div className="mb-4">
              <label className="block text-sm font-medium mb-1">Animation Name</label>
              <input 
                type="text" 
                value={newAnimationName}
                onChange={(e) => setNewAnimationName(e.target.value)}
                className="w-full p-2 border rounded"
              />
            </div>
            <div className="flex justify-end space-x-2">
              <button 
                className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300"
                onClick={() => setShowNewAnimationDialog(false)}
              >
                Cancel
              </button>
              <button 
                className="px-3 py-1 bg-blue-500 text-white rounded hover:bg-blue-600"
                onClick={createNewAnimation}
                disabled={!newAnimationName}
              >
                Create
              </button>
            </div>
          </div>
        </div>
      )}
      
      {/* Load Animation Dialog */}
      {showLoadAnimationDialog && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-40">
          <div className="bg-white p-6 rounded-lg w-96">
            <h2 className="text-xl font-semibold mb-4">Load Animation</h2>
            {availableAnimations.length > 0 ? (
              <div className="mb-4 max-h-64 overflow-y-auto">
                {availableAnimations.map((name, index) => (
                  <button 
                    key={index}
                    className="w-full text-left p-2 hover:bg-gray-100 rounded mb-1"
                    onClick={() => loadAnimation(name)}
                  >
                    {name}
                  </button>
                ))}
              </div>
            ) : (
              <p className="mb-4 text-gray-500">No animations available.</p>
            )}
            <div className="flex justify-end">
              <button 
                className="px-3 py-1 bg-gray-200 rounded hover:bg-gray-300"
                onClick={() => setShowLoadAnimationDialog(false)}
              >
                Cancel
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SpotDanceAnimator;
