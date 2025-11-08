// Copyright (c) 2025 Boston Dynamics AI Institute LLC. All rights reserved.

/**
 * API client for the Spot Dance Animator backend
 */
class SpotAnimatorClient {
  constructor(baseUrl = 'http://localhost:8000') {
    this.baseUrl = baseUrl;
    this.websocket = null;
    this.reconnectAttempts = 0;
    this.reconnectTimer = null;
    this.websocketCallbacks = {
      onMessage: () => {},
      onOpen: () => {},
      onClose: () => {},
      onError: () => {}
    };
  }

  /**
   * Initialize the WebSocket connection with reconnection logic
   */
connectWebSocket() {
  // Clear any existing connection attempts
  if (this.reconnectTimer) {
    clearTimeout(this.reconnectTimer);
    this.reconnectTimer = null;
  }

  // Close existing socket if one exists
  if (this.websocket) {
    this.websocket.onclose = null; // Remove the existing onclose handler
    if (this.websocket.readyState === WebSocket.OPEN || 
        this.websocket.readyState === WebSocket.CONNECTING) {
      this.websocket.close();
    }
  }

  const wsUrl = this.baseUrl.replace(/^http/, 'ws') + '/ws';
  console.log(`Connecting to WebSocket at ${wsUrl}`);
  
  this.websocket = new WebSocket(wsUrl);

  this.websocket.onopen = (event) => {
    console.log('WebSocket connection established');
    this.reconnectAttempts = 0; // Reset reconnect counter on successful connection
    this.websocketCallbacks.onOpen(event);
  };

  this.websocket.onmessage = (event) => {
    try {
      const message = JSON.parse(event.data);
      // Only log non-heartbeat messages to avoid too much console noise
      if (message.type !== 'pong') {
        console.log('WebSocket message received:', message);
      }
      this.websocketCallbacks.onMessage(message);
    } catch (error) {
      console.error('Error parsing WebSocket message:', error, event.data);
    }
  };

  this.websocket.onclose = (event) => {
    console.log('WebSocket connection closed, code:', event.code, 'reason:', event.reason);
    this.websocketCallbacks.onClose(event);
    
    // Only reconnect on unexpected closures or normal closures during active use
    if (event.code === 1006 || event.code === 1000) {
      // Implement reconnection with backoff but cap at 5 attempts
      if (this.reconnectAttempts < 5) {
        const reconnectDelay = Math.min(1000 * Math.pow(1.5, this.reconnectAttempts), 10000);
        console.log(`Attempting to reconnect in ${reconnectDelay}ms...`);
        
        this.reconnectTimer = setTimeout(() => {
          this.reconnectAttempts = (this.reconnectAttempts || 0) + 1;
          this.connectWebSocket();
        }, reconnectDelay);
      } else {
        console.log("Maximum reconnection attempts reached. Please refresh the page.");
      }
    }
  };

  this.websocket.onerror = (event) => {
    console.error('WebSocket error:', event);
    this.websocketCallbacks.onError(event);
  };
}
  /**
   * Set WebSocket event callbacks
   */
  setWebSocketCallbacks(callbacks) {
    this.websocketCallbacks = {
      ...this.websocketCallbacks,
      ...callbacks
    };
  }
  /**
   * Get the currently active animation in the backend
   */
  async getCurrentAnimation() {
    return this.request('/api/current-animation');
  }

  /**
   * Send a message through the WebSocket with better error handling
   */
  sendWebSocketMessage(message) {
    if (!this.websocket) {
      console.error("WebSocket is not initialized");
      this.connectWebSocket(); // Try to reconnect
      return false;
    }
    
    if (this.websocket.readyState === WebSocket.OPEN) {
      console.log("Sending WebSocket message:", message);
      this.websocket.send(JSON.stringify(message));
      return true;
    } else if (this.websocket.readyState === WebSocket.CONNECTING) {
      console.warn("WebSocket is still connecting. Message will be queued.");
      // Queue the message to send when connection opens
      const queuedMessage = JSON.stringify(message);
      const sendQueuedMessage = () => {
        if (this.websocket.readyState === WebSocket.OPEN) {
          console.log("Sending queued message:", message);
          this.websocket.send(queuedMessage);
          this.websocket.removeEventListener('open', sendQueuedMessage);
        }
      };
      this.websocket.addEventListener('open', sendQueuedMessage);
      return true;
    } else {
      console.error("WebSocket is not connected (state:", this.websocket.readyState, ")");
      this.connectWebSocket(); // Try to reconnect
      return false;
    }
  }

  /**
   * Generic HTTP request method
   */
  async request(endpoint, method = 'GET', data = null) {
    const url = `${this.baseUrl}${endpoint}`;
    const headers = {
      'Content-Type': 'application/json'
    };

    const options = {
      method,
      headers,
      // credentials: 'include'
    };

    if (data) {
      options.body = JSON.stringify(data);
    }

    try {
      const response = await fetch(url, options);
      const responseData = await response.json();

      if (!response.ok) {
        throw new Error(responseData.error || 'Request failed');
      }

      return responseData;
    } catch (error) {
      console.error('API request error:', error);
      throw error;
    }
  }

  /**
   * Get list of available animations
   */
  async getAnimations() {
    return this.request('/api/animations');
  }

  /**
   * Create a new animation
   */
  async createAnimation(name) {
    return this.request('/api/animations', 'POST', { name });
  }

  /**
   * Load an existing animation
   */
  async loadAnimation(name) {
    return this.request(`/api/animations/${name}`);
  }

  /**
   * Get a specific keyframe
   */
  async getKeyframe(animationName, index) {
    return this.request(`/api/animations/${animationName}/keyframes/${index}`);
  }

  /**
   * Save a new keyframe
   */
  async saveKeyframe(animationName, jointAngles, time) {
    return this.request(`/api/animations/${animationName}/keyframes`, 'POST', {
      animation_name: animationName,
      joint_angles: jointAngles,
      time
    });
  }

  /**
   * Update an existing keyframe
   */
  async updateKeyframe(animationName, index, jointAngles, time) {
    return this.request(`/api/animations/${animationName}/keyframes/${index}`, 'PUT', {
      animation_name: animationName,
      index,
      joint_angles: jointAngles,
      time
    });
  }

  /**
   * Name a segment in the animation
   */
  async nameSegment(animationName, segmentName, startIndex, endIndex) {
    return this.request(`/api/animations/${animationName}/segments`, 'POST', {
      animation_name: animationName,
      segment_name: segmentName,
      start_index: startIndex,
      end_index: endIndex
    });
  }

  /**
   * Adjust the timing of keyframes
   */
  async adjustTiming(animationName, keyframeIndices, newTimestamps) {
    return this.request(`/api/animations/${animationName}/timing`, 'PUT', {
      animation_name: animationName,
      keyframe_indices: keyframeIndices,
      new_timestamps: newTimestamps
    });
  }

  /**
   * Play the animation
   */
  async playAnimation(animationName) {
    return this.request(`/api/animations/${animationName}/play`, 'POST');
  }

  /**
   * Stop the animation playback
   */
  async stopAnimation(animationName) {
    return this.request(`/api/animations/${animationName}/stop`, 'POST');
  }

  /**
   * Update the robot pose in realtime
   * @returns {boolean} True if message was sent, false if connection issues
   */
  updateRobotPose(jointAngles) {
    return this.sendWebSocketMessage({
      type: 'update_robot_pose',
      joint_angles: jointAngles
    });
  }
}

export default SpotAnimatorClient;
