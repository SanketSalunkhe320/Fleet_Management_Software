const express = require('express');
const cors = require('cors');
const http = require('http');
const WebSocket = require('ws');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

// CORS configuration
app.use(cors({
  origin: ['http://localhost:5173', 'http://127.0.0.1:5173', 'http://192.168.1.119:5173'],
  credentials: true,
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS']
}));

app.use(express.json());

// In-memory storage for robots and missions
const robots = new Map(); // robotId -> { ws, status, data }
const missionsQueue = new Map(); // robotId -> queue of missions

// WebSocket connection handler
wss.on('connection', (ws, req) => {
  console.log('🔗 New WebSocket connection attempt');
  
  // Generate robot ID
  const robotId = `hiwonder-${Date.now().toString().slice(-6)}`;
  console.log(`🤖 Robot connected: ${robotId} from ${req.socket.remoteAddress}`);
  
  // Store robot connection
  robots.set(robotId, {
    ws,
    status: 'idle',
    ip: req.socket.remoteAddress,
    lastHeartbeat: Date.now(),
    currentMission: null,
    taskResults: [],
    completedMissions: 0
  });

  // Send welcome message
  ws.send(JSON.stringify({
    type: 'welcome',
    robotId,
    message: 'Connected to HiWonder Mission Control',
    timestamp: new Date().toISOString()
  }));

  // Handle messages from robot
  ws.on('message', (data) => {
    try {
      const message = JSON.parse(data.toString());
      const robot = robots.get(robotId);
      
      if (!robot) return;
      
      robot.lastHeartbeat = Date.now();
      
      switch (message.type) {
        case 'heartbeat':
          robot.status = message.status || robot.status;
          break;
          
        case 'status_update':
          robot.status = message.status;
          if (message.data) {
            robot.data = { ...robot.data, ...message.data };
          }
          console.log(`📊 ${robotId}: ${robot.status}`);
          break;
          
        case 'mission_completed':
          console.log(`✅ ${robotId} completed mission: ${message.missionId}`);
          robot.status = 'idle';
          robot.currentMission = null;
          robot.completedMissions = (robot.completedMissions || 0) + 1;
          break;
          
        case 'task_completed':
          console.log(`✅ ${robotId} completed task: ${message.taskId}`);
          if (message.success) {
            robot.taskResults.push({
              taskId: message.taskId,
              success: true,
              timestamp: new Date().toISOString()
            });
          }
          break;
          
        case 'error':
          console.error(`❌ ${robotId} error:`, message.error);
          robot.status = 'error';
          robot.lastError = message.error;
          break;
      }
    } catch (error) {
      console.error('Failed to parse message:', error);
    }
  });

  ws.on('close', () => {
    console.log(`🔌 Robot disconnected: ${robotId}`);
    robots.delete(robotId);
    missionsQueue.delete(robotId);
  });

  ws.on('error', (error) => {
    console.error(`💥 WebSocket error for ${robotId}:`, error);
  });
});

// Heartbeat monitor - check every 5 seconds
setInterval(() => {
  const now = Date.now();
  for (const [robotId, robot] of robots.entries()) {
    if (now - robot.lastHeartbeat > 15000) { // 15 seconds timeout
      console.log(`💀 Robot ${robotId} heartbeat timeout`);
      robot.status = 'disconnected';
      if (robot.ws.readyState === WebSocket.OPEN) {
        robot.ws.close();
      }
    }
  }
}, 5000);

// API Endpoints

// Health check
app.get('/api/health', (req, res) => {
  res.json({
    success: true,
    service: 'hiwonder-mission-control',
    version: '1.0.0',
    timestamp: new Date().toISOString(),
    stats: {
      connectedRobots: robots.size,
      activeMissions: Array.from(robots.values()).filter(r => r.status === 'executing').length,
      queuedMissions: Array.from(missionsQueue.values()).reduce((sum, queue) => sum + queue.length, 0)
    }
  });
});

// Get all connected robots
app.get('/api/robots', (req, res) => {
  const robotList = Array.from(robots.entries()).map(([id, data]) => ({
    id,
    status: data.status,
    ip: data.ip,
    lastHeartbeat: new Date(data.lastHeartbeat).toISOString(),
    currentMission: data.currentMission,
    completedMissions: data.completedMissions || 0
  }));
  
  res.json({
    success: true,
    count: robotList.length,
    robots: robotList,
    timestamp: new Date().toISOString()
  });
});

// Get specific robot details
app.get('/api/robot/:id', (req, res) => {
  const robot = robots.get(req.params.id);
  
  if (!robot) {
    return res.status(404).json({
      success: false,
      error: 'Robot not found'
    });
  }
  
  res.json({
    success: true,
    robot: {
      id: req.params.id,
      status: robot.status,
      ip: robot.ip,
      currentMission: robot.currentMission,
      taskResults: robot.taskResults || [],
      data: robot.data || {},
      lastError: robot.lastError,
      queue: missionsQueue.get(req.params.id) || [],
      completedMissions: robot.completedMissions || 0,
      lastHeartbeat: new Date(robot.lastHeartbeat).toISOString()
    }
  });
});

// Assign mission to robot
app.post('/api/assign-mission', (req, res) => {
  const { robotId, mission } = req.body;
  
  console.log(`🎯 Mission assignment request for ${robotId}:`, mission?.name);
  
  if (!robotId || !mission) {
    return res.status(400).json({
      success: false,
      error: 'Missing robotId or mission'
    });
  }
  
  const robot = robots.get(robotId);
  
  if (!robot) {
    return res.status(404).json({
      success: false,
      error: 'Robot not connected'
    });
  }
  
  if (robot.ws.readyState !== WebSocket.OPEN) {
    return res.status(503).json({
      success: false,
      error: 'Robot WebSocket not connected'
    });
  }
  
  // Validate mission
  if (!mission.id || !mission.name || !Array.isArray(mission.tasks)) {
    return res.status(400).json({
      success: false,
      error: 'Invalid mission format'
    });
  }
  
  let result;
  
  if (robot.status === 'idle') {
    // Assign mission immediately
    robot.currentMission = mission;
    robot.status = 'executing';
    
    // Send mission to robot via WebSocket
    robot.ws.send(JSON.stringify({
      type: 'mission_assignment',
      missionId: mission.id,
      missionName: mission.name,
      tasks: mission.tasks,
      timestamp: new Date().toISOString()
    }));
    
    console.log(`🚀 Mission "${mission.name}" assigned to ${robotId}`);
    
    result = {
      success: true,
      message: 'Mission assigned successfully',
      queuePosition: 0
    };
  } else {
    // Add to queue
    if (!missionsQueue.has(robotId)) {
      missionsQueue.set(robotId, []);
    }
    missionsQueue.get(robotId).push(mission);
    const queuePos = missionsQueue.get(robotId).length;
    
    console.log(`📋 Mission "${mission.name}" queued for ${robotId} (position: ${queuePos})`);
    
    result = {
      success: true,
      message: 'Mission queued',
      queuePosition: queuePos
    };
  }
  
  res.json(result);
});

// Send command to robot
app.post('/api/robot/:id/command', (req, res) => {
  const { command, params = {} } = req.body;
  const robotId = req.params.id;
  const robot = robots.get(robotId);
  
  console.log(`🎯 Command "${command}" for ${robotId}`);
  
  if (!robot) {
    return res.status(404).json({
      success: false,
      error: 'Robot not found'
    });
  }
  
  if (robot.ws.readyState !== WebSocket.OPEN) {
    return res.status(503).json({
      success: false,
      error: 'Robot not connected'
    });
  }
  
  // Send command via WebSocket
  robot.ws.send(JSON.stringify({
    type: 'command',
    command: command,
    params: params,
    timestamp: new Date().toISOString()
  }));
  
  // Update robot status based on command
  switch (command) {
    case 'pause':
      robot.status = 'paused';
      break;
    case 'resume':
      robot.status = 'executing';
      break;
    case 'stop':
    case 'emergency_stop':
      robot.status = 'idle';
      robot.currentMission = null;
      break;
  }
  
  res.json({
    success: true,
    message: `Command "${command}" sent to ${robotId}`,
    timestamp: new Date().toISOString()
  });
});

// Cancel mission
app.post('/api/robot/:id/cancel', (req, res) => {
  const robotId = req.params.id;
  const robot = robots.get(robotId);
  
  if (!robot) {
    return res.status(404).json({
      success: false,
      error: 'Robot not found'
    });
  }
  
  // Send cancel command
  if (robot.ws.readyState === WebSocket.OPEN) {
    robot.ws.send(JSON.stringify({
      type: 'cancel_mission',
      timestamp: new Date().toISOString()
    }));
  }
  
  // Update robot state
  robot.status = 'idle';
  robot.currentMission = null;
  
  // Clear queue for this robot
  missionsQueue.delete(robotId);
  
  res.json({
    success: true,
    message: 'Mission cancelled and queue cleared',
    timestamp: new Date().toISOString()
  });
});

// Serve a simple status page
app.get('/status', (req, res) => {
  res.send(`
    <!DOCTYPE html>
    <html>
    <head>
      <title>HiWonder Mission Control Status</title>
      <style>
        body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }
        .container { max-width: 800px; margin: 0 auto; }
        .header { background: #4f46e5; color: white; padding: 20px; border-radius: 10px; }
        .robot-card { background: white; border: 1px solid #ddd; padding: 15px; margin: 10px 0; border-radius: 8px; }
        .idle { border-left: 5px solid #10b981; }
        .executing { border-left: 5px solid #3b82f6; }
        .error { border-left: 5px solid #ef4444; }
        .disconnected { border-left: 5px solid #6b7280; }
        .stats { display: flex; gap: 20px; margin: 20px 0; }
        .stat-box { background: white; padding: 15px; border-radius: 8px; flex: 1; text-align: center; }
      </style>
    </head>
    <body>
      <div class="container">
        <div class="header">
          <h1>🤖 HiWonder Mission Control</h1>
          <p>Real-time robot monitoring dashboard</p>
        </div>
        
        <div class="stats">
          <div class="stat-box">
            <h3>${robots.size}</h3>
            <p>Connected Robots</p>
          </div>
          <div class="stat-box">
            <h3>${Array.from(robots.values()).filter(r => r.status === 'executing').length}</h3>
            <p>Active Missions</p>
          </div>
          <div class="stat-box">
            <h3>${Array.from(missionsQueue.values()).reduce((sum, queue) => sum + queue.length, 0)}</h3>
            <p>Queued Missions</p>
          </div>
        </div>
        
        <h2>Connected Robots</h2>
        ${Array.from(robots.entries()).map(([id, robot]) => `
          <div class="robot-card ${robot.status}">
            <h3>🤖 ${id}</h3>
            <p><strong>Status:</strong> <span class="status">${robot.status.toUpperCase()}</span></p>
            <p><strong>IP:</strong> ${robot.ip}</p>
            <p><strong>Current Mission:</strong> ${robot.currentMission?.name || 'None'}</p>
            <p><strong>Last Heartbeat:</strong> ${new Date(robot.lastHeartbeat).toLocaleTimeString()}</p>
          </div>
        `).join('')}
        
        ${robots.size === 0 ? '<div class="robot-card"><p>No robots connected. Start the Python client to connect robots.</p></div>' : ''}
        
        <div style="margin-top: 30px; color: #666; font-size: 14px;">
          <p>Server running on port 3002 | ${new Date().toLocaleString()}</p>
        </div>
      </div>
    </body>
    </html>
  `);
});

// Start server
const PORT = process.env.PORT || 3002;
server.listen(PORT, () => {
  console.log(`
🚀 HiWonder Mission Control Server Started!
===========================================
📡 WebSocket Server: ws://localhost:${PORT}
🌐 HTTP API Server: http://localhost:${PORT}
📊 Status Page: http://localhost:${PORT}/status
👾 API Endpoints:
   • GET  /api/health          - Health check
   • GET  /api/robots          - List connected robots
   • GET  /api/robot/:id       - Get robot details
   • POST /api/assign-mission  - Assign mission to robot
   • POST /api/robot/:id/command - Send command to robot
   • POST /api/robot/:id/cancel - Cancel mission

🤖 To connect robots:
   Run: python3 hiwonder-client.py
===========================================
  `);
});