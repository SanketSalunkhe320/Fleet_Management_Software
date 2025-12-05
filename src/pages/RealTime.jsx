// import React, { useState, useEffect, useRef } from "react";
// import ROSLIB from "roslib";
// import yaml from "js-yaml";
// import { 
//   FaSearchPlus,
//   FaSearchMinus,
//   FaExpand,
//   FaRobot,
//   FaSync,
//   FaUpload,
//   FaTrash,
//   FaImage,
//   FaRedo
// } from "react-icons/fa";

// export default function RealTime() {
//   const [rosConnected, setRosConnected] = useState(false);
//   const [mapMsg, setMapMsg] = useState(null);
//   const [editableMap, setEditableMap] = useState(null);
//   const [robotPosition, setRobotPosition] = useState(null);
//   const [debugInfo, setDebugInfo] = useState("Initializing...");
//   const [connectionStatus, setConnectionStatus] = useState("disconnected");
//   const [messageCount, setMessageCount] = useState(0);
//   const [apiConnected, setApiConnected] = useState(false);
//   const [isMobile, setIsMobile] = useState(false);
//   const [mapLoadedFromFile, setMapLoadedFromFile] = useState(false);
//   const [pgmImage, setPgmImage] = useState(null);
//   const [mapMetadata, setMapMetadata] = useState(null);
//   const [rotation, setRotation] = useState(0); // Rotation state
//   const [rotationInput, setRotationInput] = useState("0"); // Input field state

//   // --- Map Loader State ---
//   const [mapLoaderActive, setMapLoaderActive] = useState(false);
//   const [yamlFile, setYamlFile] = useState(null);
//   const [pgmFile, setPgmFile] = useState(null);
//   const [mapInfo, setMapInfo] = useState(null);

//   // --- PNG Image State ---
//   const [pngImage, setPngImage] = useState(null);
//   const [pngLoaderActive, setPngLoaderActive] = useState(false);
//   const [pngYamlFile, setPngYamlFile] = useState(null);
//   const [pngFile, setPngFile] = useState(null);
//   const [pngMapInfo, setPngMapInfo] = useState(null);

//   const [zoomState, setZoomState] = useState({
//     scale: 1,
//     offsetX: 0,
//     offsetY: 0,
//     isDragging: false,
//     lastX: 0,
//     lastY: 0
//   });

//   const canvasRef = useRef(null);
//   const mapParamsRef = useRef(null);
//   const positionIntervalRef = useRef(null);
  
//   // Flask API configuration
//   const API_BASE_URL = 'http://localhost:5001';

//   // Load saved map data from localStorage on component mount
//   useEffect(() => {
//     const savedMapData = localStorage.getItem("savedMapData");
//     const savedPngData = localStorage.getItem("savedPngData");
    
//     console.log("🔍 Checking localStorage for saved maps:", {
//       pgm: savedMapData ? "Found" : "Not found",
//       png: savedPngData ? "Found" : "Not found"
//     });
    
//     // Restore PGM map if exists
//     if (savedMapData) {
//       try {
//         const mapData = JSON.parse(savedMapData);
//         if (mapData.pgmImageData && mapData.metadata) {
//           const img = new Image();
//           img.onload = () => {
//             console.log("🖼️ PGM image loaded from localStorage:", img.width, "x", img.height);
//             setPgmImage(img);
//             setMapMetadata(mapData.metadata);
//             setMapLoadedFromFile(true);
            
//             mapParamsRef.current = {
//               width: img.width,
//               height: img.height,
//               resolution: mapData.metadata.resolution,
//               originX: mapData.metadata.origin[0],
//               originY: mapData.metadata.origin[1],
//             };

//             const restoredMapData = {
//               width: img.width,
//               height: img.height,
//               resolution: mapData.metadata.resolution,
//               data: new Int8Array(img.width * img.height).fill(-1),
//               origin: { 
//                 x: mapData.metadata.origin[0], 
//                 y: mapData.metadata.origin[1], 
//                 z: 0 
//               }
//             };

//             setMapMsg(restoredMapData);
//             setEditableMap(restoredMapData);
//             setDebugInfo(`🗺️ Restored PGM Map: ${img.width}x${img.height}`);

//             if (mapData.zoomState) {
//               setZoomState(mapData.zoomState);
//             } else {
//               centerMapOnCanvas(img.width, img.height);
//             }
//           };
//           img.onerror = () => console.error("❌ Failed to load PGM image from localStorage");
//           img.src = mapData.pgmImageData;
//         }
//       } catch (e) {
//         console.error("Error restoring PGM map from localStorage:", e);
//         localStorage.removeItem("savedMapData");
//       }
//     }

//     // Restore PNG image if exists
//     if (savedPngData) {
//       try {
//         const pngData = JSON.parse(savedPngData);
//         if (pngData.pngImageData && pngData.metadata) {
//           const img = new Image();
//           img.onload = () => {
//             console.log("🖼️ PNG image loaded from localStorage:", img.width, "x", img.height);
//             setPngImage(img);
//             setMapLoadedFromFile(true);
            
//             // Use coordinates from saved metadata
//             mapParamsRef.current = {
//               width: img.width,
//               height: img.height,
//               resolution: pngData.metadata.resolution || 0.05,
//               originX: pngData.metadata.origin?.[0] || 0,
//               originY: pngData.metadata.origin?.[1] || 0,
//             };

//             setDebugInfo(`🖼️ Restored PNG Image: ${img.width}x${img.height}`);
            
//             if (pngData.zoomState) {
//               setZoomState(pngData.zoomState);
//             } else {
//               centerMapOnCanvas(img.width, img.height);
//             }
//           };
//           img.onerror = () => console.error("❌ Failed to load PNG image from localStorage");
//           img.src = pngData.pngImageData;
//         }
//       } catch (e) {
//         console.error("Error restoring PNG image from localStorage:", e);
//         localStorage.removeItem("savedPngData");
//       }
//     }
//   }, []);

//   // Save map data to localStorage whenever it changes
//   useEffect(() => {
//     const canvas = canvasRef.current;
//     if (!canvas) return;
//     const ctx = canvas.getContext("2d", { willReadFrequently: true });
//     ctx.imageSmoothingEnabled = false;

//     const container = canvas.parentElement;
//     const containerWidth = container.clientWidth;
//     const containerHeight = container.clientHeight;
//     canvas.width = containerWidth;
//     canvas.height = containerHeight;
//     canvas.style.width = `${containerWidth}px`;
//     canvas.style.height = `${containerHeight}px`;

//     ctx.clearRect(0, 0, canvas.width, canvas.height);
    
//     // If we have a PNG image loaded, draw it with PGM coordinate logic
//     if (pngImage) {
//       const { width, height } = pngImage;
      
//       ctx.save();
//       ctx.translate(zoomState.offsetX, zoomState.offsetY);
//       ctx.scale(zoomState.scale, zoomState.scale);

//       // Draw the PNG image at (0,0) - same as PGM
//       ctx.drawImage(pngImage, 0, 0, width, height);

//       // Draw robot using the same coordinate logic as PGM
//       if (robotPosition) {
//         const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
//         // Draw robot as green circle with shadow
//         ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
//         ctx.shadowBlur = 5;
//         ctx.shadowOffsetX = 2;
//         ctx.shadowOffsetY = 2;
        
//         ctx.fillStyle = '#28a745';
//         ctx.beginPath();
//         ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
//         ctx.fill();
        
//         // Reset shadow
//         ctx.shadowColor = 'transparent';
//         ctx.shadowBlur = 0;
//         ctx.shadowOffsetX = 0;
//         ctx.shadowOffsetY = 0;
        
//         // Add black border
//         ctx.strokeStyle = '#000';
//         ctx.lineWidth = 2 / zoomState.scale;
//         ctx.stroke();
        
   
//       }

//       ctx.restore();
//       return;
//     }
    
//     // If we have a PGM image loaded, draw it
//     if (pgmImage) {
//       const { width, height } = pgmImage;
      
//       ctx.save();
//       ctx.translate(zoomState.offsetX, zoomState.offsetY);
//       ctx.scale(zoomState.scale, zoomState.scale);

//       ctx.drawImage(pgmImage, 0, 0, width, height);

//       if (robotPosition) {
//         const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
//         ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
//         ctx.shadowBlur = 5;
//         ctx.shadowOffsetX = 2;
//         ctx.shadowOffsetY = 2;
        
//         ctx.fillStyle = '#28a745';
//         ctx.beginPath();
//         ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
//         ctx.fill();
        
//         ctx.shadowColor = 'transparent';
//         ctx.shadowBlur = 0;
//         ctx.shadowOffsetX = 0;
//         ctx.shadowOffsetY = 0;
        
//         ctx.strokeStyle = '#000';
//         ctx.lineWidth = 2 / zoomState.scale;
//         ctx.stroke();
        

//       }

//       ctx.restore();
//       return;
//     }
//   }, [mapMsg, pgmImage, pngImage, zoomState, robotPosition]);



//   // Center map on canvas
//   const centerMapOnCanvas = (width, height) => {
//     if (canvasRef.current) {
//       const container = canvasRef.current.parentElement;
//       const offsetX = (container.clientWidth - width) / 2;
//       const offsetY = (container.clientHeight - height) / 2;
//       setZoomState(prev => ({ 
//         ...prev, 
//         offsetX: Math.max(0, offsetX), 
//         offsetY: Math.max(0, offsetY), 
//         scale: 1 
//       }));
//     }
//   };

//   // Check if mobile device
//   useEffect(() => {
//     const checkMobile = () => {
//       setIsMobile(window.innerWidth <= 768);
//     };
//     checkMobile();
//     window.addEventListener('resize', checkMobile);
//     return () => window.removeEventListener('resize', checkMobile);
//   }, []);

//   // --- FIXED Coordinate conversions - ROS coordinates remain consistent regardless of rotation ---
//   const rosToCanvasCoords = (rosX, rosY) => {
//     if (!mapParamsRef.current) {
//       console.log("⚠️ No map parameters for coordinate conversion");
//       return { x: 0, y: 0 };
//     }

//     const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
//     // Convert ROS coordinates to pixel coordinates
//     const pixelX = (rosX - originX) / resolution;
//     const pixelY = (rosY - originY) / resolution;
    
//     // Flip Y-axis (ROS has Y-up, canvas has Y-down)
//     const canvasX = pixelX;
//     const canvasY = height - pixelY;
    
//     // Apply rotation transformation to match the displayed image
//     let rotatedX = canvasX;
//     let rotatedY = canvasY;
    
//     switch (rotation) {
//       case 90:
//         // For 90° rotation: (x, y) -> (height - y, x)
//         rotatedX = height - canvasY;
//         rotatedY = canvasX;
//         break;
//       case 180:
//         // For 180° rotation: (x, y) -> (width - x, height - y)
//         rotatedX = width - canvasX;
//         rotatedY = height - canvasY;
//         break;
//       case 270:
//         // For 270° rotation: (x, y) -> (y, width - x)
//         rotatedX = canvasY;
//         rotatedY = width - canvasX;
//         break;
//       default:
//         // Custom rotation or 0° rotation
//         if (rotation !== 0) {
//           const rad = (rotation * Math.PI) / 180;
//           const centerX = width / 2;
//           const centerY = height / 2;
//           const translatedX = canvasX - centerX;
//           const translatedY = canvasY - centerY;
//           rotatedX = translatedX * Math.cos(rad) - translatedY * Math.sin(rad) + centerX;
//           rotatedY = translatedX * Math.sin(rad) + translatedY * Math.cos(rad) + centerY;
//         } else {
//           rotatedX = canvasX;
//           rotatedY = canvasY;
//         }
//     }
    
//     return { x: rotatedX, y: rotatedY };
//   };

//   const canvasToRosCoords = (canvasX, canvasY) => {
//     if (!mapParamsRef.current) return null;
//     const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
//     // Apply inverse rotation transformation first
//     let unrotatedX = canvasX;
//     let unrotatedY = canvasY;
    
//     switch (rotation) {
//       case 90:
//         // Inverse of 90°: (x, y) -> (y, height - x)
//         unrotatedX = unrotatedY;
//         unrotatedY = height - unrotatedX;
//         break;
//       case 180:
//         // Inverse of 180°: (x, y) -> (width - x, height - y)
//         unrotatedX = width - unrotatedX;
//         unrotatedY = height - unrotatedY;
//         break;
//       case 270:
//         // Inverse of 270°: (x, y) -> (width - y, x)
//         unrotatedX = width - unrotatedY;
//         unrotatedY = unrotatedX;
//         break;
//       default:
//         // Custom rotation or 0° rotation
//         if (rotation !== 0) {
//           const rad = (-rotation * Math.PI) / 180; // Inverse rotation
//           const centerX = width / 2;
//           const centerY = height / 2;
//           const translatedX = canvasX - centerX;
//           const translatedY = canvasY - centerY;
//           unrotatedX = translatedX * Math.cos(rad) - translatedY * Math.sin(rad) + centerX;
//           unrotatedY = translatedX * Math.sin(rad) + translatedY * Math.cos(rad) + centerY;
//         }
//         break;
//     }
    
//     // Convert back to ROS coordinates
//     const pixelY = height - unrotatedY; // Unflip Y-axis
//     const worldX = originX + unrotatedX * resolution;
//     const worldY = originY + pixelY * resolution;
    
//     return { x: +worldX.toFixed(2), y: +worldY.toFixed(2) };
//   };

//   // --- Map Loader Functions ---
//   const handleLoadMapFiles = () => {
//     setMapLoaderActive(true);
//   };

//   const handleLoadPngImage = () => {
//     setPngLoaderActive(true);
//   };

//   const handleYamlSelect = (e) => setYamlFile(e.target.files[0]);
//   const handlePgmSelect = (e) => setPgmFile(e.target.files[0]);
//   const handlePngYamlSelect = (e) => setPngYamlFile(e.target.files[0]);
//   const handlePngSelect = (e) => setPngFile(e.target.files[0]);

//   const handleLoadMap = async () => {
//     try {
//       if (!yamlFile || !pgmFile) {
//         alert("Please select both YAML and PGM files first.");
//         return;
//       }

//       // Parse YAML
//       const yamlText = await yamlFile.text();
//       const parsedYaml = yaml.load(yamlText);
//       setMapInfo(parsedYaml);

//       // Read PGM as binary
//       const buffer = await pgmFile.arrayBuffer();
//       const bytes = new Uint8Array(buffer);
//       const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

//       // Parse header
//       const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
//       const magic = headerLines[0];
//       if (magic !== "P5" && magic !== "P2") {
//         throw new Error("Unsupported PGM format: must be P5 or P2");
//       }

//       const width = parseInt(headerLines[1]);
//       const height = parseInt(headerLines[2]);
//       const maxVal = parseInt(headerLines[3]);

//       // Find start of binary data
//       const headerEnd = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
//       const dataStart = headerEnd + 1;
//       const pixelBytes = bytes.slice(dataStart);

//       console.log(`📊 PGM Info: ${width}x${height}, maxVal: ${maxVal}`);

//       // Convert PGM to canvas image
//       const canvas = document.createElement('canvas');
//       canvas.width = width;
//       canvas.height = height;
//       const ctx = canvas.getContext('2d');
//       const imageData = ctx.createImageData(width, height);

//       const negate = parsedYaml.negate || 0;

//       for (let y = 0; y < height; y++) {
//         for (let x = 0; x < width; x++) {
//           const i = y * width + x;
//           if (i >= pixelBytes.length) continue;
          
//           let value = pixelBytes[i];
//           if (negate === 1) value = 255 - value;
          
//           const dataIndex = (y * width + x) * 4;
//           imageData.data[dataIndex] = value;
//           imageData.data[dataIndex + 1] = value;
//           imageData.data[dataIndex + 2] = value;
//           imageData.data[dataIndex + 3] = 255;
//         }
//       }

//       ctx.putImageData(imageData, 0, 0);

//       const img = new Image();
//       img.onload = () => {
//         // Set map parameters from YAML file
//         mapParamsRef.current = {
//           width: width,
//           height: height,
//           resolution: parsedYaml.resolution || 0.05,
//           originX: parsedYaml.origin?.[0] || 0,
//           originY: parsedYaml.origin?.[1] || 0,
//         };

//         console.log("🗺️ PGM Map parameters:", mapParamsRef.current);

//         const loadedMap = {
//           width: width,
//           height: height,
//           resolution: parsedYaml.resolution || 0.05,
//           data: new Int8Array(width * height).fill(-1),
//           origin: {
//             x: parsedYaml.origin?.[0] || 0,
//             y: parsedYaml.origin?.[1] || 0,
//             z: parsedYaml.origin?.[2] || 0
//           }
//         };

//         setMapMsg(loadedMap);
//         setEditableMap(loadedMap);
//         setPgmImage(img);
//         setMapMetadata(parsedYaml);
//         setMapLoadedFromFile(true);
//         setMapLoaderActive(false);
//         setPngImage(null); // Clear PNG if PGM is loaded
//         setRotation(0); // Reset rotation when loading new map
//         setRotationInput("0"); // Reset input field

//         centerMapOnCanvas(width, height);
//         setDebugInfo(`🗺️ PGM Map loaded: ${width}x${height} | Resolution: ${parsedYaml.resolution || 0.05}`);
//       };

//       img.src = canvas.toDataURL('image/png');

//     } catch (err) {
//       console.error("Error loading map:", err);
//       alert("Failed to load PGM map. Check console for details.");
//     }
//   };

//   const handleLoadPng = async () => {
//     try {
//       if (!pngYamlFile || !pngFile) {
//         alert("Please select both YAML and PNG files first.");
//         return;
//       }

//       console.log("🖼️ Loading PNG file with YAML:", pngFile.name);

//       // Parse YAML for PNG
//       const yamlText = await pngYamlFile.text();
//       const parsedYaml = yaml.load(yamlText);
//       setPngMapInfo(parsedYaml);

//       const img = new Image();
//       const objectUrl = URL.createObjectURL(pngFile);
      
//       img.onload = () => {
//         console.log(`📊 PNG Dimensions: ${img.width}x${img.height}`);
        
//         // Use coordinates from YAML file for PNG
//         mapParamsRef.current = {
//           width: img.width,
//           height: img.height,
//           resolution: parsedYaml.resolution || 0.05,
//           originX: parsedYaml.origin?.[0] || 0,
//           originY: parsedYaml.origin?.[1] || 0,
//         };

//         console.log("🗺️ PNG Map parameters (from YAML):", mapParamsRef.current);

//         setPngImage(img);
//         setMapLoadedFromFile(true);
//         setPngLoaderActive(false);
//         setPgmImage(null); // Clear PGM if PNG is loaded
//         setMapMsg(null);
//         setEditableMap(null);
//         setMapMetadata(null);
//         setRotation(0); // Reset rotation when loading new image
//         setRotationInput("0"); // Reset input field

//         centerMapOnCanvas(img.width, img.height);
//         setDebugInfo(`🖼️ PNG Image loaded: ${img.width}x${img.height} | Resolution: ${parsedYaml.resolution || 0.05}`);
        
//         URL.revokeObjectURL(objectUrl);
//       };
      
//       img.onerror = () => {
//         console.error("❌ Failed to load PNG image");
//         alert("Failed to load PNG image.");
//         URL.revokeObjectURL(objectUrl);
//       };
      
//       img.src = objectUrl;

//     } catch (err) {
//       console.error("Error loading PNG:", err);
//       alert("Failed to load PNG image. Check console for details.");
//     }
//   };

//   const handleRemoveMap = () => {
//     if (window.confirm("Are you sure you want to remove the current map/image?")) {
//       setPgmImage(null);
//       setPngImage(null);
//       setMapMsg(null);
//       setEditableMap(null);
//       setMapMetadata(null);
//       setPngMapInfo(null);
//       setMapLoadedFromFile(false);
//       setRotation(0); // Reset rotation
//       setRotationInput("0"); // Reset input field
//       localStorage.removeItem("savedMapData");
//       localStorage.removeItem("savedPngData");
//       setDebugInfo("🗺️ Map/Image removed");
      
//       setZoomState({
//         scale: 1,
//         offsetX: 0,
//         offsetY: 0,
//         isDragging: false,
//         lastX: 0,
//         lastY: 0
//       });
//     }
//   };

//   // --- ROS connection / subscribers ---
//   useEffect(() => {
//     const hostname = window.location.hostname;
//     const port = 9090;
//     const ros = new ROSLIB.Ros();
//     let mapListener = null;
//     let reconnectTimer = null;

//     const connect = () => {
//       try {
//         ros.connect(`ws://${hostname}:${port}`);
//       } catch (err) {
//         console.error("ROS connect error", err);
//       }
//     };

//     ros.on("connection", () => {
//       setRosConnected(true);
//       setConnectionStatus("connected");
//       setDebugInfo("✅ ROS Connected - Ready for robot data...");

//       // Subscribe to map topic if not loaded from file
//       if (!mapLoadedFromFile) {
//         mapListener = new ROSLIB.Topic({
//           ros,
//           name: "/map",
//           messageType: "nav_msgs/OccupancyGrid",
//         });

//         mapListener.subscribe((msg) => {
//           const mapData = {
//             width: msg.info.width,
//             height: msg.info.height,
//             resolution: msg.info.resolution,
//             data: msg.data,
//             origin: msg.info.origin.position,
//           };

//           mapParamsRef.current = {
//             width: msg.info.width,
//             height: msg.info.height,
//             resolution: msg.info.resolution,
//             originX: msg.info.origin.position.x,
//             originY: msg.info.origin.position.y,
//           };

//           setMapMsg(mapData);
//           setEditableMap({ ...mapData, data: [...msg.data] });
//           setDebugInfo(`🗺️ Map loaded from ROS: ${msg.info.width}x${msg.info.height} | 🤖 Waiting for robot data...`);

//           centerMapOnCanvas(msg.info.width, msg.info.height);
//         });
//       }
//     });

//     ros.on("error", (err) => {
//       console.error("ROS error", err);
//       setRosConnected(false);
//       setConnectionStatus("error");
//       setDebugInfo("❌ ROS Connection Error");
//     });

//     ros.on("close", () => {
//       setRosConnected(false);
//       setConnectionStatus("disconnected");
//       setDebugInfo("🔌 ROS Disconnected - Reconnecting...");
//       clearTimeout(reconnectTimer);
//       reconnectTimer = setTimeout(connect, 2000);
//     });

//     connect();

//     return () => {
//       mapListener?.unsubscribe();
//       clearTimeout(reconnectTimer);
//       ros.close();
//     };
//   }, [mapLoadedFromFile]);

//   // Fetch robot position from Flask API
//   const fetchRobotPosition = async () => {
//     try {
//       const response = await fetch(`${API_BASE_URL}/robot_position`);
      
//       if (!response.ok) {
//         throw new Error(`HTTP error! status: ${response.status}`);
//       }
      
//       const data = await response.json();
      
//       if ((data.status === 'success' || data.status === 'waiting') && mapParamsRef.current) {
//         const positionData = data.data;
//         const { x, y, timestamp } = positionData;
        
//         if (x !== undefined && y !== undefined) {
//           // Always convert ROS coordinates to canvas coordinates using current rotation
//           const canvasCoords = rosToCanvasCoords(x, y);

//           setMessageCount(prev => prev + 1);
//           setDebugInfo(`🤖 Robot: (${x.toFixed(2)}, ${y.toFixed(2)}) | Rotation: ${rotation}° | Updates: ${messageCount + 1}`);
          
//           setRobotPosition({
//             rosX: x,
//             rosY: y,
//             canvasX: canvasCoords.x,
//             canvasY: canvasCoords.y,
//             timestamp: timestamp,
//           });

//           localStorage.setItem("lastRobotPose", JSON.stringify({
//             rosX: x,
//             rosY: y,
//             canvasX: canvasCoords.x,
//             canvasY: canvasCoords.y,
//             timestamp: timestamp,
//           }));

//           setApiConnected(true);
//           setConnectionStatus("connected");
//         } else if (data.status === 'waiting') {
//           setDebugInfo("⏳ Waiting for robot position data...");
//         }
//       }
//     } catch (error) {
//       console.error("❌ Error fetching robot position:", error);
//       setApiConnected(false);
//       setConnectionStatus("error");
//       setDebugInfo("❌ Failed to connect to position API");
//     }
//   };

//   // Check API health
//   const checkApiHealth = async () => {
//     try {
//       const response = await fetch(`${API_BASE_URL}/health`);
//       if (response.ok) {
//         setApiConnected(true);
//         setConnectionStatus("connected");
//         setDebugInfo("✅ API Connected - Waiting for map...");
//       }
//     } catch (error) {
//       console.error("❌ API health check failed:", error);
//       setApiConnected(false);
//       setConnectionStatus("error");
//     }
//   };

//   // API connection
//   useEffect(() => {
//     console.log("🔄 Connecting to Flask API...");
//     setDebugInfo("Connecting to position API...");
//     setConnectionStatus("connecting");

//     checkApiHealth();
//     positionIntervalRef.current = setInterval(fetchRobotPosition, 100);

//     return () => {
//       if (positionIntervalRef.current) {
//         clearInterval(positionIntervalRef.current);
//       }
//     };
//   }, []);

//   // Restore robot position from localStorage - recalculate canvas coordinates when rotation changes
//   useEffect(() => {
//     const savedPose = localStorage.getItem("lastRobotPose");
//     if (savedPose && mapParamsRef.current) {
//       try {
//         const pose = JSON.parse(savedPose);
//         // Recalculate canvas coordinates using current rotation
//         const canvasCoords = rosToCanvasCoords(pose.rosX, pose.rosY);
//         setRobotPosition({
//           rosX: pose.rosX,
//           rosY: pose.rosY,
//           canvasX: canvasCoords.x,
//           canvasY: canvasCoords.y,
//           timestamp: pose.timestamp,
//         });
//         console.log("💾 Restored robot position from localStorage with current rotation");
//       } catch (e) {
//         console.error("Error restoring robot position:", e);
//       }
//     }
//   }, [rotation]); // Re-run when rotation changes

//   // Touch event handlers
//   const handleTouchStart = (e) => {
//     if (e.touches.length === 1) {
//       const touch = e.touches[0];
//       const rect = canvasRef.current.getBoundingClientRect();
//       const x = touch.clientX - rect.left;
//       const y = touch.clientY - rect.top;
      
//       setZoomState(prev => ({
//         ...prev,
//         isDragging: true,
//         lastX: x,
//         lastY: y
//       }));
//     }
//     e.preventDefault();
//   };

//   const handleTouchMove = (e) => {
//     if (e.touches.length === 1 && zoomState.isDragging) {
//       const touch = e.touches[0];
//       const rect = canvasRef.current.getBoundingClientRect();
//       const x = touch.clientX - rect.left;
//       const y = touch.clientY - rect.top;

//       const deltaX = x - zoomState.lastX;
//       const deltaY = y - zoomState.lastY;
      
//       setZoomState(prev => ({
//         ...prev,
//         offsetX: prev.offsetX + deltaX,
//         offsetY: prev.offsetY + deltaY,
//         lastX: x,
//         lastY: y
//       }));
//     }
//     e.preventDefault();
//   };

//   const handleTouchEnd = () => {
//     setZoomState(prev => ({
//       ...prev,
//       isDragging: false
//     }));
//   };

//   // Mouse event handlers
//   const handleMouseDown = (e) => {
//     const rect = canvasRef.current.getBoundingClientRect();
//     const x = e.clientX - rect.left;
//     const y = e.clientY - rect.top;
    
//     setZoomState(prev => ({
//       ...prev,
//       isDragging: true,
//       lastX: x,
//       lastY: y
//     }));
//   };

//   const handleMouseMove = (e) => {
//     if (!zoomState.isDragging) return;

//     const rect = canvasRef.current.getBoundingClientRect();
//     const x = e.clientX - rect.left;
//     const y = e.clientY - rect.top;

//     const deltaX = x - zoomState.lastX;
//     const deltaY = y - zoomState.lastY;
    
//     setZoomState(prev => ({
//       ...prev,
//       offsetX: prev.offsetX + deltaX,
//       offsetY: prev.offsetY + deltaY,
//       lastX: x,
//       lastY: y
//     }));
//   };

//   const handleMouseUp = () => {
//     setZoomState(prev => ({
//       ...prev,
//       isDragging: false
//     }));
//   };

//   // Zoom handlers
//   const handleWheel = (e) => {
//     if (!mapMsg && !pgmImage && !pngImage) return;
//     e.preventDefault();
//     e.stopPropagation();
    
//     const rect = canvasRef.current.getBoundingClientRect();
//     const containerWidth = rect.width;
//     const containerHeight = rect.height;
    
//     const centerX = containerWidth / 2;
//     const centerY = containerHeight / 2;
    
//     const delta = -e.deltaY;
//     const zoomFactor = delta > 0 ? 1.1 : 0.9;
//     const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
//     const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
//     const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
//     setZoomState(prev => ({ 
//       ...prev, 
//       scale: newScale,
//       offsetX: centerX - zoomPointX * newScale,
//       offsetY: centerY - zoomPointY * newScale
//     }));
//   };

//   const handleButtonZoom = (zoomFactor) => {
//     if ((!mapMsg && !pgmImage && !pngImage) || !canvasRef.current) return;
    
//     const rect = canvasRef.current.getBoundingClientRect();
//     const containerWidth = rect.width;
//     const containerHeight = rect.height;
    
//     const centerX = containerWidth / 2;
//     const centerY = containerHeight / 2;
    
//     const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
//     const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
//     const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
//     setZoomState(prev => ({ 
//       ...prev, 
//       scale: newScale,
//       offsetX: centerX - zoomPointX * newScale,
//       offsetY: centerY - zoomPointY * newScale
//     }));
//   };

//   // Rotation handler - 90° increments
//   const handleRotate = () => {
//     const newRotation = (rotation + 90) % 360;
//     setRotation(newRotation);
//     setRotationInput(newRotation.toString());
//   };

//   // Reset rotation handler
//   const handleResetRotation = () => {
//     setRotation(0);
//     setRotationInput("0");
//   };

//   // Custom rotation input handler
//   const handleRotationInputChange = (e) => {
//     const value = e.target.value;
//     setRotationInput(value);
    
//     // Validate and set rotation
//     if (value === "") {
//       return; // Allow empty input
//     }
    
//     const degrees = parseFloat(value);
//     if (!isNaN(degrees) && degrees >= 0 && degrees <= 360) {
//       const normalizedDegrees = degrees % 360;
//       setRotation(normalizedDegrees);
//     }
//   };

//   // Apply custom rotation when input loses focus or Enter is pressed
//   const handleRotationInputBlur = () => {
//     if (rotationInput === "") {
//       setRotationInput("0");
//       setRotation(0);
//       return;
//     }
    
//     const degrees = parseFloat(rotationInput);
//     if (isNaN(degrees) || degrees < 0 || degrees > 360) {
//       // Reset to current rotation if invalid
//       setRotationInput(rotation.toString());
//     } else {
//       const normalizedDegrees = degrees % 360;
//       setRotation(normalizedDegrees);
//       setRotationInput(normalizedDegrees.toString());
//     }
//   };

//   const handleRotationInputKeyPress = (e) => {
//     if (e.key === 'Enter') {
//       handleRotationInputBlur();
//       e.target.blur(); // Remove focus from input
//     }
//   };

//   // --- Drawing the map ---
//   useEffect(() => {
//     const canvas = canvasRef.current;
//     if (!canvas) return;
//     const ctx = canvas.getContext("2d", { willReadFrequently: true });
//     ctx.imageSmoothingEnabled = false;

//     const container = canvas.parentElement;
//     const containerWidth = container.clientWidth;
//     const containerHeight = container.clientHeight;
//     canvas.width = containerWidth;
//     canvas.height = containerHeight;
//     canvas.style.width = `${containerWidth}px`;
//     canvas.style.height = `${containerHeight}px`;

//     ctx.clearRect(0, 0, canvas.width, canvas.height);
    
//     // If we have a PNG image loaded, draw it with rotation
//     if (pngImage) {
//       const { width, height } = pngImage;
      
//       ctx.save();
//       ctx.translate(zoomState.offsetX, zoomState.offsetY);
//       ctx.scale(zoomState.scale, zoomState.scale);

//       // Apply rotation transformation
//       if (rotation !== 0) {
//         const centerX = width / 2;
//         const centerY = height / 2;
//         ctx.translate(centerX, centerY);
//         ctx.rotate((rotation * Math.PI) / 180);
//         ctx.translate(-centerX, -centerY);
//       }

//       // Draw the PNG image
//       ctx.drawImage(pngImage, 0, 0, width, height);

//       // Draw robot - coordinates are already rotated in rosToCanvasCoords
//       if (robotPosition) {
//         const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
//         // Draw robot as green circle with shadow
//         ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
//         ctx.shadowBlur = 5;
//         ctx.shadowOffsetX = 2;
//         ctx.shadowOffsetY = 2;
        
//         ctx.fillStyle = '#28a745';
//         ctx.beginPath();
//         ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
//         ctx.fill();
        
//         // Reset shadow
//         ctx.shadowColor = 'transparent';
//         ctx.shadowBlur = 0;
//         ctx.shadowOffsetX = 0;
//         ctx.shadowOffsetY = 0;
        
//         // Add black border
//         ctx.strokeStyle = '#000';
//         ctx.lineWidth = 2 / zoomState.scale;
//         ctx.stroke();
        
//         // // Add coordinates label with background
//         // const label = `(${rosX.toFixed(2)}, ${rosY.toFixed(2)})`;
//         // ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
//         // ctx.fillRect(canvasX + 8, canvasY - 25, 100, 18);
        
//         // ctx.fillStyle = '#000';
//         // ctx.font = isMobile ? '10px Arial' : '12px Arial';
//         // ctx.fillText(label, canvasX + 12, canvasY - 12);
//       }

//       ctx.restore();
//       return;
//     }
    
//     // If we have a PGM image loaded, draw it with rotation
//     if (pgmImage) {
//       const { width, height } = pgmImage;
      
//       ctx.save();
//       ctx.translate(zoomState.offsetX, zoomState.offsetY);
//       ctx.scale(zoomState.scale, zoomState.scale);

//       // Apply rotation transformation
//       if (rotation !== 0) {
//         const centerX = width / 2;
//         const centerY = height / 2;
//         ctx.translate(centerX, centerY);
//         ctx.rotate((rotation * Math.PI) / 180);
//         ctx.translate(-centerX, -centerY);
//       }

//       ctx.drawImage(pgmImage, 0, 0, width, height);

//       if (robotPosition) {
//         const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
//         ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
//         ctx.shadowBlur = 5;
//         ctx.shadowOffsetX = 2;
//         ctx.shadowOffsetY = 2;
        
//         ctx.fillStyle = '#28a745';
//         ctx.beginPath();
//         ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
//         ctx.fill();
        
//         ctx.shadowColor = 'transparent';
//         ctx.shadowBlur = 0;
//         ctx.shadowOffsetX = 0;
//         ctx.shadowOffsetY = 0;
        
//         ctx.strokeStyle = '#000';
//         ctx.lineWidth = 2 / zoomState.scale;
//         ctx.stroke();
        
//         const label = `(${rosX.toFixed(2)}, ${rosY.toFixed(2)})`;
//         ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
//         ctx.fillRect(canvasX + 8, canvasY - 25, 100, 18);
        
//         ctx.fillStyle = '#000';
//         ctx.font = isMobile ? '10px Arial' : '12px Arial';
//         ctx.fillText(label, canvasX + 12, canvasY - 12);
//       }

//       ctx.restore();
//       return;
//     }

//     // Original ROS map rendering
//     const mapToRender = editableMap || mapMsg;
//     if (!mapToRender) {
//       ctx.fillStyle = '#f8f9fa';
//       ctx.fillRect(0, 0, canvas.width, canvas.height);
//       ctx.fillStyle = '#6c757d';
//       ctx.font = isMobile ? '14px Arial' : '16px Arial';
//       ctx.textAlign = 'center';
//       ctx.fillText('Load a PGM map or PNG image to visualize robot movement', canvas.width / 2, canvas.height / 2);
//       return;
//     }

//     const { width, height, data } = mapToRender;

//     ctx.save();
//     ctx.translate(zoomState.offsetX, zoomState.offsetY);
//     ctx.scale(zoomState.scale, zoomState.scale);

//     // Apply rotation transformation for ROS maps
//     if (rotation !== 0) {
//       const centerX = width / 2;
//       const centerY = height / 2;
//       ctx.translate(centerX, centerY);
//       ctx.rotate((rotation * Math.PI) / 180);
//       ctx.translate(-centerX, -centerY);
//     }

//     const imageData = ctx.createImageData(width, height);
//     for (let y = 0; y < height; y++) {
//       for (let x = 0; x < width; x++) {
//         const val = data[y * width + x];
//         let r, g, b;
        
//         if (val === -1) {
//           r = g = b = 200;
//         } else if (val === 0) {
//           r = g = b = 255;
//         } else if (val === 100) {
//           r = g = b = 100;
//         } else if (val > 0 && val < 100) {
//           const intensity = 255 - Math.floor(val * 2.2);
//           r = g = b = Math.max(100, Math.min(200, intensity));
//         } else {
//           r = 255; g = 200; b = 200;
//         }
        
//         const py = height - 1 - y;
//         const i = (py * width + x) * 4;
//         imageData.data[i] = r;
//         imageData.data[i + 1] = g;
//         imageData.data[i + 2] = b;
//         imageData.data[i + 3] = 255;
//       }
//     }
    
//     const off = document.createElement("canvas");
//     off.width = width; off.height = height;
//     off.getContext("2d").putImageData(imageData, 0, 0);
//     ctx.drawImage(off, 0, 0);

//     if (robotPosition) {
//       const { canvasX, canvasY, rosX, rosY } = robotPosition;
      
//       ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
//       ctx.shadowBlur = 5;
//       ctx.shadowOffsetX = 2;
//       ctx.shadowOffsetY = 2;
      
//       ctx.fillStyle = '#28a745';
//       ctx.beginPath();
//       ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
//       ctx.fill();
      
//       ctx.shadowColor = 'transparent';
//       ctx.shadowBlur = 0;
//       ctx.shadowOffsetX = 0;
//       ctx.shadowOffsetY = 0;
      
//       ctx.strokeStyle = '#000';
//       ctx.lineWidth = 2 / zoomState.scale;
//       ctx.stroke();
      
//       const label = `(${rosX.toFixed(2)}, ${rosY.toFixed(2)})`;
//       ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
//       ctx.fillRect(canvasX + 8, canvasY - 25, 100, 18);
      
//       ctx.fillStyle = '#000';
//       ctx.font = isMobile ? '10px Arial' : '12px Arial';
//       ctx.fillText(label, canvasX + 12, canvasY - 12);
//     }

//     ctx.restore();
//   }, [mapMsg, editableMap, zoomState, robotPosition, isMobile, pgmImage, pngImage, rotation]);

//   // Map Loader Modal
//   const MapLoaderModal = () => {
//     return (
//       <div style={{
//         position: 'fixed',
//         top: 0,
//         left: 0,
//         right: 0,
//         bottom: 0,
//         background: 'rgba(0,0,0,0.7)',
//         display: 'flex',
//         alignItems: 'center',
//         justifyContent: 'center',
//         zIndex: 1000
//       }}>
//         <div style={{
//           background: '#ffffff',
//           padding: 24,
//           borderRadius: 12,
//           border: '1px solid #e2e8f0',
//           maxWidth: 500,
//           width: '90%',
//           boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
//         }}>
//           <h3 style={{ margin: '0 0 20px 0', color: '#1e293b' }}>
//             🗺️ Load PGM Map from Files
//           </h3>
          
//           <div style={{ marginBottom: 15 }}>
//             <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
//               Select YAML File:
//               <div style={{ 
//                 marginTop: 6,
//                 border: '1px solid #e2e8f0',
//                 borderRadius: 8,
//                 padding: '10px 12px',
//                 background: '#ffffff',
//                 color: yamlFile ? '#1e293b' : '#64748b',
//                 fontSize: '14px',
//                 cursor: 'pointer',
//                 position: 'relative',
//                 overflow: 'hidden'
//               }}>
//                 {yamlFile ? yamlFile.name : 'Choose YAML file...'}
//                 <input 
//                   type="file" 
//                   accept=".yaml,.yml" 
//                   onChange={handleYamlSelect} 
//                   style={{ 
//                     position: 'absolute',
//                     top: 0,
//                     left: 0,
//                     width: '100%',
//                     height: '100%',
//                     opacity: 0,
//                     cursor: 'pointer'
//                   }}
//                 />
//               </div>
//             </label>
//           </div>

//           <div style={{ marginBottom: 20 }}>
//             <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
//               Select PGM File:
//               <div style={{ 
//                 marginTop: 6,
//                 borderRadius: 8,
//                 padding: '10px 12px',
//                 background: '#ffffff',
//                 color: pgmFile ? '#1e293b' : '#64748b',
//                 fontSize: '14px',
//                 cursor: 'pointer',
//                 position: 'relative',
//                 overflow: 'hidden'
//               }}>
//                 {pgmFile ? pgmFile.name : 'Choose PGM file...'}
//                 <input 
//                   type="file" 
//                   accept=".pgm" 
//                   onChange={handlePgmSelect} 
//                   style={{ 
//                     position: 'absolute',
//                     top: 0,
//                     left: 0,
//                     width: '100%',
//                     height: '100%',
//                     opacity: 0,
//                     cursor: 'pointer'
//                   }}
//                 />
//               </div>
//             </label>
//           </div>

//           {mapInfo && (
//             <div style={{
//               marginBottom: 15,
//               padding: 10,
//               background: '#f8fafc',
//               borderRadius: 8,
//               border: '1px solid #e2e8f0',
//             }}>
//               <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Map Info:</h4>
//               <pre style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
//                 {JSON.stringify(mapInfo, null, 2)}
//               </pre>
//             </div>
//           )}

//           <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
//             <button 
//               onClick={() => {
//                 setMapLoaderActive(false);
//                 setYamlFile(null);
//                 setPgmFile(null);
//               }}
//               style={{
//                 padding: "10px 12px",
//                 borderRadius: 8,
//                 border: "1px solid #e2e8f0",
//                 background: "#ffffff",
//                 color: "#1e293b",
//                 cursor: "pointer",
//                 fontSize: '14px',
//                 fontWeight: '500'
//               }}
//             >
//               Cancel
//             </button>
//             <button 
//               onClick={handleLoadMap}
//               style={{
//                 padding: "10px 12px",
//                 borderRadius: 8,
//                 border: "none",
//                 background: "#3b82f6",
//                 color: "white",
//                 cursor: "pointer",
//                 fontSize: '14px',
//                 fontWeight: '500'
//               }}
//               disabled={!yamlFile || !pgmFile}
//             >
//               Load PGM Map
//             </button>
//           </div>
//         </div>
//       </div>
//     );
//   };

//   // PNG Loader Modal
//   const PngLoaderModal = () => {
//     return (
//       <div style={{
//         position: 'fixed',
//         top: 0,
//         left: 0,
//         right: 0,
//         bottom: 0,
//         background: 'rgba(0,0,0,0.7)',
//         display: 'flex',
//         alignItems: 'center',
//         justifyContent: 'center',
//         zIndex: 1000
//       }}>
//         <div style={{
//           background: '#ffffff',
//           padding: 24,
//           borderRadius: 12,
//           border: '1px solid #e2e8f0',
//           maxWidth: 500,
//           width: '90%',
//           boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
//         }}>
//           <h3 style={{ margin: '0 0 20px 0', color: '#1e293b' }}>
//             🖼️ Load PNG Image with YAML Coordinates
//           </h3>
          
//           <div style={{ marginBottom: 15 }}>
//             <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
//               Select YAML File:
//               <div style={{ 
//                 marginTop: 6,
//                 border: '1px solid #e2e8f0',
//                 borderRadius: 8,
//                 padding: '10px 12px',
//                 background: '#ffffff',
//                 color: pngYamlFile ? '#1e293b' : '#64748b',
//                 fontSize: '14px',
//                 cursor: 'pointer',
//                 position: 'relative',
//                 overflow: 'hidden'
//               }}>
//                 {pngYamlFile ? pngYamlFile.name : 'Choose YAML file...'}
//                 <input 
//                   type="file" 
//                   accept=".yaml,.yml" 
//                   onChange={handlePngYamlSelect} 
//                   style={{ 
//                     position: 'absolute',
//                     top: 0,
//                     left: 0,
//                     width: '100%',
//                     height: '100%',
//                     opacity: 0,
//                     cursor: 'pointer'
//                   }}
//                 />
//               </div>
//             </label>
//           </div>

//           <div style={{ marginBottom: 20 }}>
//             <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
//               Select PNG File:
//               <div style={{ 
//                 marginTop: 6,
//                 borderRadius: 8,
//                 padding: '10px 12px',
//                 background: '#ffffff',
//                 color: pngFile ? '#1e293b' : '#64748b',
//                 fontSize: '14px',
//                 cursor: 'pointer',
//                 position: 'relative',
//                 overflow: 'hidden'
//               }}>
//                 {pngFile ? pngFile.name : 'Choose PNG file...'}
//                 <input 
//                   type="file" 
//                   accept=".png" 
//                   onChange={handlePngSelect}
//                   style={{ 
//                     position: 'absolute',
//                     top: 0,
//                     left: 0,
//                     width: '100%',
//                     height: '100%',
//                     opacity: 0,
//                     cursor: 'pointer'
//                   }}
//                 />
//               </div>
//             </label>
//           </div>

//           {pngMapInfo && (
//             <div style={{
//               marginBottom: 15,
//               padding: 10,
//               background: '#f8fafc',
//               borderRadius: 8,
//               border: '1px solid #e2e8f0',
//             }}>
//               <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Map Info:</h4>
//               <pre style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
//                 {JSON.stringify(pngMapInfo, null, 2)}
//               </pre>
//             </div>
//           )}

//           <div style={{
//             marginBottom: 20,
//             padding: 10,
//             background: '#f0f9ff',
//             borderRadius: 8,
//             border: '1px solid #e0f2fe',
//           }}>
//             <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Coordinate Information:</h4>
//             <p style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
//               • Using coordinates from YAML file<br/>
//               • Robot will maintain correct position during rotation<br/>
//               • Make sure YAML file has correct resolution and origin coordinates
//             </p>
//           </div>

//           <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
//             <button 
//               onClick={() => {
//                 setPngLoaderActive(false);
//                 setPngYamlFile(null);
//                 setPngFile(null);
//               }}
//               style={{
//                 padding: "10px 12px",
//                 borderRadius: 8,
//                 border: "1px solid #e2e8f0",
//                 background: "#ffffff",
//                 color: "#1e293b",
//                 cursor: "pointer",
//                 fontSize: '14px',
//                 fontWeight: '500'
//               }}
//             >
//               Cancel
//             </button>
//             <button 
//               onClick={handleLoadPng}
//               style={{
//                 padding: "10px 12px",
//                 borderRadius: 8,
//                 border: "none",
//                 background: "#10b981",
//                 color: "white",
//                 cursor: "pointer",
//                 fontSize: '14px',
//                 fontWeight: '500'
//               }}
//               disabled={!pngYamlFile || !pngFile}
//             >
//               Load PNG Image
//             </button>
//           </div>
//         </div>
//       </div>
//     );
//   };

//   const getStatusColor = () => {
//     switch(connectionStatus) {
//       case 'connected': return 'linear-gradient(135deg, #10b981, #059669)';
//       case 'connecting': return 'linear-gradient(135deg, #f59e0b, #d97706)';
//       case 'error': return 'linear-gradient(135deg, #ef4444, #dc2626)';
//       default: return 'linear-gradient(135deg, #6b7280, #4b5563)';
//     }
//   };

//   const getStatusText = () => {
//     switch(connectionStatus) {
//       case 'connected': return isMobile ? 'API Connected' : 'API Connected';
//       case 'connecting': return isMobile ? 'Connecting...' : 'Connecting...';
//       case 'error': return isMobile ? 'Error' : 'Connection Error';
//       default: return isMobile ? 'Offline' : 'Disconnected';
//     }
//   };

//   return (
//     <div style={{ 
//       display: "flex", 
//       flexDirection: "column", 
//       padding: isMobile ? "0.5rem" : "1rem", 
//       width: "100%", 
//       boxSizing: "border-box",
//       minHeight: "100vh",
//       background: "white",
//       overflow: "hidden"
//     }}>
      
//       {mapLoaderActive && <MapLoaderModal />}
//       {pngLoaderActive && <PngLoaderModal />}

//       {/* Debug Status Bar */}
//       <div style={{
//         display: "flex",
//         alignItems: "center",
//         justifyContent: "space-between",
//         padding: isMobile ? "8px 12px" : "12px 16px",
//         background: getStatusColor(),
//         borderRadius: "8px",
//         marginBottom: isMobile ? "0.5rem" : "1rem",
//         color: "white",
//         fontWeight: "600",
//         boxShadow: "0 2px 8px rgba(0,0,0,0.1)",
//         flexWrap: isMobile ? "wrap" : "nowrap",
//         gap: isMobile ? "4px" : "0"
//       }}>
//         <div style={{ 
//           display: "flex", 
//           alignItems: "center", 
//           gap: "8px",
//           flex: isMobile ? "1 1 100%" : "0 1 auto"
//         }}>
//           <div style={{
//             width: "12px",
//             height: "12px",
//             borderRadius: "50%",
//             background: connectionStatus === 'connected' ? "#4ade80" : 
//                        connectionStatus === 'connecting' ? "#fbbf24" : "#f87171",
//             animation: connectionStatus === 'connected' ? "pulse 2s infinite" : "none",
//           }}></div>
//           <span style={{ fontSize: isMobile ? "0.9rem" : "1rem" }}>{getStatusText()}</span>
//           <span style={{ 
//             fontSize: isMobile ? "0.7rem" : "0.8rem", 
//             opacity: 0.8,
//             marginLeft: "8px",
//             display: "flex",
//             alignItems: "center",
//             gap: "4px"
//           }}>
//             <div style={{
//               width: "6px",
//               height: "6px",
//               borderRadius: "50%",
//               background: rosConnected ? "#4ade80" : "#f87171"
//             }}></div>
//             ROS: {rosConnected ? "Connected" : "Disconnected"}
//           </span>
//         </div>
        
//         {/* Map Control Buttons */}
//         <div style={{ 
//           display: "flex", 
//           gap: "8px",
//           alignItems: "center",
//           flex: isMobile ? "1 1 100%" : "0 1 auto",
//           marginTop: isMobile ? "4px" : "0",
//           justifyContent: isMobile ? "center" : "flex-end"
//         }}>
//           <button
//             onClick={handleLoadMapFiles}
//             style={{
//               padding: isMobile ? "6px 8px" : "8px 12px",
//               background: "#8b5cf6",
//               color: "white",
//               border: "none",
//               borderRadius: "6px",
//               cursor: "pointer",
//               fontSize: isMobile ? "10px" : "12px",
//               display: "flex",
//               alignItems: "center",
//               gap: "4px"
//             }}
//             title="Load YAML+PGM Map"
//           >
//             <FaUpload />
//             {!isMobile && "Load PGM"}
//           </button>

//           <button
//             onClick={handleLoadPngImage}
//             style={{
//               padding: isMobile ? "6px 8px" : "8px 12px",
//               background: "#10b981",
//               color: "white",
//               border: "none",
//               borderRadius: "6px",
//               cursor: "pointer",
//               fontSize: isMobile ? "10px" : "12px",
//               display: "flex",
//               alignItems: "center",
//               gap: "4px"
//             }}
//             title="Load PNG Image with YAML"
//           >
//             <FaImage />
//             {!isMobile && "Load PNG"}
//           </button>

//           {(pgmImage || pngImage || mapMsg) && (
//             <button
//               onClick={handleRemoveMap}
//               style={{
//                 padding: isMobile ? "6px 8px" : "8px 12px",
//                 background: "#ef4444",
//                 color: "white",
//                 border: "none",
//                 borderRadius: "6px",
//                 cursor: "pointer",
//                 fontSize: isMobile ? "10px" : "12px",
//                 display: "flex",
//                 alignItems: "center",
//                 gap: "4px"
//               }}
//               title="Remove Map/Image"
//             >
//               <FaTrash />
//               {!isMobile && "Remove"}
//             </button>
//           )}
//         </div>
        
//         <div style={{ 
//           fontSize: isMobile ? "0.7rem" : "0.8rem", 
//           opacity: 0.9,
//           background: "rgba(255,255,255,0.2)",
//           padding: "4px 8px",
//           borderRadius: "4px",
//           fontFamily: "monospace",
//           maxWidth: isMobile ? "100%" : "400px",
//           overflow: "hidden",
//           textOverflow: "ellipsis",
//           whiteSpace: "nowrap",
//           flex: isMobile ? "1 1 100%" : "0 1 auto",
//           marginTop: isMobile ? "4px" : "0",
//           textAlign: isMobile ? "center" : "left"
//         }}>
//           {debugInfo}
//         </div>
//       </div>

//       {/* Map Container */}
//       <div style={{ 
//         flex: 1,
//         width: "100%",
//         minHeight: isMobile ? "60vh" : "500px"
//       }}>
//         <div style={{ 
//           width: "100%", 
//           height: "100%",
//           border: "2px solid #e2e8f0", 
//           borderRadius: "8px", 
//           background: "#ffffff", 
//           minHeight: isMobile ? "300px" : "500px",
//           position: "relative",
//           overflow: "hidden",
//           cursor: zoomState.isDragging ? "grabbing" : "grab",
//           touchAction: "none"
//         }}>
//           {/* Controls */}
//           <div style={{
//             position: "absolute",
//             top: isMobile ? "5px" : "10px",
//             right: isMobile ? "5px" : "10px",
//             display: "flex",
//             flexDirection: "column",
//             gap: isMobile ? "4px" : "6px",
//             zIndex: 10,
//             background: "rgba(255, 255, 255, 0.95)",
//             padding: isMobile ? "6px" : "10px",
//             borderRadius: "6px",
//             boxShadow: "0 4px 12px rgba(0,0,0,0.15)",
//           }}>

//             <div style={{
//               padding: "4px",
//               background: "#f8f9fa",
//               borderRadius: "4px",
//               textAlign: "center",
//               fontSize: isMobile ? "9px" : "11px",
//               fontWeight: "bold",
//               color: "#495057",
//               marginBottom: "2px"
//             }}>
//               {Math.round(zoomState.scale * 100)}%
//             </div>

//             <button
//               onClick={() => handleButtonZoom(1.2)}
//               disabled={zoomState.scale >= 5}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: zoomState.scale >= 5 ? "#6c757d" : "#007bff",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "6px",
//                 cursor: zoomState.scale >= 5 ? "not-allowed" : "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "12px" : "14px"
//               }}
//               title="Zoom In"
//             >
//               <FaSearchPlus />
//             </button>

//             <button
//               onClick={() => handleButtonZoom(0.8)}
//               disabled={zoomState.scale <= 0.1}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: zoomState.scale <= 0.1 ? "#6c757d" : "#6c757d",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "4px",
//                 cursor: zoomState.scale <= 0.1 ? "not-allowed" : "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "12px" : "14px"
//               }}
//               title="Zoom Out"
//             >
//               <FaSearchMinus />
//             </button>

//             <button
//               onClick={() => {
//                 if (!mapMsg && !pgmImage && !pngImage) return;
//                 const container = canvasRef.current.parentElement;
//                 const mapWidth = pngImage ? pngImage.width : (pgmImage ? pgmImage.width : mapMsg.width);
//                 const mapHeight = pngImage ? pngImage.height : (pgmImage ? pgmImage.height : mapMsg.height);
//                 const offsetX = (container.clientWidth - mapWidth) / 2;
//                 const offsetY = (container.clientHeight - mapHeight) / 2;
//                 setZoomState((p) => ({ ...p, scale: 1, offsetX, offsetY }));
//               }}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: "#ff6b35",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "4px",
//                 cursor: "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "10px" : "12px"
//               }}
//               title="Fit Map to Screen"
//             >
//               <FaExpand />
//             </button>

//             <button
//               onClick={handleRotate}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: "#f59e0b",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "4px",
//                 cursor: "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "10px" : "12px"
//               }}
//               title="Rotate 90°"
//             >
//               <FaRedo />
//             </button>

//             <button
//               onClick={handleResetRotation}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: rotation === 0 ? "#6c757d" : "#8b5cf6",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "4px",
//                 cursor: rotation === 0 ? "not-allowed" : "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "10px" : "12px"
//               }}
//               title="Reset Rotation"
//               disabled={rotation === 0}
//             >
//               {rotation}°
//             </button>

//             {/* Rotation Input Field */}
//             <div style={{
//               display: "flex",
//               flexDirection: "column",
//               alignItems: "center",
//               gap: "2px"
//             }}>
//               <input
//                 type="text"
//                 value={rotationInput}
//                 onChange={handleRotationInputChange}
//                 onBlur={handleRotationInputBlur}
//                 onKeyPress={handleRotationInputKeyPress}
//                 style={{
//                   width: isMobile ? "28px" : "36px",
//                   height: isMobile ? "20px" : "24px",
//                   textAlign: "center",
//                   fontSize: isMobile ? "10px" : "11px",
//                   border: "1px solid #d1d5db",
//                   borderRadius: "3px",
//                   background: "#ffffff"
//                 }}
//                 title="Enter rotation degrees (0-360)"
//                 maxLength={3}
//               />
//               <div style={{
//                 fontSize: isMobile ? "8px" : "9px",
//                 color: "#6b7280",
//                 textAlign: "center"
//               }}>
//                 deg
//               </div>
//             </div>

//             <button
//               onClick={fetchRobotPosition}
//               style={{
//                 padding: isMobile ? "6px" : "8px",
//                 background: "#8b5cf6",
//                 color: "white",
//                 border: "none",
//                 borderRadius: isMobile ? "4px" : "4px",
//                 cursor: "pointer",
//                 width: isMobile ? "28px" : "36px",
//                 height: isMobile ? "28px" : "36px",
//                 fontSize: isMobile ? "10px" : "12px"
//               }}
//               title="Refresh Position"
//             >
//               <FaSync />
//             </button>
//           </div>

//           <canvas 
//             ref={canvasRef} 
//             style={{ 
//               width: "100%",
//               height: "100%", 
//               imageRendering: "pixelated",
//               display: "block",
//               touchAction: "none"
//             }}
//             onMouseDown={!isMobile ? handleMouseDown : undefined}
//             onMouseMove={!isMobile ? handleMouseMove : undefined}
//             onMouseUp={!isMobile ? handleMouseUp : undefined}
//             onMouseLeave={!isMobile ? handleMouseUp : undefined}
//             onWheel={!isMobile ? handleWheel : undefined}
//             onTouchStart={isMobile ? handleTouchStart : undefined}
//             onTouchMove={isMobile ? handleTouchMove : undefined}
//             onTouchEnd={isMobile ? handleTouchEnd : undefined}
//             onTouchCancel={isMobile ? handleTouchEnd : undefined}
//           />
//         </div>
//       </div>
//     </div>
//   );
// }


import React, { useState, useEffect, useRef } from "react";
import ROSLIB from "roslib";
import yaml from "js-yaml";
import { 
  FaSearchPlus,
  FaSearchMinus,
  FaExpand,
  FaRobot,
  FaSync,
  FaUpload,
  FaTrash,
  FaImage,
  FaRedo
} from "react-icons/fa";

export default function RealTime() {
  const [rosConnected, setRosConnected] = useState(false);
  const [mapMsg, setMapMsg] = useState(null);
  const [editableMap, setEditableMap] = useState(null);
  const [robotPosition, setRobotPosition] = useState(null);
  const [debugInfo, setDebugInfo] = useState("Initializing...");
  const [connectionStatus, setConnectionStatus] = useState("disconnected");
  const [messageCount, setMessageCount] = useState(0);
  const [apiConnected, setApiConnected] = useState(false);
  const [isMobile, setIsMobile] = useState(false);
  const [mapLoadedFromFile, setMapLoadedFromFile] = useState(false);
  const [pgmImage, setPgmImage] = useState(null);
  const [mapMetadata, setMapMetadata] = useState(null);
  const [rotation, setRotation] = useState(0); // Rotation state

  // --- Map Loader State ---
  const [mapLoaderActive, setMapLoaderActive] = useState(false);
  const [yamlFile, setYamlFile] = useState(null);
  const [pgmFile, setPgmFile] = useState(null);
  const [mapInfo, setMapInfo] = useState(null);

  // --- PNG Image State ---
  const [pngImage, setPngImage] = useState(null);
  const [pngLoaderActive, setPngLoaderActive] = useState(false);
  const [pngYamlFile, setPngYamlFile] = useState(null);
  const [pngFile, setPngFile] = useState(null);
  const [pngMapInfo, setPngMapInfo] = useState(null);

  const [zoomState, setZoomState] = useState({
    scale: 1,
    offsetX: 0,
    offsetY: 0,
    isDragging: false,
    lastX: 0,
    lastY: 0
  });

  const canvasRef = useRef(null);
  const mapParamsRef = useRef(null);
  const positionIntervalRef = useRef(null);
  
  // Flask API configuration
  const API_BASE_URL = 'http://localhost:5001';

  // Load saved map data from localStorage on component mount
  useEffect(() => {
    const savedMapData = localStorage.getItem("savedMapData");
    const savedPngData = localStorage.getItem("savedPngData");
    
    console.log("🔍 Checking localStorage for saved maps:", {
      pgm: savedMapData ? "Found" : "Not found",
      png: savedPngData ? "Found" : "Not found"
    });
    
    // Restore PGM map if exists
    if (savedMapData) {
      try {
        const mapData = JSON.parse(savedMapData);
        if (mapData.pgmImageData && mapData.metadata) {
          const img = new Image();
          img.onload = () => {
            console.log("🖼️ PGM image loaded from localStorage:", img.width, "x", img.height);
            setPgmImage(img);
            setMapMetadata(mapData.metadata);
            setMapLoadedFromFile(true);
            
            mapParamsRef.current = {
              width: img.width,
              height: img.height,
              resolution: mapData.metadata.resolution,
              originX: mapData.metadata.origin[0],
              originY: mapData.metadata.origin[1],
            };

            const restoredMapData = {
              width: img.width,
              height: img.height,
              resolution: mapData.metadata.resolution,
              data: new Int8Array(img.width * img.height).fill(-1),
              origin: { 
                x: mapData.metadata.origin[0], 
                y: mapData.metadata.origin[1], 
                z: 0 
              }
            };

            setMapMsg(restoredMapData);
            setEditableMap(restoredMapData);
            setDebugInfo(`🗺️ Restored PGM Map: ${img.width}x${img.height}`);

            if (mapData.zoomState) {
              setZoomState(mapData.zoomState);
            } else {
              centerMapOnCanvas(img.width, img.height);
            }
          };
          img.onerror = () => console.error("❌ Failed to load PGM image from localStorage");
          img.src = mapData.pgmImageData;
        }
      } catch (e) {
        console.error("Error restoring PGM map from localStorage:", e);
        localStorage.removeItem("savedMapData");
      }
    }

    // Restore PNG image if exists
    if (savedPngData) {
      try {
        const pngData = JSON.parse(savedPngData);
        if (pngData.pngImageData && pngData.metadata) {
          const img = new Image();
          img.onload = () => {
            console.log("🖼️ PNG image loaded from localStorage:", img.width, "x", img.height);
            setPngImage(img);
            setMapLoadedFromFile(true);
            
            // Use coordinates from saved metadata
            mapParamsRef.current = {
              width: img.width,
              height: img.height,
              resolution: pngData.metadata.resolution || 0.05,
              originX: pngData.metadata.origin?.[0] || 0,
              originY: pngData.metadata.origin?.[1] || 0,
            };

            setDebugInfo(`🖼️ Restored PNG Image: ${img.width}x${img.height}`);
            
            if (pngData.zoomState) {
              setZoomState(pngData.zoomState);
            } else {
              centerMapOnCanvas(img.width, img.height);
            }
          };
          img.onerror = () => console.error("❌ Failed to load PNG image from localStorage");
          img.src = pngData.pngImageData;
        }
      } catch (e) {
        console.error("Error restoring PNG image from localStorage:", e);
        localStorage.removeItem("savedPngData");
      }
    }
  }, []);

  // Save map data to localStorage whenever it changes
  useEffect(() => {
    if (pgmImage && mapMetadata) {
      try {
        const canvas = document.createElement('canvas');
        canvas.width = pgmImage.width;
        canvas.height = pgmImage.height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(pgmImage, 0, 0);
        
        const mapData = {
          pgmImageData: canvas.toDataURL('image/png'),
          metadata: mapMetadata,
          width: pgmImage.width,
          height: pgmImage.height,
          zoomState: zoomState,
          timestamp: Date.now()
        };
        
        localStorage.setItem("savedMapData", JSON.stringify(mapData));
        console.log("💾 PGM Map saved to localStorage");
      } catch (e) {
        console.error("Error saving PGM map to localStorage:", e);
      }
    }

    if (pngImage && pngMapInfo) {
      try {
        const canvas = document.createElement('canvas');
        canvas.width = pngImage.width;
        canvas.height = pngImage.height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(pngImage, 0, 0);
        
        const pngData = {
          pngImageData: canvas.toDataURL('image/png'),
          metadata: pngMapInfo,
          width: pngImage.width,
          height: pngImage.height,
          zoomState: zoomState,
          timestamp: Date.now()
        };
        
        localStorage.setItem("savedPngData", JSON.stringify(pngData));
        console.log("💾 PNG Image saved to localStorage");
      } catch (e) {
        console.error("Error saving PNG image to localStorage:", e);
      }
    }
  }, [pgmImage, mapMetadata, pngImage, pngMapInfo, zoomState]);

  // Center map on canvas
  const centerMapOnCanvas = (width, height) => {
    if (canvasRef.current) {
      const container = canvasRef.current.parentElement;
      const offsetX = (container.clientWidth - width) / 2;
      const offsetY = (container.clientHeight - height) / 2;
      setZoomState(prev => ({ 
        ...prev, 
        offsetX: Math.max(0, offsetX), 
        offsetY: Math.max(0, offsetY), 
        scale: 1 
      }));
    }
  };

  // Check if mobile device
  useEffect(() => {
    const checkMobile = () => {
      setIsMobile(window.innerWidth <= 768);
    };
    checkMobile();
    window.addEventListener('resize', checkMobile);
    return () => window.removeEventListener('resize', checkMobile);
  }, []);

  // --- FIXED Coordinate conversions - ROS coordinates remain consistent regardless of rotation ---
  const rosToCanvasCoords = (rosX, rosY) => {
    if (!mapParamsRef.current) {
      console.log("⚠️ No map parameters for coordinate conversion");
      return { x: 0, y: 0 };
    }

    const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
    // Convert ROS coordinates to pixel coordinates
    const pixelX = (rosX - originX) / resolution;
    const pixelY = (rosY - originY) / resolution;
    
    // Flip Y-axis (ROS has Y-up, canvas has Y-down)
    const canvasX = pixelX;
    const canvasY = height - pixelY;
    
    // Apply rotation transformation to match the displayed image
    let rotatedX = canvasX;
    let rotatedY = canvasY;
    
    switch (rotation) {
      case 90:
        // For 90° rotation: (x, y) -> (height - y, x)
        rotatedX = height - canvasY;
        rotatedY = canvasX;
        break;
      case 180:
        // For 180° rotation: (x, y) -> (width - x, height - y)
        rotatedX = width - canvasX;
        rotatedY = height - canvasY;
        break;
      case 270:
        // For 270° rotation: (x, y) -> (y, width - x)
        rotatedX = canvasY;
        rotatedY = width - canvasX;
        break;
      default:
        // 0° rotation - no change
        rotatedX = canvasX;
        rotatedY = canvasY;
    }
    
    return { x: rotatedX, y: rotatedY };
  };

  const canvasToRosCoords = (canvasX, canvasY) => {
    if (!mapParamsRef.current) return null;
    const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
    // Apply inverse rotation transformation first
    let unrotatedX = canvasX;
    let unrotatedY = canvasY;
    
    switch (rotation) {
      case 90:
        // Inverse of 90°: (x, y) -> (y, height - x)
        unrotatedX = unrotatedY;
        unrotatedY = height - unrotatedX;
        break;
      case 180:
        // Inverse of 180°: (x, y) -> (width - x, height - y)
        unrotatedX = width - unrotatedX;
        unrotatedY = height - unrotatedY;
        break;
      case 270:
        // Inverse of 270°: (x, y) -> (width - y, x)
        unrotatedX = width - unrotatedY;
        unrotatedY = unrotatedX;
        break;
      default:
        // 0° rotation - no change
        break;
    }
    
    // Convert back to ROS coordinates
    const pixelY = height - unrotatedY; // Unflip Y-axis
    const worldX = originX + unrotatedX * resolution;
    const worldY = originY + pixelY * resolution;
    
    return { x: +worldX.toFixed(2), y: +worldY.toFixed(2) };
  };

  // --- Map Loader Functions ---
  const handleLoadMapFiles = () => {
    setMapLoaderActive(true);
  };

  const handleLoadPngImage = () => {
    setPngLoaderActive(true);
  };

  const handleYamlSelect = (e) => setYamlFile(e.target.files[0]);
  const handlePgmSelect = (e) => setPgmFile(e.target.files[0]);
  const handlePngYamlSelect = (e) => setPngYamlFile(e.target.files[0]);
  const handlePngSelect = (e) => setPngFile(e.target.files[0]);

  const handleLoadMap = async () => {
    try {
      if (!yamlFile || !pgmFile) {
        alert("Please select both YAML and PGM files first.");
        return;
      }

      // Parse YAML
      const yamlText = await yamlFile.text();
      const parsedYaml = yaml.load(yamlText);
      setMapInfo(parsedYaml);

      // Read PGM as binary
      const buffer = await pgmFile.arrayBuffer();
      const bytes = new Uint8Array(buffer);
      const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

      // Parse header
      const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
      const magic = headerLines[0];
      if (magic !== "P5" && magic !== "P2") {
        throw new Error("Unsupported PGM format: must be P5 or P2");
      }

      const width = parseInt(headerLines[1]);
      const height = parseInt(headerLines[2]);
      const maxVal = parseInt(headerLines[3]);

      // Find start of binary data
      const headerEnd = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
      const dataStart = headerEnd + 1;
      const pixelBytes = bytes.slice(dataStart);

      console.log(`📊 PGM Info: ${width}x${height}, maxVal: ${maxVal}`);

      // Convert PGM to canvas image
      const canvas = document.createElement('canvas');
      canvas.width = width;
      canvas.height = height;
      const ctx = canvas.getContext('2d');
      const imageData = ctx.createImageData(width, height);

      const negate = parsedYaml.negate || 0;

      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const i = y * width + x;
          if (i >= pixelBytes.length) continue;
          
          let value = pixelBytes[i];
          if (negate === 1) value = 255 - value;
          
          const dataIndex = (y * width + x) * 4;
          imageData.data[dataIndex] = value;
          imageData.data[dataIndex + 1] = value;
          imageData.data[dataIndex + 2] = value;
          imageData.data[dataIndex + 3] = 255;
        }
      }

      ctx.putImageData(imageData, 0, 0);

      const img = new Image();
      img.onload = () => {
        // Set map parameters from YAML file
        mapParamsRef.current = {
          width: width,
          height: height,
          resolution: parsedYaml.resolution || 0.05,
          originX: parsedYaml.origin?.[0] || 0,
          originY: parsedYaml.origin?.[1] || 0,
        };

        console.log("🗺️ PGM Map parameters:", mapParamsRef.current);

        const loadedMap = {
          width: width,
          height: height,
          resolution: parsedYaml.resolution || 0.05,
          data: new Int8Array(width * height).fill(-1),
          origin: {
            x: parsedYaml.origin?.[0] || 0,
            y: parsedYaml.origin?.[1] || 0,
            z: parsedYaml.origin?.[2] || 0
          }
        };

        setMapMsg(loadedMap);
        setEditableMap(loadedMap);
        setPgmImage(img);
        setMapMetadata(parsedYaml);
        setMapLoadedFromFile(true);
        setMapLoaderActive(false);
        setPngImage(null); // Clear PNG if PGM is loaded
        setRotation(0); // Reset rotation when loading new map

        centerMapOnCanvas(width, height);
        setDebugInfo(`🗺️ PGM Map loaded: ${width}x${height} | Resolution: ${parsedYaml.resolution || 0.05}`);
      };

      img.src = canvas.toDataURL('image/png');

    } catch (err) {
      console.error("Error loading map:", err);
      alert("Failed to load PGM map. Check console for details.");
    }
  };

  const handleLoadPng = async () => {
    try {
      if (!pngYamlFile || !pngFile) {
        alert("Please select both YAML and PNG files first.");
        return;
      }

      console.log("🖼️ Loading PNG file with YAML:", pngFile.name);

      // Parse YAML for PNG
      const yamlText = await pngYamlFile.text();
      const parsedYaml = yaml.load(yamlText);
      setPngMapInfo(parsedYaml);

      const img = new Image();
      const objectUrl = URL.createObjectURL(pngFile);
      
      img.onload = () => {
        console.log(`📊 PNG Dimensions: ${img.width}x${img.height}`);
        
        // Use coordinates from YAML file for PNG
        mapParamsRef.current = {
          width: img.width,
          height: img.height,
          resolution: parsedYaml.resolution || 0.05,
          originX: parsedYaml.origin?.[0] || 0,
          originY: parsedYaml.origin?.[1] || 0,
        };

        console.log("🗺️ PNG Map parameters (from YAML):", mapParamsRef.current);

        setPngImage(img);
        setMapLoadedFromFile(true);
        setPngLoaderActive(false);
        setPgmImage(null); // Clear PGM if PNG is loaded
        setMapMsg(null);
        setEditableMap(null);
        setMapMetadata(null);
        setRotation(0); // Reset rotation when loading new image

        centerMapOnCanvas(img.width, img.height);
        setDebugInfo(`🖼️ PNG Image loaded: ${img.width}x${img.height} | Resolution: ${parsedYaml.resolution || 0.05}`);
        
        URL.revokeObjectURL(objectUrl);
      };
      
      img.onerror = () => {
        console.error("❌ Failed to load PNG image");
        alert("Failed to load PNG image.");
        URL.revokeObjectURL(objectUrl);
      };
      
      img.src = objectUrl;

    } catch (err) {
      console.error("Error loading PNG:", err);
      alert("Failed to load PNG image. Check console for details.");
    }
  };

  const handleRemoveMap = () => {
    if (window.confirm("Are you sure you want to remove the current map/image?")) {
      setPgmImage(null);
      setPngImage(null);
      setMapMsg(null);
      setEditableMap(null);
      setMapMetadata(null);
      setPngMapInfo(null);
      setMapLoadedFromFile(false);
      setRotation(0); // Reset rotation
      localStorage.removeItem("savedMapData");
      localStorage.removeItem("savedPngData");
      setDebugInfo("🗺️ Map/Image removed");
      
      setZoomState({
        scale: 1,
        offsetX: 0,
        offsetY: 0,
        isDragging: false,
        lastX: 0,
        lastY: 0
      });
    }
  };

  // --- ROS connection / subscribers ---
  useEffect(() => {
    const hostname = window.location.hostname;
    const port = 9090;
    const ros = new ROSLIB.Ros();
    let mapListener = null;
    let reconnectTimer = null;

    const connect = () => {
      try {
        ros.connect(`ws://${hostname}:${port}`);
      } catch (err) {
        console.error("ROS connect error", err);
      }
    };

    ros.on("connection", () => {
      setRosConnected(true);
      setConnectionStatus("connected");
      setDebugInfo("✅ ROS Connected - Ready for robot data...");

      // Subscribe to map topic if not loaded from file
      if (!mapLoadedFromFile) {
        mapListener = new ROSLIB.Topic({
          ros,
          name: "/map",
          messageType: "nav_msgs/OccupancyGrid",
        });

        mapListener.subscribe((msg) => {
          const mapData = {
            width: msg.info.width,
            height: msg.info.height,
            resolution: msg.info.resolution,
            data: msg.data,
            origin: msg.info.origin.position,
          };

          mapParamsRef.current = {
            width: msg.info.width,
            height: msg.info.height,
            resolution: msg.info.resolution,
            originX: msg.info.origin.position.x,
            originY: msg.info.origin.position.y,
          };

          setMapMsg(mapData);
          setEditableMap({ ...mapData, data: [...msg.data] });
          setDebugInfo(`🗺️ Map loaded from ROS: ${msg.info.width}x${msg.info.height} | 🤖 Waiting for robot data...`);

          centerMapOnCanvas(msg.info.width, msg.info.height);
        });
      }
    });

    ros.on("error", (err) => {
      console.error("ROS error", err);
      setRosConnected(false);
      setConnectionStatus("error");
      setDebugInfo("❌ ROS Connection Error");
    });

    ros.on("close", () => {
      setRosConnected(false);
      setConnectionStatus("disconnected");
      setDebugInfo("🔌 ROS Disconnected - Reconnecting...");
      clearTimeout(reconnectTimer);
      reconnectTimer = setTimeout(connect, 2000);
    });

    connect();

    return () => {
      mapListener?.unsubscribe();
      clearTimeout(reconnectTimer);
      ros.close();
    };
  }, [mapLoadedFromFile]);

  // Fetch robot position from Flask API
  const fetchRobotPosition = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/robot_position`);
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      
      const data = await response.json();
      
      if ((data.status === 'success' || data.status === 'waiting') && mapParamsRef.current) {
        const positionData = data.data;
        const { x, y, timestamp } = positionData;
        
        if (x !== undefined && y !== undefined) {
          // Always convert ROS coordinates to canvas coordinates using current rotation
          const canvasCoords = rosToCanvasCoords(x, y);

          setMessageCount(prev => prev + 1);
          setDebugInfo(`🤖 Robot: (${x.toFixed(2)}, ${y.toFixed(2)}) | Rotation: ${rotation}° | Updates: ${messageCount + 1}`);
          
          setRobotPosition({
            rosX: x,
            rosY: y,
            canvasX: canvasCoords.x,
            canvasY: canvasCoords.y,
            timestamp: timestamp,
          });

          localStorage.setItem("lastRobotPose", JSON.stringify({
            rosX: x,
            rosY: y,
            canvasX: canvasCoords.x,
            canvasY: canvasCoords.y,
            timestamp: timestamp,
          }));

          setApiConnected(true);
          setConnectionStatus("connected");
        } else if (data.status === 'waiting') {
          setDebugInfo("⏳ Waiting for robot position data...");
        }
      }
    } catch (error) {
      console.error("❌ Error fetching robot position:", error);
      setApiConnected(false);
      setConnectionStatus("error");
      setDebugInfo("❌ Failed to connect to position API");
    }
  };

  // Check API health
  const checkApiHealth = async () => {
    try {
      const response = await fetch(`${API_BASE_URL}/health`);
      if (response.ok) {
        setApiConnected(true);
        setConnectionStatus("connected");
        setDebugInfo("✅ API Connected - Waiting for map...");
      }
    } catch (error) {
      console.error("❌ API health check failed:", error);
      setApiConnected(false);
      setConnectionStatus("error");
    }
  };

  // API connection
  useEffect(() => {
    console.log("🔄 Connecting to Flask API...");
    setDebugInfo("Connecting to position API...");
    setConnectionStatus("connecting");

    checkApiHealth();
    positionIntervalRef.current = setInterval(fetchRobotPosition, 100);

    return () => {
      if (positionIntervalRef.current) {
        clearInterval(positionIntervalRef.current);
      }
    };
  }, []);

  // Restore robot position from localStorage - recalculate canvas coordinates when rotation changes
  useEffect(() => {
    const savedPose = localStorage.getItem("lastRobotPose");
    if (savedPose && mapParamsRef.current) {
      try {
        const pose = JSON.parse(savedPose);
        // Recalculate canvas coordinates using current rotation
        const canvasCoords = rosToCanvasCoords(pose.rosX, pose.rosY);
        setRobotPosition({
          rosX: pose.rosX,
          rosY: pose.rosY,
          canvasX: canvasCoords.x,
          canvasY: canvasCoords.y,
          timestamp: pose.timestamp,
        });
        console.log("💾 Restored robot position from localStorage with current rotation");
      } catch (e) {
        console.error("Error restoring robot position:", e);
      }
    }
  }, [rotation]); // Re-run when rotation changes

  // Touch event handlers
  const handleTouchStart = (e) => {
    if (e.touches.length === 1) {
      const touch = e.touches[0];
      const rect = canvasRef.current.getBoundingClientRect();
      const x = touch.clientX - rect.left;
      const y = touch.clientY - rect.top;
      
      setZoomState(prev => ({
        ...prev,
        isDragging: true,
        lastX: x,
        lastY: y
      }));
    }
    e.preventDefault();
  };

  const handleTouchMove = (e) => {
    if (e.touches.length === 1 && zoomState.isDragging) {
      const touch = e.touches[0];
      const rect = canvasRef.current.getBoundingClientRect();
      const x = touch.clientX - rect.left;
      const y = touch.clientY - rect.top;

      const deltaX = x - zoomState.lastX;
      const deltaY = y - zoomState.lastY;
      
      setZoomState(prev => ({
        ...prev,
        offsetX: prev.offsetX + deltaX,
        offsetY: prev.offsetY + deltaY,
        lastX: x,
        lastY: y
      }));
    }
    e.preventDefault();
  };

  const handleTouchEnd = () => {
    setZoomState(prev => ({
      ...prev,
      isDragging: false
    }));
  };

  // Mouse event handlers
  const handleMouseDown = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    setZoomState(prev => ({
      ...prev,
      isDragging: true,
      lastX: x,
      lastY: y
    }));
  };

  const handleMouseMove = (e) => {
    if (!zoomState.isDragging) return;

    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    const deltaX = x - zoomState.lastX;
    const deltaY = y - zoomState.lastY;
    
    setZoomState(prev => ({
      ...prev,
      offsetX: prev.offsetX + deltaX,
      offsetY: prev.offsetY + deltaY,
      lastX: x,
      lastY: y
    }));
  };

  const handleMouseUp = () => {
    setZoomState(prev => ({
      ...prev,
      isDragging: false
    }));
  };

  // Zoom handlers
  const handleWheel = (e) => {
    if (!mapMsg && !pgmImage && !pngImage) return;
    e.preventDefault();
    e.stopPropagation();
    
    const rect = canvasRef.current.getBoundingClientRect();
    const containerWidth = rect.width;
    const containerHeight = rect.height;
    
    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;
    
    const delta = -e.deltaY;
    const zoomFactor = delta > 0 ? 1.1 : 0.9;
    const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
    const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
    const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
    setZoomState(prev => ({ 
      ...prev, 
      scale: newScale,
      offsetX: centerX - zoomPointX * newScale,
      offsetY: centerY - zoomPointY * newScale
    }));
  };

  const handleButtonZoom = (zoomFactor) => {
    if ((!mapMsg && !pgmImage && !pngImage) || !canvasRef.current) return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const containerWidth = rect.width;
    const containerHeight = rect.height;
    
    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;
    
    const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
    const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
    const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
    setZoomState(prev => ({ 
      ...prev, 
      scale: newScale,
      offsetX: centerX - zoomPointX * newScale,
      offsetY: centerY - zoomPointY * newScale
    }));
  };

  // Rotation handler
  const handleRotate = () => {
    setRotation((prev) => (prev + 90) % 360);
  };

  // Reset rotation handler
  const handleResetRotation = () => {
    setRotation(0);
  };

  // --- Drawing the map ---
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d", { willReadFrequently: true });
    ctx.imageSmoothingEnabled = false;

    const container = canvas.parentElement;
    const containerWidth = container.clientWidth;
    const containerHeight = container.clientHeight;
    canvas.width = containerWidth;
    canvas.height = containerHeight;
    canvas.style.width = `${containerWidth}px`;
    canvas.style.height = `${containerHeight}px`;

    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // If we have a PNG image loaded, draw it with rotation
    if (pngImage) {
      const { width, height } = pngImage;
      
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation
      if (rotation !== 0) {
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      // Draw the PNG image
      ctx.drawImage(pngImage, 0, 0, width, height);

      // Draw robot - coordinates are already rotated in rosToCanvasCoords
      if (robotPosition) {
        const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
        // Draw robot as green circle with shadow
        ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
        ctx.shadowBlur = 5;
        ctx.shadowOffsetX = 2;
        ctx.shadowOffsetY = 2;
        
        ctx.fillStyle = '#28a745';
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, Math.max(2, 4 / zoomState.scale), 0, 2 * Math.PI);
        ctx.fill();
        
        // Reset shadow
        ctx.shadowColor = 'transparent';
        ctx.shadowBlur = 0;
        ctx.shadowOffsetX = 0;
        ctx.shadowOffsetY = 0;
        
        // Add black border
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 2 / zoomState.scale;
        ctx.stroke();
        
        // Add coordinates label with background

      }

      ctx.restore();
      return;
    }
    
    // If we have a PGM image loaded, draw it with rotation
    if (pgmImage) {
      const { width, height } = pgmImage;
      
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation
      if (rotation !== 0) {
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      ctx.drawImage(pgmImage, 0, 0, width, height);

      if (robotPosition) {
        const { canvasX, canvasY, rosX, rosY } = robotPosition;
        
        ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
        ctx.shadowBlur = 5;
        ctx.shadowOffsetX = 2;
        ctx.shadowOffsetY = 2;
        
        ctx.fillStyle = '#28a745';
        ctx.beginPath();
        ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
        ctx.fill();
        
        ctx.shadowColor = 'transparent';
        ctx.shadowBlur = 0;
        ctx.shadowOffsetX = 0;
        ctx.shadowOffsetY = 0;
        
        ctx.strokeStyle = '#000';
        ctx.lineWidth = 2 / zoomState.scale;
        ctx.stroke();
        

      }

      ctx.restore();
      return;
    }

    // Original ROS map rendering
    const mapToRender = editableMap || mapMsg;
    if (!mapToRender) {
      ctx.fillStyle = '#f8f9fa';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#6c757d';
      ctx.font = isMobile ? '14px Arial' : '16px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('Load a PGM map or PNG image to visualize robot movement', canvas.width / 2, canvas.height / 2);
      return;
    }

    const { width, height, data } = mapToRender;

    ctx.save();
    ctx.translate(zoomState.offsetX, zoomState.offsetY);
    ctx.scale(zoomState.scale, zoomState.scale);

    // Apply rotation transformation for ROS maps
    if (rotation !== 0) {
      const centerX = width / 2;
      const centerY = height / 2;
      ctx.translate(centerX, centerY);
      ctx.rotate((rotation * Math.PI) / 180);
      ctx.translate(-centerX, -centerY);
    }

    const imageData = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const val = data[y * width + x];
        let r, g, b;
        
        if (val === -1) {
          r = g = b = 200;
        } else if (val === 0) {
          r = g = b = 255;
        } else if (val === 100) {
          r = g = b = 100;
        } else if (val > 0 && val < 100) {
          const intensity = 255 - Math.floor(val * 2.2);
          r = g = b = Math.max(100, Math.min(200, intensity));
        } else {
          r = 255; g = 200; b = 200;
        }
        
        const py = height - 1 - y;
        const i = (py * width + x) * 4;
        imageData.data[i] = r;
        imageData.data[i + 1] = g;
        imageData.data[i + 2] = b;
        imageData.data[i + 3] = 255;
      }
    }
    
    const off = document.createElement("canvas");
    off.width = width; off.height = height;
    off.getContext("2d").putImageData(imageData, 0, 0);
    ctx.drawImage(off, 0, 0);

    if (robotPosition) {
      const { canvasX, canvasY, rosX, rosY } = robotPosition;
      
      ctx.shadowColor = 'rgba(0, 0, 0, 0.3)';
      ctx.shadowBlur = 5;
      ctx.shadowOffsetX = 2;
      ctx.shadowOffsetY = 2;
      
      ctx.fillStyle = '#28a745';
      ctx.beginPath();
      ctx.arc(canvasX, canvasY, Math.max(4, 8 / zoomState.scale), 0, 2 * Math.PI);
      ctx.fill();
      
      ctx.shadowColor = 'transparent';
      ctx.shadowBlur = 0;
      ctx.shadowOffsetX = 0;
      ctx.shadowOffsetY = 0;
      
      ctx.strokeStyle = '#000';
      ctx.lineWidth = 2 / zoomState.scale;
      ctx.stroke();
      
      const label = `(${rosX.toFixed(2)}, ${rosY.toFixed(2)})`;
      ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
      ctx.fillRect(canvasX + 8, canvasY - 25, 100, 18);
      
      ctx.fillStyle = '#000';
      ctx.font = isMobile ? '10px Arial' : '12px Arial';
      ctx.fillText(label, canvasX + 12, canvasY - 12);
    }

    ctx.restore();
  }, [mapMsg, editableMap, zoomState, robotPosition, isMobile, pgmImage, pngImage, rotation]);

  // Map Loader Modal
  const MapLoaderModal = () => {
    return (
      <div style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        background: 'rgba(0,0,0,0.7)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 1000
      }}>
        <div style={{
          background: '#ffffff',
          padding: 24,
          borderRadius: 12,
          border: '1px solid #e2e8f0',
          maxWidth: 500,
          width: '90%',
          boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
        }}>
          <h3 style={{ margin: '0 0 20px 0', color: '#1e293b' }}>
            🗺️ Load PGM Map from Files
          </h3>
          
          <div style={{ marginBottom: 15 }}>
            <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
              Select YAML File:
              <div style={{ 
                marginTop: 6,
                border: '1px solid #e2e8f0',
                borderRadius: 8,
                padding: '10px 12px',
                background: '#ffffff',
                color: yamlFile ? '#1e293b' : '#64748b',
                fontSize: '14px',
                cursor: 'pointer',
                position: 'relative',
                overflow: 'hidden'
              }}>
                {yamlFile ? yamlFile.name : 'Choose YAML file...'}
                <input 
                  type="file" 
                  accept=".yaml,.yml" 
                  onChange={handleYamlSelect} 
                  style={{ 
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    width: '100%',
                    height: '100%',
                    opacity: 0,
                    cursor: 'pointer'
                  }}
                />
              </div>
            </label>
          </div>

          <div style={{ marginBottom: 20 }}>
            <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
              Select PGM File:
              <div style={{ 
                marginTop: 6,
                borderRadius: 8,
                padding: '10px 12px',
                background: '#ffffff',
                color: pgmFile ? '#1e293b' : '#64748b',
                fontSize: '14px',
                cursor: 'pointer',
                position: 'relative',
                overflow: 'hidden'
              }}>
                {pgmFile ? pgmFile.name : 'Choose PGM file...'}
                <input 
                  type="file" 
                  accept=".pgm" 
                  onChange={handlePgmSelect} 
                  style={{ 
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    width: '100%',
                    height: '100%',
                    opacity: 0,
                    cursor: 'pointer'
                  }}
                />
              </div>
            </label>
          </div>

          {mapInfo && (
            <div style={{
              marginBottom: 15,
              padding: 10,
              background: '#f8fafc',
              borderRadius: 8,
              border: '1px solid #e2e8f0',
            }}>
              <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Map Info:</h4>
              <pre style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
                {JSON.stringify(mapInfo, null, 2)}
              </pre>
            </div>
          )}

          <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
            <button 
              onClick={() => {
                setMapLoaderActive(false);
                setYamlFile(null);
                setPgmFile(null);
              }}
              style={{
                padding: "10px 12px",
                borderRadius: 8,
                border: "1px solid #e2e8f0",
                background: "#ffffff",
                color: "#1e293b",
                cursor: "pointer",
                fontSize: '14px',
                fontWeight: '500'
              }}
            >
              Cancel
            </button>
            <button 
              onClick={handleLoadMap}
              style={{
                padding: "10px 12px",
                borderRadius: 8,
                border: "none",
                background: "#3b82f6",
                color: "white",
                cursor: "pointer",
                fontSize: '14px',
                fontWeight: '500'
              }}
              disabled={!yamlFile || !pgmFile}
            >
              Load PGM Map
            </button>
          </div>
        </div>
      </div>
    );
  };

  // PNG Loader Modal
  const PngLoaderModal = () => {
    return (
      <div style={{
        position: 'fixed',
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        background: 'rgba(0,0,0,0.7)',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: 1000
      }}>
        <div style={{
          background: '#ffffff',
          padding: 24,
          borderRadius: 12,
          border: '1px solid #e2e8f0',
          maxWidth: 500,
          width: '90%',
          boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
        }}>
          <h3 style={{ margin: '0 0 20px 0', color: '#1e293b' }}>
            🖼️ Load PNG Image with YAML Coordinates
          </h3>
          
          <div style={{ marginBottom: 15 }}>
            <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
              Select YAML File:
              <div style={{ 
                marginTop: 6,
                border: '1px solid #e2e8f0',
                borderRadius: 8,
                padding: '10px 12px',
                background: '#ffffff',
                color: pngYamlFile ? '#1e293b' : '#64748b',
                fontSize: '14px',
                cursor: 'pointer',
                position: 'relative',
                overflow: 'hidden'
              }}>
                {pngYamlFile ? pngYamlFile.name : 'Choose YAML file...'}
                <input 
                  type="file" 
                  accept=".yaml,.yml" 
                  onChange={handlePngYamlSelect} 
                  style={{ 
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    width: '100%',
                    height: '100%',
                    opacity: 0,
                    cursor: 'pointer'
                  }}
                />
              </div>
            </label>
          </div>

          <div style={{ marginBottom: 20 }}>
            <label style={{ fontSize: "14px", color: "#1e293b", fontWeight: '600', marginBottom: 4, display: 'block' }}>
              Select PNG File:
              <div style={{ 
                marginTop: 6,
                borderRadius: 8,
                padding: '10px 12px',
                background: '#ffffff',
                color: pngFile ? '#1e293b' : '#64748b',
                fontSize: '14px',
                cursor: 'pointer',
                position: 'relative',
                overflow: 'hidden'
              }}>
                {pngFile ? pngFile.name : 'Choose PNG file...'}
                <input 
                  type="file" 
                  accept=".png" 
                  onChange={handlePngSelect}
                  style={{ 
                    position: 'absolute',
                    top: 0,
                    left: 0,
                    width: '100%',
                    height: '100%',
                    opacity: 0,
                    cursor: 'pointer'
                  }}
                />
              </div>
            </label>
          </div>

          {pngMapInfo && (
            <div style={{
              marginBottom: 15,
              padding: 10,
              background: '#f8fafc',
              borderRadius: 8,
              border: '1px solid #e2e8f0',
            }}>
              <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Map Info:</h4>
              <pre style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
                {JSON.stringify(pngMapInfo, null, 2)}
              </pre>
            </div>
          )}

          <div style={{
            marginBottom: 20,
            padding: 10,
            background: '#f0f9ff',
            borderRadius: 8,
            border: '1px solid #e0f2fe',
          }}>
            <h4 style={{ margin: '0 0 8px 0', color: '#1e293b' }}>Coordinate Information:</h4>
            <p style={{ margin: 0, fontSize: "12px", color: "#64748b" }}>
              • Using coordinates from YAML file<br/>
              • Robot will maintain correct position during rotation<br/>
              • Make sure YAML file has correct resolution and origin coordinates
            </p>
          </div>

          <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
            <button 
              onClick={() => {
                setPngLoaderActive(false);
                setPngYamlFile(null);
                setPngFile(null);
              }}
              style={{
                padding: "10px 12px",
                borderRadius: 8,
                border: "1px solid #e2e8f0",
                background: "#ffffff",
                color: "#1e293b",
                cursor: "pointer",
                fontSize: '14px',
                fontWeight: '500'
              }}
            >
              Cancel
            </button>
            <button 
              onClick={handleLoadPng}
              style={{
                padding: "10px 12px",
                borderRadius: 8,
                border: "none",
                background: "#10b981",
                color: "white",
                cursor: "pointer",
                fontSize: '14px',
                fontWeight: '500'
              }}
              disabled={!pngYamlFile || !pngFile}
            >
              Load PNG Image
            </button>
          </div>
        </div>
      </div>
    );
  };

  const getStatusColor = () => {
    switch(connectionStatus) {
      case 'connected': return 'linear-gradient(135deg, #10b981, #059669)';
      case 'connecting': return 'linear-gradient(135deg, #f59e0b, #d97706)';
      case 'error': return 'linear-gradient(135deg, #ef4444, #dc2626)';
      default: return 'linear-gradient(135deg, #6b7280, #4b5563)';
    }
  };

  const getStatusText = () => {
    switch(connectionStatus) {
      case 'connected': return isMobile ? 'API Connected' : 'API Connected';
      case 'connecting': return isMobile ? 'Connecting...' : 'Connecting...';
      case 'error': return isMobile ? 'Error' : 'Connection Error';
      default: return isMobile ? 'Offline' : 'Disconnected';
    }
  };

  return (
    <div style={{ 
      display: "flex", 
      flexDirection: "column", 
      padding: isMobile ? "0.5rem" : "1rem", 
      width: "100%", 
      boxSizing: "border-box",
      minHeight: "100vh",
      background: "white",
      overflow: "hidden"
    }}>
      
      {mapLoaderActive && <MapLoaderModal />}
      {pngLoaderActive && <PngLoaderModal />}

      {/* Debug Status Bar */}
      <div style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        padding: isMobile ? "8px 12px" : "12px 16px",
        background: getStatusColor(),
        borderRadius: "8px",
        marginBottom: isMobile ? "0.5rem" : "1rem",
        color: "white",
        fontWeight: "600",
        boxShadow: "0 2px 8px rgba(0,0,0,0.1)",
        flexWrap: isMobile ? "wrap" : "nowrap",
        gap: isMobile ? "4px" : "0"
      }}>
        <div style={{ 
          display: "flex", 
          alignItems: "center", 
          gap: "8px",
          flex: isMobile ? "1 1 100%" : "0 1 auto"
        }}>
          <div style={{
            width: "12px",
            height: "12px",
            borderRadius: "50%",
            background: connectionStatus === 'connected' ? "#4ade80" : 
                       connectionStatus === 'connecting' ? "#fbbf24" : "#f87171",
            animation: connectionStatus === 'connected' ? "pulse 2s infinite" : "none",
          }}></div>
          <span style={{ fontSize: isMobile ? "0.9rem" : "1rem" }}>{getStatusText()}</span>
          <span style={{ 
            fontSize: isMobile ? "0.7rem" : "0.8rem", 
            opacity: 0.8,
            marginLeft: "8px",
            display: "flex",
            alignItems: "center",
            gap: "4px"
          }}>
            <div style={{
              width: "6px",
              height: "6px",
              borderRadius: "50%",
              background: rosConnected ? "#4ade80" : "#f87171"
            }}></div>
            ROS: {rosConnected ? "Connected" : "Disconnected"}
          </span>
        </div>
        
        {/* Map Control Buttons */}
        <div style={{ 
          display: "flex", 
          gap: "8px",
          alignItems: "center",
          flex: isMobile ? "1 1 100%" : "0 1 auto",
          marginTop: isMobile ? "4px" : "0",
          justifyContent: isMobile ? "center" : "flex-end"
        }}>
          <button
            onClick={handleLoadMapFiles}
            style={{
              padding: isMobile ? "6px 8px" : "8px 12px",
              background: "#8b5cf6",
              color: "white",
              border: "none",
              borderRadius: "6px",
              cursor: "pointer",
              fontSize: isMobile ? "10px" : "12px",
              display: "flex",
              alignItems: "center",
              gap: "4px"
            }}
            title="Load YAML+PGM Map"
          >
            <FaUpload />
            {!isMobile && "Load PGM"}
          </button>

          <button
            onClick={handleLoadPngImage}
            style={{
              padding: isMobile ? "6px 8px" : "8px 12px",
              background: "#10b981",
              color: "white",
              border: "none",
              borderRadius: "6px",
              cursor: "pointer",
              fontSize: isMobile ? "10px" : "12px",
              display: "flex",
              alignItems: "center",
              gap: "4px"
            }}
            title="Load PNG Image with YAML"
          >
            <FaImage />
            {!isMobile && "Load PNG"}
          </button>

          {(pgmImage || pngImage || mapMsg) && (
            <button
              onClick={handleRemoveMap}
              style={{
                padding: isMobile ? "6px 8px" : "8px 12px",
                background: "#ef4444",
                color: "white",
                border: "none",
                borderRadius: "6px",
                cursor: "pointer",
                fontSize: isMobile ? "10px" : "12px",
                display: "flex",
                alignItems: "center",
                gap: "4px"
              }}
              title="Remove Map/Image"
            >
              <FaTrash />
              {!isMobile && "Remove"}
            </button>
          )}
        </div>
        
        <div style={{ 
          fontSize: isMobile ? "0.7rem" : "0.8rem", 
          opacity: 0.9,
          background: "rgba(255,255,255,0.2)",
          padding: "4px 8px",
          borderRadius: "4px",
          fontFamily: "monospace",
          maxWidth: isMobile ? "100%" : "400px",
          overflow: "hidden",
          textOverflow: "ellipsis",
          whiteSpace: "nowrap",
          flex: isMobile ? "1 1 100%" : "0 1 auto",
          marginTop: isMobile ? "4px" : "0",
          textAlign: isMobile ? "center" : "left"
        }}>
          {debugInfo}
        </div>
      </div>

      {/* Map Container */}
      <div style={{ 
        flex: 1,
        width: "100%",
        minHeight: isMobile ? "60vh" : "500px"
      }}>
        <div style={{ 
          width: "100%", 
          height: "100%",
          border: "2px solid #e2e8f0", 
          borderRadius: "8px", 
          background: "#f8fafc", 
          minHeight: isMobile ? "300px" : "500px",
          position: "relative",
          overflow: "hidden",
          cursor: zoomState.isDragging ? "grabbing" : "grab",
          touchAction: "none"
        }}>
          {/* Controls */}
          <div style={{
            position: "absolute",
            top: isMobile ? "5px" : "10px",
            right: isMobile ? "5px" : "10px",
            display: "flex",
            flexDirection: "column",
            gap: isMobile ? "4px" : "6px",
            zIndex: 10,
            background: "rgba(255, 255, 255, 0.95)",
            padding: isMobile ? "6px" : "10px",
            borderRadius: "6px",
            boxShadow: "0 4px 12px rgba(0,0,0,0.15)",
          }}>

            <div style={{
              padding: "4px",
              background: "#f8f9fa",
              borderRadius: "4px",
              textAlign: "center",
              fontSize: isMobile ? "9px" : "11px",
              fontWeight: "bold",
              color: "#495057",
              marginBottom: "2px"
            }}>
              {Math.round(zoomState.scale * 100)}%
            </div>

            <button
              onClick={() => handleButtonZoom(1.2)}
              disabled={zoomState.scale >= 5}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: zoomState.scale >= 5 ? "#6c757d" : "#007bff",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "6px",
                cursor: zoomState.scale >= 5 ? "not-allowed" : "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "12px" : "14px"
              }}
              title="Zoom In"
            >
              <FaSearchPlus />
            </button>

            <button
              onClick={() => handleButtonZoom(0.8)}
              disabled={zoomState.scale <= 0.1}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: zoomState.scale <= 0.1 ? "#6c757d" : "#6c757d",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "4px",
                cursor: zoomState.scale <= 0.1 ? "not-allowed" : "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "12px" : "14px"
              }}
              title="Zoom Out"
            >
              <FaSearchMinus />
            </button>

            <button
              onClick={() => {
                if (!mapMsg && !pgmImage && !pngImage) return;
                const container = canvasRef.current.parentElement;
                const mapWidth = pngImage ? pngImage.width : (pgmImage ? pgmImage.width : mapMsg.width);
                const mapHeight = pngImage ? pngImage.height : (pgmImage ? pgmImage.height : mapMsg.height);
                const offsetX = (container.clientWidth - mapWidth) / 2;
                const offsetY = (container.clientHeight - mapHeight) / 2;
                setZoomState((p) => ({ ...p, scale: 1, offsetX, offsetY }));
              }}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: "#ff6b35",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "4px",
                cursor: "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "10px" : "12px"
              }}
              title="Fit Map to Screen"
            >
              <FaExpand />
            </button>

            <button
              onClick={handleRotate}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: "#f59e0b",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "4px",
                cursor: "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "10px" : "12px"
              }}
              title="Rotate 90°"
            >
              <FaRedo />
            </button>

            <button
              onClick={handleResetRotation}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: rotation === 0 ? "#6c757d" : "#8b5cf6",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "4px",
                cursor: rotation === 0 ? "not-allowed" : "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "10px" : "12px"
              }}
              title="Reset Rotation"
              disabled={rotation === 0}
            >
              {rotation}°
            </button>

            <button
              onClick={fetchRobotPosition}
              style={{
                padding: isMobile ? "6px" : "8px",
                background: "#8b5cf6",
                color: "white",
                border: "none",
                borderRadius: isMobile ? "4px" : "4px",
                cursor: "pointer",
                width: isMobile ? "28px" : "36px",
                height: isMobile ? "28px" : "36px",
                fontSize: isMobile ? "10px" : "12px"
              }}
              title="Refresh Position"
            >
              <FaSync />
            </button>
          </div>

          <canvas 
            ref={canvasRef} 
            style={{ 
              width: "100%",
              height: "100%", 
              imageRendering: "pixelated",
              display: "block",
              touchAction: "none"
            }}
            onMouseDown={!isMobile ? handleMouseDown : undefined}
            onMouseMove={!isMobile ? handleMouseMove : undefined}
            onMouseUp={!isMobile ? handleMouseUp : undefined}
            onMouseLeave={!isMobile ? handleMouseUp : undefined}
            onWheel={!isMobile ? handleWheel : undefined}
            onTouchStart={isMobile ? handleTouchStart : undefined}
            onTouchMove={isMobile ? handleTouchMove : undefined}
            onTouchEnd={isMobile ? handleTouchEnd : undefined}
            onTouchCancel={isMobile ? handleTouchEnd : undefined}
          />
        </div>
      </div>
    </div>
  );
}