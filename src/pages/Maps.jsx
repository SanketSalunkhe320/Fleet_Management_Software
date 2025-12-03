
import React, { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";
import yaml from "js-yaml";
import JSZip from 'jszip';
import { 
  FaSearchPlus, 
  FaSearchMinus, 
  FaExpand, 
  FaTrashAlt, 
  FaMapMarkerAlt,
  FaArrowRight,
  FaDrawPolygon,
  FaEraser,
  FaSave,
  FaUpload,
  FaWarehouse,
  FaDownload,
  FaRedo,
  FaCropAlt,
  FaImage,
  FaMoon,
  FaSun,
  FaBan,
  FaBrush,
  FaCut
} from "react-icons/fa";

// OpenCV loader
const loadOpenCV = () => {
  return new Promise((resolve, reject) => {
    if (window.cv) {
      resolve();
      return;
    }
    
    const script = document.createElement('script');
    script.src = 'https://docs.opencv.org/4.5.0/opencv.js';
    script.onload = () => {
      const checkOpenCV = () => {
        if (window.cv && window.cv.Mat) {
          resolve();
        } else {
          setTimeout(checkOpenCV, 50);
        }
      };
      checkOpenCV();
    };
    script.onerror = reject;
    document.head.appendChild(script);
  });
};

export default function MapEditor() {
  // --- Theme state with localStorage persistence ---
  const [darkMode, setDarkMode] = useState(() => {
    const saved = localStorage.getItem("mapEditorDarkMode");
    if (saved !== null) {
      return JSON.parse(saved);
    }
    return window.matchMedia?.('(prefers-color-scheme: dark)').matches || false;
  });

  // --- ROS & map state ---
  const [rosConnected, setRosConnected] = useState(false);
  const [mapMsg, setMapMsg] = useState(null);
  const [editableMap, setEditableMap] = useState(null);
  const mapParamsRef = useRef(null);
  const [opencvLoaded, setOpencvLoaded] = useState(false);

  // --- Map Loader State ---
  const [mapLoaderActive, setMapLoaderActive] = useState(false);
  const [yamlFile, setYamlFile] = useState(null);
  const [pgmFile, setPgmFile] = useState(null);
  const [mapInfo, setMapInfo] = useState(null);

  // --- Editor state ---
  const [tool, setTool] = useState(() => {
    const saved = localStorage.getItem("tool");
    return saved || "place_node";
  });
  const [nodeType, setNodeType] = useState(() => {
    const saved = localStorage.getItem("nodeType");
    return saved || "station";
  });
  const [mapName, setMapName] = useState(() => {
    const saved = localStorage.getItem("mapName");
    return saved || "";
  });
  const [zoneName, setZoneName] = useState("");
  const [zoneType, setZoneType] = useState("normal"); // "normal" or "keep_out"
  const [nodes, setNodes] = useState(() => {
    const saved = localStorage.getItem("nodes");
    return saved ? JSON.parse(saved) : [];
  });
  const [arrows, setArrows] = useState(() => {
    const saved = localStorage.getItem("arrows");
    return saved ? JSON.parse(saved) : [];
  });
  const [zones, setZones] = useState(() => {
    const saved = localStorage.getItem("zones");
    return saved ? JSON.parse(saved) : [];
  });
  const [currentZonePoints, setCurrentZonePoints] = useState(() => {
    const saved = localStorage.getItem("currentZonePoints");
    return saved ? JSON.parse(saved) : [];
  });

  // --- Arrow drawing state ---
  const [arrowDrawing, setArrowDrawing] = useState({
    isDrawing: false,
    fromId: null,
    points: []
  });

  // --- Rotation state ---
  const [rotation, setRotation] = useState(() => {
    const saved = localStorage.getItem("rotation");
    return saved ? parseInt(saved) : 0;
  });

  // --- Scissor Crop state ---
  const [cropState, setCropState] = useState({
    isCropping: false,
    startX: 0,
    startY: 0,
    endX: 0,
    endY: 0,
    isDragging: false,
    freehandPoints: [] // Store freehand points
  });

  // --- Erase mode state ---
  const [eraseMode, setEraseMode] = useState(() => {
    const saved = localStorage.getItem("eraseMode");
    return saved || "objects"; // "objects", "noise", or "hand"
  });
  const [eraseRadius, setEraseRadius] = useState(() => {
    const saved = localStorage.getItem("eraseRadius");
    return saved ? parseInt(saved) : 3;
  });

  // --- Hand erase state ---
  const [handEraseState, setHandEraseState] = useState({
    isErasing: false,
    lastX: 0,
    lastY: 0
  });

  // --- UI helpers ---
  const canvasRef = useRef(null);
  const [zoomState, setZoomState] = useState({
    scale: 1,
    offsetX: 500,
    offsetY: 110,
    isDragging: false,
    lastX: 0,
    lastY: 0,
  });
  const [cursorCoords, setCursorCoords] = useState(null);
  const idCounter = useRef(1);

  // --- Database state ---
  const [dbStatus, setDbStatus] = useState("Ready");
  const [isSaving, setIsSaving] = useState(false);

  // --- Auto-save to localStorage ---
  useEffect(() => {
    localStorage.setItem("nodes", JSON.stringify(nodes));
  }, [nodes]);

  useEffect(() => {
    localStorage.setItem("arrows", JSON.stringify(arrows));
  }, [arrows]);

  useEffect(() => {
    localStorage.setItem("zones", JSON.stringify(zones));
  }, [zones]);

  useEffect(() => {
    localStorage.setItem("currentZonePoints", JSON.stringify(currentZonePoints));
  }, [currentZonePoints]);

  useEffect(() => {
    localStorage.setItem("mapName", mapName);
  }, [mapName]);

  useEffect(() => {
    localStorage.setItem("tool", tool);
  }, [tool]);

  useEffect(() => {
    localStorage.setItem("nodeType", nodeType);
  }, [nodeType]);

  useEffect(() => {
    localStorage.setItem("rotation", rotation.toString());
  }, [rotation]);

  useEffect(() => {
    localStorage.setItem("eraseMode", eraseMode);
  }, [eraseMode]);

  useEffect(() => {
    localStorage.setItem("eraseRadius", eraseRadius.toString());
  }, [eraseRadius]);

  // --- Dark mode persistence ---
  useEffect(() => {
    localStorage.setItem("mapEditorDarkMode", JSON.stringify(darkMode));
  }, [darkMode]);

  // --- Load OpenCV on component mount ---
  useEffect(() => {
    loadOpenCV()
      .then(() => {
        setOpencvLoaded(true);
        console.log("OpenCV.js loaded successfully");
      })
      .catch(err => {
        console.error("Failed to load OpenCV:", err);
      });
  }, []);

  // --- System dark mode detection (only if no user preference) ---
  useEffect(() => {
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    
    const handleChange = (e) => {
      const hasUserChoice = localStorage.getItem("mapEditorDarkMode") !== null;
      if (!hasUserChoice) {
        setDarkMode(e.matches);
      }
    };
    
    mediaQuery.addEventListener('change', handleChange);
    return () => mediaQuery.removeEventListener('change', handleChange);
  }, []);

  // --- Theme toggle function ---
  const toggleDarkMode = () => {
    setDarkMode(prev => !prev);
  };

  // --- Theme styles ---
  const theme = {
    light: {
      background: '#ffffff',
      surface: '#f8fafc',
      surfaceSecondary: '#ffffff',
      card: '#ffffff',
      text: '#1e293b',
      textSecondary: '#64748b',
      border: '#e2e8f0',
      accent: '#3b82f6',
      accentHover: '#2563eb',
      danger: '#ef4444',
      success: '#10b981',
      warning: '#f59e0b',
      nodeColors: {
        station: '#8b5cf6',
        docking: '#06b6d4',
        waypoint: '#f59e0b',
        home: '#10b981',
        charging: '#ef4444'
      }
    },
    dark: {
      background: '#0f172a',
      surface: '#1e293b',
      surfaceSecondary: '#1e293b',
      card: '#1e293b',
      text: '#f1f5f9',
      textSecondary: '#94a3b8',
      border: '#334155',
      accent: '#60a5fa',
      accentHover: '#3b82f6',
      danger: '#f87171',
      success: '#34d399',
      warning: '#fbbf24',
      nodeColors: {
        station: '#a78bfa',
        docking: '#22d3ee',
        waypoint: '#fbbf24',
        home: '#34d399',
        charging: '#f87171'
      }
    }
  };

  const currentTheme = darkMode ? theme.dark : theme.light;

  const styles = {
    container: {
      display: "flex",
      gap: 12,
      padding: 12,
      width: "100%",
      boxSizing: "border-box",
      background: currentTheme.background,
      color: currentTheme.text,
      minHeight: '100vh',
      fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
    },
    sidebar: {
      width: 320,
      background: currentTheme.card,
      borderRadius: 12,
      padding: 16,
      boxShadow: "0 4px 20px rgba(0,0,0,0.1)",
      border: `1px solid ${currentTheme.border}`,
    },
    canvasContainer: {
      flex: 1,
      border: `2px solid ${currentTheme.border}`,
      borderRadius: 12,
      background: currentTheme.surface,
      minHeight: 900,
      position: "relative",
      overflow: "hidden"
    },
    button: {
      padding: "10px 12px",
      borderRadius: 8,
      border: `1px solid ${currentTheme.border}`,
      background: currentTheme.card,
      color: currentTheme.text,
      cursor: "pointer",
      fontSize: '14px',
      fontWeight: '500',
      transition: 'all 0.2s ease',
      display: 'flex',
      alignItems: 'center',
      gap: 8,
      flex: 1,
      justifyContent: 'center'
    },
    buttonActive: {
      padding: "10px 12px",
      borderRadius: 8,
      border: `1px solid ${currentTheme.accent}`,
      background: currentTheme.accent,
      color: "white",
      cursor: "pointer",
      fontSize: '14px',
      fontWeight: '500',
      transition: 'all 0.2s ease',
      display: 'flex',
      alignItems: 'center',
      gap: 8,
      flex: 1,
      justifyContent: 'center'
    },
    buttonAction: {
      padding: "10px 12px",
      borderRadius: 8,
      border: "none",
      background: currentTheme.accent,
      color: "white",
      cursor: "pointer",
      fontSize: '14px',
      fontWeight: '500',
      transition: 'all 0.2s ease',
      display: 'flex',
      alignItems: 'center',
      gap: 8,
      flex: 1,
      justifyContent: 'center'
    },
    buttonDanger: {
      padding: "10px 12px",
      borderRadius: 8,
      border: "none",
      background: currentTheme.danger,
      color: "white",
      cursor: "pointer",
      fontSize: '14px',
      fontWeight: '500',
      transition: 'all 0.2s ease',
      display: 'flex',
      alignItems: 'center',
      gap: 8,
      flex: 1,
      justifyContent: 'center'
    },
    buttonKeepOut: {
  padding: "10px 12px",
  borderRadius: 8,
  border: "none",
  background: '#000000', // Changed to black
  color: '#ffffff', // White text for contrast
  cursor: "pointer",
  fontSize: '14px',
  fontWeight: '500',
  transition: 'all 0.2s ease',
  display: 'flex',
  alignItems: 'center',
  gap: 8,
  flex: 1,
  justifyContent: 'center'
},
    smallButton: {
      padding: 10,
      borderRadius: 8,
      border: "none",
      background: currentTheme.accent,
      color: "#fff",
      cursor: "pointer",
      display: 'flex',
      alignItems: 'center',
      justifyContent: 'center',
      transition: 'all 0.2s ease',
    },
    input: {
      width: "100%",
      padding: "10px 12px",
      margin: "6px 0 12px 0",
      borderRadius: 8,
      border: `1px solid ${currentTheme.border}`,
      background: currentTheme.card,
      color: currentTheme.text,
      fontSize: '14px',
      boxSizing: 'border-box'
    },
    select: {
      width: "100%",
      padding: "10px 12px",
      marginTop: 6,
      borderRadius: 8,
      border: `1px solid ${currentTheme.border}`,
      background: currentTheme.card,
      color: currentTheme.text,
      fontSize: '14px',
    },
    label: {
      fontSize: "14px",
      color: currentTheme.text,
      fontWeight: '600',
      marginBottom: 4,
      display: 'block'
    },
    hint: {
      fontSize: "12px",
      color: currentTheme.textSecondary,
      marginTop: 6,
      lineHeight: '1.4'
    },
    hud: {
      background: currentTheme.accent,
      color: "#fff",
      padding: "8px 12px",
      borderRadius: 8,
      fontSize: 14,
      fontWeight: '500',
      fontFamily: 'monospace'
    },
    status: {
      padding: "8px 12px",
      background: rosConnected ? currentTheme.success : currentTheme.danger,
      color: "white",
      borderRadius: 8,
      fontSize: 14,
      fontWeight: '500',
      display: 'flex',
      alignItems: 'center',
      gap: 6
    },
    mapInfo: {
      padding: "8px 12px",
      background: currentTheme.card,
      color: currentTheme.text,
      borderRadius: 8,
      fontSize: 14,
      fontWeight: '500',
      border: `1px solid ${currentTheme.border}`
    }
  };

  // --- Map Loader Functions ---

  const handleLoadMap = async () => {
    try {
      if (!yamlFile || !pgmFile) {
        alert("Please select both YAML and PGM files first.");
        return;
      }

      const yamlText = await yamlFile.text();
      const parsedYaml = yaml.load(yamlText);
      setMapInfo(parsedYaml);

      const buffer = await pgmFile.arrayBuffer();
      const bytes = new Uint8Array(buffer);
      const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

      const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
      const magic = headerLines[0];
      if (magic !== "P5" && magic !== "P2") {
        throw new Error("Unsupported PGM format: must be P5 or P2");
      }

      const width = parseInt(headerLines[1]);
      const height = parseInt(headerLines[2]);
      const maxVal = parseInt(headerLines[3]);

      const headerLength = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
      const dataStart = textHeader.slice(0, headerLength).length + 1;
      const pixelBytes = bytes.slice(dataStart);

      const occupancyData = new Int8Array(width * height);
      const negate = parsedYaml.negate || 0;

      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const i = y * width + x;
          let value = pixelBytes[i];
          
          if (negate === 1) {
            value = 255 - value;
          }
          
          let occupancyValue = -1;
          if (value >= 0 && value <= 10) {
            occupancyValue = 100;
          } else if (value >= 250 && value <= 255) {
            occupancyValue = 0;
          }
          
          occupancyData[(height - 1 - y) * width + x] = occupancyValue;
        }
      }

      const loadedMap = {
        width: width,
        height: height,
        resolution: parsedYaml.resolution || 0.05,
        data: occupancyData,
        origin: {
          x: parsedYaml.origin?.[0] || 0,
          y: parsedYaml.origin?.[1] || 0,
          z: parsedYaml.origin?.[2] || 0
        }
      };

      mapParamsRef.current = {
        width: width,
        height: height,
        resolution: parsedYaml.resolution || 0.05,
        originX: parsedYaml.origin?.[0] || 0,
        originY: parsedYaml.origin?.[1] || 0,
      };

      setMapMsg(loadedMap);
      setEditableMap({ ...loadedMap, data: [...occupancyData] });
      setMapLoaderActive(false);

      if (canvasRef.current) {
        const container = canvasRef.current.parentElement;
        const offsetX = (container.clientWidth - width) / 2;
        const offsetY = (container.clientHeight - height) / 2;
        setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
      }

      const mapNameFromFile = yamlFile.name.replace('.yaml', '').replace('.yml', '');
      if (mapNameFromFile && !mapName) {
        setMapName(mapNameFromFile);
      }

    } catch (err) {
      console.error("Error loading map:", err);
      alert("Failed to load map. Check console for details.");
    }
  };

  // --- Enhanced Erase Functions ---
  const eraseAtClient = (clientX, clientY) => {
    if (!mapMsg) return;
    
    if (eraseMode === "objects") {
      const c = clientToCanvasCoords(clientX, clientY);
      eraseObjectsAt(c.x, c.y);
    } else if (eraseMode === "noise") {
      eraseNoiseAt(clientX, clientY);
    }
  };

  const eraseObjectsAt = (canvasX, canvasY) => {
    // Check nodes first
    const node = findNodeAtCanvas(canvasX, canvasY, 8 / zoomState.scale);
    if (node) {
      setNodes((prev) => prev.filter((p) => p.id !== node.id));
      setArrows((prev) => prev.filter((a) => a.fromId !== node.id && a.toId !== node.id));
      return;
    }
    
    // Check zones
    for (const z of zones) {
      const poly = z.points.map((p) => [p.canvasX, p.canvasY]);
      if (pointInPolygon([canvasX, canvasY], poly)) {
        setZones((prev) => prev.filter((pz) => pz.id !== z.id));
        return;
      }
    }
  };

  // Main function for noise erasing given browser (client) coordinates
  const eraseNoiseAt = (clientX, clientY) => {
    if (!editableMap || !canvasRef.current) return;

    // Get the canvas coordinates first
    const canvasCoords = clientToCanvasCoords(clientX, clientY);
    
    // Convert canvas coordinates to map coordinates (flip Y-axis)
    const mapX = Math.floor(canvasCoords.x);
    const mapY = Math.floor(editableMap.height - canvasCoords.y);

    const { width, height, data } = editableMap;

    // Check if coordinates are within map bounds
    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) return;

    const newData = [...data];
    let erased = 0;

    for (let y = mapY - eraseRadius; y <= mapY + eraseRadius; y++) {
      for (let x = mapX - eraseRadius; x <= mapX + eraseRadius; x++) {
        // Check bounds for each erase point
        if (x >= 0 && x < width && y >= 0 && y < height) {
          const d = Math.sqrt((x - mapX) ** 2 + (y - mapY) ** 2);
          if (d <= eraseRadius) {
            const idx = y * width + x;
            // Only erase black areas (occupied cells = 100) and unknown areas (-1)
            if (newData[idx] === 100 || newData[idx] === -1) {
              newData[idx] = 0; // Set to free space
              erased++;
            }
          }
        }
      }
    }

    if (erased > 0) {
      setEditableMap({ ...editableMap, data: newData });
      setMapMsg({ ...mapMsg, data: newData });
    }
  };

  // --- Hand Erase Functions ---
  const startHandErase = (clientX, clientY) => {
    if (!editableMap || !canvasRef.current) return;

    const canvasCoords = clientToCanvasCoords(clientX, clientY);
    
    // Convert canvas coordinates to map coordinates (flip Y-axis)
    const mapX = Math.floor(canvasCoords.x);
    const mapY = Math.floor(editableMap.height - canvasCoords.y);

    const { width, height } = editableMap;

    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) return;

    setHandEraseState({
      isErasing: true,
      lastX: mapX,
      lastY: mapY
    });

    eraseNoiseInLine(mapX, mapY, mapX, mapY);
  };

  const continueHandErase = (clientX, clientY) => {
    if (!handEraseState.isErasing || !editableMap || !canvasRef.current) return;

    const canvasCoords = clientToCanvasCoords(clientX, clientY);
    
    // Convert canvas coordinates to map coordinates (flip Y-axis)
    const currentX = Math.floor(canvasCoords.x);
    const currentY = Math.floor(editableMap.height - canvasCoords.y);

    const { width, height } = editableMap;

    if (currentX < 0 || currentX >= width || currentY < 0 || currentY >= height) {
      setHandEraseState(prev => ({
        ...prev,
        lastX: currentX,
        lastY: currentY
      }));
      return;
    }

    eraseNoiseInLine(handEraseState.lastX, handEraseState.lastY, currentX, currentY);

    setHandEraseState(prev => ({
      ...prev,
      lastX: currentX,
      lastY: currentY
    }));
  };

  const stopHandErase = () => {
    setHandEraseState({
      isErasing: false,
      lastX: 0,
      lastY: 0
    });
  };

  const eraseNoiseInLine = (x0, y0, x1, y1) => {
    const { width, height, data } = editableMap;
    const newData = [...data];
    let erased = 0;

    // Use Bresenham's line algorithm to get all points between (x0,y0) and (x1,y1)
    const points = getLinePoints(x0, y0, x1, y1);

    // Erase at each point along the line with the specified radius
    points.forEach(point => {
      const { x, y } = point;
      
      // Check if the center point is within bounds
      if (x < 0 || x >= width || y < 0 || y >= height) return;
      
      for (let dy = -eraseRadius; dy <= eraseRadius; dy++) {
        for (let dx = -eraseRadius; dx <= eraseRadius; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          
          // Check bounds for each erase point
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            const distance = Math.sqrt(dx * dx + dy * dy);
            if (distance <= eraseRadius) {
              const idx = ny * width + nx;
              // Only erase black areas (occupied cells = 100) and unknown areas (-1)
              if (newData[idx] === 100 || newData[idx] === -1) {
                newData[idx] = 0; // Set to free space
                erased++;
              }
            }
          }
        }
      }
    });

    if (erased > 0) {
      setEditableMap({ ...editableMap, data: newData });
      setMapMsg({ ...mapMsg, data: newData });
    }
  };

  const getLinePoints = (x0, y0, x1, y1) => {
    const points = [];
    const dx = Math.abs(x1 - x0);
    const dy = Math.abs(y1 - y0);
    const sx = (x0 < x1) ? 1 : -1;
    const sy = (y0 < y1) ? 1 : -1;
    let err = dx - dy;

    let x = x0;
    let y = y0;

    while (true) {
      points.push({ x, y });

      if (x === x1 && y === y1) break;

      const e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }

    return points;
  };




  
// --- Enhanced PNG Export Functions ---
const saveMapAsPNG = () => {
  // If there's an active freehand crop, save the cropped area instead
  if (cropState.freehandPoints && cropState.freehandPoints.length >= 3) {
    saveFreehandCroppedMap();
    return;
  }

  // Otherwise, save the full map as before
  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded to save!");
    return;
  }

  try {
    const { width, height } = mapMsg;
    const { resolution, originX, originY } = mapParamsRef.current;
    
    // High resolution scale factor (2x, 3x, 4x, etc.)
    const SCALE_FACTOR = 4; // You can adjust this (2 = 2x resolution, 3 = 3x, etc.)
    
    const tempCanvas = document.createElement('canvas');
    const ctx = tempCanvas.getContext('2d');
    
    // Set canvas to scaled dimensions
    tempCanvas.width = width * SCALE_FACTOR;
    tempCanvas.height = height * SCALE_FACTOR;
    
    // Fill background white
    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, tempCanvas.width, tempCanvas.height);
    
    ctx.save();
    
    // Apply scaling for high resolution - this is the key change!
    // We scale the entire coordinate system
    ctx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
    if (rotation !== 0) {
      ctx.translate(width / 2, height / 2);
      ctx.rotate((rotation * Math.PI) / 180);
      ctx.translate(-width / 2, -height / 2);
    }

// Draw base map - same as before but coordinates are scaled
const imageData = ctx.createImageData(width, height);
for (let y = 0; y < height; y++) {
  for (let x = 0; x < width; x++) {
    const val = mapMsg.data[y * width + x];
    const py = height - 1 - y;
    const i = (py * width + x) * 4;
    
    let gray, alpha;
    if (val === -2) {
      // CHECKERBOARD AREAS - MAKE TRANSPARENT
      gray = 255; // White background
      alpha = 0;  // Fully transparent
    } else if (val === -1) {
      gray = 205; // Unknown - show as gray
      alpha = 255;
    } else if (val === 0) {
      gray = 255; // Free space - white
      alpha = 255;
    } else if (val === 100) {
      gray = 0;   // Occupied - black
      alpha = 255;
    } else {
      gray = 255 - Math.floor((val / 100) * 255);
      alpha = 255;
    }

    imageData.data[i] = gray;
    imageData.data[i + 1] = gray;
    imageData.data[i + 2] = gray;
    imageData.data[i + 3] = alpha; // Use alpha channel for transparency
  }
}
    const off = document.createElement("canvas");
    off.width = width;
    off.height = height;
    off.getContext("2d").putImageData(imageData, 0, 0);
    ctx.drawImage(off, 0, 0, width, height);

    const transformPoint = (point) => {
      if (rotation === 0) return { x: point.canvasX, y: point.canvasY };
      
      const nx = point.canvasX / width;
      const ny = point.canvasY / height;
      
      let rotatedX, rotatedY;
      
      switch (rotation) {
        case 90:
          rotatedX = (1 - ny) * width;
          rotatedY = nx * height;
          break;
        case 180:
          rotatedX = (1 - nx) * width;
          rotatedY = (1 - ny) * height;
          break;
        case 270:
          rotatedX = ny * width;
          rotatedY = (1 - nx) * height;
          break;
        default:
          rotatedX = point.canvasX;
          rotatedY = point.canvasY;
      }
      
      return { x: rotatedX, y: rotatedY };
    };

    // Draw zones with enhanced resolution
    zones.forEach((zone) => {
      if (zone.points && zone.points.length >= 3) {
        ctx.beginPath();
        zone.points.forEach((point, idx) => {
          const transformed = transformPoint(point);
          if (idx === 0) ctx.moveTo(transformed.x, transformed.y);
          else ctx.lineTo(transformed.x, transformed.y);
        });
        ctx.closePath();
        
        if (zone.type === 'keep_out') {
          ctx.fillStyle = "rgba(0, 0, 0, 1.0)";
          ctx.fill();
          ctx.strokeStyle = "rgba(0, 0, 0, 1.0)";
          ctx.lineWidth = 2 / SCALE_FACTOR; // Adjust line width for scaling
          ctx.stroke();
        } else {
          ctx.fillStyle = "rgba(100, 200, 100, 0.3)";
          ctx.fill();
          ctx.strokeStyle = "rgba(50, 150, 50, 0.8)";
          ctx.lineWidth = 1.4 / SCALE_FACTOR; // Adjust line width for scaling
          ctx.stroke();
        }

        const centerX = zone.points.reduce((sum, p) => sum + transformPoint(p).x, 0) / zone.points.length;
        const centerY = zone.points.reduce((sum, p) => sum + transformPoint(p).y, 0) / zone.points.length;
        
        if (zone.type === 'keep_out') {
          ctx.fillStyle = "rgba(220, 38, 38, 1)";
        } else {
          ctx.fillStyle = "rgba(50, 150, 50, 1)";
        }
        ctx.font = `${14 / SCALE_FACTOR}px Arial`; // Adjust font size for scaling
        ctx.fillText(zone.name, centerX + 16 / SCALE_FACTOR, centerY + 8 / SCALE_FACTOR);
      }
    });

    // Draw arrows with enhanced resolution
    arrows.forEach((arrow) => {
      const fromNode = nodes.find(n => n.id === arrow.fromId);
      const toNode = nodes.find(n => n.id === arrow.toId);
      
      if (fromNode && toNode) {
        const allPoints = [fromNode, ...(arrow.points || []), toNode];
        if (allPoints.length < 2) return;
        
        ctx.strokeStyle = "rgba(255, 165, 0, 0.8)";
        ctx.lineWidth = 4 / SCALE_FACTOR; // Adjust line width for scaling
        ctx.beginPath();
        
        const firstPoint = transformPoint(allPoints[0]);
        ctx.moveTo(firstPoint.x, firstPoint.y);
        
        for (let i = 1; i < allPoints.length; i++) {
          const point = transformPoint(allPoints[i]);
          ctx.lineTo(point.x, point.y);
        }
        ctx.stroke();
        
        const lastPoint = transformPoint(allPoints[allPoints.length - 1]);
        const secondLastPoint = transformPoint(allPoints[allPoints.length - 2]);
        const angle = Math.atan2(
          lastPoint.y - secondLastPoint.y,
          lastPoint.x - secondLastPoint.x
        );
        const headLength = 24 / SCALE_FACTOR; // Adjust arrowhead size for scaling
        
        ctx.beginPath();
        ctx.moveTo(lastPoint.x, lastPoint.y);
        ctx.lineTo(
          lastPoint.x - headLength * Math.cos(angle - Math.PI / 6),
          lastPoint.y - headLength * Math.sin(angle - Math.PI / 6)
        );
        ctx.lineTo(
          lastPoint.x - headLength * Math.cos(angle + Math.PI / 6),
          lastPoint.y - headLength * Math.sin(angle + Math.PI / 6)
        );
        ctx.closePath();
        ctx.fillStyle = "rgba(255, 165, 0, 1)";
        ctx.fill();
        
        for (let i = 1; i < allPoints.length - 1; i++) {
          const point = transformPoint(allPoints[i]);
          ctx.beginPath();
          ctx.arc(point.x, point.y, 8 / SCALE_FACTOR, 0, Math.PI * 2); // Adjust control point size
          ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
          ctx.fill();
          ctx.strokeStyle = "rgba(255, 165, 0, 1)";
          ctx.lineWidth = 2 / SCALE_FACTOR;
          ctx.stroke();
        }
      }
    });

    // Draw nodes with enhanced resolution
    nodes.forEach((node) => {
      const nodeColors = {
        station: '#8b5cf6',
        docking: '#06b6d4',
        waypoint: '#f59e0b',
        home: '#10b981',
        charging: '#ef4444'
      };
      
      const nodeColor = nodeColors[node.type] || "#3b82f6";
      const transformed = transformPoint(node);
      
      ctx.beginPath();
      ctx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2); // Adjust node size
      ctx.fillStyle = nodeColor;
      ctx.fill();
      
      // Optional: Add node labels
      // ctx.fillStyle = "black";
      // ctx.font = `${12 / SCALE_FACTOR}px Arial`;
      // ctx.fillText(node.label || node.type, transformed.x + 12 / SCALE_FACTOR, transformed.y - 8 / SCALE_FACTOR);
    });

    ctx.restore();

    // Generate YAML file with corrected resolution
    const newResolution = resolution / SCALE_FACTOR;
    const yamlContent = `image: ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png
mode: trinary
resolution: ${newResolution.toFixed(6)}
origin: [${originX}, ${originY}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# High-resolution export
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Original size: ${width}x${height} pixels
# Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
# Rotation: ${rotation}°`;

    tempCanvas.toBlob((blob) => {
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
      
      // Also save the YAML file
      const yamlBlob = new Blob([yamlContent], { type: 'application/x-yaml' });
      const yamlUrl = URL.createObjectURL(yamlBlob);
      const yamlLink = document.createElement('a');
      yamlLink.href = yamlUrl;
      yamlLink.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml`;
      document.body.appendChild(yamlLink);
      yamlLink.click();
      document.body.removeChild(yamlLink);
      URL.revokeObjectURL(yamlUrl);
      
      alert(`✅ High-resolution map saved!\n\n📁 Files:\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel (adjusted)\n🔄 Rotation: ${rotation}°\n\n📊 Statistics:\n- 🏭 Nodes: ${nodes.length}\n- ➡️ Arrows: ${arrows.length}\n- 🗺️ Zones: ${zones.length}\n\n⚠️ Important: Use the provided YAML file for correct coordinate mapping!`);
    }, 'image/png', 1.0);

  } catch (error) {
    console.error('Error saving high-resolution map as PNG:', error);
    alert('❌ Error saving high-resolution map as PNG. Check console for details.');
  }
};


  // Point in polygon algorithm for freehand selection
  const pointInFreehandPolygon = (point, vs) => {
    if (!vs || vs.length < 3) return false;
    
    const x = point[0], y = point[1];
    let inside = false;
    for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
      const xi = vs[i].x, yi = vs[i].y;
      const xj = vs[j].x, yj = vs[j].y;
      
      const intersect = ((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 0.0000001) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  };

  // Get bounding box of freehand selection
  const getFreehandBoundingBox = (points) => {
    if (!points || points.length === 0) return { minX: 0, minY: 0, maxX: 0, maxY: 0 };
    
    let minX = points[0].x;
    let minY = points[0].y;
    let maxX = points[0].x;
    let maxY = points[0].y;
    
    points.forEach(point => {
      minX = Math.min(minX, point.x);
      minY = Math.min(minY, point.y);
      maxX = Math.max(maxX, point.x);
      maxY = Math.max(maxY, point.y);
    });
    
    return { minX, minY, maxX, maxY };
  };

  // Helper function to rotate points
  const rotatePoint = (point, width, height, rotation) => {
    if (rotation === 0) return { x: point.x, y: point.y };
    
    const centerX = width / 2;
    const centerY = height / 2;
    
    // Translate to origin
    const translatedX = point.x - centerX;
    const translatedY = point.y - centerY;
    
    let rotatedX, rotatedY;
    
    switch (rotation) {
      case 90:
        rotatedX = -translatedY;
        rotatedY = translatedX;
        break;
      case 180:
        rotatedX = -translatedX;
        rotatedY = -translatedY;
        break;
      case 270:
        rotatedX = translatedY;
        rotatedY = -translatedX;
        break;
      default:
        rotatedX = translatedX;
        rotatedY = translatedY;
    }
    
    // Translate back
    return { 
      x: Math.round(rotatedX + centerX), 
      y: Math.round(rotatedY + centerY) 
    };
  };


  
// --- Scissor Crop Functions with Complete Removal ---
const startCrop = () => {
  setCropState({
    isCropping: true,
    startX: 0,
    startY: 0,
    endX: 0,
    endY: 0,
    isDragging: false,
    freehandPoints: [] // Store freehand points
  });
  setTool("crop");
};

const cancelCrop = () => {
  setCropState({
    isCropping: false,
    startX: 0,
    startY: 0,
    endX: 0,
    endY: 0,
    isDragging: false,
    freehandPoints: []
  });
  setTool("place_node");
};


// --- TRUE SCISSOR CUT: Remove actual map region, not erase grayscale ---
const handleScissorCut = () => {
  console.log("✂️ Cutting actual map region - COMPLETE REMOVAL");

  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded!");
    return;
  }

  if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
    alert("Draw a closed freehand shape first!");
    return;
  }

  const { width, height } = mapParamsRef.current;

  // Convert freehand (canvas coords) → map coords
  const mapPolygon = cropState.freehandPoints.map(p => ({
    x: p.x,
    y: height - 1 - p.y  // flip Y axis
  }));

  // Bounding box for optimization
  const bbox = getFreehandBoundingBox(mapPolygon);
  const minX = Math.max(0, Math.floor(bbox.minX));
  const minY = Math.max(0, Math.floor(bbox.minY));
  const maxX = Math.min(width, Math.ceil(bbox.maxX));
  const maxY = Math.min(height, Math.ceil(bbox.maxY));

  const newData = new Int8Array([...mapMsg.data]);
  let removedCount = 0;

  // COMPLETE REMOVAL — set to -2 for checkerboard display
  for (let y = minY; y < maxY; y++) {
    for (let x = minX; x < maxX; x++) {
      if (isPointInPolygon({ x, y }, mapPolygon)) {
        const idx = y * width + x;
        newData[idx] = -2;      // Use -2 for checkerboard pattern
        removedCount++;
      }
    }
  }

  console.log(`✂️ Completely removed ${removedCount} pixels`);

  if (removedCount === 0) {
    alert("No map pixels found inside the selection!");
    return;
  }

  // Update map
  const newEditableMap = { ...editableMap, data: newData };
  setMapMsg(prev => prev ? { ...prev, data: newData } : null);
  setEditableMap(prev => prev ? { ...prev, data: [...newData] } : null);

  // Save to history for undo/redo
  saveToHistory({
    nodes: [...nodes],
    arrows: [...arrows],
    zones: [...zones],
    editableMap: newEditableMap,
    rotation: rotation
  });

  // Reset state
  setCropState({
    isCropping: false,
    startX: 0,
    startY: 0,
    endX: 0,
    endY: 0,
    isDragging: false,
    freehandPoints: []
  });

  setTool("place_node");

  alert(`✂️ Complete removal successful!\nRemoved ${removedCount} pixels - area is now checkerboard (completely removed)`);
};
// Visual debug function to show where gray areas actually are
const showGrayAreas = () => {
  if (!mapMsg || !mapParamsRef.current) return;
  
  const { width, height } = mapParamsRef.current;
  const grayAreas = [];
  
  // Find some gray areas
  for (let y = 0; y < height; y += 10) { // Sample every 10th row
    for (let x = 0; x < width; x += 10) { // Sample every 10th column
      const index = y * width + x;
      if (mapMsg.data[index] === -1) {
        grayAreas.push({
          mapX: x,
          mapY: y,
          canvasX: x,
          canvasY: height - 1 - y // Convert to canvas coordinates
        });
      }
    }
  }
  
  console.log("📍 Gray areas locations (first 10):", grayAreas.slice(0, 10));
  
  if (grayAreas.length > 0) {
    const sample = grayAreas[0];
    alert(`Gray areas found! Sample location:\nMap: (${sample.mapX}, ${sample.mapY})\nCanvas: (${sample.canvasX}, ${sample.canvasY})\n\nCheck console for more locations.`);
  } else {
    alert("No gray areas found in the sampled locations.");
  }
};


const saveFreehandCroppedMap = async () => {
  console.log("🔧 Exporting cleaned map without gray areas");

  const SCALE_FACTOR = 4;

  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded!");
    return;
  }

  if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
    alert("Invalid freehand selection! Please draw at least 3 points.");
    return;
  }

  try {
    const freehandPoints = cropState.freehandPoints;
    const { width, height, resolution, originX, originY, rotation = 0 } = mapParamsRef.current;

    // Calculate bounding box of selection
    const bbox = getFreehandBoundingBox(freehandPoints);
    const cropMinX = Math.max(0, Math.floor(bbox.minX));
    const cropMinY = Math.max(0, Math.floor(bbox.minY));
    const cropMaxX = Math.min(width, Math.ceil(bbox.maxX));
    const cropMaxY = Math.min(height, Math.ceil(bbox.maxY));

    const cropWidth = cropMaxX - cropMinX;
    const cropHeight = cropMaxY - cropMinY;

    if (cropWidth <= 0 || cropHeight <= 0) {
      alert("Invalid crop area!");
      return;
    }

    console.log(`📐 Crop area: ${cropWidth}x${cropHeight} at (${cropMinX},${cropMinY})`);

    // Count gray areas in selection for info
    let grayCountInSelection = 0;
    for (let y = cropMinY; y < cropMaxY; y++) {
      for (let x = cropMinX; x < cropMaxX; x++) {
        if (isPointInPolygon({ x, y }, freehandPoints)) {
          const index = y * width + x;
          if (mapMsg.data[index] === -1) {
            grayCountInSelection++;
          }
        }
      }
    }

    // Find content bounds (non-gray areas)
    let contentMinX = cropWidth, contentMinY = cropHeight, contentMaxX = 0, contentMaxY = 0;
    let hasContent = false;

    for (let y = 0; y < cropHeight; y++) {
      for (let x = 0; x < cropWidth; x++) {
        const mapX = cropMinX + x;
        const mapY = cropMinY + y;
        
        if (!isPointInPolygon({ x: mapX, y: mapY }, freehandPoints)) continue;

        const srcIndex = mapY * width + mapX;
        if (mapMsg.data[srcIndex] !== -1) {
          contentMinX = Math.min(contentMinX, x);
          contentMinY = Math.min(contentMinY, y);
          contentMaxX = Math.max(contentMaxX, x);
          contentMaxY = Math.max(contentMaxY, y);
          hasContent = true;
        }
      }
    }

    // Use entire area if no content found
    if (!hasContent) {
      contentMinX = 0;
      contentMinY = 0;
      contentMaxX = cropWidth - 1;
      contentMaxY = cropHeight - 1;
    }

    // Add padding
    const padding = 2;
    contentMinX = Math.max(0, contentMinX - padding);
    contentMinY = Math.max(0, contentMinY - padding);
    contentMaxX = Math.min(cropWidth - 1, contentMaxX + padding);
    contentMaxY = Math.min(cropHeight - 1, contentMaxY + padding);

    const contentWidth = contentMaxX - contentMinX + 1;
    const contentHeight = contentMaxY - contentMinY + 1;

    if (contentWidth <= 0 || contentHeight <= 0) {
      alert("No map content found in selected area!");
      return;
    }

    console.log(`📐 Content area: ${contentWidth}x${contentHeight} pixels`);

    // Extract content data (remove gray areas)
    const contentData = new Int8Array(contentWidth * contentHeight);
    let grayAreasRemoved = 0;

    for (let y = 0; y < contentHeight; y++) {
      for (let x = 0; x < contentWidth; x++) {
        const absX = cropMinX + contentMinX + x;
        const absY = cropMinY + contentMinY + y;
        const srcIndex = absY * width + absX;
        const dstIndex = y * contentWidth + x;
        
        // Convert gray to white, keep other values
        if (mapMsg.data[srcIndex] === -1) {
          contentData[dstIndex] = 0; // Gray → White
          grayAreasRemoved++;
        } else {
          contentData[dstIndex] = mapMsg.data[srcIndex];
        }
      }
    }

    // Calculate new origin
    const absContentMinX = cropMinX + contentMinX;
    const absContentMinY = cropMinY + contentMinY;
    const newOriginX = originX + absContentMinX * resolution;
    const newOriginY = originY + (height - (absContentMinY + contentHeight)) * resolution;

    // Create high-res canvas
    const scaledW = contentWidth * SCALE_FACTOR;
    const scaledH = contentHeight * SCALE_FACTOR;
    const canvas = document.createElement("canvas");
    canvas.width = scaledW;
    canvas.height = scaledH;
    const ctx = canvas.getContext("2d");
    ctx.scale(SCALE_FACTOR, SCALE_FACTOR);

    // Fill background and draw cleaned map
    ctx.fillStyle = "white";
    ctx.fillRect(0, 0, contentWidth, contentHeight);

    ctx.save();
    if (rotation !== 0) {
      ctx.translate(contentWidth / 2, contentHeight / 2);
      ctx.rotate((rotation * Math.PI) / 180);
      ctx.translate(-contentWidth / 2, -contentHeight / 2);
    }

    // Create and draw image data
    const imgData = ctx.createImageData(contentWidth, contentHeight);
    for (let y = 0; y < contentHeight; y++) {
      for (let x = 0; x < contentWidth; x++) {
        const val = contentData[y * contentWidth + x];
        const py = contentHeight - 1 - y;
        const i = (py * contentWidth + x) * 4;
        const gray = val === 100 ? 0 : 255; // 100 = obstacle, else free space
        
        imgData.data[i] = gray;
        imgData.data[i + 1] = gray;
        imgData.data[i + 2] = gray;
        imgData.data[i + 3] = 255;
      }
    }

    const offCanvas = document.createElement("canvas");
    offCanvas.width = contentWidth;
    offCanvas.height = contentHeight;
    offCanvas.getContext("2d").putImageData(imgData, 0, 0);
    ctx.drawImage(offCanvas, 0, 0);
    ctx.restore();

    // Save files
    const fileBase = `${mapName || "cleaned_map"}_${contentWidth}x${contentHeight}_hires${SCALE_FACTOR}x_rot${rotation}`;
    const newResolution = resolution / SCALE_FACTOR;

    // Save PNG
    canvas.toBlob((blob) => {
      const url = URL.createObjectURL(blob);
      const link = document.createElement("a");
      link.href = url;
      link.download = `${fileBase}.png`;
      link.click();
      URL.revokeObjectURL(url);
    }, "image/png");

    // Save YAML
    const yamlContent = `image: ${fileBase}.png
mode: trinary
resolution: ${newResolution.toFixed(6)}
origin: [${newOriginX.toFixed(6)}, ${newOriginY.toFixed(6)}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# Cleaned map export - gray areas removed
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Content size: ${contentWidth}x${contentHeight} pixels
# Scaled size: ${scaledW}x${scaledH} pixels
# Rotation: ${rotation}°
# Gray areas removed: ${grayAreasRemoved}`;

    const yamlBlob = new Blob([yamlContent], { type: "application/x-yaml" });
    const yamlUrl = URL.createObjectURL(yamlBlob);
    const yamlLink = document.createElement("a");
    yamlLink.href = yamlUrl;
    yamlLink.download = `${fileBase}.yaml`;
    yamlLink.click();
    URL.revokeObjectURL(yamlUrl);

    // Reset state
    setCropState({
      isCropping: false,
      startX: 0,
      startY: 0,
      endX: 0,
      endY: 0,
      isDragging: false,
      freehandPoints: []
    });

    setTool("place_node");

    setTimeout(() => {
      alert(`✅ Cleaned Map Exported!\n\n📄 ${fileBase}.png\n✂️ Removed ${grayAreasRemoved} gray areas\n📏 Size: ${contentWidth}x${contentHeight} pixels\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel`);
    }, 100);

  } catch (e) {
    console.error("Error exporting cleaned map:", e);
    alert("❌ Error exporting cleaned map");
  }
};

// Improved point-in-polygon function
const isPointInPolygon = (point, polygon) => {
  const x = point.x, y = point.y;
  
  // Check if polygon is valid
  if (!polygon || polygon.length < 3) {
    console.warn("Invalid polygon for point-in-polygon check:", polygon);
    return false;
  }

  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x || polygon[i].canvasX;
    const yi = polygon[i].y || polygon[i].canvasY;
    const xj = polygon[j].x || polygon[j].canvasX;
    const yj = polygon[j].y || polygon[j].canvasY;
    
    const intersect = ((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 0.0000001) + xi);
    if (intersect) inside = !inside;
  }
  
  return inside;
};

  // --- Save Map Functions (for full map) ---
const saveMapToComputer = async () => {
  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded to save!");
    return;
  }

  if (!opencvLoaded) {
    alert("OpenCV is still loading. Please wait a moment and try again.");
    return;
  }

  try {
    const { width, height } = mapMsg;
    const { resolution, originX, originY } = mapParamsRef.current;
    
    // High resolution scale factor
    const SCALE_FACTOR = 4; // You can adjust this (2 = 2x resolution, 3 = 3x, etc.)
    
    const tempCanvas = document.createElement('canvas');
    
    // Apply scaling to canvas dimensions
    if (rotation === 90 || rotation === 270) {
      tempCanvas.width = height * SCALE_FACTOR;
      tempCanvas.height = width * SCALE_FACTOR;
    } else {
      tempCanvas.width = width * SCALE_FACTOR;
      tempCanvas.height = height * SCALE_FACTOR;
    }
    
    const ctx = tempCanvas.getContext('2d');
    
    ctx.save();
    
    // Apply scaling to the coordinate system
    ctx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
    if (rotation !== 0) {
      const centerX = tempCanvas.width / (2 * SCALE_FACTOR);
      const centerY = tempCanvas.height / (2 * SCALE_FACTOR);
      ctx.translate(centerX, centerY);"rgba(0, 0, 0, 1.0)"
      ctx.rotate((rotation * Math.PI) / 180);
      
      if (rotation === 90) {
        ctx.translate(-centerY, -centerX);
      } else if (rotation === 180) {
        ctx.translate(-centerX, -centerY);
      } else if (rotation === 270) {
        ctx.translate(-centerY, -centerX);
      } else {
        ctx.translate(-centerX, -centerY);
      }
    }

    // Draw the base map
    const imageData = ctx.createImageData(width, height);
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const val = mapMsg.data[y * width + x];
        const py = height - 1 - y;
        const i = (py * width + x) * 4;

        let gray;
        if (val === -1) gray = 205;
        else if (val === 0) gray = 255;
        else if (val === 100) gray = 0;
        else gray = 255 - Math.floor((val / 100) * 255);

        imageData.data[i] = gray;
        imageData.data[i + 1] = gray;
        imageData.data[i + 2] = gray;
        imageData.data[i + 3] = 255;
      }
    }
    
    const off = document.createElement("canvas");
    off.width = width;
    off.height = height;
    off.getContext("2d").putImageData(imageData, 0, 0);
    ctx.drawImage(off, 0, 0, width, height);

    // Helper function to transform points based on rotation
    const transformPoint = (point) => {
      if (rotation === 0) return { x: point.canvasX, y: point.canvasY };
      
      const nx = point.canvasX / width;
      const ny = point.canvasY / height;
      
      let rotatedX, rotatedY;
      
      switch (rotation) {
        case 90:
          rotatedX = (1 - ny) * width;
          rotatedY = nx * height;
          break;
        case 180:
          rotatedX = (1 - nx) * width;
          rotatedY = (1 - ny) * height;
          break;
        case 270:
          rotatedX = ny * width;
          rotatedY = (1 - nx) * height;
          break;
        default:
          rotatedX = point.canvasX;
          rotatedY = point.canvasY;
      }
      
      return { x: rotatedX, y: rotatedY };
    };

    // Draw zones with scaled sizes
    zones.forEach((zone) => {
      if (zone.points && zone.points.length >= 3) {
        ctx.beginPath();
        zone.points.forEach((point, idx) => {
          const transformed = transformPoint(point);
          if (idx === 0) ctx.moveTo(transformed.x, transformed.y);
          else ctx.lineTo(transformed.x, transformed.y);
        });
        ctx.closePath();
        
        if (zone.type === 'keep_out') {
          // For keepout zones, draw as black areas (occupied)
          ctx.fillStyle = "rgba(0, 0, 0, 1.0)";
          ctx.fill();
        } else {
          // For normal zones, draw as dark gray areas
          ctx.fillStyle = "rgba(100, 100, 100, 1.0)";
          ctx.fill();
        }
      }
    });

    // Draw arrows as black lines with scaled sizes
    arrows.forEach((arrow) => {
      const fromNode = nodes.find(n => n.id === arrow.fromId);
      const toNode = nodes.find(n => n.id === arrow.toId);
      
      if (fromNode && toNode) {
        const allPoints = [fromNode, ...(arrow.points || []), toNode];
        if (allPoints.length < 2) return;
        
    ctx.strokeStyle = "rgba(255, 255, 255, 1.0)"; // White = free space      
      ctx.lineWidth = 2 / SCALE_FACTOR; // Scale line width
        ctx.beginPath();
        
        const firstPoint = transformPoint(allPoints[0]);
        ctx.moveTo(firstPoint.x, firstPoint.y);
        
        for (let i = 1; i < allPoints.length; i++) {
          const point = transformPoint(allPoints[i]);
          ctx.lineTo(point.x, point.y);
        }
        ctx.stroke();
      }
    });

    // Draw nodes as black dots with scaled sizes
    nodes.forEach((node) => {
      const transformed = transformPoint(node);
      
      ctx.beginPath();
      ctx.arc(transformed.x, transformed.y, 3 / SCALE_FACTOR, 0, Math.PI * 2); // Scale node size
      ctx.fillStyle = "rgba(255, 255, 255, 1.0)";
      ctx.fill();
    });

    ctx.restore();

    // Convert canvas to grayscale for PGM
    const imageDataFromCanvas = ctx.getImageData(0, 0, tempCanvas.width, tempCanvas.height);
    const finalMat = new cv.Mat(tempCanvas.height, tempCanvas.width, cv.CV_8UC1);
    
    for (let y = 0; y < tempCanvas.height; y++) {
      for (let x = 0; x < tempCanvas.width; x++) {
        const i = (y * tempCanvas.width + x) * 4;
        const r = imageDataFromCanvas.data[i];
        const g = imageDataFromCanvas.data[i + 1];
        const b = imageDataFromCanvas.data[i + 2];
        
        const gray = Math.round(0.299 * r + 0.587 * g + 0.114 * b);
        finalMat.ucharPtr(y, x)[0] = gray;
      }
    }

    const pgmHeader = `P5\n${tempCanvas.width} ${tempCanvas.height}\n255\n`;
    const pgmData = new Uint8Array(tempCanvas.width * tempCanvas.height);
    
    for (let y = 0; y < tempCanvas.height; y++) {
      for (let x = 0; x < tempCanvas.width; x++) {
        pgmData[y * tempCanvas.width + x] = finalMat.ucharPtr(y, x)[0];
      }
    }

    const pgmContent = new Uint8Array(pgmHeader.length + pgmData.length);
    const encoder = new TextEncoder();
    const headerBytes = encoder.encode(pgmHeader);
    pgmContent.set(headerBytes);
    pgmContent.set(pgmData, headerBytes.length);

    // Update resolution in YAML to match the scaled image
    const newResolution = resolution / SCALE_FACTOR;
    const yamlContent = `image: ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm
mode: trinary
resolution: ${newResolution.toFixed(6)}
origin: [${originX}, ${originY}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# High-resolution PGM Export
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Original size: ${width}x${height} pixels
# Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
# Rotation: ${rotation}°
# Nodes: ${nodes.length}, Arrows: ${arrows.length}, Zones: ${zones.length}
# Export time: ${new Date().toISOString()}`;

    const pgmBlob = new Blob([pgmContent], { type: 'image/x-portable-graymap' });
    const pgmUrl = URL.createObjectURL(pgmBlob);
    const pgmLink = document.createElement('a');
    pgmLink.href = pgmUrl;
    pgmLink.download = `${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm`;
    document.body.appendChild(pgmLink);
    pgmLink.click();
    document.body.removeChild(pgmLink);
    
    const yamlBlob = new Blob([yamlContent], { type: 'application/x-yaml' });
    const yamlUrl = URL.createObjectURL(yamlBlob);
    const yamlLink = document.createElement('a');
    yamlLink.href = yamlUrl;
    yamlLink.download = `${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml`;
    document.body.appendChild(yamlLink);
    yamlLink.click();
    document.body.removeChild(yamlLink);
    
    finalMat.delete();
    
    setTimeout(() => {
      URL.revokeObjectURL(pgmUrl);
      URL.revokeObjectURL(yamlUrl);
    }, 100);

    alert(`✅ High-resolution PGM map exported successfully!\n\n📁 Files saved:\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n🔄 Rotation: ${rotation}°\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel (adjusted)\n\n📊 Statistics:\n- 🏭 Nodes: ${nodes.length}\n- ➡️ Arrows: ${arrows.length}\n- 🗺️ Zones: ${zones.length}\n\n⚠️ Important: Use the provided YAML file for correct coordinate mapping!`);

  } catch (error) {
    console.error('Error exporting high-resolution PGM map:', error);
    alert('❌ Error exporting high-resolution PGM map. Check console for details.');
  }
};
  // --- Database Functions (simplified) ---
  const saveNodesToDatabase = async () => {
    if (nodes.length === 0 && arrows.length === 0 && zones.length === 0) {
      alert("No nodes, arrows, or zones to save!");
      return;
    }

    try {
      setIsSaving(true);
      setDbStatus("Saving...");

      const payload = {
        mapName: mapName || "default",
        nodes: nodes,
        arrows: arrows,
        zones: zones,
      };

      const response = await fetch('http://localhost:5000/save-nodes', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
      });

      if (response.ok) {
        const result = await response.json();
        setDbStatus("Saved successfully!");
        alert(`✅ Successfully saved to database!\n\n🗂️ Database: ${result.database}\n🏷️ Map: ${result.map_name}`);
      } else {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
    } catch (error) {
      console.error('❌ Error saving nodes to database:', error);
      setDbStatus("Save failed!");
      alert('❌ Error saving to database. Make sure Flask server is running on port 5000.');
    } finally {
      setIsSaving(false);
    }
  };

  const loadNodesFromDatabase = async () => {
    try {
      const mapNameToLoad = mapName || "default";
      const response = await fetch(`http://localhost:5000/load-nodes?map_name=${mapNameToLoad}`);
      
      if (response.ok) {
        const result = await response.json();
        
        if (result.status === "success") {
          const mapNodes = result.nodes.map(dbNode => {
            const canvasCoords = rosToCanvasCoords(dbNode.x, dbNode.y);
            return {
              id: `node_${dbNode.id}`,
              type: dbNode.type,
              label: dbNode.name,
              rosX: dbNode.x,
              rosY: dbNode.y,
              canvasX: canvasCoords.x,
              canvasY: canvasCoords.y,
              dbId: dbNode.id
            };
          });
          
          setNodes(mapNodes);
          setArrows(result.arrows || []);
          setZones(result.zones || []);
          
          alert(`✅ Successfully loaded from database!\n\n📊 Nodes: ${result.nodes.length}\n➡️ Arrows: ${result.arrows.length}\n🗺️ Zones: ${result.zones.length}`);
        }
      }
    } catch (error) {
      console.error('❌ Error loading nodes from database:', error);
      alert('❌ Error connecting to database server.');
    }
  };

  // --- Coordinate conversions ---
  const rosToCanvasCoords = (rosX, rosY) => {
    if (!mapParamsRef.current) return { x: 0, y: 0 };
    const { resolution, originX, originY, width, height } = mapParamsRef.current;
    const pixelX = (rosX - originX) / resolution;
    const pixelY = height - (rosY - originY) / resolution;
    return { x: pixelX, y: pixelY };
  };

  const canvasToRosCoords = (canvasX, canvasY) => {
    if (!mapParamsRef.current) return null;
    const { resolution, originX, originY, height } = mapParamsRef.current;
    const flippedY = height - canvasY;
    const worldX = originX + canvasX * resolution;
    const worldY = originY + flippedY * resolution;
    return { x: +worldX.toFixed(2), y: +worldY.toFixed(2) };
  };

  const clientToCanvasCoords = (clientX, clientY) => {
    const canvas = canvasRef.current;
    if (!canvas) return { x: 0, y: 0 };
    const rect = canvas.getBoundingClientRect();
    
    const mouseX = clientX - rect.left;
    const mouseY = clientY - rect.top;
    
    if (rotation === 0) {
      const rawX = (mouseX - zoomState.offsetX) / zoomState.scale;
      const rawY = (mouseY - zoomState.offsetY) / zoomState.scale;
      return { x: rawX, y: rawY };
    }
    
    const containerWidth = rect.width;
    const containerHeight = rect.height;
    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;
    
    const translatedMouseX = mouseX - centerX;
    const translatedMouseY = mouseY - centerY;
    
    const angle = -rotation * Math.PI / 180;
    const rotatedMouseX = translatedMouseX * Math.cos(angle) - translatedMouseY * Math.sin(angle);
    const rotatedMouseY = translatedMouseX * Math.sin(angle) + translatedMouseY * Math.cos(angle);
    
    const unrotatedX = rotatedMouseX + centerX;
    const unrotatedY = rotatedMouseY + centerY;
    
    const rawX = (unrotatedX - zoomState.offsetX) / zoomState.scale;
    const rawY = (unrotatedY - zoomState.offsetY) / zoomState.scale;
    
    return { x: rawX, y: rawY };
  };

  const clientToRosCoords = (clientX, clientY) => {
    const { x, y } = clientToCanvasCoords(clientX, clientY);
    return canvasToRosCoords(x, y);
  };

  // --- Rotation function ---
  const handleRotate = () => {
    setRotation((prevRotation) => (prevRotation + 90) % 360);
  };

  const handleRotationInput = (e) => {
    const value = parseInt(e.target.value);
    if (!isNaN(value)) {
      const normalizedRotation = ((value % 360) + 360) % 360;
      setRotation(normalizedRotation);
    }
  };

  // --- Editor helpers ---
  const makeId = (prefix = "n") => `${prefix}_${idCounter.current++}`;

  const placeNodeAtClient = (clientX, clientY) => {
    if (!mapParamsRef.current) return;
    const rosCoords = clientToRosCoords(clientX, clientY);
    if (!rosCoords) return;
    const canvasCoords = rosToCanvasCoords(rosCoords.x, rosCoords.y);

    const newNode = {
      id: makeId("node"),
      type: nodeType,
      label: `${nodeType}_${nodes.length + 1}`,
      rosX: rosCoords.x,
      rosY: rosCoords.y,
      canvasX: canvasCoords.x,
      canvasY: canvasCoords.y,
    };
    setNodes((prev) => [...prev, newNode]);
  };

  const findNodeAtCanvas = (cx, cy, radius = 10) => {
    for (let i = nodes.length - 1; i >= 0; --i) {
      const n = nodes[i];
      const dx = n.canvasX - cx;
      const dy = n.canvasY - cy;
      if (Math.hypot(dx, dy) <= radius) return n;
    }
    return null;
  };

  // --- Arrow Drawing Functions ---
  const handleConnectClick = (clientX, clientY) => {
    const c = clientToCanvasCoords(clientX, clientY);
    const node = findNodeAtCanvas(c.x, c.y, 8 / zoomState.scale);
    
    if (!node) {
      if (arrowDrawing.isDrawing) {
        setArrowDrawing(prev => ({
          ...prev,
          points: [...prev.points, { x: c.x, y: c.y }]
        }));
      }
      return;
    }

    if (!arrowDrawing.isDrawing) {
      setArrowDrawing({
        isDrawing: true,
        fromId: node.id,
        points: []
      });
    } else {
      if (arrowDrawing.fromId === node.id) {
        setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
      } else {
        const arrow = { 
          id: makeId("arrow"), 
          fromId: arrowDrawing.fromId, 
          toId: node.id,
          points: [...arrowDrawing.points]
        };
        setArrows((prev) => [...prev, arrow]);
        setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
      }
    }
  };

  const cancelArrowDrawing = () => {
    setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
  };

  const pointInPolygon = (point, vs) => {
    if (!vs.length) return false;
    const x = point[0], y = point[1];
    let inside = false;
    for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
      const xi = vs[i][0], yi = vs[i][1];
      const xj = vs[j][0], yj = vs[j][1];
      const intersect = ((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 0.0000001) + xi);
      if (intersect) inside = !inside;
    }
    return inside;
  };

  const finishZone = () => {
    if (!currentZonePoints.length) return;
    const z = {
      id: makeId("zone"),
      name: zoneName || `Zone_${zones.length + 1}`,
      type: zoneType,
      points: currentZonePoints.slice(),
    };
    setZones((prev) => [...prev, z]);
    setCurrentZonePoints([]);
    setZoneName("");
    setZoneType("normal");
  };

  // --- Mouse handlers ---
  const handleMouseDown = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    if (e.button === 1 || tool === "pan") {
      setZoomState((prev) => ({ ...prev, isDragging: true, lastX: x, lastY: y }));
      return;
    }
    
    if (tool === "place_node") {
      placeNodeAtClient(e.clientX, e.clientY);
      return;
    }
    if (tool === "connect") {
      handleConnectClick(e.clientX, e.clientY);
      return;
    }
    if (tool === "zone") {
      if (!mapParamsRef.current) return;
      const ros = clientToRosCoords(e.clientX, e.clientY);
      if (!ros) return;
      const canvas = rosToCanvasCoords(ros.x, ros.y);
      setCurrentZonePoints((prev) => [...prev, { rosX: ros.x, rosY: ros.y, canvasX: canvas.x, canvasY: canvas.y }]);
      return;
    }
    if (tool === "erase") {
      if (eraseMode === "hand") {
        startHandErase(e.clientX, e.clientY);
      } else {
        eraseAtClient(e.clientX, e.clientY);
      }
      return;
    }
    if (tool === "crop") {
      const c = clientToCanvasCoords(e.clientX, e.clientY);
      // Start freehand selection
      setCropState({
        ...cropState,
        startX: c.x,
        startY: c.y,
        endX: c.x,
        endY: c.y,
        isDragging: true,
        freehandPoints: [{x: c.x, y: c.y}] // Start with first point
      });
      return;
    }
  };

  const handleMouseMove = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const rectX = e.clientX - rect.left;
    const rectY = e.clientY - rect.top;
    const ros = clientToRosCoords(e.clientX, e.clientY);
    
    if (ros) {
      const canvas = rosToCanvasCoords(ros.x, ros.y);
      setCursorCoords({ rosX: ros.x, rosY: ros.y, canvasX: canvas.x, canvasY: canvas.y });
    } else {
      setCursorCoords(null);
    }

    if (zoomState.isDragging) {
      const x = e.clientX - rect.left;
      const y = e.clientY - rect.top;
      setZoomState((prev) => ({ 
        ...prev, 
        offsetX: prev.offsetX + (x - prev.lastX), 
        offsetY: prev.offsetY + (y - prev.lastY), 
        lastX: x, 
        lastY: y 
      }));
    }

    // Freehand selection drawing
    if (cropState.isDragging && tool === "crop") {
      const c = clientToCanvasCoords(e.clientX, e.clientY);
      setCropState(prev => ({
        ...prev,
        endX: c.x,
        endY: c.y,
        freehandPoints: [...prev.freehandPoints, {x: c.x, y: c.y}]
      }));
    }

    if (handEraseState.isErasing) {
      continueHandErase(e.clientX, e.clientY);
    }
  };

  const handleMouseUp = (e) => {
    if (zoomState.isDragging) setZoomState((prev) => ({ ...prev, isDragging: false }));
    
    if (cropState.isDragging && tool === "crop") {
      // Complete the freehand selection by closing the polygon
      const c = clientToCanvasCoords(e.clientX, e.clientY);
      const completedPoints = [...cropState.freehandPoints, {x: c.x, y: c.y}];
      
      setCropState(prev => ({
        ...prev,
        endX: c.x,
        endY: c.y,
        isDragging: false,
        freehandPoints: completedPoints
      }));
      
      // REMOVED AUTO-SAVE - User will manually click save button
      return;
    }
    
    if (handEraseState.isErasing) stopHandErase();
  };

  const handleWheel = (e) => {
    if (!mapMsg) return;
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
    
    setZoomState((prev) => ({ 
      ...prev, 
      scale: newScale,
      offsetX: centerX - zoomPointX * newScale,
      offsetY: centerY - zoomPointY * newScale
    }));
  };

  const handleButtonZoom = (zoomFactor) => {
    if (!mapMsg || !canvasRef.current) return;
    
    const rect = canvasRef.current.getBoundingClientRect();
    const containerWidth = rect.width;
    const containerHeight = rect.height;
    
    const centerX = containerWidth / 2;
    const centerY = containerHeight / 2;
    
    const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
    const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
    const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
    setZoomState((prev) => ({ 
      ...prev, 
      scale: newScale,
      offsetX: centerX - zoomPointX * newScale,
      offsetY: centerY - zoomPointY * newScale
    }));
  };

  // --- Save / Load / Clear ---
  const handleSaveJSON = () => {
    const payload = {
      mapName,
      nodes,
      arrows,
      zones,
      mapParams: mapParamsRef.current,
      timestamp: Date.now(),
    };
    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = `${mapName || "map"}_editor.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  const handleUploadJSON = (file) => {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const obj = JSON.parse(ev.target.result);
        if (obj.nodes) setNodes(obj.nodes);
        if (obj.arrows) setArrows(obj.arrows);
        if (obj.zones) setZones(obj.zones);
        if (obj.mapName) setMapName(obj.mapName);
        if (!mapParamsRef.current && obj.mapParams) mapParamsRef.current = obj.mapParams;
      } catch (err) {
        console.error("Invalid JSON", err);
        alert("Invalid JSON file");
      }
    };
    reader.readAsText(file);
  };

  const handleClearAll = () => {
    if (window.confirm("Are you sure you want to clear ALL nodes, arrows, and zones?")) {
      setNodes([]);
      setArrows([]);
      setZones([]);
      setCurrentZonePoints([]);
      setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
      setRotation(0);
    }
  };

  // --- ROS connection ---
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

        if (canvasRef.current) {
          const container = canvasRef.current.parentElement;
          const offsetX = (container.clientWidth - msg.info.width) / 2;
          const offsetY = (container.clientHeight - msg.info.height) / 2;
          setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
        }
      });
    });

    ros.on("error", (err) => {
      console.error("ROS error", err);
      setRosConnected(false);
    });
    ros.on("close", () => {
      setRosConnected(false);
      clearTimeout(reconnectTimer);
      reconnectTimer = setTimeout(connect, 2000);
    });

    connect();

    return () => {
      mapListener?.unsubscribe();
      clearTimeout(reconnectTimer);
      ros.close();
    };
  }, []);

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

    const mapToRender = editableMap || mapMsg;
    if (!mapToRender) return;

    const { width, height, data } = mapToRender;

    ctx.save();
    
    if (rotation !== 0) {
      const centerX = containerWidth / 2;
      const centerY = containerHeight / 2;
      ctx.translate(centerX, centerY);
      ctx.rotate((rotation * Math.PI) / 180);
      ctx.translate(-centerX, -centerY);
    }

    ctx.translate(zoomState.offsetX, zoomState.offsetY);
    ctx.scale(zoomState.scale, zoomState.scale);

   // In your drawing useEffect, replace the imageData creation part:
const imageData = ctx.createImageData(width, height);
for (let y = 0; y < height; y++) {
  for (let x = 0; x < width; x++) {
    const val = data[y * width + x];
    const py = height - 1 - y;
    const i = (py * width + x) * 4;

    let gray;
    if (val === -2) {
      // Checkerboard pattern for cut areas
      const checkerSize = 8; // Size of checkerboard squares
      const isCheckered = ((Math.floor(x / checkerSize) + Math.floor(y / checkerSize)) % 2) === 0;
      gray = isCheckered ? 200 : 230; // Light gray checkerboard
    } else if (val === -1) gray = 205;
    else if (val === 0) gray = 255;
    else if (val === 100) gray = 0;
    else gray = 255 - Math.floor((val / 100) * 255);

    imageData.data[i] = gray;
    imageData.data[i + 1] = gray;
    imageData.data[i + 2] = gray;
    imageData.data[i + 3] = 255;
  }
}

    const off = document.createElement("canvas");
    off.width = width;
    off.height = height;
    off.getContext("2d").putImageData(imageData, 0, 0);
    ctx.drawImage(off, 0, 0, width, height);

    // Draw freehand crop selection
    if (cropState.isCropping && cropState.freehandPoints.length > 0) {
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2 / zoomState.scale;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      
      cropState.freehandPoints.forEach((point, idx) => {
        if (idx === 0) ctx.moveTo(point.x, point.y);
        else ctx.lineTo(point.x, point.y);
      });
      
      // Draw line to current cursor position if still dragging
      if (cropState.isDragging && cursorCoords) {
        ctx.lineTo(cursorCoords.canvasX, cursorCoords.canvasY);
      }
      
      ctx.stroke();
      ctx.setLineDash([]);

      // Fill the polygon with semi-transparent color
      if (cropState.freehandPoints.length >= 3) {
        ctx.fillStyle = 'rgba(0, 255, 0, 0.1)';
        ctx.beginPath();
        cropState.freehandPoints.forEach((point, idx) => {
          if (idx === 0) ctx.moveTo(point.x, point.y);
          else ctx.lineTo(point.x, point.y);
        });
        ctx.closePath();
        ctx.fill();
      }

      // Show point count
      if (cropState.freehandPoints.length > 0) {
        const lastPoint = cropState.freehandPoints[cropState.freehandPoints.length - 1];
        ctx.fillStyle = '#00ff00';
        ctx.font = `${12 / zoomState.scale}px Arial`;
        ctx.fillText(`${cropState.freehandPoints.length} points`, lastPoint.x + 5, lastPoint.y - 5);
      }
    }

    // Draw zones
    zones.forEach((z) => {
      if (!z.points || z.points.length < 2) return;
      ctx.beginPath();
      z.points.forEach((p, idx) => {
        if (idx === 0) ctx.moveTo(p.canvasX, p.canvasY);
        else ctx.lineTo(p.canvasX, p.canvasY);
      });
      ctx.closePath();
      
       if (z.type === 'keep_out') {
    // Solid black fill for keepout zones
    ctx.fillStyle = "rgba(0, 0, 0, 1.0)";
    ctx.fill();
    ctx.strokeStyle = "rgba(0, 0, 0, 1.0)";
    ctx.lineWidth = 2 / zoomState.scale;
    ctx.stroke();
  } else {
        ctx.fillStyle = "rgba(16,185,129,0.12)";
        ctx.fill();
        ctx.strokeStyle = "rgba(16,185,129,0.6)";
        ctx.lineWidth = 1 / zoomState.scale;
        ctx.stroke();
      }

      const cx = z.points.reduce((s, p) => s + p.canvasX, 0) / z.points.length;
      const cy = z.points.reduce((s, p) => s + p.canvasY, 0) / z.points.length;
      
      if (z.type === 'keep_out') {
        ctx.fillStyle = "#dc2626";
      } else {
        ctx.fillStyle = "#064e3b";
      }
      ctx.font = `${12 / zoomState.scale}px Arial`;
      ctx.fillText(z.name, cx + 6 / zoomState.scale, cy);
    });

    // Current zone points
    if (currentZonePoints.length) {
      ctx.beginPath();
      currentZonePoints.forEach((p, idx) => {
        if (idx === 0) ctx.moveTo(p.canvasX, p.canvasY);
        else ctx.lineTo(p.canvasX, p.canvasY);
      });
      
       if (zoneType === 'keep_out') {
    ctx.strokeStyle = "rgba(0, 0, 0, 0.9)"; // Black for keepout zones
    ctx.lineWidth = 2 / zoomState.scale; // Thicker line for visibility
  } else {
    ctx.strokeStyle = "rgba(59,130,246,0.9)";
    ctx.lineWidth = 1 / zoomState.scale;
  }
  ctx.stroke();
}

    // Draw arrows
    arrows.forEach((a) => {
      const from = nodes.find((n) => n.id === a.fromId);
      const to = nodes.find((n) => n.id === a.toId);
      if (!from || !to) return;
      
      const allPoints = [from, ...(a.points || []), to];
      if (allPoints.length < 2) return;
      
      ctx.strokeStyle = "#f97316";
      ctx.lineWidth = 2 / zoomState.scale;
      ctx.beginPath();
      ctx.moveTo(allPoints[0].canvasX, allPoints[0].canvasY);
      
      for (let i = 1; i < allPoints.length; i++) {
        ctx.lineTo(allPoints[i].canvasX, allPoints[i].canvasY);
      }
      ctx.stroke();
      
      const lastPoint = allPoints[allPoints.length - 1];
      const secondLastPoint = allPoints[allPoints.length - 2];
      const angle = Math.atan2(
        lastPoint.canvasY - secondLastPoint.canvasY,
        lastPoint.canvasX - secondLastPoint.canvasX
      );
      const headlen = 12 / zoomState.scale;
      
      ctx.beginPath();
      ctx.moveTo(lastPoint.canvasX, lastPoint.canvasY);
      ctx.lineTo(
        lastPoint.canvasX - headlen * Math.cos(angle - Math.PI / 6),
        lastPoint.canvasY - headlen * Math.sin(angle - Math.PI / 6)
      );
      ctx.lineTo(
        lastPoint.canvasX - headlen * Math.cos(angle + Math.PI / 6),
        lastPoint.canvasY - headlen * Math.sin(angle + Math.PI / 6)
      );
      ctx.closePath();
      ctx.fillStyle = "#f97316";
      ctx.fill();
      
      for (let i = 1; i < allPoints.length - 1; i++) {
        const point = allPoints[i];
        ctx.beginPath();
        ctx.arc(point.canvasX, point.canvasY, 4 / zoomState.scale, 0, Math.PI * 2);
        ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
        ctx.fill();
        ctx.strokeStyle = "#f97316";
        ctx.lineWidth = 1 / zoomState.scale;
        ctx.stroke();
      }
    });

    // Draw the currently-being-drawn arrow
    if (arrowDrawing.isDrawing) {
      const from = nodes.find((n) => n.id === arrowDrawing.fromId);
      if (from && cursorCoords) {
        const allPoints = [from, ...arrowDrawing.points, cursorCoords];
        if (allPoints.length >= 2) {
          ctx.strokeStyle = "rgba(255, 165, 0, 0.8)";
          ctx.lineWidth = 2 / zoomState.scale;
          ctx.beginPath();
          ctx.moveTo(allPoints[0].canvasX, allPoints[0].canvasY);
          
          for (let i = 1; i < allPoints.length; i++) {
            ctx.lineTo(allPoints[i].canvasX, allPoints[i].canvasY);
          }
          ctx.stroke();
          
          for (let i = 1; i < allPoints.length - 1; i++) {
            const point = allPoints[i];
            ctx.beginPath();
            ctx.arc(point.x, point.y, 4 / zoomState.scale, 0, Math.PI * 2);
            ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
            ctx.fill();
            ctx.strokeStyle = "#f97316";
            ctx.lineWidth = 1 / zoomState.scale;
            ctx.stroke();
          }
        }
      }
    }

    // Draw nodes
    nodes.forEach((n) => {
      ctx.beginPath();
      ctx.fillStyle = currentTheme.nodeColors[n.type] || "#1e40af";
      ctx.arc(n.canvasX, n.canvasY, Math.max(3, 6 / zoomState.scale), 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = "black";
      ctx.font = `${Math.max(6, 10 / zoomState.scale)}px Arial`;
      ctx.fillText(n.label || n.type, n.canvasX + 8 / zoomState.scale, n.canvasY - 6 / zoomState.scale);
    });

    // Hover crosshair
    if (cursorCoords) {
      ctx.strokeStyle = "rgba(0,0,0,0.4)";
      ctx.lineWidth = 0.5 / zoomState.scale;
      ctx.beginPath();
      ctx.moveTo(cursorCoords.canvasX, 0);
      ctx.lineTo(cursorCoords.canvasX, height);
      ctx.moveTo(0, cursorCoords.canvasY);
      ctx.lineTo(width, cursorCoords.canvasY);
      ctx.stroke();
    }

    ctx.restore();
  }, [mapMsg, editableMap, zoomState, nodes, arrows, zones, currentZonePoints, cursorCoords, currentTheme, rotation, arrowDrawing, cropState, zoneType, tool, eraseMode, eraseRadius]);

  // Map Loader Modal
const MapLoaderModal = () => {
  const handleZipSelect = async (e) => {
    const file = e.target.files[0];
    if (!file) return;

    try {
      const jsZip = new JSZip();
      const zip = await jsZip.loadAsync(file);
      
      // Find YAML and PGM files in the ZIP
      const yamlFiles = Object.keys(zip.files).filter(name => 
        name.toLowerCase().endsWith('.yaml') || name.toLowerCase().endsWith('.yml')
      );
      
      const pgmFiles = Object.keys(zip.files).filter(name => 
        name.toLowerCase().endsWith('.pgm')
      );

      if (yamlFiles.length === 0) {
        alert("No YAML files found in the ZIP archive");
        return;
      }

      if (pgmFiles.length === 0) {
        alert("No PGM files found in the ZIP archive");
        return;
      }

      // Use the first YAML file found
      const yamlFileEntry = zip.files[yamlFiles[0]];
      const yamlText = await yamlFileEntry.async('text');
      const parsedYaml = yaml.load(yamlText);

      // Find the corresponding PGM file
      const expectedPgmName = parsedYaml.image;
      let pgmFileEntry;

      if (expectedPgmName) {
        // Look for exact match first, then try different variations
        const possibleNames = [
          expectedPgmName,
          `${expectedPgmName}.pgm`,
          expectedPgmName.replace('.png', '.pgm').replace('.jpg', '.pgm').replace('.jpeg', '.pgm'),
          ...pgmFiles.filter(name => name.toLowerCase().includes(expectedPgmName.toLowerCase()))
        ];

        for (const name of possibleNames) {
          if (zip.files[name]) {
            pgmFileEntry = zip.files[name];
            break;
          }
        }
      }

      // Fallback to first PGM file if no match found
      if (!pgmFileEntry && pgmFiles.length > 0) {
        pgmFileEntry = zip.files[pgmFiles[0]];
      }

      if (!pgmFileEntry) {
        alert("Could not find matching PGM file in the ZIP archive");
        return;
      }

      // Get PGM file as array buffer
      const pgmArrayBuffer = await pgmFileEntry.async('arraybuffer');
      
      // Create file objects for consistency with existing code
      const yamlFile = new File([yamlText], yamlFiles[0], { type: 'application/x-yaml' });
      const pgmFile = new File([pgmArrayBuffer], pgmFileEntry.name, { type: 'image/x-portable-graymap' });

      // Load the map directly
      await loadMapWithFiles(yamlFile, pgmFile);

    } catch (err) {
      console.error("Error processing ZIP file:", err);
      alert("Failed to process ZIP file. Check console for details.");
    }
  };

  const loadMapWithFiles = async (yamlFile, pgmFile) => {
    try {
      const yamlText = await yamlFile.text();
      const parsedYaml = yaml.load(yamlText);
      setMapInfo(parsedYaml);

      const buffer = await pgmFile.arrayBuffer();
      const bytes = new Uint8Array(buffer);
      const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

      const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
      const magic = headerLines[0];
      if (magic !== "P5" && magic !== "P2") {
        throw new Error("Unsupported PGM format: must be P5 or P2");
      }

      const width = parseInt(headerLines[1]);
      const height = parseInt(headerLines[2]);
      const maxVal = parseInt(headerLines[3]);

      const headerLength = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
      const dataStart = textHeader.slice(0, headerLength).length + 1;
      const pixelBytes = bytes.slice(dataStart);

      const occupancyData = new Int8Array(width * height);
      const negate = parsedYaml.negate || 0;

      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const i = y * width + x;
          let value = pixelBytes[i];
          
          if (negate === 1) {
            value = 255 - value;
          }
          
          let occupancyValue = -1;
          if (value >= 0 && value <= 10) {
            occupancyValue = 100;
          } else if (value >= 250 && value <= 255) {
            occupancyValue = 0;
          }
          
          occupancyData[(height - 1 - y) * width + x] = occupancyValue;
        }
      }

      const loadedMap = {
        width: width,
        height: height,
        resolution: parsedYaml.resolution || 0.05,
        data: occupancyData,
        origin: {
          x: parsedYaml.origin?.[0] || 0,
          y: parsedYaml.origin?.[1] || 0,
          z: parsedYaml.origin?.[2] || 0
        }
      };

      mapParamsRef.current = {
        width: width,
        height: height,
        resolution: parsedYaml.resolution || 0.05,
        originX: parsedYaml.origin?.[0] || 0,
        originY: parsedYaml.origin?.[1] || 0,
      };

      setMapMsg(loadedMap);
      setEditableMap({ ...loadedMap, data: [...occupancyData] });
      setMapLoaderActive(false);

      if (canvasRef.current) {
        const container = canvasRef.current.parentElement;
        const offsetX = (container.clientWidth - width) / 2;
        const offsetY = (container.clientHeight - height) / 2;
        setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
      }

      const mapNameFromFile = yamlFile.name.replace('.yaml', '').replace('.yml', '');
      if (mapNameFromFile && !mapName) {
        setMapName(mapNameFromFile);
      }

      alert(`✅ Map loaded successfully!\n\n📦 Source: ZIP archive\n📊 Size: ${width} x ${height} pixels\n📍 Resolution: ${parsedYaml.resolution} m/pixel`);

    } catch (err) {
      console.error("Error loading map:", err);
      alert("Failed to load map. Check console for details.");
    }
  };

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
        background: currentTheme.card,
        padding: 24,
        borderRadius: 12,
        border: `1px solid ${currentTheme.border}`,
        maxWidth: 500,
        width: '90%',
        boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
      }}>
        <h3 style={{ margin: '0 0 20px 0', color: currentTheme.text }}>
          🗺️ Load Map from ZIP
        </h3>

        <div style={{ marginBottom: 20 }}>
          <label style={styles.label}>
            Select ZIP File:
            <div style={{ 
              marginTop: 6,
              border: `1px solid ${currentTheme.border}`,
              borderRadius: 8,
              padding: '10px 12px',
              background: currentTheme.card,
              color: currentTheme.text,
              fontSize: '14px',
              cursor: 'pointer',
              position: 'relative',
              overflow: 'hidden'
            }}>
              Choose ZIP file containing YAML and PGM...
              <input 
                type="file" 
                accept=".zip" 
                onChange={handleZipSelect} 
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
          <div style={styles.hint}>
            Select a ZIP file containing both YAML and PGM map files. The map will load automatically.
          </div>
        </div>

        {mapInfo && (
          <div style={{
            marginBottom: 15,
            padding: 10,
            background: currentTheme.surface,
            borderRadius: 8,
            border: `1px solid ${currentTheme.border}`,
          }}>
            <h4 style={{ margin: '0 0 8px 0', color: currentTheme.text }}>Map Info:</h4>
            <pre style={{ margin: 0, fontSize: "12px", color: currentTheme.textSecondary }}>
              {JSON.stringify(mapInfo, null, 2)}
            </pre>
            {mapInfo.image && (
              <div style={{ marginTop: 8, fontSize: "12px", color: currentTheme.accent }}>
                📄 PGM File: {mapInfo.image}
              </div>
            )}
          </div>
        )}

        <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
          <button 
            onClick={() => {
              setMapLoaderActive(false);
              setMapInfo(null);
            }}
            style={styles.button}
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
};

  return (
    <div style={styles.container}>
      {mapLoaderActive && <MapLoaderModal />}

      <div style={styles.sidebar}>
        <div style={{ marginBottom: 20 }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 8 }}>
            <h3 style={{ margin: 0, color: currentTheme.text, display: 'flex', alignItems: 'center', gap: 8 }}>
              <FaMapMarkerAlt style={{ color: currentTheme.accent }} />
              Map Editor {opencvLoaded ? "✓" : "⏳"}
            </h3>
            <button 
              onClick={toggleDarkMode}
              style={{
                ...styles.smallButton,
                background: 'transparent',
                color: currentTheme.text,
                border: `1px solid ${currentTheme.border}`
              }}
              title={darkMode ? "Switch to Light Mode" : "Switch to Dark Mode"}
            >
              {darkMode ? <FaSun size={14} /> : <FaMoon size={14} />}
            </button>
          </div>
          <div style={{ fontSize: 12, color: currentTheme.textSecondary }}>
            {darkMode ? 'Dark' : 'Light'} mode • {opencvLoaded ? 'OpenCV Ready' : 'Loading OpenCV...'}
          </div>
        </div>

        <label style={styles.label}>Map name</label>
        <input 
          value={mapName} 
          onChange={(e) => setMapName(e.target.value)} 
          placeholder="My Map" 
          style={styles.input} 
        />

        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Tools</label>
          <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8, marginBottom: 16 }}>
            <button 
              onClick={() => setTool("place_node")} 
              style={tool === "place_node" ? styles.buttonActive : styles.button}
            >
              <FaMapMarkerAlt />
              Node
            </button>
            <button 
              onClick={() => setTool("connect")} 
              style={tool === "connect" ? styles.buttonActive : styles.button}
            >
              <FaArrowRight />
              Arrow
            </button>
            <button 
              onClick={() => setTool("zone")} 
              style={tool === "zone" ? styles.buttonActive : styles.button}
            >
              <FaDrawPolygon />
              Zone
            </button>
            <button 
              onClick={() => setTool("erase")} 
              style={tool === "erase" ? styles.buttonActive : styles.button}
            >
              <FaEraser />
              Erase
            </button>
            <button 
              onClick={startCrop} 
              style={tool === "crop" ? styles.buttonActive : styles.button}
            >
              <FaCut />
              Scissor
            </button>
            <button 
              onClick={() => {
                setTool("zone");
                setZoneType("keep_out");
              }} 
              style={zoneType === "keep_out" ? styles.buttonKeepOut : styles.button}
            >
              <FaBan />
              Restricted Zone
            </button>
          </div>
        </div>

        {/* Erase Mode Controls */}
        {tool === "erase" && (
          <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
            <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
              🧹 Erase Mode: {eraseMode === "objects" ? "Objects" : eraseMode === "noise" ? "Map Noise (Click)" : "Hand Erase (Draw)"}
            </div>
            
            <div style={{ marginBottom: 12 }}>
              <label style={styles.label}>Erase Mode</label>
              <select 
                value={eraseMode} 
                onChange={(e) => setEraseMode(e.target.value)} 
                style={styles.select}
              >
                <option value="objects">🗑️ Erase Objects (Nodes, Zones)</option>
                <option value="noise">🧹 Erase Map Noise (Click)</option>
                <option value="hand">✋ Hand Erase (Draw Freely)</option>
              </select>
            </div>

            {(eraseMode === "noise" || eraseMode === "hand") && (
              <div style={{ marginBottom: 12 }}>
                <label style={styles.label}>Erase Radius: {eraseRadius}px</label>
                <input 
                  type="range"
                  min="1"
                  max="20"
                  value={eraseRadius}
                  onChange={(e) => setEraseRadius(parseInt(e.target.value))}
                  style={{ width: '100%', marginTop: 6 }}
                />
                <div style={styles.hint}>
                  {eraseMode === "hand" 
                    ? "Draw freely to erase black noise areas"
                    : "Larger radius for cleaning large noise areas"}
                </div>
              </div>
            )}

            <div style={styles.hint}>
              {eraseMode === "objects" 
                ? "Click on nodes or zones to delete them"
                : eraseMode === "noise"
                ? `Click on black noise areas to erase them (${eraseRadius}px radius)`
                : `Click and drag to draw erase paths (${eraseRadius}px radius)`}
            </div>
          </div>
        )}

{/* Scissor Crop Controls */}
{cropState.isCropping && (
  <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
    
    <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
      ✂️ Scissor Mode Active
    </div>

    <div style={styles.hint}>
      Draw a freehand selection. Click CUT to remove that exact shape from the map.
    </div>

    <div style={{ display: "flex", gap: 8, marginTop: 8, flexWrap: "wrap" }}>
      
      {/* Actual Cutting Function */}
      <button
        onClick={handleScissorCut}
        style={{
          ...styles.buttonAction,
          background: '#ef4444',
          border: '1px solid #dc2626'
        }}
        disabled={!cropState.freehandPoints || cropState.freehandPoints.length < 3}
        title="Cut actual map region"
      >
        <FaCut />
        Cut Selection
      </button>

      {/* Cancel */}
      <button 
        onClick={cancelCrop}
        style={styles.buttonDanger}
      >
        <FaBan />
        Cancel
      </button>
    </div>

    {cropState.freehandPoints && (
      <div style={{ marginTop: 8, fontSize: '12px', color: currentTheme.textSecondary }}>
        Points: {cropState.freehandPoints.length}  
        (need 3+)
      </div>
    )}

    <div style={{ marginTop: 12, padding: 8, background: currentTheme.background, borderRadius: 4 }}>
      <div style={{ fontSize: '11px', color: currentTheme.textSecondary, lineHeight: 1.4 }}>
        <strong>Cut Selection:</strong> Removes the selected freehand area from the map completely (sets pixels to EMPTY -2).
      </div>
    </div>

  </div>
)}


        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Node type</label>
          <select 
            value={nodeType} 
            onChange={(e) => setNodeType(e.target.value)} 
            style={styles.select}
          >
            <option value="station">🏭 Station</option>
            <option value="docking">📍 Docking Point</option>
            <option value="waypoint">🚩 Waypoint</option>
            <option value="home">🏠 Home Position</option>
            <option value="charging">🔋 Charging Station</option>
          </select>
          <div style={styles.hint}>
            Click on map to place {nodeType} node
          </div>
        </div>

        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Zone name</label>
          <input 
            value={zoneName} 
            onChange={(e) => setZoneName(e.target.value)} 
            placeholder="Zone A" 
            style={styles.input} 
          />
          <div style={styles.hint}>
            Click to add polygon points, then finish zone
          </div>
          <div style={{ display: "flex", gap: 8, marginTop: 12 }}>
            <button onClick={finishZone} style={styles.button}>
              <FaDrawPolygon />
              Finish Zone
            </button>
            <button onClick={() => setCurrentZonePoints([])} style={styles.button}>
              <FaTrashAlt />
              Cancel Zone
            </button>
          </div>
        </div>

        {/* Arrow Drawing Controls */}
        {arrowDrawing.isDrawing && (
          <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
            <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
              Drawing Arrow...
            </div>
            <div style={styles.hint}>
              Click on empty space to add bend points
            </div>
            <button 
              onClick={cancelArrowDrawing}
              style={{...styles.buttonDanger, marginTop: 8 }}
            >
              <FaTrashAlt />
              Cancel Arrow
            </button>
          </div>
        )}

        {/* Rotation Control Section */}
        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Map Rotation</label>
          <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
            <input 
              type="number"
              min="0"
              max="359"
              value={rotation}
              onChange={handleRotationInput}
              style={{
                ...styles.input,
                margin: 0,
                flex: 1,
                textAlign: 'center'
              }}
              placeholder="Enter degrees (0-359)"
            />
            <span style={{ 
              fontSize: '14px', 
              color: currentTheme.textSecondary,
              minWidth: '30px'
            }}>
              °
            </span>
          </div>
          <div style={styles.hint}>
            Enter rotation angle (0-359 degrees)
          </div>
          <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
            <button onClick={handleRotate} style={styles.button}>
              <FaRedo />
              +90°
            </button>
            <button 
              onClick={() => setRotation(0)} 
              style={styles.button}
            >
              <FaExpand />
              Reset
            </button>
          </div>
        </div>

        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Map Operations</label>
          <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
            <button 
              onClick={() => setMapLoaderActive(true)} 
              style={styles.buttonAction}
            >
              <FaUpload />
              Load Map
            </button>
            <button onClick={saveNodesToDatabase} style={styles.buttonAction}>
              <FaSave />
              Save to DB
            </button>
            <button onClick={loadNodesFromDatabase} style={styles.buttonAction}>
              <FaUpload />
              Load from DB
            </button>
            <button 
              onClick={saveMapToComputer} 
              style={styles.buttonAction}
              disabled={!opencvLoaded}
            >
              <FaDownload />
              {opencvLoaded ? "Save Full Map" : "Loading OpenCV..."}
            </button>
            <button 
              onClick={saveMapAsPNG} 
              style={styles.buttonAction}
            >
              <FaImage />
              Save Full PNG
            </button>
          </div>
        </div>

        <div style={{ display: "flex", gap: 8, flexWrap: "wrap", marginBottom: 16 }}>
          <button onClick={handleSaveJSON} style={styles.button}>
            <FaSave />
            Save JSON
          </button>
          <label style={{...styles.button, cursor: 'pointer', textAlign: 'center'}}>
            <FaUpload />
            Upload JSON
            <input type="file" accept="application/json" style={{ display: "none" }} onChange={(e) => handleUploadJSON(e.target.files?.[0])} />
          </label>
          <button onClick={handleClearAll} style={styles.buttonDanger}>
            <FaTrashAlt />
            Clear All
          </button>
        </div>
      </div>

      <div style={{ flex: 1 }}>
        <div style={{ marginBottom: 12, display: "flex", justifyContent: "space-between", alignItems: "center" }}>
          <div style={styles.hud}>
            {cursorCoords ? `ROS: ${cursorCoords.rosX.toFixed(2)}, ${cursorCoords.rosY.toFixed(2)}` : "Hover map for coordinates"}
          </div>
          <div style={{ display: "flex", gap: 8, alignItems: 'center' }}>
            <div style={styles.status}>
              <div style={{ 
                width: 8, 
                height: 8, 
                borderRadius: '50%', 
                background: 'currentColor',
                opacity: rosConnected ? 1 : 0.5
              }} />
              {rosConnected ? "ROS Connected" : "ROS Disconnected"}
            </div>
            <div style={styles.mapInfo}>
              Map: {mapName || "—"}
            </div>
            <div style={styles.mapInfo}>
              Rotation: {rotation}°
            </div>
            <div style={styles.mapInfo}>
              OpenCV: {opencvLoaded ? "✓" : "⏳"}
            </div>
            {tool === "erase" && (
              <div style={styles.mapInfo}>
                Erase: {eraseMode === "objects" ? "Objects" : eraseMode === "noise" ? `Noise (${eraseRadius}px)` : `Hand (${eraseRadius}px)`}
              </div>
            )}
            {tool === "crop" && (
              <div style={styles.mapInfo}>
                Scissor: {cropState.freehandPoints?.length || 0} points
              </div>
            )}
          </div>
        </div>

        <div style={styles.canvasContainer}>
          <div style={{
            position: "absolute",
            top: 12,
            right: 12,
            display: "flex",
            flexDirection: "column",
            gap: 8,
            zIndex: 20,
            background: currentTheme.card,
            padding: 22,
            borderRadius: 12,
            boxShadow: "0 6px 18px rgba(0,0,0,0.15)",
            border: `1px solid ${currentTheme.border}`
          }}>
            <div style={{ 
              textAlign: "center", 
              fontSize: 13, 
              fontWeight: '700', 
              color: currentTheme.text,
              marginBottom: 4 
            }}>
              {Math.round(zoomState.scale * 100)}%
            </div>
            <button 
              onClick={() => handleButtonZoom(1.2)}
              style={styles.smallButton}
              title="Zoom In"
            >
              <FaSearchPlus />
            </button>
            <button 
              onClick={() => handleButtonZoom(0.8)}
              style={styles.smallButton}
              title="Zoom Out"
            >
              <FaSearchMinus />
            </button>
            <button 
              onClick={handleRotate}
              style={styles.smallButton}
              title="Rotate 90°"
            >
              <FaRedo />
            </button>
            <button 
              onClick={() => {
                if (!mapMsg || !canvasRef.current) return;
                const container = canvasRef.current.parentElement;
                const offsetX = (container.clientWidth - mapMsg.width) / 2;
                const offsetY = (container.clientHeight - mapMsg.height) / 2;
                setZoomState((p) => ({ ...p, scale: 1, offsetX, offsetY }));
                setRotation(0);
              }} 
              style={styles.smallButton}
              title="Reset View"
            >
              <FaExpand />
            </button>
          </div>

          <canvas
            ref={canvasRef}
            style={{ 
              width: "100%", 
              height: "100%", 
              display: "block", 
              cursor: cropState.isCropping ? "crosshair" : 
                      zoomState.isDragging ? "grabbing" : 
                      tool === "erase" && eraseMode === "hand" ? "crosshair" : "crosshair" 
            }}
            onMouseDown={handleMouseDown}
            onMouseMove={handleMouseMove}
            onMouseUp={handleMouseUp}
            onMouseLeave={() => { 
              setCursorCoords(null); 
              setZoomState((p)=> ({...p, isDragging:false})); 
              setCropState(prev => ({...prev, isDragging: false}));
              if (handEraseState.isErasing) stopHandErase();
            }}
            onWheel={handleWheel}
          />
        </div>
      </div>
    </div>
  );
}


