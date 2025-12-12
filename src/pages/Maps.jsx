
// import React, { useEffect, useRef, useState } from "react";
// import ROSLIB from "roslib";
// import yaml from "js-yaml";
// import JSZip from 'jszip';
// import { 
//   FaSearchPlus, 
//   FaSearchMinus, 
//   FaExpand, 
//   FaTrashAlt, 
//   FaMapMarkerAlt,
//   FaArrowRight,
//   FaDrawPolygon,
//   FaEraser,
//   FaSave,
//   FaUpload,
//   FaWarehouse,
//   FaDownload,
//   FaRedo,
//   FaCropAlt,
//   FaImage,
//   FaMoon,
//   FaSun,
//   FaBan,
//   FaBrush,
//   FaCut,
//   FaUndo,
//   FaRedoAlt
// } from "react-icons/fa";

// // OpenCV loader
// const loadOpenCV = () => {
//   return new Promise((resolve, reject) => {
//     if (window.cv) {
//       resolve();
//       return;
//     }
    
//     const script = document.createElement('script');
//     script.src = 'https://docs.opencv.org/4.5.0/opencv.js';
//     script.onload = () => {
//       const checkOpenCV = () => {
//         if (window.cv && window.cv.Mat) {
//           resolve();
//         } else {
//           setTimeout(checkOpenCV, 50);
//         }
//       };
//       checkOpenCV();
//     };
//     script.onerror = reject;
//     document.head.appendChild(script);
//   });
// };

// export default function MapEditor() {
//   // --- Theme state with localStorage persistence ---
//   const [darkMode, setDarkMode] = useState(() => {
//     const saved = localStorage.getItem("mapEditorDarkMode");
//     if (saved !== null) {
//       return JSON.parse(saved);
//     }
//     return window.matchMedia?.('(prefers-color-scheme: dark)').matches || false;
//   });

//   // --- ROS & map state ---
//   const [rosConnected, setRosConnected] = useState(false);
//   const [mapMsg, setMapMsg] = useState(null);
//   const [editableMap, setEditableMap] = useState(null);
//   const mapParamsRef = useRef(null);
//   const [opencvLoaded, setOpencvLoaded] = useState(false);

//   // --- Map Loader State ---
//   const [mapLoaderActive, setMapLoaderActive] = useState(false);
//   const [yamlFile, setYamlFile] = useState(null);
//   const [pgmFile, setPgmFile] = useState(null);
//   const [mapInfo, setMapInfo] = useState(null);

//   // --- Editor state ---
//   const [tool, setTool] = useState(() => {
//     const saved = localStorage.getItem("tool");
//     return saved || "place_node";
//   });
//   const [nodeType, setNodeType] = useState(() => {
//     const saved = localStorage.getItem("nodeType");
//     return saved || "station";
//   });
//   const [mapName, setMapName] = useState(() => {
//     const saved = localStorage.getItem("mapName");
//     return saved || "";
//   });
//   const [zoneName, setZoneName] = useState("");
//   const [zoneType, setZoneType] = useState("normal"); // "normal" or "keep_out"
//   const [nodes, setNodes] = useState(() => {
//     const saved = localStorage.getItem("nodes");
//     return saved ? JSON.parse(saved) : [];
//   });
//   const [arrows, setArrows] = useState(() => {
//     const saved = localStorage.getItem("arrows");
//     return saved ? JSON.parse(saved) : [];
//   });
//   const [zones, setZones] = useState(() => {
//     const saved = localStorage.getItem("zones");
//     return saved ? JSON.parse(saved) : [];
//   });
//   const [currentZonePoints, setCurrentZonePoints] = useState(() => {
//     const saved = localStorage.getItem("currentZonePoints");
//     return saved ? JSON.parse(saved) : [];
//   });

//   // --- Arrow drawing state ---
//   const [arrowDrawing, setArrowDrawing] = useState({
//     isDrawing: false,
//     fromId: null,
//     points: []
//   });

//   // --- Rotation state ---
//   const [rotation, setRotation] = useState(() => {
//     const saved = localStorage.getItem("rotation");
//     return saved ? parseInt(saved) : 0;
//   });

//   // --- Crop state ---
//   const [cropState, setCropState] = useState({
//     isCropping: false,
//     startX: 0,
//     startY: 0,
//     endX: 0,
//     endY: 0,
//     isDragging: false,
//     freehandPoints: [] // Store freehand points
//   });

//   // --- Erase mode state ---
//   const [eraseMode, setEraseMode] = useState(() => {
//     const saved = localStorage.getItem("eraseMode");
//     return saved || "objects"; // "objects", "noise", or "hand"
//   });
//   const [eraseRadius, setEraseRadius] = useState(() => {
//     const saved = localStorage.getItem("eraseRadius");
//     return saved ? parseInt(saved) : 3;
//   });

//   // --- Hand erase state ---
//   const [handEraseState, setHandEraseState] = useState({
//     isErasing: false,
//     lastX: 0,
//     lastY: 0
//   });

//   // --- UI helpers ---
//   const canvasRef = useRef(null);
//   const [zoomState, setZoomState] = useState({
//     scale: 1,
//     offsetX: 500,
//     offsetY: 110,
//     isDragging: false,
//     lastX: 0,
//     lastY: 0,
//   });
//   const [cursorCoords, setCursorCoords] = useState(null);
//   const idCounter = useRef(1);

//   // --- Database state ---
//   const [dbStatus, setDbStatus] = useState("Ready");
//   const [isSaving, setIsSaving] = useState(false);

//   // --- Undo/Redo State ---
//   const [history, setHistory] = useState(() => {
//     const saved = localStorage.getItem("mapEditorHistory");
//     return saved ? JSON.parse(saved) : [];
//   });
//   const [historyIndex, setHistoryIndex] = useState(() => {
//     const saved = localStorage.getItem("mapEditorHistoryIndex");
//     return saved ? parseInt(saved) : -1;
//   });

//   // --- Save current state to history ---
//   const saveToHistory = () => {
//     const currentState = {
//       nodes: JSON.parse(JSON.stringify(nodes)),
//       arrows: JSON.parse(JSON.stringify(arrows)),
//       zones: JSON.parse(JSON.stringify(zones)),
//       editableMap: editableMap ? {
//         ...editableMap,
//         data: [...editableMap.data]
//       } : null,
//       rotation: rotation,
//       timestamp: Date.now()
//     };

//     const newHistory = history.slice(0, historyIndex + 1);
//     newHistory.push(currentState);
    
//     if (newHistory.length > 50) {
//       newHistory.shift();
//     }
    
//     setHistory(newHistory);
//     setHistoryIndex(newHistory.length - 1);
    
//     localStorage.setItem("mapEditorHistory", JSON.stringify(newHistory));
//     localStorage.setItem("mapEditorHistoryIndex", (newHistory.length - 1).toString());
//   };

//   // --- Undo function ---
//   const handleUndo = () => {
//     if (historyIndex > 0) {
//       const newIndex = historyIndex - 1;
//       const state = history[newIndex];
      
//       setNodes(state.nodes);
//       setArrows(state.arrows);
//       setZones(state.zones);
//       setRotation(state.rotation);
      
//       if (state.editableMap) {
//         setEditableMap(state.editableMap);
//         setMapMsg(prev => prev ? { ...prev, data: state.editableMap.data } : null);
//       }
      
//       setHistoryIndex(newIndex);
//       localStorage.setItem("mapEditorHistoryIndex", newIndex.toString());
//     }
//   };

//   // --- Redo function ---
//   const handleRedo = () => {
//     if (historyIndex < history.length - 1) {
//       const newIndex = historyIndex + 1;
//       const state = history[newIndex];
      
//       setNodes(state.nodes);
//       setArrows(state.arrows);
//       setZones(state.zones);
//       setRotation(state.rotation);
      
//       if (state.editableMap) {
//         setEditableMap(state.editableMap);
//         setMapMsg(prev => prev ? { ...prev, data: state.editableMap.data } : null);
//       }
      
//       setHistoryIndex(newIndex);
//       localStorage.setItem("mapEditorHistoryIndex", newIndex.toString());
//     }
//   };

//   // --- Clear history when map is loaded ---
//   const clearHistory = () => {
//     setHistory([]);
//     setHistoryIndex(-1);
//     localStorage.removeItem("mapEditorHistory");
//     localStorage.removeItem("mapEditorHistoryIndex");
//   };

//   // --- Auto-save to localStorage ---
//   useEffect(() => {
//     localStorage.setItem("nodes", JSON.stringify(nodes));
//   }, [nodes]);

//   useEffect(() => {
//     localStorage.setItem("arrows", JSON.stringify(arrows));
//   }, [arrows]);

//   useEffect(() => {
//     localStorage.setItem("zones", JSON.stringify(zones));
//   }, [zones]);

//   useEffect(() => {
//     localStorage.setItem("currentZonePoints", JSON.stringify(currentZonePoints));
//   }, [currentZonePoints]);

//   useEffect(() => {
//     localStorage.setItem("mapName", mapName);
//   }, [mapName]);

//   useEffect(() => {
//     localStorage.setItem("tool", tool);
//   }, [tool]);

//   useEffect(() => {
//     localStorage.setItem("nodeType", nodeType);
//   }, [nodeType]);

//   useEffect(() => {
//     localStorage.setItem("rotation", rotation.toString());
//   }, [rotation]);

//   useEffect(() => {
//     localStorage.setItem("eraseMode", eraseMode);
//   }, [eraseMode]);

//   useEffect(() => {
//     localStorage.setItem("eraseRadius", eraseRadius.toString());
//   }, [eraseRadius]);

//   // --- Dark mode persistence ---
//   useEffect(() => {
//     localStorage.setItem("mapEditorDarkMode", JSON.stringify(darkMode));
//   }, [darkMode]);

//   // --- Load OpenCV on component mount ---
//   useEffect(() => {
//     loadOpenCV()
//       .then(() => {
//         setOpencvLoaded(true);
//         console.log("OpenCV.js loaded successfully");
//       })
//       .catch(err => {
//         console.error("Failed to load OpenCV:", err);
//       });
//   }, []);

//   // --- System dark mode detection ---
//   useEffect(() => {
//     const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    
//     const handleChange = (e) => {
//       const hasUserChoice = localStorage.getItem("mapEditorDarkMode") !== null;
//       if (!hasUserChoice) {
//         setDarkMode(e.matches);
//       }
//     };
    
//     mediaQuery.addEventListener('change', handleChange);
//     return () => mediaQuery.removeEventListener('change', handleChange);
//   }, []);

//   // --- Theme toggle function ---
//   const toggleDarkMode = () => {
//     setDarkMode(prev => !prev);
//   };

//   // --- Theme styles ---
//   const theme = {
//     light: {
//       background: '#ffffff',
//       surface: '#f8fafc',
//       surfaceSecondary: '#ffffff',
//       card: '#ffffff',
//       text: '#1e293b',
//       textSecondary: '#64748b',
//       border: '#e2e8f0',
//       accent: '#3b82f6',
//       accentHover: '#2563eb',
//       danger: '#ef4444',
//       success: '#10b981',
//       warning: '#f59e0b',
//       nodeColors: {
//         station: '#8b5cf6',
//         docking: '#06b6d4',
//         waypoint: '#f59e0b',
//         home: '#10b981',
//         charging: '#ef4444'
//       }
//     },
//     dark: {
//       background: '#0f172a',
//       surface: '#1e293b',
//       surfaceSecondary: '#1e293b',
//       card: '#1e293b',
//       text: '#f1f5f9',
//       textSecondary: '#94a3b8',
//       border: '#334155',
//       accent: '#60a5fa',
//       accentHover: '#3b82f6',
//       danger: '#f87171',
//       success: '#34d399',
//       warning: '#fbbf24',
//       nodeColors: {
//         station: '#a78bfa',
//         docking: '#22d3ee',
//         waypoint: '#fbbf24',
//         home: '#34d399',
//         charging: '#f87171'
//       }
//     }
//   };

//   const currentTheme = darkMode ? theme.dark : theme.light;

//   const styles = {
//     container: {
//       display: "flex",
//       gap: 12,
//       padding: 12,
//       width: "100%",
//       boxSizing: "border-box",
//       background: currentTheme.background,
//       color: currentTheme.text,
//       minHeight: '100vh',
//       fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
//     },
//     sidebar: {
//       width: 320,
//       background: currentTheme.card,
//       borderRadius: 12,
//       padding: 16,
//       boxShadow: "0 4px 20px rgba(0,0,0,0.1)",
//       border: `1px solid ${currentTheme.border}`,
//     },
//     canvasContainer: {
//       flex: 1,
//       border: `2px solid ${currentTheme.border}`,
//       borderRadius: 12,
//       background: currentTheme.surface,
//       minHeight: 900,
//       position: "relative",
//       overflow: "hidden"
//     },
//     button: {
//       padding: "10px 12px",
//       borderRadius: 8,
//       border: `1px solid ${currentTheme.border}`,
//       background: currentTheme.card,
//       color: currentTheme.text,
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 8,
//       flex: 1,
//       justifyContent: 'center'
//     },
//     buttonActive: {
//       padding: "10px 12px",
//       borderRadius: 8,
//       border: `1px solid ${currentTheme.accent}`,
//       background: currentTheme.accent,
//       color: "white",
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 8,
//       flex: 1,
//       justifyContent: 'center'
//     },
//     buttonAction: {
//       padding: "10px 12px",
//       borderRadius: 8,
//       border: "none",
//       background: currentTheme.accent,
//       color: "white",
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 8,
//       flex: 1,
//       justifyContent: 'center'
//     },
//     buttonDanger: {
//       padding: "10px 12px",
//       borderRadius: 8,
//       border: "none",
//       background: currentTheme.danger,
//       color: "white",
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 8,
//       flex: 1,
//       justifyContent: 'center'
//     },
//     buttonKeepOut: {
//       padding: "10px 12px",
//       borderRadius: 8,
//       border: "none",
//       background: '#000000',
//       color: '#ffffff',
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 8,
//       flex: 1,
//       justifyContent: 'center'
//     },
//     buttonUndoRedo: {
//       padding: "8px 12px",
//       borderRadius: 8,
//       border: `1px solid ${currentTheme.border}`,
//       background: currentTheme.card,
//       color: currentTheme.text,
//       cursor: "pointer",
//       fontSize: '14px',
//       fontWeight: '500',
//       transition: 'all 0.2s ease',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 6,
//       justifyContent: 'center',
//       minWidth: '44px'
//     },
//     smallButton: {
//       padding: 10,
//       borderRadius: 8,
//       border: "none",
//       background: currentTheme.accent,
//       color: "#fff",
//       cursor: "pointer",
//       display: 'flex',
//       alignItems: 'center',
//       justifyContent: 'center',
//       transition: 'all 0.2s ease',
//     },
//     input: {
//       width: "100%",
//       padding: "10px 12px",
//       margin: "6px 0 12px 0",
//       borderRadius: 8,
//       border: `1px solid ${currentTheme.border}`,
//       background: currentTheme.card,
//       color: currentTheme.text,
//       fontSize: '14px',
//       boxSizing: 'border-box'
//     },
//     select: {
//       width: "100%",
//       padding: "10px 12px",
//       marginTop: 6,
//       borderRadius: 8,
//       border: `1px solid ${currentTheme.border}`,
//       background: currentTheme.card,
//       color: currentTheme.text,
//       fontSize: '14px',
//     },
//     label: {
//       fontSize: "14px",
//       color: currentTheme.text,
//       fontWeight: '600',
//       marginBottom: 4,
//       display: 'block'
//     },
//     hint: {
//       fontSize: "12px",
//       color: currentTheme.textSecondary,
//       marginTop: 6,
//       lineHeight: '1.4'
//     },
//     hud: {
//       background: currentTheme.accent,
//       color: "#fff",
//       padding: "8px 12px",
//       borderRadius: 8,
//       fontSize: 14,
//       fontWeight: '500',
//       fontFamily: 'monospace'
//     },
//     status: {
//       padding: "8px 12px",
//       background: rosConnected ? currentTheme.success : currentTheme.danger,
//       color: "white",
//       borderRadius: 8,
//       fontSize: 14,
//       fontWeight: '500',
//       display: 'flex',
//       alignItems: 'center',
//       gap: 6
//     },
//     mapInfo: {
//       padding: "8px 12px",
//       background: currentTheme.card,
//       color: currentTheme.text,
//       borderRadius: 8,
//       fontSize: 14,
//       fontWeight: '500',
//       border: `1px solid ${currentTheme.border}`
//     }
//   };

//   // --- Helper Functions ---

//   // Point in polygon algorithm
//   const isPointInPolygon = (point, polygon) => {
//     const x = point.x, y = point.y;
    
//     if (!polygon || polygon.length < 3) {
//       console.warn("Invalid polygon for point-in-polygon check:", polygon);
//       return false;
//     }

//     let inside = false;
//     for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
//       const xi = polygon[i].x || polygon[i].canvasX;
//       const yi = polygon[i].y || polygon[i].canvasY;
//       const xj = polygon[j].x || polygon[j].canvasX;
//       const yj = polygon[j].y || polygon[j].canvasY;
      
//       const intersect = ((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 0.0000001) + xi);
//       if (intersect) inside = !inside;
//     }
    
//     return inside;
//   };

//   // Get bounding box of freehand selection
//   const getFreehandBoundingBox = (points) => {
//     if (!points || points.length === 0) return { minX: 0, minY: 0, maxX: 0, maxY: 0 };
    
//     let minX = points[0].x;
//     let minY = points[0].y;
//     let maxX = points[0].x;
//     let maxY = points[0].y;
    
//     points.forEach(point => {
//       minX = Math.min(minX, point.x);
//       minY = Math.min(minY, point.y);
//       maxX = Math.max(maxX, point.x);
//       maxY = Math.max(maxY, point.y);
//     });
    
//     return { minX, minY, maxX, maxY };
//   };

//   // --- Map Loader Functions ---
//   const handleLoadMap = async () => {
//     try {
//       if (!yamlFile || !pgmFile) {
//         alert("Please select both YAML and PGM files first.");
//         return;
//       }

//       const yamlText = await yamlFile.text();
//       const parsedYaml = yaml.load(yamlText);
//       setMapInfo(parsedYaml);

//       const buffer = await pgmFile.arrayBuffer();
//       const bytes = new Uint8Array(buffer);
//       const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

//       const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
//       const magic = headerLines[0];
//       if (magic !== "P5" && magic !== "P2") {
//         throw new Error("Unsupported PGM format: must be P5 or P2");
//       }

//       const width = parseInt(headerLines[1]);
//       const height = parseInt(headerLines[2]);
//       const maxVal = parseInt(headerLines[3]);

//       const headerLength = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
//       const dataStart = textHeader.slice(0, headerLength).length + 1;
//       const pixelBytes = bytes.slice(dataStart);

//       const occupancyData = new Int8Array(width * height);
//       const negate = parsedYaml.negate || 0;

//       for (let y = 0; y < height; y++) {
//         for (let x = 0; x < width; x++) {
//           const i = y * width + x;
//           let value = pixelBytes[i];
          
//           if (negate === 1) {
//             value = 255 - value;
//           }
          
//           let occupancyValue = -1;
//           if (value >= 0 && value <= 10) {
//             occupancyValue = 100;
//           } else if (value >= 250 && value <= 255) {
//             occupancyValue = 0;
//           }
          
//           occupancyData[(height - 1 - y) * width + x] = occupancyValue;
//         }
//       }

//       const loadedMap = {
//         width: width,
//         height: height,
//         resolution: parsedYaml.resolution || 0.05,
//         data: occupancyData,
//         origin: {
//           x: parsedYaml.origin?.[0] || 0,
//           y: parsedYaml.origin?.[1] || 0,
//           z: parsedYaml.origin?.[2] || 0
//         }
//       };

//       mapParamsRef.current = {
//         width: width,
//         height: height,
//         resolution: parsedYaml.resolution || 0.05,
//         originX: parsedYaml.origin?.[0] || 0,
//         originY: parsedYaml.origin?.[1] || 0,
//       };

//       setMapMsg(loadedMap);
//       setEditableMap({ ...loadedMap, data: [...occupancyData] });
//       setMapLoaderActive(false);

//       if (canvasRef.current) {
//         const container = canvasRef.current.parentElement;
//         const offsetX = (container.clientWidth - width) / 2;
//         const offsetY = (container.clientHeight - height) / 2;
//         setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
//       }

//       const mapNameFromFile = yamlFile.name.replace('.yaml', '').replace('.yml', '');
//       if (mapNameFromFile && !mapName) {
//         setMapName(mapNameFromFile);
//       }

//       // Clear history when new map is loaded
//       clearHistory();

//     } catch (err) {
//       console.error("Error loading map:", err);
//       alert("Failed to load map. Check console for details.");
//     }
//   };

//   // --- CROP FUNCTIONS ---
//   const startCrop = () => {
//     setCropState({
//       isCropping: true,
//       startX: 0,
//       startY: 0,
//       endX: 0,
//       endY: 0,
//       isDragging: false,
//       freehandPoints: []
//     });
//     setTool("crop");
//   };

//   const cancelCrop = () => {
//     setCropState({
//       isCropping: false,
//       startX: 0,
//       startY: 0,
//       endX: 0,
//       endY: 0,
//       isDragging: false,
//       freehandPoints: []
//     });
//     setTool("place_node");
//   };
// // Apply crop to current map (visual checkerboard for SELECTED area)
// // Apply crop to current map (KEEP the SELECTED area, DELETE everything else)
// // Apply crop to current map (DELETE everything OUTSIDE selected area with transparency)
// const handleApplyCrop = () => {
//   console.log("✂️ Applying crop to current map - KEEP selected area, DELETE everything else WITH TRANSPARENCY");

//   if (!mapMsg || !mapParamsRef.current) {
//     alert("No map loaded!");
//     return;
//   }

//   if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
//     alert("Draw a closed freehand shape first!");
//     return;
//   }

//   // Save state before cropping for undo
//   saveToHistory();

//   const { width, height } = mapParamsRef.current;

//   // Convert freehand (canvas coords) → map coords (flip Y axis)
//   const mapPolygon = cropState.freehandPoints.map(p => ({
//     x: p.x,
//     y: height - 1 - p.y
//   }));

//   const newData = new Int8Array([...mapMsg.data]);
//   let keptPixels = 0;
//   let deletedPixels = 0;

//   // DELETE everything OUTSIDE the selected area with TRUE TRANSPARENCY
//   for (let y = 0; y < height; y++) {
//     for (let x = 0; x < width; x++) {
//       const idx = y * width + x;
      
//       if (isPointInPolygon({ x, y }, mapPolygon)) {
//         // INSIDE polygon - KEEP original value
//         // But make sure it's not already transparent
//         if (newData[idx] !== -2 && newData[idx] !== -3) {
//           keptPixels++;
//         }
//       } else {
//         // OUTSIDE polygon - MAKE IT TRANSPARENT (REAL TRANSPARENCY)
//         // Use -3 for true transparency (not checkerboard)
//         newData[idx] = -3; // This will be transparent in PNG export
//         deletedPixels++;
//       }
//     }
//   }

//   console.log(`✅ Applied crop with transparency: Kept ${keptPixels} pixels INSIDE selection, made ${deletedPixels} pixels TRANSPARENT`);

//   // Update map
//   setMapMsg(prev => prev ? { ...prev, data: newData } : null);
//   setEditableMap(prev => prev ? { ...prev, data: [...newData] } : null);

//   // Reset state
//   setCropState({
//     isCropping: false,
//     startX: 0,
//     startY: 0,
//     endX: 0,
//     endY: 0,
//     isDragging: false,
//     freehandPoints: []
//   });

//   setTool("place_node");

//   alert(`✅ Crop applied with TRANSPARENCY!\n\n✅ Kept ${keptPixels} pixels INSIDE selection\n🌫️ Made ${deletedPixels} pixels TRANSPARENT\n📸 Will be transparent in PNG export`);
// };

// // Export TRULY cropped map (KEEP only INSIDE selected area)
// const handleExportCroppedMap = async () => {
//   console.log("📐 Exporting truly cropped map - KEEP only INSIDE selected area");

//   const SCALE_FACTOR = 4;

//   if (!mapMsg || !mapParamsRef.current) {
//     alert("No map loaded!");
//     return;
//   }

//   if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
//     alert("Invalid crop selection! Please draw at least 3 points.");
//     return;
//   }

//   try {
//     const freehandPoints = cropState.freehandPoints;
//     const { width, height, resolution, originX, originY, rotation = 0 } = mapParamsRef.current;

//     // Convert freehand points from canvas to map coordinates
//     const mapPolygon = freehandPoints.map(p => ({
//       x: p.x,
//       y: height - 1 - p.y  // flip Y axis
//     }));

//     // Find bounds of the SELECTED area only
//     let minX = width, minY = height, maxX = 0, maxY = 0;
//     let hasContent = false;

//     for (let y = 0; y < height; y++) {
//       for (let x = 0; x < width; x++) {
//         if (isPointInPolygon({ x, y }, mapPolygon)) { // INSIDE selection
//           const srcIndex = y * width + x;
//           // Only consider non-deleted, non-empty pixels
//           if (mapMsg.data[srcIndex] !== -2 && mapMsg.data[srcIndex] !== 0) {
//             minX = Math.min(minX, x);
//             minY = Math.min(minY, y);
//             maxX = Math.max(maxX, x);
//             maxY = Math.max(maxY, y);
//             hasContent = true;
//           }
//         }
//       }
//     }

//     if (!hasContent) {
//       alert("No valid map content found INSIDE the selected area!");
//       return;
//     }

//     // Add padding
//     const padding = 2;
//     minX = Math.max(0, minX - padding);
//     minY = Math.max(0, minY - padding);
//     maxX = Math.min(width - 1, maxX + padding);
//     maxY = Math.min(height - 1, maxY + padding);

//     const contentWidth = maxX - minX + 1;
//     const contentHeight = maxY - minY + 1;

//     console.log(`📐 True crop bounds: ${minX},${minY} to ${maxX},${maxY} (${contentWidth}x${contentHeight})`);

//     // Create new map data with ONLY content from INSIDE selected area
//     const contentData = new Int8Array(contentWidth * contentHeight);
//     let keptPixels = 0;

//     for (let y = 0; y < contentHeight; y++) {
//       for (let x = 0; x < contentWidth; x++) {
//         const srcX = minX + x;
//         const srcY = minY + y;
//         const srcIndex = srcY * width + srcX;
//         const dstIndex = y * contentWidth + x;
        
//         if (isPointInPolygon({ x: srcX, y: srcY }, mapPolygon)) {
//           // INSIDE polygon - keep if not deleted
//           if (mapMsg.data[srcIndex] !== -2) {
//             contentData[dstIndex] = mapMsg.data[srcIndex];
//             if (mapMsg.data[srcIndex] !== 0 && mapMsg.data[srcIndex] !== -1) {
//               keptPixels++;
//             }
//           } else {
//             // Deleted area inside polygon - make it white
//             contentData[dstIndex] = 0;
//           }
//         } else {
//           // OUTSIDE polygon - make it white (transparent)
//           contentData[dstIndex] = 0;
//         }
//       }
//     }

//     console.log(`✅ True crop: ${keptPixels} non-white pixels kept (inside selected area)`);

//     // Calculate new origin for the truly cropped map
//     const newOriginX = originX + minX * resolution;
//     const newOriginY = originY + (height - (minY + contentHeight)) * resolution;

//     // Create high-res canvas
//     const scaledW = contentWidth * SCALE_FACTOR;
//     const scaledH = contentHeight * SCALE_FACTOR;
//     const canvas = document.createElement("canvas");
//     canvas.width = scaledW;
//     canvas.height = scaledH;
//     const ctx = canvas.getContext("2d");
//     ctx.scale(SCALE_FACTOR, SCALE_FACTOR);

//     // Fill background with transparent
//     ctx.clearRect(0, 0, contentWidth, contentHeight);

//     ctx.save();
//     if (rotation !== 0) {
//       ctx.translate(contentWidth / 2, contentHeight / 2);
//       ctx.rotate((rotation * Math.PI) / 180);
//       ctx.translate(-contentWidth / 2, -contentHeight / 2);
//     }

//     // Create and draw image data
//     const imgData = ctx.createImageData(contentWidth, contentHeight);
//     for (let y = 0; y < contentHeight; y++) {
//       for (let x = 0; x < contentWidth; x++) {
//         const val = contentData[y * contentWidth + x];
//         const py = contentHeight - 1 - y;
//         const i = (py * contentWidth + x) * 4;
        
//         let gray, alpha;
//         if (val === 100) {
//           gray = 0;   // Black
//           alpha = 255;
//         } else if (val === -1) {
//           gray = 128; // Gray for unknown
//           alpha = 255;
//         } else if (val === 0) {
//           // White areas should be transparent if they're outside selection
//           const srcX = minX + x;
//           const srcY = minY + y;
//           if (isPointInPolygon({ x: srcX, y: srcY }, mapPolygon)) {
//             // Inside selection - white but opaque
//             gray = 255;
//             alpha = 255;
//           } else {
//             // Outside selection - transparent
//             gray = 255;
//             alpha = 0;
//           }
//         } else {
//           gray = 255;
//           alpha = 255;
//         }
        
//         imgData.data[i] = gray;
//         imgData.data[i + 1] = gray;
//         imgData.data[i + 2] = gray;
//         imgData.data[i + 3] = alpha;
//       }
//     }

//     const offCanvas = document.createElement("canvas");
//     offCanvas.width = contentWidth;
//     offCanvas.height = contentHeight;
//     offCanvas.getContext("2d").putImageData(imgData, 0, 0);
//     ctx.drawImage(offCanvas, 0, 0);
//     ctx.restore();

//     // Save files
//     const fileBase = `${mapName || "true_cropped"}_${contentWidth}x${contentHeight}_hires${SCALE_FACTOR}x_rot${rotation}`;
//     const newResolution = resolution / SCALE_FACTOR;

//     // Save PNG
//     canvas.toBlob((blob) => {
//       const url = URL.createObjectURL(blob);
//       const link = document.createElement("a");
//       link.href = url;
//       link.download = `${fileBase}.png`;
//       link.click();
//       URL.revokeObjectURL(url);
//     }, "image/png");

//     // Save YAML
//     const yamlContent = `image: ${fileBase}.png
// mode: trinary
// resolution: ${newResolution.toFixed(6)}
// origin: [${newOriginX.toFixed(6)}, ${newOriginY.toFixed(6)}, 0]
// negate: 0
// occupied_thresh: 0.65
// free_thresh: 0.25
// # True cropped map - KEEPING ONLY SELECTED AREA
// # Selected area inside freehand polygon is kept
// # Everything outside is removed
// # Original resolution: ${resolution} m/pixel
// # Scaled resolution: ${newResolution.toFixed(6)} m/pixel
// # Scale factor: ${SCALE_FACTOR}x
// # Content size: ${contentWidth}x${contentHeight} pixels
// # Scaled size: ${scaledW}x${scaledH} pixels
// # Rotation: ${rotation}°
// # Pixels kept: ${keptPixels}
// # True crop bounds: x[${minX},${maxX}], y[${minY},${maxY}]
// # Map size reduced from ${width}x${height} to ${contentWidth}x${contentHeight}`;

//     const yamlBlob = new Blob([yamlContent], { type: "application/x-yaml" });
//     const yamlUrl = URL.createObjectURL(yamlBlob);
//     const yamlLink = document.createElement("a");
//     yamlLink.href = yamlUrl;
//     yamlLink.download = `${fileBase}.yaml`;
//     yamlLink.click();
//     URL.revokeObjectURL(yamlUrl);

//     // Reset state
//     setCropState({
//       isCropping: false,
//       startX: 0,
//       startY: 0,
//       endX: 0,
//       endY: 0,
//       isDragging: false,
//       freehandPoints: []
//     });

//     setTool("place_node");

//     setTimeout(() => {
//       alert(`✅ True Cropped Map Exported!\n\n📄 ${fileBase}.png\n✅ Kept ${keptPixels} non-white pixels INSIDE selection\n🗑️ Completely removed EVERYTHING OUTSIDE selection\n📏 Map size: ${contentWidth}x${contentHeight} pixels (reduced from ${width}x${height})\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n⚪ Everything outside selection completely removed`);
//     }, 100);

//   } catch (e) {
//     console.error("Error exporting true cropped map:", e);
//     alert("❌ Error exporting true cropped map");
//   }
// };

// const saveMapAsPNG = () => {
//   if (!mapMsg || !mapParamsRef.current) {
//     alert("No map loaded to save!");
//     return;
//   }

//   // If there's an active crop selection, ask user what they want
//   if (cropState.freehandPoints && cropState.freehandPoints.length >= 3) {
//     const choice = confirm(
//       "Crop selection detected!\n\n" +
//       "Click OK to export the truly cropped map (only selected area, reduced size).\n" +
//       "Click Cancel to save current map with transparent deleted areas."
//     );
    
//     if (choice) {
//       // Export truly cropped map (reduced size)
//       handleExportCroppedMap();
//       return;
//     }
//     // Continue to save current map with transparency
//   }

//   // Save the full map (or currently displayed map) with transparency for deleted areas
//   try {
//     const { width, height } = mapMsg;
//     const { resolution, originX, originY } = mapParamsRef.current;
    
//     const SCALE_FACTOR = 4;
    
//     const tempCanvas = document.createElement('canvas');
//     const ctx = tempCanvas.getContext('2d');
    
//     tempCanvas.width = width * SCALE_FACTOR;
//     tempCanvas.height = height * SCALE_FACTOR;
    
//     // Clear with transparent background
//     ctx.clearRect(0, 0, tempCanvas.width, tempCanvas.height);
    
//     ctx.save();
//     ctx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
//     if (rotation !== 0) {
//       ctx.translate(width / 2, height / 2);
//       ctx.rotate((rotation * Math.PI) / 180);
//       ctx.translate(-width / 2, -height / 2);
//     }

//     // STEP 1: Draw base map with TRUE TRANSPARENCY for deleted areas
//     const imageData = ctx.createImageData(width, height);
    
//     // Count transparent pixels for feedback
// let transparentPixels = 0;
// let occupiedPixels = 0;
// let freePixels = 0;
// let unknownPixels = 0;
// let checkerboardPixels = 0;
    
//    for (let y = 0; y < height; y++) {
//   for (let x = 0; x < width; x++) {
//     const val = mapMsg.data[y * width + x];
//     const py = height - 1 - y;
//     const i = (py * width + x) * 4;
    
//     let gray, alpha;
    
//     if (val === -3) {
//       // CROPPED AREAS (value -3) - MAKE TRANSPARENT
//       gray = 255; // White
//       alpha = 0;  // Fully transparent
//       transparentPixels++;
//     } else if (val === -2) {
//       // OLD DELETED AREAS - ALSO TRANSPARENT
//       gray = 255;
//       alpha = 0;
//       transparentPixels++;
//     } else if (val === -1) {
//       gray = 205;
//       alpha = 255;
//       unknownPixels++;
//     } else if (val === 0) {
//       gray = 255;
//       alpha = 255;
//       freePixels++;
//     } else if (val === 100) {
//       gray = 0;
//       alpha = 255;
//       occupiedPixels++;
//     } else {
//       gray = 255 - Math.floor((val / 100) * 255);
//       alpha = 255;
//       if (val > 50) occupiedPixels++;
//       else freePixels++;
//     }

//     imageData.data[i] = gray;
//     imageData.data[i + 1] = gray;
//     imageData.data[i + 2] = gray;
//     imageData.data[i + 3] = alpha;
//   }
// }
    
//     const off = document.createElement("canvas");
//     off.width = width;
//     off.height = height;
//     off.getContext("2d").putImageData(imageData, 0, 0);
//     ctx.drawImage(off, 0, 0, width, height);

//     // STEP 2: Create a mask for transparent areas so annotations don't overwrite them
//     // We'll check if a pixel is transparent before drawing annotations
    
//     // Helper function to check if a point is in a transparent area
//     const isPointTransparent = (canvasX, canvasY) => {
//       // Convert canvas coordinates to map coordinates
//       const mapX = Math.floor(canvasX);
//       const mapY = Math.floor(canvasY);
      
//       if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) {
//         return true; // Outside map is transparent
//       }
      
//       const val = mapMsg.data[mapY * width + mapX];
//       return val === -2; // -2 means transparent
//     };

//     // Transform point helper function
//     const transformPoint = (point) => {
//       if (rotation === 0) return { x: point.canvasX, y: point.canvasY };
      
//       const nx = point.canvasX / width;
//       const ny = point.canvasY / height;
      
//       let rotatedX, rotatedY;
      
//       switch (rotation) {
//         case 90:
//           rotatedX = (1 - ny) * width;
//           rotatedY = nx * height;
//           break;
//         case 180:
//           rotatedX = (1 - nx) * width;
//           rotatedY = (1 - ny) * height;
//           break;
//         case 270:
//           rotatedX = ny * width;
//           rotatedY = (1 - nx) * height;
//           break;
//         default:
//           rotatedX = point.canvasX;
//           rotatedY = point.canvasY;
//       }
      
//       return { x: rotatedX, y: rotatedY };
//     };

//     // STEP 3: Draw zones with transparency check
//     zones.forEach((zone) => {
//       if (zone.points && zone.points.length >= 3) {
//         // Check if zone center is in transparent area
//         const centerX = zone.points.reduce((sum, p) => sum + transformPoint(p).x, 0) / zone.points.length;
//         const centerY = zone.points.reduce((sum, p) => sum + transformPoint(p).y, 0) / zone.points.length;
        
//         if (isPointTransparent(centerX, centerY)) {
//           // Skip drawing zone if center is in transparent area
//           return;
//         }
        
//         ctx.beginPath();
//         zone.points.forEach((point, idx) => {
//           const transformed = transformPoint(point);
//           if (idx === 0) ctx.moveTo(transformed.x, transformed.y);
//           else ctx.lineTo(transformed.x, transformed.y);
//         });
//         ctx.closePath();
        
//         if (zone.type === 'keep_out') {
//           ctx.fillStyle = "rgba(0, 0, 0, 1.0)";
//           ctx.fill();
//           ctx.strokeStyle = "rgba(0, 0, 0, 1.0)";
//           ctx.lineWidth = 2 / SCALE_FACTOR;
//           ctx.stroke();
//         } else {
//           ctx.fillStyle = "rgba(100, 200, 100, 0.3)";
//           ctx.fill();
//           ctx.strokeStyle = "rgba(50, 150, 50, 0.8)";
//           ctx.lineWidth = 1.4 / SCALE_FACTOR;
//           ctx.stroke();
//         }

//         if (zone.type === 'keep_out') {
//           ctx.fillStyle = "rgba(220, 38, 38, 1)";
//         } else {
//           ctx.fillStyle = "rgba(50, 150, 50, 1)";
//         }
//         ctx.font = `${14 / SCALE_FACTOR}px Arial`;
//         ctx.fillText(zone.name, centerX + 16 / SCALE_FACTOR, centerY + 8 / SCALE_FACTOR);
//       }
//     });

//     // STEP 4: Draw arrows with transparency check
//     arrows.forEach((arrow) => {
//       const fromNode = nodes.find(n => n.id === arrow.fromId);
//       const toNode = nodes.find(n => n.id === arrow.toId);
      
//       if (fromNode && toNode) {
//         // Check if either endpoint is in transparent area
//         const fromTransformed = transformPoint(fromNode);
//         const toTransformed = transformPoint(toNode);
        
//         if (isPointTransparent(fromTransformed.x, fromTransformed.y) || 
//             isPointTransparent(toTransformed.x, toTransformed.y)) {
//           // Skip drawing arrow if endpoints are in transparent areas
//           return;
//         }
        
//         const allPoints = [fromNode, ...(arrow.points || []), toNode];
//         if (allPoints.length < 2) return;
        
//         // Check all points for transparency
//         let hasTransparentPoint = false;
//         for (const point of allPoints) {
//           const transformed = transformPoint(point);
//           if (isPointTransparent(transformed.x, transformed.y)) {
//             hasTransparentPoint = true;
//             break;
//           }
//         }
        
//         if (hasTransparentPoint) {
//           // Skip drawing arrow if any point is in transparent area
//           return;
//         }
        
//         ctx.strokeStyle = "rgba(255, 165, 0, 0.8)";
//         ctx.lineWidth = 4 / SCALE_FACTOR;
//         ctx.beginPath();
        
//         const firstPoint = transformPoint(allPoints[0]);
//         ctx.moveTo(firstPoint.x, firstPoint.y);
        
//         for (let i = 1; i < allPoints.length; i++) {
//           const point = transformPoint(allPoints[i]);
//           ctx.lineTo(point.x, point.y);
//         }
//         ctx.stroke();
        
//         const lastPoint = transformPoint(allPoints[allPoints.length - 1]);
//         const secondLastPoint = transformPoint(allPoints[allPoints.length - 2]);
//         const angle = Math.atan2(
//           lastPoint.y - secondLastPoint.y,
//           lastPoint.x - secondLastPoint.x
//         );
//         const headLength = 24 / SCALE_FACTOR;
        
//         ctx.beginPath();
//         ctx.moveTo(lastPoint.x, lastPoint.y);
//         ctx.lineTo(
//           lastPoint.x - headLength * Math.cos(angle - Math.PI / 6),
//           lastPoint.y - headLength * Math.sin(angle - Math.PI / 6)
//         );
//         ctx.lineTo(
//           lastPoint.x - headLength * Math.cos(angle + Math.PI / 6),
//           lastPoint.y - headLength * Math.sin(angle + Math.PI / 6)
//         );
//         ctx.closePath();
//         ctx.fillStyle = "rgba(255, 165, 0, 1)";
//         ctx.fill();
        
//         for (let i = 1; i < allPoints.length - 1; i++) {
//           const point = transformPoint(allPoints[i]);
//           ctx.beginPath();
//           ctx.arc(point.x, point.y, 8 / SCALE_FACTOR, 0, Math.PI * 2);
//           ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
//           ctx.fill();
//           ctx.strokeStyle = "rgba(255, 165, 0, 1)";
//           ctx.lineWidth = 2 / SCALE_FACTOR;
//           ctx.stroke();
//         }
//       }
//     });

//     // STEP 5: Draw nodes with transparency check
//     nodes.forEach((node) => {
//       const transformed = transformPoint(node);
      
//       // Check if node is in transparent area
//       if (isPointTransparent(transformed.x, transformed.y)) {
//         // Skip drawing node if it's in transparent area
//         return;
//       }
      
//       const nodeColors = {
//         station: '#8b5cf6',
//         docking: '#06b6d4',
//         waypoint: '#f59e0b',
//         home: '#10b981',
//         charging: '#ef4444'
//       };
      
//       const nodeColor = nodeColors[node.type] || "#3b82f6";
      
//       ctx.beginPath();
//       ctx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
//       ctx.fillStyle = nodeColor;
//       ctx.fill();
//     });

//     ctx.restore();

//     // Generate YAML file
//     const newResolution = resolution / SCALE_FACTOR;
//     const yamlContent = `image: ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png
// mode: trinary
// resolution: ${newResolution.toFixed(6)}
// origin: [${originX}, ${originY}, 0]
// negate: 0
// occupied_thresh: 0.65
// free_thresh: 0.25
// # High-resolution export WITH REAL TRANSPARENCY
// # Original resolution: ${resolution} m/pixel
// # Scaled resolution: ${newResolution.toFixed(6)} m/pixel
// # Scale factor: ${SCALE_FACTOR}x
// # Original size: ${width}x${height} pixels
// # Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
// # Rotation: ${rotation}°
// # Pixel Statistics:
// #   Transparent (deleted/cropped): ${transparentPixels} pixels
// #   Occupied (black): ${occupiedPixels} pixels
// #   Free (white): ${freePixels} pixels
// #   Unknown (gray): ${unknownPixels} pixels
// # Deleted areas (-2): REAL TRANSPARENCY (alpha=0)
// # Annotations NOT drawn on transparent areas
// # Annotations included: Nodes (${nodes.length}), Arrows (${arrows.length}), Zones (${zones.length})`;

//     tempCanvas.toBlob((blob) => {
//       const url = URL.createObjectURL(blob);
//       const link = document.createElement('a');
//       link.href = url;
//       link.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png`;
//       document.body.appendChild(link);
//       link.click();
//       document.body.removeChild(link);
//       URL.revokeObjectURL(url);
      
//       const yamlBlob = new Blob([yamlContent], { type: "application/x-yaml" });
//       const yamlUrl = URL.createObjectURL(yamlBlob);
//       const yamlLink = document.createElement('a');
//       yamlLink.href = yamlUrl;
//       yamlLink.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml`;
//       document.body.appendChild(yamlLink);
//       yamlLink.click();
//       document.body.removeChild(yamlLink);
//       URL.revokeObjectURL(yamlUrl);
      
//       alert(`✅ High-resolution map saved WITH REAL TRANSPARENCY!\n\n📁 Files:\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n🔄 Rotation: ${rotation}°\n🌫️ Transparent areas: ${transparentPixels} pixels (REAL transparency - check with transparency checker)\n📝 Annotations: ${nodes.length} nodes, ${arrows.length} arrows, ${zones.length} zones\n⚠️ Annotations NOT drawn on transparent areas`);

//     }, 'image/png', 1.0);

//   } catch (error) {
//     console.error('Error saving high-resolution map as PNG:', error);
//     alert('❌ Error saving high-resolution map as PNG. Check console for details.');
//   }
// };
//   // --- Enhanced Erase Functions ---
//   const eraseAtClient = (clientX, clientY) => {
//     if (!mapMsg) return;
    
//     if (eraseMode === "objects") {
//       const c = clientToCanvasCoords(clientX, clientY);
//       eraseObjectsAt(c.x, c.y);
//     } else if (eraseMode === "noise") {
//       eraseNoiseAt(clientX, clientY);
//     }
//   };

//   const eraseObjectsAt = (canvasX, canvasY) => {
//     // Save state before erasing for undo
//     saveToHistory();
    
//     // Check nodes first
//     const node = findNodeAtCanvas(canvasX, canvasY, 8 / zoomState.scale);
//     if (node) {
//       setNodes((prev) => prev.filter((p) => p.id !== node.id));
//       setArrows((prev) => prev.filter((a) => a.fromId !== node.id && a.toId !== node.id));
//       return;
//     }
    
//     // Check zones
//     for (const z of zones) {
//       const poly = z.points.map((p) => [p.canvasX, p.canvasY]);
//       if (pointInPolygon([canvasX, canvasY], poly)) {
//         setZones((prev) => prev.filter((pz) => pz.id !== z.id));
//         return;
//       }
//     }
//   };

//   const eraseNoiseAt = (clientX, clientY) => {
//     if (!editableMap || !canvasRef.current) return;

//     // Save state before erasing for undo
//     saveToHistory();

//     const canvasCoords = clientToCanvasCoords(clientX, clientY);
//     const mapX = Math.floor(canvasCoords.x);
//     const mapY = Math.floor(editableMap.height - canvasCoords.y);

//     const { width, height, data } = editableMap;

//     if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) return;

//     const newData = [...data];
//     let erased = 0;

//     for (let y = mapY - eraseRadius; y <= mapY + eraseRadius; y++) {
//       for (let x = mapX - eraseRadius; x <= mapX + eraseRadius; x++) {
//         if (x >= 0 && x < width && y >= 0 && y < height) {
//           const d = Math.sqrt((x - mapX) ** 2 + (y - mapY) ** 2);
//           if (d <= eraseRadius) {
//             const idx = y * width + x;
//             if (newData[idx] === 100 || newData[idx] === -1) {
//               newData[idx] = 0;
//               erased++;
//             }
//           }
//         }
//       }
//     }

//     if (erased > 0) {
//       setEditableMap({ ...editableMap, data: newData });
//       setMapMsg({ ...mapMsg, data: newData });
//     }
//   };

//   // --- Hand Erase Functions ---
//   const startHandErase = (clientX, clientY) => {
//     if (!editableMap || !canvasRef.current) return;

//     saveToHistory();

//     const canvasCoords = clientToCanvasCoords(clientX, clientY);
//     const mapX = Math.floor(canvasCoords.x);
//     const mapY = Math.floor(editableMap.height - canvasCoords.y);

//     const { width, height } = editableMap;

//     if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) return;

//     setHandEraseState({
//       isErasing: true,
//       lastX: mapX,
//       lastY: mapY
//     });

//     eraseNoiseInLine(mapX, mapY, mapX, mapY);
//   };

//   const continueHandErase = (clientX, clientY) => {
//     if (!handEraseState.isErasing || !editableMap || !canvasRef.current) return;

//     const canvasCoords = clientToCanvasCoords(clientX, clientY);
//     const currentX = Math.floor(canvasCoords.x);
//     const currentY = Math.floor(editableMap.height - canvasCoords.y);

//     const { width, height } = editableMap;

//     if (currentX < 0 || currentX >= width || currentY < 0 || currentY >= height) {
//       setHandEraseState(prev => ({
//         ...prev,
//         lastX: currentX,
//         lastY: currentY
//       }));
//       return;
//     }

//     eraseNoiseInLine(handEraseState.lastX, handEraseState.lastY, currentX, currentY);

//     setHandEraseState(prev => ({
//       ...prev,
//       lastX: currentX,
//       lastY: currentY
//     }));
//   };

//   const stopHandErase = () => {
//     setHandEraseState({
//       isErasing: false,
//       lastX: 0,
//       lastY: 0
//     });
//   };

//   const eraseNoiseInLine = (x0, y0, x1, y1) => {
//     const { width, height, data } = editableMap;
//     const newData = [...data];
//     let erased = 0;

//     const points = getLinePoints(x0, y0, x1, y1);

//     points.forEach(point => {
//       const { x, y } = point;
      
//       if (x < 0 || x >= width || y < 0 || y >= height) return;
      
//       for (let dy = -eraseRadius; dy <= eraseRadius; dy++) {
//         for (let dx = -eraseRadius; dx <= eraseRadius; dx++) {
//           const nx = x + dx;
//           const ny = y + dy;
          
//           if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
//             const distance = Math.sqrt(dx * dx + dy * dy);
//             if (distance <= eraseRadius) {
//               const idx = ny * width + nx;
//               if (newData[idx] === 100 || newData[idx] === -1) {
//                 newData[idx] = 0;
//                 erased++;
//               }
//             }
//           }
//         }
//       }
//     });

//     if (erased > 0) {
//       setEditableMap({ ...editableMap, data: newData });
//       setMapMsg({ ...mapMsg, data: newData });
//     }
//   };

//   const getLinePoints = (x0, y0, x1, y1) => {
//     const points = [];
//     const dx = Math.abs(x1 - x0);
//     const dy = Math.abs(y1 - y0);
//     const sx = (x0 < x1) ? 1 : -1;
//     const sy = (y0 < y1) ? 1 : -1;
//     let err = dx - dy;

//     let x = x0;
//     let y = y0;

//     while (true) {
//       points.push({ x, y });

//       if (x === x1 && y === y1) break;

//       const e2 = 2 * err;
//       if (e2 > -dy) {
//         err -= dy;
//         x += sx;
//       }
//       if (e2 < dx) {
//         err += dx;
//         y += sy;
//       }
//     }

//     return points;
//   };

//   // --- Save Map Functions (for full map) ---
// // --- Save Map Functions (for full map) ---
// // --- Save Map Functions (for full map) ---
// const saveMapToComputer = async () => {
//   if (!mapMsg || !mapParamsRef.current) {
//     alert("No map loaded to save!");
//     return;
//   }

//   if (!opencvLoaded) {
//     alert("OpenCV is still loading. Please wait a moment and try again.");
//     return;
//   }

//   try {
//     const { width, height } = mapMsg;
//     const { resolution, originX, originY } = mapParamsRef.current;
    
//     const SCALE_FACTOR = 4;
    
//     // Create a temporary canvas for drawing everything
//     const tempCanvas = document.createElement('canvas');
//     const tempCtx = tempCanvas.getContext('2d');
    
//     if (rotation === 90 || rotation === 270) {
//       tempCanvas.width = height * SCALE_FACTOR;
//       tempCanvas.height = width * SCALE_FACTOR;
//     } else {
//       tempCanvas.width = width * SCALE_FACTOR;
//       tempCanvas.height = height * SCALE_FACTOR;
//     }
    
//     // Fill with white background (PGM needs white for free space)
//     tempCtx.fillStyle = 'white';
//     tempCtx.fillRect(0, 0, tempCanvas.width, tempCanvas.height);
    
//     tempCtx.save();
//     tempCtx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
//     if (rotation !== 0) {
//       const centerX = tempCanvas.width / (2 * SCALE_FACTOR);
//       const centerY = tempCanvas.height / (2 * SCALE_FACTOR);
//       tempCtx.translate(centerX, centerY);
//       tempCtx.rotate((rotation * Math.PI) / 180);
      
//       if (rotation === 90) {
//         tempCtx.translate(-centerY, -centerX);
//       } else if (rotation === 180) {
//         tempCtx.translate(-centerX, -centerY);
//       } else if (rotation === 270) {
//         tempCtx.translate(-centerY, -centerX);
//       } else {
//         tempCtx.translate(-centerX, -centerY);
//       }
//     }

//     // FIRST: Draw zones (they should be behind everything)
//     const transformPoint = (point) => {
//       if (rotation === 0) return { x: point.canvasX, y: point.canvasY };
      
//       const nx = point.canvasX / width;
//       const ny = point.canvasY / height;
      
//       let rotatedX, rotatedY;
      
//       switch (rotation) {
//         case 90:
//           rotatedX = (1 - ny) * width;
//           rotatedY = nx * height;
//           break;
//         case 180:
//           rotatedX = (1 - nx) * width;
//           rotatedY = (1 - ny) * height;
//           break;
//         case 270:
//           rotatedX = ny * width;
//           rotatedY = (1 - nx) * height;
//           break;
//         default:
//           rotatedX = point.canvasX;
//           rotatedY = point.canvasY;
//       }
      
//       return { x: rotatedX, y: rotatedY };
//     };

//     // Draw zones with solid black/gray fill
//     zones.forEach((zone) => {
//       if (zone.points && zone.points.length >= 3) {
//         tempCtx.beginPath();
//         zone.points.forEach((point, idx) => {
//           const transformed = transformPoint(point);
//           if (idx === 0) tempCtx.moveTo(transformed.x, transformed.y);
//           else tempCtx.lineTo(transformed.x, transformed.y);
//         });
//         tempCtx.closePath();
        
//         if (zone.type === 'keep_out') {
//           tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)"; // Solid black
//           tempCtx.fill();
//         } else {
//           tempCtx.fillStyle = "rgba(100, 100, 100, 1.0)"; // Solid dark gray
//           tempCtx.fill();
//         }
//       }
//     });

//     // SECOND: Draw the base map OVER the zones with CHECKERBOARD for deleted areas
//     const imageData = tempCtx.createImageData(width, height);
//     const checkerSize = 8; // Size of checkerboard squares
//     let checkerboardPixels = 0;
//     let deletedPixels = 0;
    
//     for (let y = 0; y < height; y++) {
//       for (let x = 0; x < width; x++) {
//         const val = mapMsg.data[y * width + x];
//         const py = height - 1 - y;
//         const i = (py * width + x) * 4;

//         let gray;
//         if (val === -2) {
//           // DELETED AREAS - DRAW CHECKERBOARD PATTERN
//           const isCheckered = ((Math.floor(x / checkerSize) + Math.floor(y / checkerSize)) % 2) === 0;
//           gray = isCheckered ? 180 : 220; // Checkerboard pattern (dark gray / light gray)
//           checkerboardPixels++;
//           deletedPixels++;
//         } else if (val === -1) {
//           gray = 205; // Unknown - medium gray
//         } else if (val === 0) {
//           gray = 255; // Free space - white
//         } else if (val === 100) {
//           gray = 0; // Occupied - black
//         } else {
//           gray = 255 - Math.floor((val / 100) * 255); // Grayscale based on occupancy
//         }

//         imageData.data[i] = gray;
//         imageData.data[i + 1] = gray;
//         imageData.data[i + 2] = gray;
//         imageData.data[i + 3] = 255; // Fully opaque
//       }
//     }
    
//     const offCanvas = document.createElement("canvas");
//     offCanvas.width = width;
//     offCanvas.height = height;
//     offCanvas.getContext("2d").putImageData(imageData, 0, 0);
//     tempCtx.drawImage(offCanvas, 0, 0, width, height);

//     // THIRD: Draw arrows (on top of map)
//     arrows.forEach((arrow) => {
//       const fromNode = nodes.find(n => n.id === arrow.fromId);
//       const toNode = nodes.find(n => n.id === arrow.toId);
      
//       if (fromNode && toNode) {
//         const allPoints = [fromNode, ...(arrow.points || []), toNode];
//         if (allPoints.length < 2) return;
        
//         tempCtx.strokeStyle = "rgba(0, 0, 0, 1.0)"; // Solid black
//         tempCtx.lineWidth = 4 / SCALE_FACTOR;
//         tempCtx.lineJoin = 'round';
//         tempCtx.lineCap = 'round';
//         tempCtx.beginPath();
        
//         const firstPoint = transformPoint(allPoints[0]);
//         tempCtx.moveTo(firstPoint.x, firstPoint.y);
        
//         for (let i = 1; i < allPoints.length; i++) {
//           const point = transformPoint(allPoints[i]);
//           tempCtx.lineTo(point.x, point.y);
//         }
//         tempCtx.stroke();
        
//         // Draw arrow head
//         const lastPoint = transformPoint(allPoints[allPoints.length - 1]);
//         const secondLastPoint = transformPoint(allPoints[allPoints.length - 2]);
//         const angle = Math.atan2(
//           lastPoint.y - secondLastPoint.y,
//           lastPoint.x - secondLastPoint.x
//         );
//         const headLength = 20 / SCALE_FACTOR;
        
//         tempCtx.save();
//         tempCtx.translate(lastPoint.x, lastPoint.y);
//         tempCtx.rotate(angle);
//         tempCtx.beginPath();
//         tempCtx.moveTo(0, 0);
//         tempCtx.lineTo(-headLength, -headLength / 2);
//         tempCtx.lineTo(-headLength, headLength / 2);
//         tempCtx.closePath();
//         tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)";
//         tempCtx.fill();
//         tempCtx.restore();
//       }
//     });

//     // FOURTH: Draw nodes (on top of everything)
//     nodes.forEach((node) => {
//       const transformed = transformPoint(node);
      
//       // Draw node as solid black circle
//       tempCtx.beginPath();
//       tempCtx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
//       tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)";
//       tempCtx.fill();
      
//       // Draw white border for contrast
//       tempCtx.beginPath();
//       tempCtx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
//       tempCtx.strokeStyle = "rgba(255, 255, 255, 1.0)";
//       tempCtx.lineWidth = 1 / SCALE_FACTOR;
//       tempCtx.stroke();
//     });

//     tempCtx.restore();

//     // Save debug preview as PNG to see the checkerboard
//     const debugUrl = tempCanvas.toDataURL('image/png');
//     const debugLink = document.createElement('a');
//     debugLink.href = debugUrl;
//     debugLink.download = 'debug_pgm_preview_with_checkerboard.png';
//     debugLink.click();

//     // Convert canvas to grayscale for PGM
//     const imageDataFromCanvas = tempCtx.getImageData(0, 0, tempCanvas.width, tempCanvas.height);
//     const finalMat = new cv.Mat(tempCanvas.height, tempCanvas.width, cv.CV_8UC1);
    
//     for (let y = 0; y < tempCanvas.height; y++) {
//       for (let x = 0; x < tempCanvas.width; x++) {
//         const i = (y * tempCanvas.width + x) * 4;
//         const r = imageDataFromCanvas.data[i];
//         const g = imageDataFromCanvas.data[i + 1];
//         const b = imageDataFromCanvas.data[i + 2];
        
//         const gray = Math.round(0.299 * r + 0.587 * g + 0.114 * b);
//         finalMat.ucharPtr(y, x)[0] = gray;
//       }
//     }

//     const pgmHeader = `P5\n${tempCanvas.width} ${tempCanvas.height}\n255\n`;
//     const pgmData = new Uint8Array(tempCanvas.width * tempCanvas.height);
    
//     for (let y = 0; y < tempCanvas.height; y++) {
//       for (let x = 0; x < tempCanvas.width; x++) {
//         pgmData[y * tempCanvas.width + x] = finalMat.ucharPtr(y, x)[0];
//       }
//     }

//     const pgmContent = new Uint8Array(pgmHeader.length + pgmData.length);
//     const encoder = new TextEncoder();
//     const headerBytes = encoder.encode(pgmHeader);
//     pgmContent.set(headerBytes);
//     pgmContent.set(pgmData, headerBytes.length);

//     // Update resolution in YAML to match the scaled image
//     const newResolution = resolution / SCALE_FACTOR;
//     const yamlContent = `image: ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm
// mode: trinary
// resolution: ${newResolution.toFixed(6)}
// origin: [${originX}, ${originY}, 0]
// negate: 0
// occupied_thresh: 0.65
// free_thresh: 0.25
// # High-resolution PGM Export WITH CHECKERBOARD FOR DELETED AREAS
// # Original resolution: ${resolution} m/pixel
// # Scaled resolution: ${newResolution.toFixed(6)} m/pixel
// # Scale factor: ${SCALE_FACTOR}x
// # Original size: ${width}x${height} pixels
// # Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
// # Rotation: ${rotation}°
// # Annotations: ${nodes.length} nodes, ${arrows.length} arrows, ${zones.length} zones
// # Zones: Black/dark gray
// # Arrows: Black lines
// # Nodes: Black circles with white border
// # Deleted areas (-2): CHECKERBOARD PATTERN (180/220 gray)
// # Deleted pixels: ${deletedPixels} (${checkerboardPixels} shown as checkerboard)
// # Checkerboard pattern: 8px squares alternating between gray 180 and 220
// # Note: In PNG export, these areas would be transparent
// # For PGM, checkerboard indicates "removed/not part of map"`;

//     const pgmBlob = new Blob([pgmContent], { type: 'image/x-portable-graymap' });
//     const pgmUrl = URL.createObjectURL(pgmBlob);
//     const pgmLink = document.createElement('a');
//     pgmLink.href = pgmUrl;
//     pgmLink.download = `${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm`;
//     document.body.appendChild(pgmLink);
//     pgmLink.click();
//     document.body.removeChild(pgmLink);
    
//     const yamlBlob = new Blob([yamlContent], { type: 'application/x-yaml' });
//     const yamlUrl = URL.createObjectURL(yamlBlob);
//     const yamlLink = document.createElement('a');
//     yamlLink.href = yamlUrl;
//     yamlLink.download = `${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml`;
//     document.body.appendChild(yamlLink);
//     yamlLink.click();
//     document.body.removeChild(yamlLink);
    
//     finalMat.delete();
    
//     setTimeout(() => {
//       URL.revokeObjectURL(pgmUrl);
//       URL.revokeObjectURL(yamlUrl);
//     }, 100);

//     alert(`✅ High-resolution PGM map exported with CHECKERBOARD!\n\n📁 Files saved:\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n- debug_pgm_preview_with_checkerboard.png (preview)\n🔄 Rotation: ${rotation}°\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n◻️ Deleted areas: CHECKERBOARD PATTERN (not white)\n🗑️ Deleted pixels: ${deletedPixels}\n⚫ Annotations: Black (zones, arrows, nodes)`);

//   } catch (error) {
//     console.error('Error exporting high-resolution PGM map:', error);
//     alert('❌ Error exporting high-resolution PGM map. Check console for details.');
//   }
// };
//   // --- Database Functions ---
//   const saveNodesToDatabase = async () => {
//   if (nodes.length === 0 && arrows.length === 0 && zones.length === 0) {
//     alert("No nodes, arrows, or zones to save!");
//     return;
//   }

//   // Get current map name or use filename
//   const currentMapName = mapName || 
//     (yamlFile ? yamlFile.name.replace('.yaml', '').replace('.yml', '') : 'default');

//   try {
//     setIsSaving(true);
//     setDbStatus("Saving...");

//     const payload = {
//       mapName: currentMapName,
//       nodes: nodes,
//       arrows: arrows,
//       zones: zones,
//     };

//     const response = await fetch('http://localhost:5000/save-nodes', {
//       method: 'POST',
//       headers: {
//         'Content-Type': 'application/json',
//       },
//       body: JSON.stringify(payload),
//     });

//     if (response.ok) {
//       const result = await response.json();
//       setDbStatus("Saved successfully!");
//       alert(`✅ Successfully saved to database!\n\n🗂️ Database: ${result.database_file}\n🏷️ Map: ${result.map_name}\n📁 File: ${result.database_path}`);
//     } else {
//       throw new Error(`HTTP error! status: ${response.status}`);
//     }
//   } catch (error) {
//     console.error('❌ Error saving nodes to database:', error);
//     setDbStatus("Save failed!");
//     alert('❌ Error saving to database. Make sure Flask server is running on port 5000.');
//   } finally {
//     setIsSaving(false);
//   }
// };

//  const loadNodesFromDatabase = async () => {
//   try {
//     // Get current map name or use filename
//     const mapNameToLoad = mapName || 
//       (yamlFile ? yamlFile.name.replace('.yaml', '').replace('.yml', '') : 'default');
    
//     const response = await fetch(`http://localhost:5000/load-nodes?map_name=${mapNameToLoad}`);
    
//     if (response.ok) {
//       const result = await response.json();
      
//       if (result.status === "success") {
//         const mapNodes = result.nodes.map(dbNode => {
//           const canvasCoords = rosToCanvasCoords(dbNode.x, dbNode.y);
//           return {
//             id: `node_${dbNode.id}`,
//             type: dbNode.type,
//             label: dbNode.name,
//             rosX: dbNode.x,
//             rosY: dbNode.y,
//             canvasX: canvasCoords.x,
//             canvasY: canvasCoords.y,
//             dbId: dbNode.id
//           };
//         });
        
//         setNodes(mapNodes);
//         setArrows(result.arrows || []);
//         setZones(result.zones || []);
        
//         alert(`✅ Successfully loaded from database!\n\n📊 Nodes: ${result.nodes.length}\n➡️ Arrows: ${result.arrows.length}\n🗺️ Zones: ${result.zones.length}\n📁 Database: ${result.database_file}`);
//       } else {
//         alert(`❌ Error: ${result.message}`);
//       }
//     } else {
//       const errorResult = await response.json();
//       alert(`❌ Error loading: ${errorResult.message}`);
//     }
//   } catch (error) {
//     console.error('❌ Error loading nodes from database:', error);
//     alert('❌ Error connecting to database server.');
//   }
// };

//   // --- Coordinate conversions ---
//   const rosToCanvasCoords = (rosX, rosY) => {
//     if (!mapParamsRef.current) return { x: 0, y: 0 };
//     const { resolution, originX, originY, width, height } = mapParamsRef.current;
//     const pixelX = (rosX - originX) / resolution;
//     const pixelY = height - (rosY - originY) / resolution;
//     return { x: pixelX, y: pixelY };
//   };

//   const canvasToRosCoords = (canvasX, canvasY) => {
//     if (!mapParamsRef.current) return null;
//     const { resolution, originX, originY, height } = mapParamsRef.current;
//     const flippedY = height - canvasY;
//     const worldX = originX + canvasX * resolution;
//     const worldY = originY + flippedY * resolution;
//     return { x: +worldX.toFixed(2), y: +worldY.toFixed(2) };
//   };

//   const clientToCanvasCoords = (clientX, clientY) => {
//     const canvas = canvasRef.current;
//     if (!canvas) return { x: 0, y: 0 };
//     const rect = canvas.getBoundingClientRect();
    
//     const mouseX = clientX - rect.left;
//     const mouseY = clientY - rect.top;
    
//     if (rotation === 0) {
//       const rawX = (mouseX - zoomState.offsetX) / zoomState.scale;
//       const rawY = (mouseY - zoomState.offsetY) / zoomState.scale;
//       return { x: rawX, y: rawY };
//     }
    
//     const containerWidth = rect.width;
//     const containerHeight = rect.height;
//     const centerX = containerWidth / 2;
//     const centerY = containerHeight / 2;
    
//     const translatedMouseX = mouseX - centerX;
//     const translatedMouseY = mouseY - centerY;
    
//     const angle = -rotation * Math.PI / 180;
//     const rotatedMouseX = translatedMouseX * Math.cos(angle) - translatedMouseY * Math.sin(angle);
//     const rotatedMouseY = translatedMouseX * Math.sin(angle) + translatedMouseY * Math.cos(angle);
    
//     const unrotatedX = rotatedMouseX + centerX;
//     const unrotatedY = rotatedMouseY + centerY;
    
//     const rawX = (unrotatedX - zoomState.offsetX) / zoomState.scale;
//     const rawY = (unrotatedY - zoomState.offsetY) / zoomState.scale;
    
//     return { x: rawX, y: rawY };
//   };

//   const clientToRosCoords = (clientX, clientY) => {
//     const { x, y } = clientToCanvasCoords(clientX, clientY);
//     return canvasToRosCoords(x, y);
//   };

//   // --- Rotation function ---
//   const handleRotate = () => {
//     setRotation((prevRotation) => (prevRotation + 90) % 360);
//   };

//   const handleRotationInput = (e) => {
//     const value = parseInt(e.target.value);
//     if (!isNaN(value)) {
//       const normalizedRotation = ((value % 360) + 360) % 360;
//       setRotation(normalizedRotation);
//     }
//   };

//   // --- Editor helpers ---
//   const makeId = (prefix = "n") => `${prefix}_${idCounter.current++}`;

//   const placeNodeAtClient = (clientX, clientY) => {
//     if (!mapParamsRef.current) return;
//     const rosCoords = clientToRosCoords(clientX, clientY);
//     if (!rosCoords) return;
//     const canvasCoords = rosToCanvasCoords(rosCoords.x, rosCoords.y);

//     const newNode = {
//       id: makeId("node"),
//       type: nodeType,
//       label: `${nodeType}_${nodes.length + 1}`,
//       rosX: rosCoords.x,
//       rosY: rosCoords.y,
//       canvasX: canvasCoords.x,
//       canvasY: canvasCoords.y,
//     };
    
//     saveToHistory();
//     setNodes((prev) => [...prev, newNode]);
//   };

//   const findNodeAtCanvas = (cx, cy, radius = 10) => {
//     for (let i = nodes.length - 1; i >= 0; --i) {
//       const n = nodes[i];
//       const dx = n.canvasX - cx;
//       const dy = n.canvasY - cy;
//       if (Math.hypot(dx, dy) <= radius) return n;
//     }
//     return null;
//   };

//   // --- Arrow Drawing Functions ---
//   const handleConnectClick = (clientX, clientY) => {
//     const c = clientToCanvasCoords(clientX, clientY);
//     const node = findNodeAtCanvas(c.x, c.y, 8 / zoomState.scale);
    
//     if (!node) {
//       if (arrowDrawing.isDrawing) {
//         setArrowDrawing(prev => ({
//           ...prev,
//           points: [...prev.points, { x: c.x, y: c.y }]
//         }));
//       }
//       return;
//     }

//     if (!arrowDrawing.isDrawing) {
//       setArrowDrawing({
//         isDrawing: true,
//         fromId: node.id,
//         points: []
//       });
//     } else {
//       if (arrowDrawing.fromId === node.id) {
//         setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
//       } else {
//         saveToHistory();
        
//         const arrow = { 
//           id: makeId("arrow"), 
//           fromId: arrowDrawing.fromId, 
//           toId: node.id,
//           points: [...arrowDrawing.points]
//         };
//         setArrows((prev) => [...prev, arrow]);
//         setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
//       }
//     }
//   };

//   const cancelArrowDrawing = () => {
//     setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
//   };

//   const pointInPolygon = (point, vs) => {
//     if (!vs.length) return false;
//     const x = point[0], y = point[1];
//     let inside = false;
//     for (let i = 0, j = vs.length - 1; i < vs.length; j = i++) {
//       const xi = vs[i][0], yi = vs[i][1];
//       const xj = vs[j][0], yj = vs[j][1];
//       const intersect = ((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 0.0000001) + xi);
//       if (intersect) inside = !inside;
//     }
//     return inside;
//   };

//   const finishZone = () => {
//     if (!currentZonePoints.length) return;
    
//     saveToHistory();
    
//     const z = {
//       id: makeId("zone"),
//       name: zoneName || `Zone_${zones.length + 1}`,
//       type: zoneType,
//       points: currentZonePoints.slice(),
//     };
//     setZones((prev) => [...prev, z]);
//     setCurrentZonePoints([]);
//     setZoneName("");
//     setZoneType("normal");
//   };

//   // --- Mouse handlers ---
//   const handleMouseDown = (e) => {
//     const rect = canvasRef.current.getBoundingClientRect();
//     const x = e.clientX - rect.left;
//     const y = e.clientY - rect.top;
    
//     if (e.button === 1 || tool === "pan") {
//       setZoomState((prev) => ({ ...prev, isDragging: true, lastX: x, lastY: y }));
//       return;
//     }
    
//     if (tool === "place_node") {
//       placeNodeAtClient(e.clientX, e.clientY);
//       return;
//     }
//     if (tool === "connect") {
//       handleConnectClick(e.clientX, e.clientY);
//       return;
//     }
//     if (tool === "zone") {
//       if (!mapParamsRef.current) return;
//       const ros = clientToRosCoords(e.clientX, e.clientY);
//       if (!ros) return;
//       const canvas = rosToCanvasCoords(ros.x, ros.y);
//       setCurrentZonePoints((prev) => [...prev, { rosX: ros.x, rosY: ros.y, canvasX: canvas.x, canvasY: canvas.y }]);
//       return;
//     }
//     if (tool === "erase") {
//       if (eraseMode === "hand") {
//         startHandErase(e.clientX, e.clientY);
//       } else {
//         eraseAtClient(e.clientX, e.clientY);
//       }
//       return;
//     }
//     if (tool === "crop") {
//       const c = clientToCanvasCoords(e.clientX, e.clientY);
//       setCropState({
//         ...cropState,
//         startX: c.x,
//         startY: c.y,
//         endX: c.x,
//         endY: c.y,
//         isDragging: true,
//         freehandPoints: [{x: c.x, y: c.y}]
//       });
//       return;
//     }
//   };

//   const handleMouseMove = (e) => {
//     const rect = canvasRef.current.getBoundingClientRect();
//     const rectX = e.clientX - rect.left;
//     const rectY = e.clientY - rect.top;
//     const ros = clientToRosCoords(e.clientX, e.clientY);
    
//     if (ros) {
//       const canvas = rosToCanvasCoords(ros.x, ros.y);
//       setCursorCoords({ rosX: ros.x, rosY: ros.y, canvasX: canvas.x, canvasY: canvas.y });
//     } else {
//       setCursorCoords(null);
//     }

//     if (zoomState.isDragging) {
//       const x = e.clientX - rect.left;
//       const y = e.clientY - rect.top;
//       setZoomState((prev) => ({ 
//         ...prev, 
//         offsetX: prev.offsetX + (x - prev.lastX), 
//         offsetY: prev.offsetY + (y - prev.lastY), 
//         lastX: x, 
//         lastY: y 
//       }));
//     }

//     // Freehand crop drawing
//     if (cropState.isDragging && tool === "crop") {
//       const c = clientToCanvasCoords(e.clientX, e.clientY);
//       setCropState(prev => ({
//         ...prev,
//         endX: c.x,
//         endY: c.y,
//         freehandPoints: [...prev.freehandPoints, {x: c.x, y: c.y}]
//       }));
//     }

//     if (handEraseState.isErasing) {
//       continueHandErase(e.clientX, e.clientY);
//     }
//   };

//   const handleMouseUp = (e) => {
//     if (zoomState.isDragging) setZoomState((prev) => ({ ...prev, isDragging: false }));
    
//     if (cropState.isDragging && tool === "crop") {
//       const c = clientToCanvasCoords(e.clientX, e.clientY);
//       const completedPoints = [...cropState.freehandPoints, {x: c.x, y: c.y}];
      
//       setCropState(prev => ({
//         ...prev,
//         endX: c.x,
//         endY: c.y,
//         isDragging: false,
//         freehandPoints: completedPoints
//       }));
//       return;
//     }
    
//     if (handEraseState.isErasing) stopHandErase();
//   };

//   const handleWheel = (e) => {
//     if (!mapMsg) return;
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
    
//     setZoomState((prev) => ({ 
//       ...prev, 
//       scale: newScale,
//       offsetX: centerX - zoomPointX * newScale,
//       offsetY: centerY - zoomPointY * newScale
//     }));
//   };

//   const handleButtonZoom = (zoomFactor) => {
//     if (!mapMsg || !canvasRef.current) return;
    
//     const rect = canvasRef.current.getBoundingClientRect();
//     const containerWidth = rect.width;
//     const containerHeight = rect.height;
    
//     const centerX = containerWidth / 2;
//     const centerY = containerHeight / 2;
    
//     const newScale = Math.max(0.1, Math.min(5, zoomState.scale * zoomFactor));
    
//     const zoomPointX = (centerX - zoomState.offsetX) / zoomState.scale;
//     const zoomPointY = (centerY - zoomState.offsetY) / zoomState.scale;
    
//     setZoomState((prev) => ({ 
//       ...prev, 
//       scale: newScale,
//       offsetX: centerX - zoomPointX * newScale,
//       offsetY: centerY - zoomPointY * newScale
//     }));
//   };

//   // --- Save / Load / Clear ---
//   const handleSaveJSON = () => {
//     const payload = {
//       mapName,
//       nodes,
//       arrows,
//       zones,
//       mapParams: mapParamsRef.current,
//       timestamp: Date.now(),
//     };
//     const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
//     const url = URL.createObjectURL(blob);
//     const a = document.createElement("a");
//     a.href = url;
//     a.download = `${mapName || "map"}_editor.json`;
//     a.click();
//     URL.revokeObjectURL(url);
//   };

//   const handleUploadJSON = (file) => {
//     if (!file) return;
//     const reader = new FileReader();
//     reader.onload = (ev) => {
//       try {
//         const obj = JSON.parse(ev.target.result);
//         if (obj.nodes) setNodes(obj.nodes);
//         if (obj.arrows) setArrows(obj.arrows);
//         if (obj.zones) setZones(obj.zones);
//         if (obj.mapName) setMapName(obj.mapName);
//         if (!mapParamsRef.current && obj.mapParams) mapParamsRef.current = obj.mapParams;
//       } catch (err) {
//         console.error("Invalid JSON", err);
//         alert("Invalid JSON file");
//       }
//     };
//     reader.readAsText(file);
//   };

//   const handleClearAll = () => {
//     if (window.confirm("Are you sure you want to clear ALL nodes, arrows, and zones?")) {
//       saveToHistory();
//       setNodes([]);
//       setArrows([]);
//       setZones([]);
//       setCurrentZonePoints([]);
//       setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
//       setRotation(0);
//     }
//   };

//   // --- ROS connection ---
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

//       mapListener = new ROSLIB.Topic({
//         ros,
//         name: "/map",
//         messageType: "nav_msgs/OccupancyGrid",
//       });

//       mapListener.subscribe((msg) => {
//         const mapData = {
//           width: msg.info.width,
//           height: msg.info.height,
//           resolution: msg.info.resolution,
//           data: msg.data,
//           origin: msg.info.origin.position,
//         };

//         mapParamsRef.current = {
//           width: msg.info.width,
//           height: msg.info.height,
//           resolution: msg.info.resolution,
//           originX: msg.info.origin.position.x,
//           originY: msg.info.origin.position.y,
//         };

//         setMapMsg(mapData);
//         setEditableMap({ ...mapData, data: [...msg.data] });

//         if (canvasRef.current) {
//           const container = canvasRef.current.parentElement;
//           const offsetX = (container.clientWidth - msg.info.width) / 2;
//           const offsetY = (container.clientHeight - msg.info.height) / 2;
//           setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
//         }
//       });
//     });

//     ros.on("error", (err) => {
//       console.error("ROS error", err);
//       setRosConnected(false);
//     });
//     ros.on("close", () => {
//       setRosConnected(false);
//       clearTimeout(reconnectTimer);
//       reconnectTimer = setTimeout(connect, 2000);
//     });

//     connect();

//     return () => {
//       mapListener?.unsubscribe();
//       clearTimeout(reconnectTimer);
//       ros.close();
//     };
//   }, []);

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

//     const mapToRender = editableMap || mapMsg;
//     if (!mapToRender) return;

//     const { width, height, data } = mapToRender;

//     ctx.save();
    
//     if (rotation !== 0) {
//       const centerX = containerWidth / 2;
//       const centerY = containerHeight / 2;
//       ctx.translate(centerX, centerY);
//       ctx.rotate((rotation * Math.PI) / 180);
//       ctx.translate(-centerX, -centerY);
//     }

//     ctx.translate(zoomState.offsetX, zoomState.offsetY);
//     ctx.scale(zoomState.scale, zoomState.scale);

//     // Draw base map
//    // In the main useEffect for drawing:
// // Draw base map
// // Draw base map
// const imageData = ctx.createImageData(width, height);
// for (let y = 0; y < height; y++) {
//   for (let x = 0; x < width; x++) {
//     const val = data[y * width + x];
//     const py = height - 1 - y;
//     const i = (py * width + x) * 4;

// // In the imageData creation loop:
// let gray, alpha = 255;
// if (val === -3) {
//   // NEW: True transparency (for cropped areas)
//   gray = 255; // White
//   alpha = 0;  // Fully transparent
// } else if (val === -2) {
//   // Keep checkerboard for old deleted areas if you want
//   const checkerSize = 8;
//   const isCheckered = ((Math.floor(x / checkerSize) + Math.floor(y / checkerSize)) % 2) === 0;
//   gray = isCheckered ? 180 : 220;
//   alpha = 255;
// } else if (val === -1) gray = 205;
// else if (val === 0) gray = 255;
// else if (val === 100) gray = 0;
// else gray = 255 - Math.floor((val / 100) * 255);

// imageData.data[i] = gray;
// imageData.data[i + 1] = gray;
// imageData.data[i + 2] = gray;
// imageData.data[i + 3] = alpha; // This controls transparency
//   }
// }

//     const off = document.createElement("canvas");
//     off.width = width;
//     off.height = height;
//     off.getContext("2d").putImageData(imageData, 0, 0);
//     ctx.drawImage(off, 0, 0, width, height);

//     // Draw freehand crop selection
//     if (cropState.isCropping && cropState.freehandPoints.length > 0) {
//       ctx.strokeStyle = '#00ff00';
//       ctx.lineWidth = 2 / zoomState.scale;
//       ctx.setLineDash([5, 5]);
//       ctx.beginPath();
      
//       cropState.freehandPoints.forEach((point, idx) => {
//         if (idx === 0) ctx.moveTo(point.x, point.y);
//         else ctx.lineTo(point.x, point.y);
//       });
      
//       if (cropState.isDragging && cursorCoords) {
//         ctx.lineTo(cursorCoords.canvasX, cursorCoords.canvasY);
//       }
      
//       ctx.stroke();
//       ctx.setLineDash([]);

//       if (cropState.freehandPoints.length >= 3) {
//         ctx.fillStyle = 'rgba(0, 255, 0, 0.1)';
//         ctx.beginPath();
//         cropState.freehandPoints.forEach((point, idx) => {
//           if (idx === 0) ctx.moveTo(point.x, point.y);
//           else ctx.lineTo(point.x, point.y);
//         });
//         ctx.closePath();
//         ctx.fill();
//       }

//       if (cropState.freehandPoints.length > 0) {
//         const lastPoint = cropState.freehandPoints[cropState.freehandPoints.length - 1];
//         ctx.fillStyle = '#00ff00';
//         ctx.font = `${12 / zoomState.scale}px Arial`;
//         ctx.fillText(`${cropState.freehandPoints.length} points`, lastPoint.x + 5, lastPoint.y - 5);
//       }
//     }

//     // Draw zones
//     zones.forEach((z) => {
//       if (!z.points || z.points.length < 2) return;
//       ctx.beginPath();
//       z.points.forEach((p, idx) => {
//         if (idx === 0) ctx.moveTo(p.canvasX, p.canvasY);
//         else ctx.lineTo(p.canvasX, p.canvasY);
//       });
//       ctx.closePath();
      
//       if (z.type === 'keep_out') {
//         ctx.fillStyle = "rgba(0, 0, 0, 1.0)";
//         ctx.fill();
//         ctx.strokeStyle = "rgba(0, 0, 0, 1.0)";
//         ctx.lineWidth = 2 / zoomState.scale;
//         ctx.stroke();
//       } else {
//         ctx.fillStyle = "rgba(16,185,129,0.12)";
//         ctx.fill();
//         ctx.strokeStyle = "rgba(16,185,129,0.6)";
//         ctx.lineWidth = 1 / zoomState.scale;
//         ctx.stroke();
//       }

//       const cx = z.points.reduce((s, p) => s + p.canvasX, 0) / z.points.length;
//       const cy = z.points.reduce((s, p) => s + p.canvasY, 0) / z.points.length;
      
//       if (z.type === 'keep_out') {
//         ctx.fillStyle = "#dc2626";
//       } else {
//         ctx.fillStyle = "#064e3b";
//       }
//       ctx.font = `${12 / zoomState.scale}px Arial`;
//       ctx.fillText(z.name, cx + 6 / zoomState.scale, cy);
//     });

//     // Current zone points
//     if (currentZonePoints.length) {
//       ctx.beginPath();
//       currentZonePoints.forEach((p, idx) => {
//         if (idx === 0) ctx.moveTo(p.canvasX, p.canvasY);
//         else ctx.lineTo(p.canvasX, p.canvasY);
//       });
      
//       if (zoneType === 'keep_out') {
//         ctx.strokeStyle = "rgba(0, 0, 0, 0.9)";
//         ctx.lineWidth = 2 / zoomState.scale;
//       } else {
//         ctx.strokeStyle = "rgba(59,130,246,0.9)";
//         ctx.lineWidth = 1 / zoomState.scale;
//       }
//       ctx.stroke();
//     }

//     // Draw arrows
//     arrows.forEach((a) => {
//       const from = nodes.find((n) => n.id === a.fromId);
//       const to = nodes.find((n) => n.id === a.toId);
//       if (!from || !to) return;
      
//       const allPoints = [from, ...(a.points || []), to];
//       if (allPoints.length < 2) return;
      
//       ctx.strokeStyle = "#f97316";
//       ctx.lineWidth = 2 / zoomState.scale;
//       ctx.beginPath();
//       ctx.moveTo(allPoints[0].canvasX, allPoints[0].canvasY);
      
//       for (let i = 1; i < allPoints.length; i++) {
//         ctx.lineTo(allPoints[i].canvasX, allPoints[i].canvasY);
//       }
//       ctx.stroke();
      
//       const lastPoint = allPoints[allPoints.length - 1];
//       const secondLastPoint = allPoints[allPoints.length - 2];
//       const angle = Math.atan2(
//         lastPoint.canvasY - secondLastPoint.canvasY,
//         lastPoint.canvasX - secondLastPoint.canvasX
//       );
//       const headlen = 12 / zoomState.scale;
      
//       ctx.beginPath();
//       ctx.moveTo(lastPoint.canvasX, lastPoint.canvasY);
//       ctx.lineTo(
//         lastPoint.canvasX - headlen * Math.cos(angle - Math.PI / 6),
//         lastPoint.canvasY - headlen * Math.sin(angle - Math.PI / 6)
//       );
//       ctx.lineTo(
//         lastPoint.canvasX - headlen * Math.cos(angle + Math.PI / 6),
//         lastPoint.canvasY - headlen * Math.sin(angle + Math.PI / 6)
//       );
//       ctx.closePath();
//       ctx.fillStyle = "#f97316";
//       ctx.fill();
      
//       for (let i = 1; i < allPoints.length - 1; i++) {
//         const point = allPoints[i];
//         ctx.beginPath();
//         ctx.arc(point.canvasX, point.canvasY, 4 / zoomState.scale, 0, Math.PI * 2);
//         ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
//         ctx.fill();
//         ctx.strokeStyle = "#f97316";
//         ctx.lineWidth = 1 / zoomState.scale;
//         ctx.stroke();
//       }
//     });

//     // Draw the currently-being-drawn arrow
//     if (arrowDrawing.isDrawing) {
//       const from = nodes.find((n) => n.id === arrowDrawing.fromId);
//       if (from && cursorCoords) {
//         const allPoints = [from, ...arrowDrawing.points, cursorCoords];
//         if (allPoints.length >= 2) {
//           ctx.strokeStyle = "rgba(255, 165, 0, 0.8)";
//           ctx.lineWidth = 2 / zoomState.scale;
//           ctx.beginPath();
//           ctx.moveTo(allPoints[0].canvasX, allPoints[0].canvasY);
          
//           for (let i = 1; i < allPoints.length; i++) {
//             ctx.lineTo(allPoints[i].canvasX, allPoints[i].canvasY);
//           }
//           ctx.stroke();
          
//           for (let i = 1; i < allPoints.length - 1; i++) {
//             const point = allPoints[i];
//             ctx.beginPath();
//             ctx.arc(point.x, point.y, 4 / zoomState.scale, 0, Math.PI * 2);
//             ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
//             ctx.fill();
//             ctx.strokeStyle = "#f97316";
//             ctx.lineWidth = 1 / zoomState.scale;
//             ctx.stroke();
//           }
//         }
//       }
//     }

//     // Draw nodes
//     nodes.forEach((n) => {
//       ctx.beginPath();
//       ctx.fillStyle = currentTheme.nodeColors[n.type] || "#1e40af";
//       ctx.arc(n.canvasX, n.canvasY, Math.max(3, 6 / zoomState.scale), 0, Math.PI * 2);
//       ctx.fill();
//       ctx.fillStyle = "black";
//       ctx.font = `${Math.max(6, 10 / zoomState.scale)}px Arial`;
//       ctx.fillText(n.label || n.type, n.canvasX + 8 / zoomState.scale, n.canvasY - 6 / zoomState.scale);
//     });

//     // Hover crosshair
//     if (cursorCoords) {
//       ctx.strokeStyle = "rgba(0,0,0,0.4)";
//       ctx.lineWidth = 0.5 / zoomState.scale;
//       ctx.beginPath();
//       ctx.moveTo(cursorCoords.canvasX, 0);
//       ctx.lineTo(cursorCoords.canvasX, height);
//       ctx.moveTo(0, cursorCoords.canvasY);
//       ctx.lineTo(width, cursorCoords.canvasY);
//       ctx.stroke();
//     }

//     ctx.restore();
//   }, [mapMsg, editableMap, zoomState, nodes, arrows, zones, currentZonePoints, cursorCoords, currentTheme, rotation, arrowDrawing, cropState, zoneType, tool, eraseMode, eraseRadius]);

//   // Map Loader Modal
//   const MapLoaderModal = () => {
//     const handleZipSelect = async (e) => {
//       const file = e.target.files[0];
//       if (!file) return;

//       try {
//         const jsZip = new JSZip();
//         const zip = await jsZip.loadAsync(file);
        
//         const yamlFiles = Object.keys(zip.files).filter(name => 
//           name.toLowerCase().endsWith('.yaml') || name.toLowerCase().endsWith('.yml')
//         );
        
//         const pgmFiles = Object.keys(zip.files).filter(name => 
//           name.toLowerCase().endsWith('.pgm')
//         );

//         if (yamlFiles.length === 0) {
//           alert("No YAML files found in the ZIP archive");
//           return;
//         }

//         if (pgmFiles.length === 0) {
//           alert("No PGM files found in the ZIP archive");
//           return;
//         }

//         const yamlFileEntry = zip.files[yamlFiles[0]];
//         const yamlText = await yamlFileEntry.async('text');
//         const parsedYaml = yaml.load(yamlText);

//         const expectedPgmName = parsedYaml.image;
//         let pgmFileEntry;

//         if (expectedPgmName) {
//           const possibleNames = [
//             expectedPgmName,
//             `${expectedPgmName}.pgm`,
//             expectedPgmName.replace('.png', '.pgm').replace('.jpg', '.pgm').replace('.jpeg', '.pgm'),
//             ...pgmFiles.filter(name => name.toLowerCase().includes(expectedPgmName.toLowerCase()))
//           ];

//           for (const name of possibleNames) {
//             if (zip.files[name]) {
//               pgmFileEntry = zip.files[name];
//               break;
//             }
//           }
//         }

//         if (!pgmFileEntry && pgmFiles.length > 0) {
//           pgmFileEntry = zip.files[pgmFiles[0]];
//         }

//         if (!pgmFileEntry) {
//           alert("Could not find matching PGM file in the ZIP archive");
//           return;
//         }

//         const pgmArrayBuffer = await pgmFileEntry.async('arraybuffer');
        
//         const yamlFile = new File([yamlText], yamlFiles[0], { type: 'application/x-yaml' });
//         const pgmFile = new File([pgmArrayBuffer], pgmFileEntry.name, { type: 'image/x-portable-graymap' });

//         await loadMapWithFiles(yamlFile, pgmFile);

//       } catch (err) {
//         console.error("Error processing ZIP file:", err);
//         alert("Failed to process ZIP file. Check console for details.");
//       }
//     };

//     const loadMapWithFiles = async (yamlFile, pgmFile) => {
//       try {
//         const yamlText = await yamlFile.text();
//         const parsedYaml = yaml.load(yamlText);
//         setMapInfo(parsedYaml);

//         const buffer = await pgmFile.arrayBuffer();
//         const bytes = new Uint8Array(buffer);
//         const textHeader = new TextDecoder("ascii").decode(bytes.slice(0, 1000));

//         const headerLines = textHeader.split(/\s+/).filter((l) => l.length > 0);
//         const magic = headerLines[0];
//         if (magic !== "P5" && magic !== "P2") {
//           throw new Error("Unsupported PGM format: must be P5 or P2");
//         }

//         const width = parseInt(headerLines[1]);
//         const height = parseInt(headerLines[2]);
//         const maxVal = parseInt(headerLines[3]);

//         const headerLength = textHeader.indexOf(maxVal.toString()) + maxVal.toString().length;
//         const dataStart = textHeader.slice(0, headerLength).length + 1;
//         const pixelBytes = bytes.slice(dataStart);

//         const occupancyData = new Int8Array(width * height);
//         const negate = parsedYaml.negate || 0;

//         for (let y = 0; y < height; y++) {
//           for (let x = 0; x < width; x++) {
//             const i = y * width + x;
//             let value = pixelBytes[i];
            
//             if (negate === 1) {
//               value = 255 - value;
//             }
            
//             let occupancyValue = -1;
//             if (value >= 0 && value <= 10) {
//               occupancyValue = 100;
//             } else if (value >= 250 && value <= 255) {
//               occupancyValue = 0;
//             }
            
//             occupancyData[(height - 1 - y) * width + x] = occupancyValue;
//           }
//         }

//         const loadedMap = {
//           width: width,
//           height: height,
//           resolution: parsedYaml.resolution || 0.05,
//           data: occupancyData,
//           origin: {
//             x: parsedYaml.origin?.[0] || 0,
//             y: parsedYaml.origin?.[1] || 0,
//             z: parsedYaml.origin?.[2] || 0
//           }
//         };

//         mapParamsRef.current = {
//           width: width,
//           height: height,
//           resolution: parsedYaml.resolution || 0.05,
//           originX: parsedYaml.origin?.[0] || 0,
//           originY: parsedYaml.origin?.[1] || 0,
//         };

//         setMapMsg(loadedMap);
//         setEditableMap({ ...loadedMap, data: [...occupancyData] });
//         setMapLoaderActive(false);

//         if (canvasRef.current) {
//           const container = canvasRef.current.parentElement;
//           const offsetX = (container.clientWidth - width) / 2;
//           const offsetY = (container.clientHeight - height) / 2;
//           setZoomState((prev) => ({ ...prev, offsetX: Math.max(0, offsetX), offsetY: Math.max(0, offsetY), scale: 1 }));
//         }

//         const mapNameFromFile = yamlFile.name.replace('.yaml', '').replace('.yml', '');
//         if (mapNameFromFile && !mapName) {
//           setMapName(mapNameFromFile);
//         }

//         clearHistory();

//         alert(`✅ Map loaded successfully!\n\n📦 Source: ZIP archive\n📊 Size: ${width} x ${height} pixels\n📍 Resolution: ${parsedYaml.resolution} m/pixel`);

//       } catch (err) {
//         console.error("Error loading map:", err);
//         alert("Failed to load map. Check console for details.");
//       }
//     };

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
//           background: currentTheme.card,
//           padding: 24,
//           borderRadius: 12,
//           border: `1px solid ${currentTheme.border}`,
//           maxWidth: 500,
//           width: '90%',
//           boxShadow: '0 10px 50px rgba(0,0,0,0.3)'
//         }}>
//           <h3 style={{ margin: '0 0 20px 0', color: currentTheme.text }}>
//             🗺️ Load Map from ZIP
//           </h3>

//           <div style={{ marginBottom: 20 }}>
//             <label style={styles.label}>
//               Select ZIP File:
//               <div style={{ 
//                 marginTop: 6,
//                 border: `1px solid ${currentTheme.border}`,
//                 borderRadius: 8,
//                 padding: '10px 12px',
//                 background: currentTheme.card,
//                 color: currentTheme.text,
//                 fontSize: '14px',
//                 cursor: 'pointer',
//                 position: 'relative',
//                 overflow: 'hidden'
//               }}>
//                 Choose ZIP file containing YAML and PGM...
//                 <input 
//                   type="file" 
//                   accept=".zip" 
//                   onChange={handleZipSelect} 
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
//             <div style={styles.hint}>
//               Select a ZIP file containing both YAML and PGM map files. The map will load automatically.
//             </div>
//           </div>

//           {mapInfo && (
//             <div style={{
//               marginBottom: 15,
//               padding: 10,
//               background: currentTheme.surface,
//               borderRadius: 8,
//               border: `1px solid ${currentTheme.border}`,
//             }}>
//               <h4 style={{ margin: '0 0 8px 0', color: currentTheme.text }}>Map Info:</h4>
//               <pre style={{ margin: 0, fontSize: "12px", color: currentTheme.textSecondary }}>
//                 {JSON.stringify(mapInfo, null, 2)}
//               </pre>
//               {mapInfo.image && (
//                 <div style={{ marginTop: 8, fontSize: "12px", color: currentTheme.accent }}>
//                   📄 PGM File: {mapInfo.image}
//                 </div>
//               )}
//             </div>
//           )}

//           <div style={{ display: 'flex', gap: 10, justifyContent: 'flex-end' }}>
//             <button 
//               onClick={() => {
//                 setMapLoaderActive(false);
//                 setMapInfo(null);
//               }}
//               style={styles.button}
//             >
//               Cancel
//             </button>
//           </div>
//         </div>
//       </div>
//     );
//   };

//   return (
//     <div style={styles.container}>
//       {mapLoaderActive && <MapLoaderModal />}

//       <div style={styles.sidebar}>
//         <div style={{ marginBottom: 20 }}>
//           <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 8 }}>
//             <h3 style={{ margin: 0, color: currentTheme.text, display: 'flex', alignItems: 'center', gap: 8 }}>
//               <FaMapMarkerAlt style={{ color: currentTheme.accent }} />
//               Map Editor {opencvLoaded ? "✓" : "⏳"}
//             </h3>
//             <button 
//               onClick={toggleDarkMode}
//               style={{
//                 ...styles.smallButton,
//                 background: 'transparent',
//                 color: currentTheme.text,
//                 border: `1px solid ${currentTheme.border}`
//               }}
//               title={darkMode ? "Switch to Light Mode" : "Switch to Dark Mode"}
//             >
//               {darkMode ? <FaSun size={14} /> : <FaMoon size={14} />}
//             </button>
//           </div>
//           <div style={{ fontSize: 12, color: currentTheme.textSecondary }}>
//             {darkMode ? 'Dark' : 'Light'} mode • {opencvLoaded ? 'OpenCV Ready' : 'Loading OpenCV...'}
//           </div>
//         </div>

//         <label style={styles.label}>Map name</label>
//         <input 
//           value={mapName} 
//           onChange={(e) => setMapName(e.target.value)} 
//           placeholder="My Map" 
//           style={styles.input} 
//         />

//         {/* Undo/Redo Controls */}
//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Undo/Redo</label>
//           <div style={{ display: "flex", gap: 8, marginBottom: 16 }}>
//             <button 
//               onClick={handleUndo} 
//               style={{
//                 ...styles.buttonUndoRedo,
//                 opacity: historyIndex > 0 ? 1 : 0.5,
//                 cursor: historyIndex > 0 ? "pointer" : "not-allowed"
//               }}
//               disabled={historyIndex <= 0}
//               title="Undo last action"
//             >
//               <FaUndo />
//               Undo
//             </button>
//             <button 
//               onClick={handleRedo} 
//               style={{
//                 ...styles.buttonUndoRedo,
//                 opacity: historyIndex < history.length - 1 ? 1 : 0.5,
//                 cursor: historyIndex < history.length - 1 ? "pointer" : "not-allowed"
//               }}
//               disabled={historyIndex >= history.length - 1}
//               title="Redo last undone action"
//             >
//               <FaRedoAlt />
//               Redo
//             </button>
//           </div>
//           <div style={styles.hint}>
//             History: {historyIndex + 1}/{history.length} steps
//           </div>
//         </div>

//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Tools</label>
//           <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8, marginBottom: 16 }}>
//             <button 
//               onClick={() => setTool("place_node")} 
//               style={tool === "place_node" ? styles.buttonActive : styles.button}
//             >
//               <FaMapMarkerAlt />
//               Node
//             </button>
//             <button 
//               onClick={() => setTool("connect")} 
//               style={tool === "connect" ? styles.buttonActive : styles.button}
//             >
//               <FaArrowRight />
//               Arrow
//             </button>
//             <button 
//               onClick={() => setTool("zone")} 
//               style={tool === "zone" ? styles.buttonActive : styles.button}
//             >
//               <FaDrawPolygon />
//               Zone
//             </button>
//             <button 
//               onClick={() => setTool("erase")} 
//               style={tool === "erase" ? styles.buttonActive : styles.button}
//             >
//               <FaEraser />
//               Erase
//             </button>
//             <button 
//               onClick={startCrop} 
//               style={tool === "crop" ? styles.buttonActive : styles.button}
//             >
//               <FaCropAlt />
//               Crop
//             </button>
//             <button 
//               onClick={() => {
//                 setTool("zone");
//                 setZoneType("keep_out");
//               }} 
//               style={zoneType === "keep_out" ? styles.buttonKeepOut : styles.button}
//             >
//               <FaBan />
//               Restricted Zone
//             </button>
//           </div>
//         </div>

//         {/* Erase Mode Controls */}
//         {tool === "erase" && (
//           <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
//             <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
//               🧹 Erase Mode: {eraseMode === "objects" ? "Objects" : eraseMode === "noise" ? "Map Noise (Click)" : "Hand Erase (Draw)"}
//             </div>
            
//             <div style={{ marginBottom: 12 }}>
//               <label style={styles.label}>Erase Mode</label>
//               <select 
//                 value={eraseMode} 
//                 onChange={(e) => setEraseMode(e.target.value)} 
//                 style={styles.select}
//               >
//                 <option value="objects">🗑️ Erase Objects (Nodes, Zones)</option>
//                 <option value="noise">🧹 Erase Map Noise (Click)</option>
//                 <option value="hand">✋ Hand Erase (Draw Freely)</option>
//               </select>
//             </div>

//             {(eraseMode === "noise" || eraseMode === "hand") && (
//               <div style={{ marginBottom: 12 }}>
//                 <label style={styles.label}>Erase Radius: {eraseRadius}px</label>
//                 <input 
//                   type="range"
//                   min="1"
//                   max="20"
//                   value={eraseRadius}
//                   onChange={(e) => setEraseRadius(parseInt(e.target.value))}
//                   style={{ width: '100%', marginTop: 6 }}
//                 />
//                 <div style={styles.hint}>
//                   {eraseMode === "hand" 
//                     ? "Draw freely to erase black noise areas"
//                     : "Larger radius for cleaning large noise areas"}
//                 </div>
//               </div>
//             )}

//             <div style={styles.hint}>
//               {eraseMode === "objects" 
//                 ? "Click on nodes or zones to delete them"
//                 : eraseMode === "noise"
//                 ? `Click on black noise areas to erase them (${eraseRadius}px radius)`
//                 : `Click and drag to draw erase paths (${eraseRadius}px radius)`}
//             </div>
//           </div>
//         )}

//         {/* Crop Controls */}
//         {cropState.isCropping && (
//           <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
            
//             <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
//               ✂️ Crop Mode Active
//             </div>

//             <div style={styles.hint}>
//               Draw a freehand selection around the area you want to KEEP.
//             </div>

//             <div style={{ display: "flex", gap: 8, marginTop: 8, flexWrap: "wrap" }}>
              
//               {/* Apply Crop (visual checkerboard) */}
//               <button
//                 onClick={handleApplyCrop}
//                 style={{
//                   ...styles.buttonAction,
//                   background: '#10b981',
//                   border: '1px solid #059669'
//                 }}
//                 disabled={!cropState.freehandPoints || cropState.freehandPoints.length < 3}
//                 title="Apply crop to current map (checkerboard for deleted areas)"
//               >
//                 <FaCropAlt />
//                 Apply Crop
//               </button>

           
//               {/* Cancel */}
//               <button 
//                 onClick={cancelCrop}
//                 style={styles.buttonDanger}
//               >
//                 <FaBan />
//                 Cancel
//               </button>
//             </div>

//             {cropState.freehandPoints && (
//               <div style={{ marginTop: 8, fontSize: '12px', color: currentTheme.textSecondary }}>
//                 Points: {cropState.freehandPoints.length} (need 3+)
//               </div>
//             )}

//             <div style={{ marginTop: 12, padding: 8, background: currentTheme.background, borderRadius: 4 }}>
//               <div style={{ fontSize: '11px', color: currentTheme.textSecondary, lineHeight: 1.4 }}>
//                 <strong>Apply Crop:</strong> Marks deleted areas as checkerboard in current map.<br/>
//                 <strong>Export True Crop:</strong> Creates new map with deleted areas completely removed (reduced map size).
//               </div>
//             </div>
//           </div>
//         )}

//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Node type</label>
//           <select 
//             value={nodeType} 
//             onChange={(e) => setNodeType(e.target.value)} 
//             style={styles.select}
//           >
//             <option value="station">🏭 Station</option>
//             <option value="docking">📍 Docking Point</option>
//             <option value="waypoint">🚩 Waypoint</option>
//             <option value="home">🏠 Home Position</option>
//             <option value="charging">🔋 Charging Station</option>
//           </select>
//           <div style={styles.hint}>
//             Click on map to place {nodeType} node
//           </div>
//         </div>

//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Zone name</label>
//           <input 
//             value={zoneName} 
//             onChange={(e) => setZoneName(e.target.value)} 
//             placeholder="Zone A" 
//             style={styles.input} 
//           />
//           <div style={styles.hint}>
//             Click to add polygon points, then finish zone
//           </div>
//           <div style={{ display: "flex", gap: 8, marginTop: 12 }}>
//             <button onClick={finishZone} style={styles.button}>
//               <FaDrawPolygon />
//               Finish Zone
//             </button>
//             <button onClick={() => setCurrentZonePoints([])} style={styles.button}>
//               <FaTrashAlt />
//               Cancel Zone
//             </button>
//           </div>
//         </div>

//         {/* Arrow Drawing Controls */}
//         {arrowDrawing.isDrawing && (
//           <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
//             <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
//               Drawing Arrow...
//             </div>
//             <div style={styles.hint}>
//               Click on empty space to add bend points
//             </div>
//             <button 
//               onClick={cancelArrowDrawing}
//               style={{...styles.buttonDanger, marginTop: 8 }}
//             >
//               <FaTrashAlt />
//               Cancel Arrow
//             </button>
//           </div>
//         )}

//         {/* Rotation Control Section */}
//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Map Rotation</label>
//           <div style={{ display: 'flex', gap: 8, alignItems: 'center' }}>
//             <input 
//               type="number"
//               min="0"
//               max="359"
//               value={rotation}
//               onChange={handleRotationInput}
//               style={{
//                 ...styles.input,
//                 margin: 0,
//                 flex: 1,
//                 textAlign: 'center'
//               }}
//               placeholder="Enter degrees (0-359)"
//             />
//             <span style={{ 
//               fontSize: '14px', 
//               color: currentTheme.textSecondary,
//               minWidth: '30px'
//             }}>
//               °
//             </span>
//           </div>
//           <div style={styles.hint}>
//             Enter rotation angle (0-359 degrees)
//           </div>
//           <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
//             <button onClick={handleRotate} style={styles.button}>
//               <FaRedo />
//               +90°
//             </button>
//             <button 
//               onClick={() => setRotation(0)} 
//               style={styles.button}
//             >
//               <FaExpand />
//               Reset
//             </button>
//           </div>
//         </div>

//         <div style={{ marginBottom: 16 }}>
//           <label style={styles.label}>Map Operations</label>
//           <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
//             <button 
//               onClick={() => setMapLoaderActive(true)} 
//               style={styles.buttonAction}
//             >
//               <FaUpload />
//               Load Map
//             </button>
//             <button onClick={saveNodesToDatabase} style={styles.buttonAction}>
//               <FaSave />
//               Save to DB
//             </button>
//             <button onClick={loadNodesFromDatabase} style={styles.buttonAction}>
//               <FaUpload />
//               Load from DB
//             </button>
//             <button 
//               onClick={saveMapToComputer} 
//               style={styles.buttonAction}
//               disabled={!opencvLoaded}
//             >
//               <FaDownload />
//               {opencvLoaded ? "Save Full Map" : "Loading OpenCV..."}
//             </button>
//             <button 
//               onClick={saveMapAsPNG} 
//               style={styles.buttonAction}
//             >
//               <FaImage />
//               Save Full PNG
//             </button>
//           </div>
//         </div>

//         <div style={{ display: "flex", gap: 8, flexWrap: "wrap", marginBottom: 16 }}>
//           <button onClick={handleSaveJSON} style={styles.button}>
//             <FaSave />
//             Save JSON
//           </button>
//           <label style={{...styles.button, cursor: 'pointer', textAlign: 'center'}}>
//             <FaUpload />
//             Upload JSON
//             <input type="file" accept="application/json" style={{ display: "none" }} onChange={(e) => handleUploadJSON(e.target.files?.[0])} />
//           </label>
//           <button onClick={handleClearAll} style={styles.buttonDanger}>
//             <FaTrashAlt />
//             Clear All
//           </button>
//         </div>
//       </div>

//       <div style={{ flex: 1 }}>
//         <div style={{ marginBottom: 12, display: "flex", justifyContent: "space-between", alignItems: "center" }}>
//           <div style={styles.hud}>
//             {cursorCoords ? `ROS: ${cursorCoords.rosX.toFixed(2)}, ${cursorCoords.rosY.toFixed(2)}` : "Hover map for coordinates"}
//           </div>
//           <div style={{ display: "flex", gap: 8, alignItems: 'center' }}>
//             <div style={styles.status}>
//               <div style={{ 
//                 width: 8, 
//                 height: 8, 
//                 borderRadius: '50%', 
//                 background: 'currentColor',
//                 opacity: rosConnected ? 1 : 0.5
//               }} />
//               {rosConnected ? "ROS Connected" : "ROS Disconnected"}
//             </div>
//             <div style={styles.mapInfo}>
//               Map: {mapName || "—"}
//             </div>
//             <div style={styles.mapInfo}>
//               Rotation: {rotation}°
//             </div>
//             <div style={styles.mapInfo}>
//               OpenCV: {opencvLoaded ? "✓" : "⏳"}
//             </div>
//             {tool === "erase" && (
//               <div style={styles.mapInfo}>
//                 Erase: {eraseMode === "objects" ? "Objects" : eraseMode === "noise" ? `Noise (${eraseRadius}px)` : `Hand (${eraseRadius}px)`}
//               </div>
//             )}
//             {tool === "crop" && (
//               <div style={styles.mapInfo}>
//                 Crop: {cropState.freehandPoints?.length || 0} points
//               </div>
//             )}
//             <div style={styles.mapInfo}>
//               History: {historyIndex + 1}/{history.length}
//             </div>
//           </div>
//         </div>

//         <div style={styles.canvasContainer}>
//           <div style={{
//             position: "absolute",
//             top: 12,
//             right: 12,
//             display: "flex",
//             flexDirection: "column",
//             gap: 8,
//             zIndex: 20,
//             background: currentTheme.card,
//             padding: 22,
//             borderRadius: 12,
//             boxShadow: "0 6px 18px rgba(0,0,0,0.15)",
//             border: `1px solid ${currentTheme.border}`
//           }}>
//             <div style={{ 
//               textAlign: "center", 
//               fontSize: 13, 
//               fontWeight: '700', 
//               color: currentTheme.text,
//               marginBottom: 4 
//             }}>
//               {Math.round(zoomState.scale * 100)}%
//             </div>
//             <button 
//               onClick={() => handleButtonZoom(1.2)}
//               style={styles.smallButton}
//               title="Zoom In"
//             >
//               <FaSearchPlus />
//             </button>
//             <button 
//               onClick={() => handleButtonZoom(0.8)}
//               style={styles.smallButton}
//               title="Zoom Out"
//             >
//               <FaSearchMinus />
//             </button>
//             {/* <button 
//               onClick={handleRotate}
//               style={styles.smallButton}
//               title="Rotate 90°"
//             >
//               <FaRedo />
//             </button> */}
//             <button 
//               onClick={() => {
//                 if (!mapMsg || !canvasRef.current) return;
//                 const container = canvasRef.current.parentElement;
//                 const offsetX = (container.clientWidth - mapMsg.width) / 2;
//                 const offsetY = (container.clientHeight - mapMsg.height) / 2;
//                 setZoomState((p) => ({ ...p, scale: 1, offsetX, offsetY }));
//                 setRotation(0);
//               }} 
//               style={styles.smallButton}
//               title="Reset View"
//             >
//               <FaExpand />
//             </button>
//             <button 
//               onClick={handleUndo}
//               style={{
//                 ...styles.smallButton,
//                 opacity: historyIndex > 0 ? 1 : 0.5,
//                 cursor: historyIndex > 0 ? "pointer" : "not-allowed",
//                 background: 'transparent',
//                 color: currentTheme.text,
//                 border: `1px solid ${currentTheme.border}`
//               }}
//               disabled={historyIndex <= 0}
//               title="Undo (Ctrl+Z)"
//             >
//               <FaUndo />
//             </button>
//             <button 
//               onClick={handleRedo}
//               style={{
//                 ...styles.smallButton,
//                 opacity: historyIndex < history.length - 1 ? 1 : 0.5,
//                 cursor: historyIndex < history.length - 1 ? "pointer" : "not-allowed",
//                 background: 'transparent',
//                 color: currentTheme.text,
//                 border: `1px solid ${currentTheme.border}`
//               }}
//               disabled={historyIndex >= history.length - 1}
//               title="Redo (Ctrl+Y)"
//             >
//               <FaRedoAlt />
//             </button>
//           </div>

//           <canvas
//             ref={canvasRef}
//             style={{ 
//               width: "100%", 
//               height: "100%", 
//               display: "block", 
//               cursor: cropState.isCropping ? "crosshair" : 
//                       zoomState.isDragging ? "grabbing" : 
//                       tool === "erase" && eraseMode === "hand" ? "crosshair" : "crosshair" 
//             }}
//             onMouseDown={handleMouseDown}
//             onMouseMove={handleMouseMove}
//             onMouseUp={handleMouseUp}
//             onMouseLeave={() => { 
//               setCursorCoords(null); 
//               setZoomState((p)=> ({...p, isDragging:false})); 
//               setCropState(prev => ({...prev, isDragging: false}));
//               if (handEraseState.isErasing) stopHandErase();
//             }}
//             onWheel={handleWheel}
//           />
//         </div>
//       </div>
//     </div>
//   );
// }


import React, { useEffect, useRef, useState } from "react";
import { useLanguage } from "../context/LanguageContext";
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
  FaHandPaper, // ADD THIS LINE
  FaBrush,
  FaCut,
  FaUndo,
  FaRedoAlt
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
    const { t } = useLanguage();
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

  // --- Crop state ---
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

  // --- Undo/Redo State ---
  const [history, setHistory] = useState(() => {
    const saved = localStorage.getItem("mapEditorHistory");
    return saved ? JSON.parse(saved) : [];
  });
  const [historyIndex, setHistoryIndex] = useState(() => {
    const saved = localStorage.getItem("mapEditorHistoryIndex");
    return saved ? parseInt(saved) : -1;
  });

  // --- Save current state to history ---
  const saveToHistory = () => {
    const currentState = {
      nodes: JSON.parse(JSON.stringify(nodes)),
      arrows: JSON.parse(JSON.stringify(arrows)),
      zones: JSON.parse(JSON.stringify(zones)),
      editableMap: editableMap ? {
        ...editableMap,
        data: [...editableMap.data]
      } : null,
      rotation: rotation,
      timestamp: Date.now()
    };

    const newHistory = history.slice(0, historyIndex + 1);
    newHistory.push(currentState);
    
    if (newHistory.length > 50) {
      newHistory.shift();
    }
    
    setHistory(newHistory);
    setHistoryIndex(newHistory.length - 1);
    
    localStorage.setItem("mapEditorHistory", JSON.stringify(newHistory));
    localStorage.setItem("mapEditorHistoryIndex", (newHistory.length - 1).toString());
  };

  // --- Undo function ---
  const handleUndo = () => {
    if (historyIndex > 0) {
      const newIndex = historyIndex - 1;
      const state = history[newIndex];
      
      setNodes(state.nodes);
      setArrows(state.arrows);
      setZones(state.zones);
      setRotation(state.rotation);
      
      if (state.editableMap) {
        setEditableMap(state.editableMap);
        setMapMsg(prev => prev ? { ...prev, data: state.editableMap.data } : null);
      }
      
      setHistoryIndex(newIndex);
      localStorage.setItem("mapEditorHistoryIndex", newIndex.toString());
    }
  };

  // --- Redo function ---
  const handleRedo = () => {
    if (historyIndex < history.length - 1) {
      const newIndex = historyIndex + 1;
      const state = history[newIndex];
      
      setNodes(state.nodes);
      setArrows(state.arrows);
      setZones(state.zones);
      setRotation(state.rotation);
      
      if (state.editableMap) {
        setEditableMap(state.editableMap);
        setMapMsg(prev => prev ? { ...prev, data: state.editableMap.data } : null);
      }
      
      setHistoryIndex(newIndex);
      localStorage.setItem("mapEditorHistoryIndex", newIndex.toString());
    }
  };

  // --- Clear history when map is loaded ---
  const clearHistory = () => {
    setHistory([]);
    setHistoryIndex(-1);
    localStorage.removeItem("mapEditorHistory");
    localStorage.removeItem("mapEditorHistoryIndex");
  };

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

  // --- System dark mode detection ---
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
      background: '#000000',
      color: '#ffffff',
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
    buttonUndoRedo: {
      padding: "8px 12px",
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
      gap: 6,
      justifyContent: 'center',
      minWidth: '44px'
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

  // --- Helper Functions ---

  // Point in polygon algorithm
  const isPointInPolygon = (point, polygon) => {
    const x = point.x, y = point.y;
    
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

      // Clear history when new map is loaded
      clearHistory();

    } catch (err) {
      console.error("Error loading map:", err);
      alert("Failed to load map. Check console for details.");
    }
  };

  // --- CROP FUNCTIONS ---
  const startCrop = () => {
    setCropState({
      isCropping: true,
      startX: 0,
      startY: 0,
      endX: 0,
      endY: 0,
      isDragging: false,
      freehandPoints: []
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
// Apply crop to current map (visual checkerboard for SELECTED area)
// Apply crop to current map (KEEP the SELECTED area, DELETE everything else)
// Apply crop to current map (DELETE everything OUTSIDE selected area with transparency)
const handleApplyCrop = () => {
  console.log("✂️ Applying crop to current map - KEEP selected area, DELETE everything else WITH TRANSPARENCY");

  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded!");
    return;
  }

  if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
    alert("Draw a closed freehand shape first!");
    return;
  }

  // Save state before cropping for undo
  saveToHistory();

  const { width, height } = mapParamsRef.current;

  // Convert freehand (canvas coords) → map coords (flip Y axis)
  const mapPolygon = cropState.freehandPoints.map(p => ({
    x: p.x,
    y: height - 1 - p.y
  }));

  const newData = new Int8Array([...mapMsg.data]);
  let keptPixels = 0;
  let deletedPixels = 0;

  // DELETE everything OUTSIDE the selected area with TRUE TRANSPARENCY
  for (let y = 0; y < height; y++) {
    for (let x = 0; x < width; x++) {
      const idx = y * width + x;
      
      if (isPointInPolygon({ x, y }, mapPolygon)) {
        // INSIDE polygon - KEEP original value
        // But make sure it's not already transparent
        if (newData[idx] !== -2 && newData[idx] !== -3) {
          keptPixels++;
        }
      } else {
        // OUTSIDE polygon - MAKE IT TRANSPARENT (REAL TRANSPARENCY)
        // Use -3 for true transparency (not checkerboard)
        newData[idx] = -3; // This will be transparent in PNG export
        deletedPixels++;
      }
    }
  }

  console.log(`✅ Applied crop with transparency: Kept ${keptPixels} pixels INSIDE selection, made ${deletedPixels} pixels TRANSPARENT`);

  // Update map
  setMapMsg(prev => prev ? { ...prev, data: newData } : null);
  setEditableMap(prev => prev ? { ...prev, data: [...newData] } : null);

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

  alert(`✅ Crop applied with TRANSPARENCY!\n\n✅ Kept ${keptPixels} pixels INSIDE selection\n🌫️ Made ${deletedPixels} pixels TRANSPARENT\n📸 Will be transparent in PNG export`);
};

// Export TRULY cropped map (KEEP only INSIDE selected area)
const handleExportCroppedMap = async () => {
  console.log("📐 Exporting truly cropped map - KEEP only INSIDE selected area");

  const SCALE_FACTOR = 4;

  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded!");
    return;
  }

  if (!cropState.freehandPoints || cropState.freehandPoints.length < 3) {
    alert("Invalid crop selection! Please draw at least 3 points.");
    return;
  }

  try {
    const freehandPoints = cropState.freehandPoints;
    const { width, height, resolution, originX, originY, rotation = 0 } = mapParamsRef.current;

    // Convert freehand points from canvas to map coordinates
    const mapPolygon = freehandPoints.map(p => ({
      x: p.x,
      y: height - 1 - p.y  // flip Y axis
    }));

    // Find bounds of the SELECTED area only
    let minX = width, minY = height, maxX = 0, maxY = 0;
    let hasContent = false;

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        if (isPointInPolygon({ x, y }, mapPolygon)) { // INSIDE selection
          const srcIndex = y * width + x;
          // Only consider non-deleted, non-empty pixels
          if (mapMsg.data[srcIndex] !== -2 && mapMsg.data[srcIndex] !== 0) {
            minX = Math.min(minX, x);
            minY = Math.min(minY, y);
            maxX = Math.max(maxX, x);
            maxY = Math.max(maxY, y);
            hasContent = true;
          }
        }
      }
    }

    if (!hasContent) {
      alert("No valid map content found INSIDE the selected area!");
      return;
    }

    // Add padding
    const padding = 2;
    minX = Math.max(0, minX - padding);
    minY = Math.max(0, minY - padding);
    maxX = Math.min(width - 1, maxX + padding);
    maxY = Math.min(height - 1, maxY + padding);

    const contentWidth = maxX - minX + 1;
    const contentHeight = maxY - minY + 1;

    console.log(`📐 True crop bounds: ${minX},${minY} to ${maxX},${maxY} (${contentWidth}x${contentHeight})`);

    // Create new map data with ONLY content from INSIDE selected area
    const contentData = new Int8Array(contentWidth * contentHeight);
    let keptPixels = 0;

    for (let y = 0; y < contentHeight; y++) {
      for (let x = 0; x < contentWidth; x++) {
        const srcX = minX + x;
        const srcY = minY + y;
        const srcIndex = srcY * width + srcX;
        const dstIndex = y * contentWidth + x;
        
        if (isPointInPolygon({ x: srcX, y: srcY }, mapPolygon)) {
          // INSIDE polygon - keep if not deleted
          if (mapMsg.data[srcIndex] !== -2) {
            contentData[dstIndex] = mapMsg.data[srcIndex];
            if (mapMsg.data[srcIndex] !== 0 && mapMsg.data[srcIndex] !== -1) {
              keptPixels++;
            }
          } else {
            // Deleted area inside polygon - make it white
            contentData[dstIndex] = 0;
          }
        } else {
          // OUTSIDE polygon - make it white (transparent)
          contentData[dstIndex] = 0;
        }
      }
    }

    console.log(`✅ True crop: ${keptPixels} non-white pixels kept (inside selected area)`);

    // Calculate new origin for the truly cropped map
    const newOriginX = originX + minX * resolution;
    const newOriginY = originY + (height - (minY + contentHeight)) * resolution;

    // Create high-res canvas
    const scaledW = contentWidth * SCALE_FACTOR;
    const scaledH = contentHeight * SCALE_FACTOR;
    const canvas = document.createElement("canvas");
    canvas.width = scaledW;
    canvas.height = scaledH;
    const ctx = canvas.getContext("2d");
    ctx.scale(SCALE_FACTOR, SCALE_FACTOR);

    // Fill background with transparent
    ctx.clearRect(0, 0, contentWidth, contentHeight);

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
        
        let gray, alpha;
        if (val === 100) {
          gray = 0;   // Black
          alpha = 255;
        } else if (val === -1) {
          gray = 128; // Gray for unknown
          alpha = 255;
        } else if (val === 0) {
          // White areas should be transparent if they're outside selection
          const srcX = minX + x;
          const srcY = minY + y;
          if (isPointInPolygon({ x: srcX, y: srcY }, mapPolygon)) {
            // Inside selection - white but opaque
            gray = 255;
            alpha = 255;
          } else {
            // Outside selection - transparent
            gray = 255;
            alpha = 0;
          }
        } else {
          gray = 255;
          alpha = 255;
        }
        
        imgData.data[i] = gray;
        imgData.data[i + 1] = gray;
        imgData.data[i + 2] = gray;
        imgData.data[i + 3] = alpha;
      }
    }

    const offCanvas = document.createElement("canvas");
    offCanvas.width = contentWidth;
    offCanvas.height = contentHeight;
    offCanvas.getContext("2d").putImageData(imgData, 0, 0);
    ctx.drawImage(offCanvas, 0, 0);
    ctx.restore();

    // Save files
    const fileBase = `${mapName || "true_cropped"}_${contentWidth}x${contentHeight}_hires${SCALE_FACTOR}x_rot${rotation}`;
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
# True cropped map - KEEPING ONLY SELECTED AREA
# Selected area inside freehand polygon is kept
# Everything outside is removed
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Content size: ${contentWidth}x${contentHeight} pixels
# Scaled size: ${scaledW}x${scaledH} pixels
# Rotation: ${rotation}°
# Pixels kept: ${keptPixels}
# True crop bounds: x[${minX},${maxX}], y[${minY},${maxY}]
# Map size reduced from ${width}x${height} to ${contentWidth}x${contentHeight}`;

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
      alert(`✅ True Cropped Map Exported!\n\n📄 ${fileBase}.png\n✅ Kept ${keptPixels} non-white pixels INSIDE selection\n🗑️ Completely removed EVERYTHING OUTSIDE selection\n📏 Map size: ${contentWidth}x${contentHeight} pixels (reduced from ${width}x${height})\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n⚪ Everything outside selection completely removed`);
    }, 100);

  } catch (e) {
    console.error("Error exporting true cropped map:", e);
    alert("❌ Error exporting true cropped map");
  }
};

const saveMapAsPNG = () => {
  if (!mapMsg || !mapParamsRef.current) {
    alert("No map loaded to save!");
    return;
  }

  // If there's an active crop selection, ask user what they want
  if (cropState.freehandPoints && cropState.freehandPoints.length >= 3) {
    const choice = confirm(
      "Crop selection detected!\n\n" +
      "Click OK to export the truly cropped map (only selected area, reduced size).\n" +
      "Click Cancel to save current map with transparent deleted areas."
    );
    
    if (choice) {
      // Export truly cropped map (reduced size)
      handleExportCroppedMap();
      return;
    }
    // Continue to save current map with transparency
  }

  // Save the full map (or currently displayed map) with transparency for deleted areas
  try {
    const { width, height } = mapMsg;
    const { resolution, originX, originY } = mapParamsRef.current;
    
    const SCALE_FACTOR = 4;
    
    const tempCanvas = document.createElement('canvas');
    const ctx = tempCanvas.getContext('2d');
    
    tempCanvas.width = width * SCALE_FACTOR;
    tempCanvas.height = height * SCALE_FACTOR;
    
    // Clear with transparent background
    ctx.clearRect(0, 0, tempCanvas.width, tempCanvas.height);
    
    ctx.save();
    ctx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
    if (rotation !== 0) {
      ctx.translate(width / 2, height / 2);
      ctx.rotate((rotation * Math.PI) / 180);
      ctx.translate(-width / 2, -height / 2);
    }

    // STEP 1: Draw base map with TRUE TRANSPARENCY for deleted areas
    const imageData = ctx.createImageData(width, height);
    
    // Count transparent pixels for feedback
let transparentPixels = 0;
let occupiedPixels = 0;
let freePixels = 0;
let unknownPixels = 0;
let checkerboardPixels = 0;
    
   for (let y = 0; y < height; y++) {
  for (let x = 0; x < width; x++) {
    const val = mapMsg.data[y * width + x];
    const py = height - 1 - y;
    const i = (py * width + x) * 4;
    
    let gray, alpha;
    
    if (val === -3) {
      // CROPPED AREAS (value -3) - MAKE TRANSPARENT
      gray = 255; // White
      alpha = 0;  // Fully transparent
      transparentPixels++;
    } else if (val === -2) {
      // OLD DELETED AREAS - ALSO TRANSPARENT
      gray = 255;
      alpha = 0;
      transparentPixels++;
    } else if (val === -1) {
      gray = 205;
      alpha = 255;
      unknownPixels++;
    } else if (val === 0) {
      gray = 255;
      alpha = 255;
      freePixels++;
    } else if (val === 100) {
      gray = 0;
      alpha = 255;
      occupiedPixels++;
    } else {
      gray = 255 - Math.floor((val / 100) * 255);
      alpha = 255;
      if (val > 50) occupiedPixels++;
      else freePixels++;
    }

    imageData.data[i] = gray;
    imageData.data[i + 1] = gray;
    imageData.data[i + 2] = gray;
    imageData.data[i + 3] = alpha;
  }
}
    
    const off = document.createElement("canvas");
    off.width = width;
    off.height = height;
    off.getContext("2d").putImageData(imageData, 0, 0);
    ctx.drawImage(off, 0, 0, width, height);

    // STEP 2: Create a mask for transparent areas so annotations don't overwrite them
    // We'll check if a pixel is transparent before drawing annotations
    
    // Helper function to check if a point is in a transparent area
    const isPointTransparent = (canvasX, canvasY) => {
      // Convert canvas coordinates to map coordinates
      const mapX = Math.floor(canvasX);
      const mapY = Math.floor(canvasY);
      
      if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) {
        return true; // Outside map is transparent
      }
      
      const val = mapMsg.data[mapY * width + mapX];
      return val === -2; // -2 means transparent
    };

    // Transform point helper function
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

    // STEP 3: Draw zones with transparency check
    zones.forEach((zone) => {
      if (zone.points && zone.points.length >= 3) {
        // Check if zone center is in transparent area
        const centerX = zone.points.reduce((sum, p) => sum + transformPoint(p).x, 0) / zone.points.length;
        const centerY = zone.points.reduce((sum, p) => sum + transformPoint(p).y, 0) / zone.points.length;
        
        if (isPointTransparent(centerX, centerY)) {
          // Skip drawing zone if center is in transparent area
          return;
        }
        
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
          ctx.lineWidth = 2 / SCALE_FACTOR;
          ctx.stroke();
        } else {
          ctx.fillStyle = "rgba(100, 200, 100, 0.3)";
          ctx.fill();
          ctx.strokeStyle = "rgba(50, 150, 50, 0.8)";
          ctx.lineWidth = 1.4 / SCALE_FACTOR;
          ctx.stroke();
        }

        if (zone.type === 'keep_out') {
          ctx.fillStyle = "rgba(220, 38, 38, 1)";
        } else {
          ctx.fillStyle = "rgba(50, 150, 50, 1)";
        }
        ctx.font = `${14 / SCALE_FACTOR}px Arial`;
        ctx.fillText(zone.name, centerX + 16 / SCALE_FACTOR, centerY + 8 / SCALE_FACTOR);
      }
    });

    // STEP 4: Draw arrows with transparency check
    arrows.forEach((arrow) => {
      const fromNode = nodes.find(n => n.id === arrow.fromId);
      const toNode = nodes.find(n => n.id === arrow.toId);
      
      if (fromNode && toNode) {
        // Check if either endpoint is in transparent area
        const fromTransformed = transformPoint(fromNode);
        const toTransformed = transformPoint(toNode);
        
        if (isPointTransparent(fromTransformed.x, fromTransformed.y) || 
            isPointTransparent(toTransformed.x, toTransformed.y)) {
          // Skip drawing arrow if endpoints are in transparent areas
          return;
        }
        
        const allPoints = [fromNode, ...(arrow.points || []), toNode];
        if (allPoints.length < 2) return;
        
        // Check all points for transparency
        let hasTransparentPoint = false;
        for (const point of allPoints) {
          const transformed = transformPoint(point);
          if (isPointTransparent(transformed.x, transformed.y)) {
            hasTransparentPoint = true;
            break;
          }
        }
        
        if (hasTransparentPoint) {
          // Skip drawing arrow if any point is in transparent area
          return;
        }
        
        ctx.strokeStyle = "rgba(255, 165, 0, 0.8)";
        ctx.lineWidth = 4 / SCALE_FACTOR;
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
        const headLength = 24 / SCALE_FACTOR;
        
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
          ctx.arc(point.x, point.y, 8 / SCALE_FACTOR, 0, Math.PI * 2);
          ctx.fillStyle = "rgba(255, 165, 0, 0.6)";
          ctx.fill();
          ctx.strokeStyle = "rgba(255, 165, 0, 1)";
          ctx.lineWidth = 2 / SCALE_FACTOR;
          ctx.stroke();
        }
      }
    });

    // STEP 5: Draw nodes with transparency check
    nodes.forEach((node) => {
      const transformed = transformPoint(node);
      
      // Check if node is in transparent area
      if (isPointTransparent(transformed.x, transformed.y)) {
        // Skip drawing node if it's in transparent area
        return;
      }
      
      const nodeColors = {
        station: '#8b5cf6',
        docking: '#06b6d4',
        waypoint: '#f59e0b',
        home: '#10b981',
        charging: '#ef4444'
      };
      
      const nodeColor = nodeColors[node.type] || "#3b82f6";
      
      ctx.beginPath();
      ctx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
      ctx.fillStyle = nodeColor;
      ctx.fill();
    });

    ctx.restore();

    // Generate YAML file
    const newResolution = resolution / SCALE_FACTOR;
    const yamlContent = `image: ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png
mode: trinary
resolution: ${newResolution.toFixed(6)}
origin: [${originX}, ${originY}, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
# High-resolution export WITH REAL TRANSPARENCY
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Original size: ${width}x${height} pixels
# Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
# Rotation: ${rotation}°
# Pixel Statistics:
#   Transparent (deleted/cropped): ${transparentPixels} pixels
#   Occupied (black): ${occupiedPixels} pixels
#   Free (white): ${freePixels} pixels
#   Unknown (gray): ${unknownPixels} pixels
# Deleted areas (-2): REAL TRANSPARENCY (alpha=0)
# Annotations NOT drawn on transparent areas
# Annotations included: Nodes (${nodes.length}), Arrows (${arrows.length}), Zones (${zones.length})`;

    tempCanvas.toBlob((blob) => {
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
      
      const yamlBlob = new Blob([yamlContent], { type: "application/x-yaml" });
      const yamlUrl = URL.createObjectURL(yamlBlob);
      const yamlLink = document.createElement('a');
      yamlLink.href = yamlUrl;
      yamlLink.download = `${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml`;
      document.body.appendChild(yamlLink);
      yamlLink.click();
      document.body.removeChild(yamlLink);
      URL.revokeObjectURL(yamlUrl);
      
      alert(`✅ High-resolution map saved WITH REAL TRANSPARENCY!\n\n📁 Files:\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.png\n- ${mapName || 'map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n🔄 Rotation: ${rotation}°\n🌫️ Transparent areas: ${transparentPixels} pixels (REAL transparency - check with transparency checker)\n📝 Annotations: ${nodes.length} nodes, ${arrows.length} arrows, ${zones.length} zones\n⚠️ Annotations NOT drawn on transparent areas`);

    }, 'image/png', 1.0);

  } catch (error) {
    console.error('Error saving high-resolution map as PNG:', error);
    alert('❌ Error saving high-resolution map as PNG. Check console for details.');
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
    // Save state before erasing for undo
    saveToHistory();
    
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

  const eraseNoiseAt = (clientX, clientY) => {
    if (!editableMap || !canvasRef.current) return;

    // Save state before erasing for undo
    saveToHistory();

    const canvasCoords = clientToCanvasCoords(clientX, clientY);
    const mapX = Math.floor(canvasCoords.x);
    const mapY = Math.floor(editableMap.height - canvasCoords.y);

    const { width, height, data } = editableMap;

    if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) return;

    const newData = [...data];
    let erased = 0;

    for (let y = mapY - eraseRadius; y <= mapY + eraseRadius; y++) {
      for (let x = mapX - eraseRadius; x <= mapX + eraseRadius; x++) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
          const d = Math.sqrt((x - mapX) ** 2 + (y - mapY) ** 2);
          if (d <= eraseRadius) {
            const idx = y * width + x;
            if (newData[idx] === 100 || newData[idx] === -1) {
              newData[idx] = 0;
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

    saveToHistory();

    const canvasCoords = clientToCanvasCoords(clientX, clientY);
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

    const points = getLinePoints(x0, y0, x1, y1);

    points.forEach(point => {
      const { x, y } = point;
      
      if (x < 0 || x >= width || y < 0 || y >= height) return;
      
      for (let dy = -eraseRadius; dy <= eraseRadius; dy++) {
        for (let dx = -eraseRadius; dx <= eraseRadius; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          
          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            const distance = Math.sqrt(dx * dx + dy * dy);
            if (distance <= eraseRadius) {
              const idx = ny * width + nx;
              if (newData[idx] === 100 || newData[idx] === -1) {
                newData[idx] = 0;
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

  // --- Save Map Functions (for full map) ---
// --- Save Map Functions (for full map) ---
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
    
    const SCALE_FACTOR = 4;
    const CHECKER_SIZE = 8; // Size of checkerboard squares
    
    // Create a temporary canvas for drawing everything
    const tempCanvas = document.createElement('canvas');
    const tempCtx = tempCanvas.getContext('2d');
    
    if (rotation === 90 || rotation === 270) {
      tempCanvas.width = height * SCALE_FACTOR;
      tempCanvas.height = width * SCALE_FACTOR;
    } else {
      tempCanvas.width = width * SCALE_FACTOR;
      tempCanvas.height = height * SCALE_FACTOR;
    }
    
    // Fill with white background (PGM needs white for free space)
    tempCtx.fillStyle = 'white';
    tempCtx.fillRect(0, 0, tempCanvas.width, tempCanvas.height);
    
    tempCtx.save();
    tempCtx.scale(SCALE_FACTOR, SCALE_FACTOR);
    
    if (rotation !== 0) {
      const centerX = tempCanvas.width / (2 * SCALE_FACTOR);
      const centerY = tempCanvas.height / (2 * SCALE_FACTOR);
      tempCtx.translate(centerX, centerY);
      tempCtx.rotate((rotation * Math.PI) / 180);
      
      if (rotation === 90) {
        tempCtx.translate(-centerY, -centerX);
      } else if (rotation === 180) {
        tempCtx.translate(-centerX, -centerY);
      } else if (rotation === 270) {
        tempCtx.translate(-centerY, -centerX);
      } else {
        tempCtx.translate(-centerX, -centerY);
      }
    }

    // Transform point helper function (same as saveMapAsPNG)
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

    // Helper function to check if a point is in a deleted area (for annotation skipping)
    const isPointDeleted = (canvasX, canvasY) => {
      // Convert canvas coordinates to map coordinates
      const mapX = Math.floor(canvasX);
      const mapY = Math.floor(canvasY);
      
      if (mapX < 0 || mapX >= width || mapY < 0 || mapY >= height) {
        return true; // Outside map is considered deleted
      }
      
      const val = mapMsg.data[mapY * width + mapX];
      return val === -2 || val === -3; // -2 or -3 means deleted/cropped
    };

    // STEP 1: Draw zones with deletion check (same as saveMapAsPNG)
    zones.forEach((zone) => {
      if (zone.points && zone.points.length >= 3) {
        // Check if zone center is in deleted area (skip if true)
        const centerX = zone.points.reduce((sum, p) => sum + transformPoint(p).x, 0) / zone.points.length;
        const centerY = zone.points.reduce((sum, p) => sum + transformPoint(p).y, 0) / zone.points.length;
        
        if (isPointDeleted(centerX, centerY)) {
          // Skip drawing zone if center is in deleted area
          return;
        }
        
        tempCtx.beginPath();
        zone.points.forEach((point, idx) => {
          const transformed = transformPoint(point);
          if (idx === 0) tempCtx.moveTo(transformed.x, transformed.y);
          else tempCtx.lineTo(transformed.x, transformed.y);
        });
        tempCtx.closePath();
        
        if (zone.type === 'keep_out') {
          tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)"; // Solid black
          tempCtx.fill();
        } else {
          tempCtx.fillStyle = "rgba(100, 100, 100, 1.0)"; // Solid dark gray
          tempCtx.fill();
        }
      }
    });

    // STEP 2: Draw the base map OVER the zones with CHECKERBOARD for deleted areas
    const imageData = tempCtx.createImageData(width, height);
    let checkerboardPixels = 0;
    let deletedPixels = 0;
    let occupiedPixels = 0;
    let freePixels = 0;
    let unknownPixels = 0;
    
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const val = mapMsg.data[y * width + x];
        const py = height - 1 - y;
        const i = (py * width + x) * 4;

        let gray;
        if (val === -3 || val === -2) {
          // CROPPED AREAS (value -3) or DELETED AREAS (-2) - DRAW CHECKERBOARD PATTERN
          const isCheckered = ((Math.floor(x / CHECKER_SIZE) + Math.floor(y / CHECKER_SIZE)) % 2) === 0;
          gray = isCheckered ? 180 : 220; // Checkerboard pattern (dark gray / light gray)
          checkerboardPixels++;
          deletedPixels++;
        } else if (val === -1) {
          gray = 205; // Unknown - medium gray
          unknownPixels++;
        } else if (val === 0) {
          gray = 255; // Free space - white
          freePixels++;
        } else if (val === 100) {
          gray = 0; // Occupied - black
          occupiedPixels++;
        } else {
          gray = 255 - Math.floor((val / 100) * 255); // Grayscale based on occupancy
          if (val > 50) occupiedPixels++;
          else freePixels++;
        }

        imageData.data[i] = gray;
        imageData.data[i + 1] = gray;
        imageData.data[i + 2] = gray;
        imageData.data[i + 3] = 255; // Fully opaque
      }
    }
    
    const offCanvas = document.createElement("canvas");
    offCanvas.width = width;
    offCanvas.height = height;
    offCanvas.getContext("2d").putImageData(imageData, 0, 0);
    tempCtx.drawImage(offCanvas, 0, 0, width, height);

    // STEP 3: Draw arrows with deletion check (same as saveMapAsPNG)
    arrows.forEach((arrow) => {
      const fromNode = nodes.find(n => n.id === arrow.fromId);
      const toNode = nodes.find(n => n.id === arrow.toId);
      
      if (fromNode && toNode) {
        // Check if either endpoint is in deleted area
        const fromTransformed = transformPoint(fromNode);
        const toTransformed = transformPoint(toNode);
        
        if (isPointDeleted(fromTransformed.x, fromTransformed.y) || 
            isPointDeleted(toTransformed.x, toTransformed.y)) {
          // Skip drawing arrow if endpoints are in deleted areas
          return;
        }
        
        const allPoints = [fromNode, ...(arrow.points || []), toNode];
        if (allPoints.length < 2) return;
        
        // Check all points for deletion
        let hasDeletedPoint = false;
        for (const point of allPoints) {
          const transformed = transformPoint(point);
          if (isPointDeleted(transformed.x, transformed.y)) {
            hasDeletedPoint = true;
            break;
          }
        }
        
        if (hasDeletedPoint) {
          // Skip drawing arrow if any point is in deleted area
          return;
        }
        
        tempCtx.strokeStyle = "rgba(0, 0, 0, 1.0)"; // Solid black
        tempCtx.lineWidth = 4 / SCALE_FACTOR;
        tempCtx.lineJoin = 'round';
        tempCtx.lineCap = 'round';
        tempCtx.beginPath();
        
        const firstPoint = transformPoint(allPoints[0]);
        tempCtx.moveTo(firstPoint.x, firstPoint.y);
        
        for (let i = 1; i < allPoints.length; i++) {
          const point = transformPoint(allPoints[i]);
          tempCtx.lineTo(point.x, point.y);
        }
        tempCtx.stroke();
        
        // Draw arrow head
        const lastPoint = transformPoint(allPoints[allPoints.length - 1]);
        const secondLastPoint = transformPoint(allPoints[allPoints.length - 2]);
        const angle = Math.atan2(
          lastPoint.y - secondLastPoint.y,
          lastPoint.x - secondLastPoint.x
        );
        const headLength = 20 / SCALE_FACTOR;
        
        tempCtx.save();
        tempCtx.translate(lastPoint.x, lastPoint.y);
        tempCtx.rotate(angle);
        tempCtx.beginPath();
        tempCtx.moveTo(0, 0);
        tempCtx.lineTo(-headLength, -headLength / 2);
        tempCtx.lineTo(-headLength, headLength / 2);
        tempCtx.closePath();
        tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)";
        tempCtx.fill();
        tempCtx.restore();
      }
    });

    // STEP 4: Draw nodes with deletion check (same as saveMapAsPNG)
    nodes.forEach((node) => {
      const transformed = transformPoint(node);
      
      // Check if node is in deleted area
      if (isPointDeleted(transformed.x, transformed.y)) {
        // Skip drawing node if it's in deleted area
        return;
      }
      
      // Draw node as solid black circle
      tempCtx.beginPath();
      tempCtx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
      tempCtx.fillStyle = "rgba(0, 0, 0, 1.0)";
      tempCtx.fill();
      
      // Draw white border for contrast
      tempCtx.beginPath();
      tempCtx.arc(transformed.x, transformed.y, 6 / SCALE_FACTOR, 0, Math.PI * 2);
      tempCtx.strokeStyle = "rgba(255, 255, 255, 1.0)";
      tempCtx.lineWidth = 1 / SCALE_FACTOR;
      tempCtx.stroke();
    });

    tempCtx.restore();

    // Save debug preview as PNG to see the checkerboard
    const debugUrl = tempCanvas.toDataURL('image/png');
    const debugLink = document.createElement('a');
    debugLink.href = debugUrl;
    debugLink.download = 'debug_pgm_preview_with_checkerboard.png';
    debugLink.click();

    // Convert canvas to grayscale for PGM
    const imageDataFromCanvas = tempCtx.getImageData(0, 0, tempCanvas.width, tempCanvas.height);
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
# High-resolution PGM Export WITH CHECKERBOARD FOR DELETED AREAS
# Original resolution: ${resolution} m/pixel
# Scaled resolution: ${newResolution.toFixed(6)} m/pixel
# Scale factor: ${SCALE_FACTOR}x
# Original size: ${width}x${height} pixels
# Scaled size: ${tempCanvas.width}x${tempCanvas.height} pixels
# Rotation: ${rotation}°
# Pixel Statistics:
#   Checkerboard (deleted/cropped): ${checkerboardPixels} pixels
#   Occupied (black): ${occupiedPixels} pixels
#   Free (white): ${freePixels} pixels
#   Unknown (gray): ${unknownPixels} pixels
# Annotations: ${nodes.length} nodes, ${arrows.length} arrows, ${zones.length} zones
# Zones: Black/dark gray (NOT drawn on deleted areas)
# Arrows: Black lines (NOT drawn through deleted areas)
# Nodes: Black circles with white border (NOT drawn on deleted areas)
# Deleted areas (-2, -3): CHECKERBOARD PATTERN (180/220 gray)
# Deleted pixels: ${deletedPixels} (${checkerboardPixels} shown as checkerboard)
# Checkerboard pattern: ${CHECKER_SIZE}px squares alternating between gray 180 and 220
# Note: In PNG export, these areas would be transparent
# For PGM, checkerboard indicates "removed/not part of map"`;

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

    alert(`✅ High-resolution PGM map exported with CHECKERBOARD!\n\n📁 Files saved:\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.pgm\n- ${mapName || 'exported_map'}_rotated_${rotation}_${SCALE_FACTOR}x.yaml\n- debug_pgm_preview_with_checkerboard.png (preview)\n🔄 Rotation: ${rotation}°\n📊 Size: ${tempCanvas.width} x ${tempCanvas.height} pixels (${SCALE_FACTOR}x)\n📍 Resolution: ${newResolution.toFixed(6)} m/pixel\n◻️ Deleted areas: CHECKERBOARD PATTERN (180/220 gray)\n🗑️ Deleted pixels: ${deletedPixels}\n⚫ Annotations: Black (zones, arrows, nodes)\n⚠️ Annotations NOT drawn on deleted areas`);

  } catch (error) {
    console.error('Error exporting high-resolution PGM map:', error);
    alert('❌ Error exporting high-resolution PGM map. Check console for details.');
  }
};
  // --- Database Functions ---
  const saveNodesToDatabase = async () => {
  if (nodes.length === 0 && arrows.length === 0 && zones.length === 0) {
    alert("No nodes, arrows, or zones to save!");
    return;
  }

  // Get current map name or use filename
  const currentMapName = mapName || 
    (yamlFile ? yamlFile.name.replace('.yaml', '').replace('.yml', '') : 'default');

  try {
    setIsSaving(true);
    setDbStatus("Saving...");

    const payload = {
      mapName: currentMapName,
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
      alert(`✅ Successfully saved to database!\n\n🗂️ Database: ${result.database_file}\n🏷️ Map: ${result.map_name}\n📁 File: ${result.database_path}`);
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
    // Get current map name or use filename
    const mapNameToLoad = mapName || 
      (yamlFile ? yamlFile.name.replace('.yaml', '').replace('.yml', '') : 'default');
    
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
        
        alert(`✅ Successfully loaded from database!\n\n📊 Nodes: ${result.nodes.length}\n➡️ Arrows: ${result.arrows.length}\n🗺️ Zones: ${result.zones.length}\n📁 Database: ${result.database_file}`);
      } else {
        alert(`❌ Error: ${result.message}`);
      }
    } else {
      const errorResult = await response.json();
      alert(`❌ Error loading: ${errorResult.message}`);
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
    
    saveToHistory();
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
        saveToHistory();
        
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
    
    saveToHistory();
    
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
      setCropState({
        ...cropState,
        startX: c.x,
        startY: c.y,
        endX: c.x,
        endY: c.y,
        isDragging: true,
        freehandPoints: [{x: c.x, y: c.y}]
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

    // Freehand crop drawing
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
      const c = clientToCanvasCoords(e.clientX, e.clientY);
      const completedPoints = [...cropState.freehandPoints, {x: c.x, y: c.y}];
      
      setCropState(prev => ({
        ...prev,
        endX: c.x,
        endY: c.y,
        isDragging: false,
        freehandPoints: completedPoints
      }));
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
// Add these functions after your handleWheel function:

const handleTouchStart = (e) => {
  if (e.touches.length === 1) {
    const touch = e.touches[0];
    const rect = canvasRef.current.getBoundingClientRect();
    const x = touch.clientX - rect.left;
    const y = touch.clientY - rect.top;
    
    // Always allow panning with touch
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
      saveToHistory();
      setNodes([]);
      setArrows([]);
      setZones([]);
      setCurrentZonePoints([]);
      setArrowDrawing({ isDrawing: false, fromId: null, points: [] });
      setRotation(0);
    }
  };
// Add keyboard shortcuts after your other useEffect hooks

  // Add keyboard shortcuts after your other useEffect hooks
  useEffect(() => {
    const handleKeyDown = (e) => {
      // Spacebar for temporary pan tool
      if (e.code === 'Space' && !e.repeat) {
        e.preventDefault();
        const prevTool = tool;
        setTool("pan");
        
        // Store previous tool to restore it when space is released
        const restoreTool = (e) => {
          if (e.code === 'Space') {
            setTool(prevTool);
            window.removeEventListener('keyup', restoreTool);
          }
        };
        window.addEventListener('keyup', restoreTool);
      }
      
      // Ctrl+Z for undo, Ctrl+Shift+Z or Ctrl+Y for redo
      if (e.ctrlKey) {
        if (e.key === 'z' || e.key === 'Z') {
          e.preventDefault();
          if (e.shiftKey) {
            // Ctrl+Shift+Z for redo
            handleRedo();
          } else {
            // Ctrl+Z for undo
            handleUndo();
          }
        }
        if (e.key === 'y' || e.key === 'Y') {
          e.preventDefault();
          // Ctrl+Y for redo
          handleRedo();
        }
      }
      
      // Alternative: Cmd+Z and Cmd+Shift+Z for Mac
      if (e.metaKey) {
        if (e.key === 'z' || e.key === 'Z') {
          e.preventDefault();
          if (e.shiftKey) {
            // Cmd+Shift+Z for redo
            handleRedo();
          } else {
            // Cmd+Z for undo
            handleUndo();
          }
        }
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [tool, handleUndo, handleRedo]); // Now these functions are stable

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

    // Draw base map
   // In the main useEffect for drawing:
// Draw base map
// Draw base map
const imageData = ctx.createImageData(width, height);
for (let y = 0; y < height; y++) {
  for (let x = 0; x < width; x++) {
    const val = data[y * width + x];
    const py = height - 1 - y;
    const i = (py * width + x) * 4;

// In the imageData creation loop:
let gray, alpha = 255;
if (val === -3) {
  // NEW: True transparency (for cropped areas)
  gray = 255; // White
  alpha = 0;  // Fully transparent
} else if (val === -2) {
  // Keep checkerboard for old deleted areas if you want
  const checkerSize = 8;
  const isCheckered = ((Math.floor(x / checkerSize) + Math.floor(y / checkerSize)) % 2) === 0;
  gray = isCheckered ? 180 : 220;
  alpha = 255;
} else if (val === -1) gray = 205;
else if (val === 0) gray = 255;
else if (val === 100) gray = 0;
else gray = 255 - Math.floor((val / 100) * 255);

imageData.data[i] = gray;
imageData.data[i + 1] = gray;
imageData.data[i + 2] = gray;
imageData.data[i + 3] = alpha; // This controls transparency
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
      
      if (cropState.isDragging && cursorCoords) {
        ctx.lineTo(cursorCoords.canvasX, cursorCoords.canvasY);
      }
      
      ctx.stroke();
      ctx.setLineDash([]);

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
        ctx.strokeStyle = "rgba(0, 0, 0, 0.9)";
        ctx.lineWidth = 2 / zoomState.scale;
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

        const yamlFileEntry = zip.files[yamlFiles[0]];
        const yamlText = await yamlFileEntry.async('text');
        const parsedYaml = yaml.load(yamlText);

        const expectedPgmName = parsedYaml.image;
        let pgmFileEntry;

        if (expectedPgmName) {
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

        if (!pgmFileEntry && pgmFiles.length > 0) {
          pgmFileEntry = zip.files[pgmFiles[0]];
        }

        if (!pgmFileEntry) {
          alert("Could not find matching PGM file in the ZIP archive");
          return;
        }

        const pgmArrayBuffer = await pgmFileEntry.async('arraybuffer');
        
        const yamlFile = new File([yamlText], yamlFiles[0], { type: 'application/x-yaml' });
        const pgmFile = new File([pgmArrayBuffer], pgmFileEntry.name, { type: 'image/x-portable-graymap' });

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

        clearHistory();

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

        {/* Undo/Redo Controls */}
        <div style={{ marginBottom: 16 }}>
          <label style={styles.label}>Undo/Redo</label>
          <div style={{ display: "flex", gap: 8, marginBottom: 16 }}>
            <button 
              onClick={handleUndo} 
              style={{
                ...styles.buttonUndoRedo,
                opacity: historyIndex > 0 ? 1 : 0.5,
                cursor: historyIndex > 0 ? "pointer" : "not-allowed"
              }}
              disabled={historyIndex <= 0}
              title="Undo last action"
            >
              <FaUndo />
              Undo
            </button>
            <button 
              onClick={handleRedo} 
              style={{
                ...styles.buttonUndoRedo,
                opacity: historyIndex < history.length - 1 ? 1 : 0.5,
                cursor: historyIndex < history.length - 1 ? "pointer" : "not-allowed"
              }}
              disabled={historyIndex >= history.length - 1}
              title="Redo last undone action"
            >
              <FaRedoAlt />
              Redo
            </button>
          </div>
          <div style={styles.hint}>
            History: {historyIndex + 1}/{history.length} steps
          </div>
        </div>

       <div style={{ marginBottom: 16 }}>
  <label style={styles.label}>Tools</label>
  <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: 8, marginBottom: 16 }}>
    {/* Hand (Pan) Tool - NEW */}
    <button 
      onClick={() => setTool("pan")} 
      style={tool === "pan" ? styles.buttonActive : styles.button}
      title="Drag to pan the map (Middle mouse button also works)"
    >
      <FaHandPaper />
      Pan
    </button>
    
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
      <FaCropAlt />
      Crop
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

        {/* Crop Controls */}
        {cropState.isCropping && (
          <div style={{ marginBottom: 16, padding: 12, background: currentTheme.surface, borderRadius: 8, border: `2px solid ${currentTheme.accent}` }}>
            
            <div style={{...styles.label, color: currentTheme.accent, marginBottom: 8 }}>
              ✂️ Crop Mode Active
            </div>

            <div style={styles.hint}>
              Draw a freehand selection around the area you want to KEEP.
            </div>

            <div style={{ display: "flex", gap: 8, marginTop: 8, flexWrap: "wrap" }}>
              
              {/* Apply Crop (visual checkerboard) */}
              <button
                onClick={handleApplyCrop}
                style={{
                  ...styles.buttonAction,
                  background: '#10b981',
                  border: '1px solid #059669'
                }}
                disabled={!cropState.freehandPoints || cropState.freehandPoints.length < 3}
                title="Apply crop to current map (checkerboard for deleted areas)"
              >
                <FaCropAlt />
                Apply Crop
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
                Points: {cropState.freehandPoints.length} (need 3+)
              </div>
            )}

            <div style={{ marginTop: 12, padding: 8, background: currentTheme.background, borderRadius: 4 }}>
              <div style={{ fontSize: '11px', color: currentTheme.textSecondary, lineHeight: 1.4 }}>
                <strong>Apply Crop:</strong> Marks deleted areas as checkerboard in current map.<br/>
                <strong>Export True Crop:</strong> Creates new map with deleted areas completely removed (reduced map size).
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
                Crop: {cropState.freehandPoints?.length || 0} points
              </div>
            )}
            <div style={styles.mapInfo}>
              History: {historyIndex + 1}/{history.length}
            </div>
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
            {/* <button 
              onClick={handleRotate}
              style={styles.smallButton}
              title="Rotate 90°"
            >
              <FaRedo />
            </button> */}
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
            <button 
              onClick={handleUndo}
              style={{
                ...styles.smallButton,
                opacity: historyIndex > 0 ? 1 : 0.5,
                cursor: historyIndex > 0 ? "pointer" : "not-allowed",
                background: 'transparent',
                color: currentTheme.text,
                border: `1px solid ${currentTheme.border}`
              }}
              disabled={historyIndex <= 0}
              title="Undo (Ctrl+Z)"
            >
              <FaUndo />
            </button>
            <button 
              onClick={handleRedo}
              style={{
                ...styles.smallButton,
                opacity: historyIndex < history.length - 1 ? 1 : 0.5,
                cursor: historyIndex < history.length - 1 ? "pointer" : "not-allowed",
                background: 'transparent',
                color: currentTheme.text,
                border: `1px solid ${currentTheme.border}`
              }}
              disabled={historyIndex >= history.length - 1}
              title="Redo (Ctrl+Y)"
            >
              <FaRedoAlt />
            </button>
          </div>

   <canvas
  ref={canvasRef}
  style={{ 
    width: "100%", 
    height: "100%", 
    display: "block", 
    cursor: tool === "pan" ? (zoomState.isDragging ? "grabbing" : "grab") :
            cropState.isCropping ? "crosshair" : 
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
  onTouchStart={handleTouchStart}
  onTouchMove={handleTouchMove}
  onTouchEnd={handleTouchEnd}
  onTouchCancel={handleTouchEnd}
/>
        </div>
      </div>
    </div>
  );
}



