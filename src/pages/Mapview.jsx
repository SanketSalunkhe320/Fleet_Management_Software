import React, { useState, useEffect, useRef } from "react";
import yaml from "js-yaml";
import { 
  FaSearchPlus,
  FaSearchMinus,
  FaExpand,
  FaUpload,
  FaTrash,
  FaImage,
  FaRedo,
  FaMapMarkerAlt,
  FaCrosshairs,
  FaMousePointer,
  FaDownload
} from "react-icons/fa";

export default function MapView() {
  const [mapMsg, setMapMsg] = useState(null);
  const [editableMap, setEditableMap] = useState(null);
  const [debugInfo, setDebugInfo] = useState("Load a map to get started");
  const [isMobile, setIsMobile] = useState(false);
  const [mapLoadedFromFile, setMapLoadedFromFile] = useState(false);
  const [pgmImage, setPgmImage] = useState(null);
  const [mapMetadata, setMapMetadata] = useState(null);
  const [rotation, setRotation] = useState(0);
  
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

  // --- Initial Pose State ---
  const [initialPose, setInitialPose] = useState(null);
  const [isSettingPose, setIsSettingPose] = useState(false);
  const [isDrawingPose, setIsDrawingPose] = useState(false);
  const [poseStart, setPoseStart] = useState(null);
  const [currentMousePos, setCurrentMousePos] = useState(null);
  const [isDraggingPose, setIsDraggingPose] = useState(false);
  const [poseDragOffset, setPoseDragOffset] = useState({ x: 0, y: 0 });

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
  
  // Load saved map data from localStorage on component mount
  useEffect(() => {
    const savedMapData = localStorage.getItem("savedMapData");
    const savedPngData = localStorage.getItem("savedPngData");
    const savedInitialPose = localStorage.getItem("savedInitialPose");
    
    console.log("🔍 Checking localStorage for saved data:", {
      pgm: savedMapData ? "Found" : "Not found",
      png: savedPngData ? "Found" : "Not found",
      pose: savedInitialPose ? "Found" : "Not found"
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

    // Restore initial pose if exists
    if (savedInitialPose) {
      try {
        const pose = JSON.parse(savedInitialPose);
        setInitialPose(pose);
        setDebugInfo(`🎯 Restored initial pose at (${pose.rosX.toFixed(2)}, ${pose.rosY.toFixed(2)})`);
      } catch (e) {
        console.error("Error restoring initial pose:", e);
        localStorage.removeItem("savedInitialPose");
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

    if (initialPose) {
      localStorage.setItem("savedInitialPose", JSON.stringify(initialPose));
    }
  }, [pgmImage, mapMetadata, pngImage, pngMapInfo, zoomState, initialPose]);

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

  // Coordinate conversions
  const rosToCanvasCoords = (rosX, rosY, applyRotation = true) => {
    if (!mapParamsRef.current) {
      return { x: 0, y: 0 };
    }

    const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
    // Convert ROS coordinates to pixel coordinates
    const pixelX = (rosX - originX) / resolution;
    const pixelY = (rosY - originY) / resolution;
    
    // Flip Y-axis (ROS has Y-up, canvas has Y-down)
    let canvasX = pixelX;
    let canvasY = height - pixelY;
    
    // Apply rotation transformation if needed
    if (applyRotation && rotation !== 0) {
      const centerX = width / 2;
      const centerY = height / 2;
      
      // Translate to origin
      const translatedX = canvasX - centerX;
      const translatedY = canvasY - centerY;
      
      let rotatedX, rotatedY;
      
      // Apply rotation
      switch (rotation) {
        case 90:
          rotatedX = translatedY;
          rotatedY = -translatedX;
          break;
        case 180:
          rotatedX = -translatedX;
          rotatedY = -translatedY;
          break;
        case 270:
          rotatedX = -translatedY;
          rotatedY = translatedX;
          break;
        default:
          rotatedX = translatedX;
          rotatedY = translatedY;
      }
      
      // Translate back
      canvasX = rotatedX + centerX;
      canvasY = rotatedY + centerY;
    }
    
    return { x: canvasX, y: canvasY };
  };

  const canvasToRosCoords = (canvasX, canvasY, applyRotation = true) => {
    if (!mapParamsRef.current) return null;
    const { resolution, originX, originY, width, height } = mapParamsRef.current;
    
    // Apply inverse rotation first if needed
    let unrotatedX = canvasX;
    let unrotatedY = canvasY;
    
    if (applyRotation && rotation !== 0) {
      const centerX = width / 2;
      const centerY = height / 2;
      
      // Translate to origin
      const translatedX = unrotatedX - centerX;
      const translatedY = unrotatedY - centerY;
      
      let inverseX, inverseY;
      
      // Apply inverse rotation
      switch (rotation) {
        case 90:
          inverseX = -translatedY;
          inverseY = translatedX;
          break;
        case 180:
          inverseX = -translatedX;
          inverseY = -translatedY;
          break;
        case 270:
          inverseX = translatedY;
          inverseY = -translatedX;
          break;
        default:
          inverseX = translatedX;
          inverseY = translatedY;
      }
      
      // Translate back
      unrotatedX = inverseX + centerX;
      unrotatedY = inverseY + centerY;
    }
    
    // Convert back to ROS coordinates
    const pixelY = height - unrotatedY;
    const worldX = originX + unrotatedX * resolution;
    const worldY = originY + pixelY * resolution;
    
    return { x: +worldX.toFixed(2), y: +worldY.toFixed(2) };
  };

  // Client to canvas coordinates (handles zoom and pan)
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
        setPngImage(null);
        setRotation(0);
        setInitialPose(null); // Clear initial pose when loading new map

        centerMapOnCanvas(width, height);
        setDebugInfo(`🗺️ PGM Map loaded: ${width}x${height}`);
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
        setPgmImage(null);
        setMapMsg(null);
        setEditableMap(null);
        setMapMetadata(null);
        setRotation(0);
        setInitialPose(null); // Clear initial pose when loading new image

        centerMapOnCanvas(img.width, img.height);
        setDebugInfo(`🖼️ PNG Image loaded: ${img.width}x${img.height}`);
        
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
      setRotation(0);
      setInitialPose(null);
      localStorage.removeItem("savedMapData");
      localStorage.removeItem("savedPngData");
      localStorage.removeItem("savedInitialPose");
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

  // --- Initial Pose Functions ---
  const startSettingPose = () => {
    setIsSettingPose(true);
    setIsDrawingPose(false);
    setPoseStart(null);
    setCurrentMousePos(null);
    setDebugInfo("🎯 Click on map to set START point, then DRAG to set direction");
  };

  const cancelSettingPose = () => {
    setIsSettingPose(false);
    setIsDrawingPose(false);
    setPoseStart(null);
    setCurrentMousePos(null);
    setDebugInfo("✅ Pose setting cancelled");
  };

  const handleMouseDownForPose = (clientX, clientY) => {
    if (!mapParamsRef.current) return;
    
    // If we already have an initial pose and are clicking on it
    if (initialPose) {
      const canvasCoords = clientToCanvasCoords(clientX, clientY);
      const distance = Math.sqrt(
        Math.pow(canvasCoords.x - initialPose.canvasX, 2) + 
        Math.pow(canvasCoords.y - initialPose.canvasY, 2)
      );
      
      const hitRadius = 20 / zoomState.scale;
      
      // Check if clicking near the arrow end
      const arrowLength = 50 / zoomState.scale;
      const arrowEndX = initialPose.canvasX + arrowLength * Math.cos(initialPose.orientation * Math.PI / 180);
      const arrowEndY = initialPose.canvasY + arrowLength * Math.sin(initialPose.orientation * Math.PI / 180);
      const arrowEndDistance = Math.sqrt(
        Math.pow(canvasCoords.x - arrowEndX, 2) + 
        Math.pow(canvasCoords.y - arrowEndY, 2)
      );
      
      // Check if clicking along the arrow line
      const arrowStartX = initialPose.canvasX;
      const arrowStartY = initialPose.canvasY;
      const A = canvasCoords.x - arrowStartX;
      const B = canvasCoords.y - arrowStartY;
      const C = arrowEndX - arrowStartX;
      const D = arrowEndY - arrowStartY;
      const dot = A * C + B * D;
      const lenSq = C * C + D * D;
      let param = -1;
      if (lenSq !== 0) param = dot / lenSq;
      
      let xx, yy;
      if (param < 0) {
        xx = arrowStartX;
        yy = arrowStartY;
      } else if (param > 1) {
        xx = arrowEndX;
        yy = arrowEndY;
      } else {
        xx = arrowStartX + param * C;
        yy = arrowStartY + param * D;
      }
      
      const dx = canvasCoords.x - xx;
      const dy = canvasCoords.y - yy;
      const lineDistance = Math.sqrt(dx * dx + dy * dy);
      
      // If clicking on or near the arrow, start dragging
      if (distance <= hitRadius * 2 || arrowEndDistance <= hitRadius * 1.5 || 
          (lineDistance <= hitRadius * 1.2 && param >= 0 && param <= 1)) {
        setIsDraggingPose(true);
        setPoseDragOffset({
          x: canvasCoords.x - initialPose.canvasX,
          y: canvasCoords.y - initialPose.canvasY
        });
        setDebugInfo("🎯 Dragging existing pose");
        return true; // Indicate that pose was clicked
      }
    }
    
    // If we're setting a new pose (button was clicked)
    if (isSettingPose && !isDrawingPose && !poseStart) {
      // First click: set start point
      const rosCoords = clientToRosCoords(clientX, clientY);
      if (!rosCoords) return false;
      
      const canvasCoords = rosToCanvasCoords(rosCoords.x, rosCoords.y, true);
      
      setPoseStart({
        rosX: rosCoords.x,
        rosY: rosCoords.y,
        canvasX: canvasCoords.x,
        canvasY: canvasCoords.y
      });
      
      setIsDrawingPose(true);
      setCurrentMousePos(canvasCoords);
      setDebugInfo("🎯 Drag to set direction, release to finish");
      return true; // Indicate that pose drawing started
    }
    
    return false; // No pose interaction
  };

  const handleMouseMoveForPose = (clientX, clientY) => {
    if (!isDrawingPose && !isDraggingPose) return;
    
    const canvasCoords = clientToCanvasCoords(clientX, clientY);
    setCurrentMousePos(canvasCoords);
    
    if (isDraggingPose && initialPose) {
      // Calculate new ROS coordinates from canvas position
      const newCanvasX = canvasCoords.x - poseDragOffset.x;
      const newCanvasY = canvasCoords.y - poseDragOffset.y;
      const newRosCoords = canvasToRosCoords(newCanvasX, newCanvasY, true);
      
      if (newRosCoords) {
        const updatedPose = {
          ...initialPose,
          rosX: newRosCoords.x,
          rosY: newRosCoords.y,
          canvasX: newCanvasX,
          canvasY: newCanvasY
        };
        
        setInitialPose(updatedPose);
        setDebugInfo(`🎯 Moving pose to (${newRosCoords.x.toFixed(2)}, ${newRosCoords.y.toFixed(2)})`);
      }
    }
  };

  const handleMouseUpForPose = (clientX, clientY) => {
    if (isDraggingPose) {
      setIsDraggingPose(false);
      return true; // Indicate pose was being dragged
    }
    
    if (isDrawingPose && poseStart) {
      // Calculate orientation from start to end point
      const endCanvasCoords = clientToCanvasCoords(clientX, clientY);
      const dx = endCanvasCoords.x - poseStart.canvasX;
      const dy = endCanvasCoords.y - poseStart.canvasY;
      let orientation = Math.atan2(dy, dx) * 180 / Math.PI;
      
      // Snap to 45 degree increments if holding Shift key
      if (window.event?.shiftKey) {
        orientation = Math.round(orientation / 45) * 45;
      }
      
      const endRosCoords = canvasToRosCoords(endCanvasCoords.x, endCanvasCoords.y, true);
      
      const newPose = {
        rosX: poseStart.rosX,
        rosY: poseStart.rosY,
        canvasX: poseStart.canvasX,
        canvasY: poseStart.canvasY,
        orientation: orientation,
        timestamp: Date.now()
      };
      
      setInitialPose(newPose);
      setIsDrawingPose(false);
      setIsSettingPose(false);
      setPoseStart(null);
      setCurrentMousePos(null);
      setDebugInfo(`🎯 Initial pose set at (${poseStart.rosX.toFixed(2)}, ${poseStart.rosY.toFixed(2)}) - ${Math.round(orientation)}°`);
      return true; // Indicate pose was drawn
    }
    
    return false; // No pose interaction
  };

  const handleClearInitialPose = () => {
    setInitialPose(null);
    localStorage.removeItem("savedInitialPose");
    setDebugInfo("🗑️ Initial pose cleared");
  };

  const rotatePoseOrientation = (degrees) => {
    if (!initialPose) return;
    
    const newOrientation = (initialPose.orientation + degrees) % 360;
    const updatedPose = {
      ...initialPose,
      orientation: newOrientation
    };
    
    setInitialPose(updatedPose);
    setDebugInfo(`🎯 Pose orientation: ${newOrientation.toFixed(0)}°`);
  };

  // Export pose as JSON
  const exportPose = () => {
    if (!initialPose) {
      alert("No initial pose set!");
      return;
    }
    
    const poseData = {
      pose: {
        position: {
          x: initialPose.rosX,
          y: initialPose.rosY,
          z: 0
        },
        orientation: {
          x: 0,
          y: 0,
          z: Math.sin(initialPose.orientation * Math.PI / 360),
          w: Math.cos(initialPose.orientation * Math.PI / 360)
        }
      },
      mapInfo: {
        name: pngMapInfo ? pngMapInfo.image : (mapMetadata ? mapMetadata.image : "unknown"),
        resolution: mapParamsRef.current?.resolution || 0.05,
        origin: [mapParamsRef.current?.originX || 0, mapParamsRef.current?.originY || 0, 0]
      },
      timestamp: new Date().toISOString()
    };
    
    const blob = new Blob([JSON.stringify(poseData, null, 2)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "initial_pose.json";
    a.click();
    URL.revokeObjectURL(url);
  };

  // --- Mouse Event Handlers ---
  const handleMouseDown = (e) => {
    const rect = canvasRef.current.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    
    // First check if we're interacting with a pose
    const poseWasClicked = handleMouseDownForPose(e.clientX, e.clientY);
    
    // Only start panning if we're NOT interacting with a pose AND we're NOT in setting pose mode
    if (!poseWasClicked && !isSettingPose) {
      setZoomState(prev => ({
        ...prev,
        isDragging: true,
        lastX: x,
        lastY: y
      }));
    }
  };

  const handleMouseMove = (e) => {
    // Always update pose movement if we're drawing or dragging
    if (isDrawingPose || isDraggingPose) {
      handleMouseMoveForPose(e.clientX, e.clientY);
      return;
    }
    
    // Only handle map panning if we're dragging the map
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

  const handleMouseUp = (e) => {
    // First check if we're finishing a pose interaction
    const poseWasInteracted = handleMouseUpForPose(e.clientX, e.clientY);
    
    // Only stop map dragging if we weren't interacting with a pose
    if (!poseWasInteracted) {
      setZoomState(prev => ({
        ...prev,
        isDragging: false
      }));
    }
  };

  const handleMouseLeave = () => {
    // Cancel pose drawing if mouse leaves canvas
    if (isDrawingPose) {
      cancelSettingPose();
    }
    
    if (isDraggingPose) {
      setIsDraggingPose(false);
    }
    
    setCurrentMousePos(null);
    setZoomState(prev => ({ ...prev, isDragging: false }));
  };

  // --- Touch Event Handlers ---
  const handleTouchStart = (e) => {
    if (e.touches.length === 1) {
      const touch = e.touches[0];
      const rect = canvasRef.current.getBoundingClientRect();
      const x = touch.clientX - rect.left;
      const y = touch.clientY - rect.top;
      
      // Check if we're interacting with a pose
      const poseWasClicked = handleMouseDownForPose(touch.clientX, touch.clientY);
      
      // Only start panning if we're NOT interacting with a pose AND we're NOT in setting pose mode
      if (!poseWasClicked && !isSettingPose) {
        setZoomState(prev => ({
          ...prev,
          isDragging: true,
          lastX: x,
          lastY: y
        }));
      }
    }
    e.preventDefault();
  };

  const handleTouchMove = (e) => {
    if (e.touches.length === 1) {
      const touch = e.touches[0];
      
      // Always update pose movement if we're drawing or dragging
      if (isDrawingPose || isDraggingPose) {
        handleMouseMoveForPose(touch.clientX, touch.clientY);
      } else if (zoomState.isDragging) {
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
    }
    e.preventDefault();
  };

  const handleTouchEnd = (e) => {
    if (e.changedTouches.length === 1) {
      const touch = e.changedTouches[0];
      
      // Check if we're finishing a pose interaction
      const poseWasInteracted = handleMouseUpForPose(touch.clientX, touch.clientY);
      
      // Only stop map dragging if we weren't interacting with a pose
      if (!poseWasInteracted) {
        setZoomState(prev => ({
          ...prev,
          isDragging: false
        }));
      }
    }
    e.preventDefault();
  };

  // --- Zoom Handlers ---
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

  // --- Drawing the map and initial pose ---
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
    
    // If we have a PNG image loaded
    if (pngImage) {
      const { width, height } = pngImage;
      
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation to the map
      if (rotation !== 0) {
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      // Draw the PNG image (rotated)
      ctx.drawImage(pngImage, 0, 0, width, height);
      
      ctx.restore();
    }
    
    // If we have a PGM image loaded
    if (pgmImage) {
      const { width, height } = pgmImage;
      
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation to the map
      if (rotation !== 0) {
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      ctx.drawImage(pgmImage, 0, 0, width, height);
      
      ctx.restore();
    }

    // Original ROS map rendering
    const mapToRender = editableMap || mapMsg;
    if (!mapToRender && !pngImage && !pgmImage) {
      ctx.fillStyle = '#f8f9fa';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = '#6c757d';
      ctx.font = isMobile ? '14px Arial' : '16px Arial';
      ctx.textAlign = 'center';
      ctx.fillText('Load a PGM map or PNG image to set initial pose', canvas.width / 2, canvas.height / 2);
      return;
    }

    if (mapToRender) {
      const { width, height, data } = mapToRender;

      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation to the ROS map
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
      
      ctx.restore();
    }

    // Draw pose being drawn (if in progress)
    if (isDrawingPose && poseStart && currentMousePos) {
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation to the pose
      if (rotation !== 0) {
        const width = pngImage ? pngImage.width : (pgmImage ? pgmImage.width : (mapMsg ? mapMsg.width : 0));
        const height = pngImage ? pngImage.height : (pgmImage ? pgmImage.height : (mapMsg ? mapMsg.height : 0));
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      const { canvasX, canvasY } = poseStart;
      const { x: mouseX, y: mouseY } = currentMousePos;
      
      // Calculate orientation
      const dx = mouseX - canvasX;
      const dy = mouseY - canvasY;
      const distance = Math.sqrt(dx * dx + dy * dy);
      const orientation = Math.atan2(dy, dx);
      
      // Calculate arrow end point - use actual distance to cursor
      const arrowEndX = canvasX + distance * Math.cos(orientation);
      const arrowEndY = canvasY + distance * Math.sin(orientation);
      const arrowHeadLength = 20 / zoomState.scale;
      const arrowWidth = 6 / zoomState.scale;
      
      // Create gradient for the arrow (same as final arrow)
      const gradient = ctx.createLinearGradient(canvasX, canvasY, arrowEndX, arrowEndY);
      gradient.addColorStop(0, '#007bff'); // Blue at start
      gradient.addColorStop(1, '#1d4ed8'); // Dark blue at end
      
      // Draw arrow line (solid blue line - same as final arrow)
      ctx.shadowColor = 'rgba(0, 123, 255, 0.3)';
      ctx.shadowBlur = 8;
      ctx.shadowOffsetX = 2;
      ctx.shadowOffsetY = 2;
      
      ctx.strokeStyle = gradient;
      ctx.lineWidth = arrowWidth;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(canvasX, canvasY);
      ctx.lineTo(arrowEndX, arrowEndY);
      ctx.stroke();
      
      // Draw arrow head
      ctx.fillStyle = '#1d4ed8';
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowEndY);
      ctx.lineTo(
        arrowEndX - arrowHeadLength * Math.cos(orientation - Math.PI / 6),
        arrowEndY - arrowHeadLength * Math.sin(orientation - Math.PI / 6)
      );
      ctx.lineTo(
        arrowEndX - arrowHeadLength * Math.cos(orientation + Math.PI / 6),
        arrowEndY - arrowHeadLength * Math.sin(orientation + Math.PI / 6)
      );
      ctx.closePath();
      ctx.fill();
      
      ctx.shadowColor = 'transparent';
      ctx.shadowBlur = 0;
      ctx.shadowOffsetX = 0;
      ctx.shadowOffsetY = 0;
      
     
      
      // Draw coordinates label
      const angleDegrees = orientation * 180 / Math.PI;
      const snappedAngle = Math.round(angleDegrees);
      const rosEnd = canvasToRosCoords(mouseX, mouseY, true);
      
      if (rosEnd) {
        const distanceMeters = Math.sqrt(
          Math.pow(rosEnd.x - poseStart.rosX, 2) + 
          Math.pow(rosEnd.y - poseStart.rosY, 2)
        );
        
        // Draw angle and distance label
        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)';
        ctx.font = `${Math.max(10, 12 / zoomState.scale)}px Arial`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        
        // Position label above the arrow midpoint
        const midX = (canvasX + arrowEndX) / 2;
        const midY = (canvasY + arrowEndY) / 2;
        
        // Calculate perpendicular offset for label
        const perpendicularX = -Math.sin(orientation) * 20 / zoomState.scale;
        const perpendicularY = Math.cos(orientation) * 20 / zoomState.scale;
        
        ctx.fillText(
          `${snappedAngle}° • ${distanceMeters.toFixed(2)} m`, 
          midX + perpendicularX, 
          midY + perpendicularY
        );
      }
      
      // If SHIFT key is pressed (snapping to 45°), show visual feedback
      if (window.event?.shiftKey) {
        // Draw snap indicator
        ctx.fillStyle = 'rgba(245, 158, 11, 0.8)';
        ctx.font = `${Math.max(8, 10 / zoomState.scale)}px Arial`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(
          'SNAP: 45°', 
          canvasX, 
          canvasY - 25 / zoomState.scale
        );
      }
      
      ctx.restore();
    }

    // Draw existing initial pose
    if (initialPose && !isDrawingPose) {
      ctx.save();
      ctx.translate(zoomState.offsetX, zoomState.offsetY);
      ctx.scale(zoomState.scale, zoomState.scale);

      // Apply rotation transformation to the pose
      if (rotation !== 0) {
        const width = pngImage ? pngImage.width : (pgmImage ? pgmImage.width : (mapMsg ? mapMsg.width : 0));
        const height = pngImage ? pngImage.height : (pgmImage ? pgmImage.height : (mapMsg ? mapMsg.height : 0));
        const centerX = width / 2;
        const centerY = height / 2;
        ctx.translate(centerX, centerY);
        ctx.rotate((rotation * Math.PI) / 180);
        ctx.translate(-centerX, -centerY);
      }

      const { canvasX, canvasY, orientation } = initialPose;
      const arrowLength = 50 / zoomState.scale;
      const arrowHeadLength = 20 / zoomState.scale;
      const arrowWidth = 6 / zoomState.scale;

      // Calculate arrow end point
      const arrowEndX = canvasX + arrowLength * Math.cos(orientation * Math.PI / 180);
      const arrowEndY = canvasY + arrowLength * Math.sin(orientation * Math.PI / 180);

      // Create gradient for the arrow
      const gradient = ctx.createLinearGradient(canvasX, canvasY, arrowEndX, arrowEndY);
      gradient.addColorStop(0, '#007bff');
      gradient.addColorStop(1, '#1d4ed8');

      // Draw arrow line with shadow
      ctx.shadowColor = 'rgba(0, 123, 255, 0.5)';
      ctx.shadowBlur = 8;
      ctx.shadowOffsetX = 2;
      ctx.shadowOffsetY = 2;
      
      ctx.strokeStyle = gradient;
      ctx.lineWidth = arrowWidth;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(canvasX, canvasY);
      ctx.lineTo(arrowEndX, arrowEndY);
      ctx.stroke();
      
      // Draw arrow head
      ctx.fillStyle = '#1d4ed8';
      ctx.beginPath();
      ctx.moveTo(arrowEndX, arrowEndY);
      ctx.lineTo(
        arrowEndX - arrowHeadLength * Math.cos((orientation * Math.PI / 180) - Math.PI / 6),
        arrowEndY - arrowHeadLength * Math.sin((orientation * Math.PI / 180) - Math.PI / 6)
      );
      ctx.lineTo(
        arrowEndX - arrowHeadLength * Math.cos((orientation * Math.PI / 180) + Math.PI / 6),
        arrowEndY - arrowHeadLength * Math.sin((orientation * Math.PI / 180) + Math.PI / 6)
      );
      ctx.closePath();
      ctx.fill();
      
      ctx.shadowColor = 'transparent';
      ctx.shadowBlur = 0;
      ctx.shadowOffsetX = 0;
      ctx.shadowOffsetY = 0;
     
      
      // Draw coordinates label
      const label = `(${initialPose.rosX.toFixed(2)}, ${initialPose.rosY.toFixed(2)}) - ${Math.round(initialPose.orientation)}°`;
      ctx.fillStyle = 'rgba(0, 0, 0, 0.7)';
      ctx.font = `${Math.max(10, 12 / zoomState.scale)}px Arial`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, canvasX, canvasY - 20 / zoomState.scale);
      
      ctx.restore();
    }
  }, [mapMsg, editableMap, zoomState, isMobile, pgmImage, pngImage, rotation, initialPose, isDrawingPose, poseStart, currentMousePos]);

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

      {/* Header */}
      <div style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "space-between",
        padding: isMobile ? "8px 12px" : "12px 16px",
        background: "linear-gradient(135deg, #3b82f6, #1d4ed8)",
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
          <FaMapMarkerAlt />
          <span style={{ fontSize: isMobile ? "1rem" : "1.2rem" }}>Map View - Initial Pose Setter</span>
        </div>
        
        {/* Pose Control Buttons */}
        <div style={{ 
          display: "flex", 
          gap: "8px",
          alignItems: "center",
          flex: isMobile ? "1 1 100%" : "0 1 auto",
          marginTop: isMobile ? "4px" : "0",
          justifyContent: isMobile ? "center" : "flex-end"
        }}>
          {!isSettingPose && !initialPose && (
            <button
              onClick={startSettingPose}
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
              title="Set Initial Pose"
              disabled={!mapLoadedFromFile && !mapMsg}
            >
              <FaCrosshairs />
              {!isMobile && "Set Pose"}
            </button>
          )}

          {isSettingPose && (
            <button
              onClick={cancelSettingPose}
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
              title="Cancel Setting Pose"
            >
              <FaMousePointer />
              {!isMobile && "Cancel"}
            </button>
          )}

          {initialPose && (
            <>
              <button
                onClick={exportPose}
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
                title="Export Pose as JSON"
              >
                <FaDownload />
                {!isMobile && "Export Pose"}
              </button>

              <button
                onClick={() => rotatePoseOrientation(45)}
                style={{
                  padding: isMobile ? "6px 8px" : "8px 12px",
                  background: "#f59e0b",
                  color: "white",
                  border: "none",
                  borderRadius: "6px",
                  cursor: "pointer",
                  fontSize: isMobile ? "10px" : "12px",
                  display: "flex",
                  alignItems: "center",
                  gap: "4px"
                }}
                title="Rotate Pose +45°"
              >
                <FaRedo />
                {!isMobile && "+45°"}
              </button>

              <button
                onClick={handleClearInitialPose}
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
                title="Clear Initial Pose"
              >
                <FaTrash />
                {!isMobile && "Clear"}
              </button>
            </>
          )}
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
              {!isMobile && "Remove Map"}
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

      {/* Instructions */}
      {(isSettingPose || initialPose) && (
        <div style={{
          padding: "10px 15px",
          background: isSettingPose ? "#fef3c7" : "#dbeafe",
          borderRadius: "6px",
          marginBottom: "10px",
          border: `1px solid ${isSettingPose ? "#f59e0b" : "#3b82f6"}`,
          fontSize: isMobile ? "0.8rem" : "0.9rem",
          color: isSettingPose ? "#92400e" : "#1e40af"
        }}>
          {isSettingPose ? (
            <div>
              <div>🎯 <strong>Step 1:</strong> Click to set START point (blue dot)</div>
              <div>🎯 <strong>Step 2:</strong> DRAG to set direction (blue arrow appears)</div>
              <div>🔄 Hold <strong>SHIFT</strong> to snap to 45° increments</div>
              <div>✅ <strong>Release mouse</strong> to finish</div>
            </div>
          ) : (
            <div>
              <div>🎯 <strong>Click and drag the BLUE ARROW</strong> to move the pose</div>
              <div>🔄 <strong>Use +45° button</strong> to rotate the pose orientation</div>
              <div>📁 <strong>Export Pose</strong> to save as JSON file</div>
            </div>
          )}
        </div>
      )}

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
          cursor: isSettingPose ? "crosshair" : (zoomState.isDragging ? "grabbing" : (isDraggingPose ? "move" : "grab")),
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
                const mapWidth = pngImage ? pngImage.width : (pgmImage ? pgmImage.width : (mapMsg ? mapMsg.width : 0));
                const mapHeight = pngImage ? pngImage.height : (pgmImage ? pgmImage.height : (mapMsg ? mapMsg.height : 0));
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
              title="Rotate Map 90°"
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
              title="Reset Map Rotation"
              disabled={rotation === 0}
            >
              {rotation}°
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
            onMouseLeave={!isMobile ? handleMouseLeave : undefined}
            onWheel={!isMobile ? handleWheel : undefined}
            onTouchStart={isMobile ? handleTouchStart : undefined}
            onTouchMove={isMobile ? handleTouchMove : undefined}
            onTouchEnd={isMobile ? handleTouchEnd : undefined}
            onTouchCancel={isMobile ? handleTouchEnd : undefined}
          />
        </div>
      </div>

      {/* Pose Info Panel */}
      {initialPose && (
        <div style={{
          marginTop: "10px",
          padding: "15px",
          background: "#f8fafc",
          borderRadius: "8px",
          border: "1px solid #e2e8f0",
          boxShadow: "0 2px 4px rgba(0,0,0,0.05)"
        }}>
          <h3 style={{ margin: "0 0 10px 0", color: "#1e293b", fontSize: isMobile ? "1rem" : "1.1rem" }}>
            <FaMapMarkerAlt style={{ marginRight: "8px", color: "#007bff" }} />
            Initial Pose Configuration
          </h3>
          
          <div style={{ 
            display: "grid", 
            gridTemplateColumns: isMobile ? "1fr" : "repeat(3, 1fr)", 
            gap: "15px",
            marginBottom: "15px"
          }}>
            <div style={{ background: "white", padding: "10px", borderRadius: "6px", border: "1px solid #e2e8f0" }}>
              <div style={{ fontSize: "0.8rem", color: "#64748b", marginBottom: "4px" }}>ROS X Position</div>
              <div style={{ fontSize: "1.2rem", fontWeight: "bold", color: "#1e293b" }}>{initialPose.rosX.toFixed(3)} m</div>
            </div>
            
            <div style={{ background: "white", padding: "10px", borderRadius: "6px", border: "1px solid #e2e8f0" }}>
              <div style={{ fontSize: "0.8rem", color: "#64748b", marginBottom: "4px" }}>ROS Y Position</div>
              <div style={{ fontSize: "1.2rem", fontWeight: "bold", color: "#1e293b" }}>{initialPose.rosY.toFixed(3)} m</div>
            </div>
            
            <div style={{ background: "white", padding: "10px", borderRadius: "6px", border: "1px solid #e2e8f0" }}>
              <div style={{ fontSize: "0.8rem", color: "#64748b", marginBottom: "4px" }}>Orientation</div>
              <div style={{ fontSize: "1.2rem", fontWeight: "bold", color: "#1e293b" }}>{Math.round(initialPose.orientation)}°</div>
            </div>
          </div>
          
          <div style={{ display: "flex", gap: "10px", justifyContent: "flex-end" }}>
            <button
              onClick={exportPose}
              style={{
                padding: "10px 15px",
                background: "#8b5cf6",
                color: "white",
                border: "none",
                borderRadius: "6px",
                cursor: "pointer",
                fontSize: isMobile ? "0.9rem" : "1rem",
                display: "flex",
                alignItems: "center",
                gap: "6px"
              }}
            >
              <FaDownload />
              Export Pose as JSON
            </button>
            
            <button
              onClick={handleClearInitialPose}
              style={{
                padding: "10px 15px",
                background: "#ef4444",
                color: "white",
                border: "none",
                borderRadius: "6px",
                cursor: "pointer",
                fontSize: isMobile ? "0.9rem" : "1rem",
                display: "flex",
                alignItems: "center",
                gap: "6px"
              }}
            >
              <FaTrash />
              Clear Pose
            </button>
          </div>
        </div>
      )}
    </div>
  );
}