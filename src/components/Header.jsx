// import React, { useState, useRef, useEffect } from "react";
// import { Link, useNavigate } from "react-router-dom";
// import { FiBell, FiUser, FiLogOut, FiHome, FiActivity, FiClipboard, FiHeart, FiBarChart2, FiSettings } from "react-icons/fi";
// import TaikishaLogo from "../assets/Taikishaimage1.png";

// export default function Header() {
//   const [menuOpen, setMenuOpen] = useState(false);
//   const [userDropdownOpen, setUserDropdownOpen] = useState(false);
//   const navigate = useNavigate();
//   const dropdownRef = useRef(null);

//   const loggedInUser = localStorage.getItem("loggedInUser") || "Guest";

//   const handleLogout = () => {
//     localStorage.removeItem("loggedInUser");
//     navigate("/animation", { replace: true });
//   };

//   // Close dropdown on outside click
//   useEffect(() => {
//     const handleClickOutside = (event) => {
//       if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
//         setUserDropdownOpen(false);
//       }
//     };
//     document.addEventListener("mousedown", handleClickOutside);
//     return () => document.removeEventListener("mousedown", handleClickOutside);
//   }, []);

//   return (
//     <header className="w-full bg-gradient-to-r from-sky-100 to-sky-700 text-white shadow px-4 py-3 flex flex-col sm:flex-row items-center sm:justify-between relative">
//       {/* Left: Logo & Title */}
//       <div className="flex items-center gap-3 flex-1 min-w-0">
//         <button
//           className="lg:hidden p-2 rounded-md border bg-sky-600 text-white flex-shrink-0"
//           onClick={() => setMenuOpen(!menuOpen)}
//         >
//           ☰
//         </button>

//         <div className="flex flex-col sm:flex-row items-center gap-2">
//          <img
//   src={TaikishaLogo}
//   alt="Taikisha Logo"
//   className="h-10 sm:h-12 w-auto object-contain flex-shrink-0 select-none"
// />

//           <h1 className="text-sm sm:text-base md:text-lg lg:text-xl font-bold text-sky-100 text-center sm:text-left">
//             {/* Optional title */}
//           </h1>
//         </div>
//       </div>

//       {/* Right: Actions */}
//       <div className="flex items-center gap-3 mt-2 sm:mt-0 relative" ref={dropdownRef}>
//         <button className="px-2 sm:px-3 py-2 bg-slate-100 text-black rounded shadow flex items-center justify-center hover:bg-slate-200 transition">
//           <FiBell className="text-lg sm:text-xl" />
//         </button>

//         {/* User Dropdown */}
//         <div className="relative">
//           <button
//             onClick={() => setUserDropdownOpen(!userDropdownOpen)}
//             className="px-2 sm:px-3 py-2 bg-slate-100 text-black rounded shadow flex items-center justify-center hover:bg-slate-200 transition"
//           >
//             <FiUser className="text-lg sm:text-xl" />
//           </button>

//           {userDropdownOpen && (
//             <div className="absolute right-0 mt-2 w-40 sm:w-44 bg-white text-black rounded shadow-lg p-2 z-50">
//               <p className="py-1 px-2 text-sm font-medium border-b truncate">{loggedInUser}</p>
//               <button
//                 onClick={handleLogout}
//                 className="w-full text-left py-1 px-2 text-sm hover:bg-red-100 text-red-600 rounded mt-1 flex items-center gap-1"
//               >
//                 <FiLogOut /> Logout
//               </button>
//             </div>
//           )}
//         </div>
//       </div>

//       {/* Mobile Menu */}
//       {menuOpen && (
//         <div className="absolute top-24 left-0 w-full bg-white shadow-lg rounded-b-lg p-4 lg:hidden z-50">
//           <nav className="space-y-2 text-gray-800">
//             {[
//               { to: "/", label: "Dashboard", icon: FiHome },
//                 { to: "/maps", label: "Maps", icon: FiBarChart2 }, // 🗺️ new line
//               { to: "/realtime", label: "Real-Time Visualisation", icon: FiActivity },
//               { to: "/tasks", label: "Task Allocation", icon: FiClipboard },
//               { to: "/health", label: "Health Check", icon: FiHeart },
//               { to: "/reports", label: "Reports", icon: FiBarChart2 },
//               { to: "/settings", label: "Settings", icon: FiSettings },
//             ].map(({ to, label, icon: Icon }) => (
//               <Link
//                 key={to}
//                 to={to}
//                 className="flex items-center gap-2 py-2 px-3 rounded hover:bg-sky-100 transition"
//                 onClick={() => setMenuOpen(false)}
//               >
//                 <Icon className="text-base sm:text-lg" /> {label}
//               </Link>
//             ))}
//           </nav>
//         </div>
//       )}
//     </header>
//   );
// }


import React, { useState, useRef, useEffect } from "react";
import { Link, useNavigate } from "react-router-dom";
import ROSLIB from "roslib";
import { FiBell, FiUser, FiLogOut, FiHome, FiActivity, FiClipboard, FiHeart, FiBarChart2, FiSettings, FiCheck, FiX, FiSearch, FiRefreshCw, FiWifi, FiWifiOff } from "react-icons/fi";
import TaikishaLogo from "../assets/Taikishaimage1.png";

export default function Header() {
  const [menuOpen, setMenuOpen] = useState(false);
  const [userDropdownOpen, setUserDropdownOpen] = useState(false);
  const [agvDropdownOpen, setAgvDropdownOpen] = useState(false);
  const [selectedAgvs, setSelectedAgvs] = useState([]);
  const [searchQuery, setSearchQuery] = useState("");
  const [agvList, setAgvList] = useState([
    {
      id: "amr-01",
      name: "AMR-01",
      status: "offline",
      battery: 0,
      task: "N/A",
      error: "NONE",
      camera: "FAULT",
      lidar: "FAULT",
      lastUpdate: null,
      position: { x: null, y: null },
      wifiConnected: false,
      lastDataReceived: null
    }
  ]);
  const [rosConnected, setRosConnected] = useState(false);
  const [loading, setLoading] = useState(false);
  const [backendConnected, setBackendConnected] = useState(false);
  const [lastUpdate, setLastUpdate] = useState(null);
  const [wifiConnected, setWifiConnected] = useState(false);
  
  const navigate = useNavigate();
  const dropdownRef = useRef(null);
  const agvDropdownRef = useRef(null);
  const rosRef = useRef(null);
  const poseListenerRef = useRef(null);
  const fetchIntervalRef = useRef(null);
  const dataTimeoutRef = useRef(null);

  const loggedInUser = localStorage.getItem("loggedInUser") || "Guest";
  const DATA_TIMEOUT_MS = 5000; // 5 seconds timeout - if no data for 5s, robot is offline

  // Check if robot data is stale (not received in last 5 seconds)
  const isDataStale = (lastDataTime) => {
    if (!lastDataTime) return true;
    return Date.now() - lastDataTime > DATA_TIMEOUT_MS;
  };

  // Fetch AGV status from backend
  useEffect(() => {
    let lastSuccessfulFetch = null;

    const fetchAgvStatus = async () => {
      try {
        const startTime = Date.now();
        const controller = new AbortController();
        const timeoutId = setTimeout(() => controller.abort(), 3000); // 3 second timeout

        const res = await fetch("http://localhost:5000/robot_position", {
          signal: controller.signal
        });
        clearTimeout(timeoutId);

        if (!res.ok) {
          throw new Error(`HTTP ${res.status}`);
        }

        const data = await res.json();
        const fetchDuration = Date.now() - startTime;
        
        // Check if this is fresh data or cached/stale data
        const isFreshData = fetchDuration < 1000; // If fetch took less than 1s, it's likely fresh
        
        setBackendConnected(true);
        
        // Determine robot status based on actual connection
        let robotStatus = "offline";
        let isWifiConnected = false;

        if (data.status === "success" && data.data && data.data.message_count > 0) {
          // We have data from robot
          lastSuccessfulFetch = Date.now();
          isWifiConnected = true;
          
          // Check if data is recent (within 5 seconds)
          const dataAge = Date.now() - (data.data.timestamp * 1000);
          const isRecentData = dataAge < DATA_TIMEOUT_MS;
          
          robotStatus = isRecentData ? "online" : "offline";
          
          // Update WiFi status
          setWifiConnected(true);
        } else if (data.status === "waiting") {
          // No data from robot
          robotStatus = "offline";
          isWifiConnected = false;
          setWifiConnected(false);
        } else {
          // Error or no data
          robotStatus = "offline";
          isWifiConnected = false;
          setWifiConnected(false);
        }

        // Update AGV list
        setAgvList([
          {
            id: "amr-01",
            name: "AMR-01",
            status: robotStatus,
            battery: typeof data.data?.battery === "number" ? data.data.battery : 0,
            task: data.robot_status === "RUNNING" ? "Running" : "Idle",
            error: data.error_status || "NONE",
            camera: data.camera_status || "FAULT",
            lidar: data.lidar_status || "FAULT",
            position: data.data?.x !== undefined ? { 
              x: data.data.x, 
              y: data.data.y 
            } : { x: null, y: null },
            wifiConnected: isWifiConnected,
            lastDataReceived: isWifiConnected ? Date.now() : null,
            lastUpdate: new Date().toISOString()
          }
        ]);
        
        setLastUpdate(new Date());

      } catch (error) {
        console.error("Error fetching AGV status:", error);
        setBackendConnected(false);
        
        // Mark robot as offline
        setWifiConnected(false);
        setAgvList(prev => prev.map(agv => ({
          ...agv,
          status: "offline",
          error: "WIFI_DISCONNECTED",
          wifiConnected: false,
          position: { x: null, y: null },
          lastUpdate: new Date().toISOString()
        })));
      }
    };

    fetchAgvStatus();
    
    // Set up interval for periodic updates (every 2 seconds)
    fetchIntervalRef.current = setInterval(fetchAgvStatus, 2000);

    // Cleanup timeout for data staleness
    dataTimeoutRef.current = setInterval(() => {
      setAgvList(prev => prev.map(agv => {
        if (agv.status === "online" && agv.lastDataReceived) {
          const isStale = isDataStale(agv.lastDataReceived);
          if (isStale) {
            return {
              ...agv,
              status: "offline",
              wifiConnected: false,
              error: "DATA_TIMEOUT",
              task: "N/A"
            };
          }
        }
        return agv;
      }));
    }, 1000);

    return () => {
      if (fetchIntervalRef.current) {
        clearInterval(fetchIntervalRef.current);
      }
      if (dataTimeoutRef.current) {
        clearInterval(dataTimeoutRef.current);
      }
    };
  }, []);

  // Manual refresh function
  const handleRefresh = () => {
    setLoading(true);
    
    // Clear any cached data
    setAgvList(prev => prev.map(agv => ({
      ...agv,
      status: "offline",
      wifiConnected: false,
      position: { x: null, y: null },
      lastDataReceived: null
    })));
    
    // Force fresh fetch
    fetch("http://localhost:5000/robot_position")
      .then(res => res.json())
      .then(data => {
        let robotStatus = "offline";
        let isWifiConnected = false;

        if (data.status === "success" && data.data && data.data.message_count > 0) {
          isWifiConnected = true;
          const dataAge = Date.now() - (data.data.timestamp * 1000);
          const isRecentData = dataAge < DATA_TIMEOUT_MS;
          robotStatus = isRecentData ? "online" : "offline";
          setWifiConnected(true);
        } else {
          setWifiConnected(false);
        }

        setAgvList([
          {
            id: "amr-01",
            name: "AMR-01",
            status: robotStatus,
            battery: typeof data.data?.battery === "number" ? data.data.battery : 0,
            task: data.robot_status === "RUNNING" ? "Running" : "Idle",
            error: data.error_status || "NONE",
            camera: data.camera_status || "FAULT",
            lidar: data.lidar_status || "FAULT",
            position: data.data?.x !== undefined ? { 
              x: data.data.x, 
              y: data.data.y 
            } : { x: null, y: null },
            wifiConnected: isWifiConnected,
            lastDataReceived: isWifiConnected ? Date.now() : null,
            lastUpdate: new Date().toISOString()
          }
        ]);
        setBackendConnected(true);
      })
      .catch(() => {
        setBackendConnected(false);
        setWifiConnected(false);
        setAgvList(prev => prev.map(agv => ({
          ...agv,
          status: "offline",
          wifiConnected: false,
          error: "CONNECTION_FAILED",
          position: { x: null, y: null }
        })));
      })
      .finally(() => setLoading(false));
  };

  // Filter AGVs based on search query
  const filteredAgvs = agvList.filter(agv =>
    agv.name.toLowerCase().includes(searchQuery.toLowerCase()) ||
    agv.id.toLowerCase().includes(searchQuery.toLowerCase())
  );

  const handleLogout = () => {
    localStorage.removeItem("loggedInUser");
    navigate("/animation", { replace: true });
  };

  const toggleAgvSelection = (agvId) => {
    setSelectedAgvs(prev => 
      prev.includes(agvId) 
        ? prev.filter(id => id !== agvId)
        : [...prev, agvId]
    );
  };

  const selectAllAgvs = () => {
    const allOnlineAgvs = filteredAgvs.filter(agv => agv.status === "online").map(agv => agv.id);
    setSelectedAgvs(allOnlineAgvs);
  };

  const clearSelection = () => {
    setSelectedAgvs([]);
  };

  // Close dropdowns on outside click
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
        setUserDropdownOpen(false);
      }
      if (agvDropdownRef.current && !agvDropdownRef.current.contains(event.target)) {
        setAgvDropdownOpen(false);
      }
    };
    document.addEventListener("mousedown", handleClickOutside);
    return () => document.removeEventListener("mousedown", handleClickOutside);
  }, []);

  const getStatusColor = (status) => {
    switch (status) {
      case "online": return "bg-green-500";
      case "offline": return "bg-red-500";
      case "maintenance": return "bg-yellow-500";
      default: return "bg-gray-500";
    }
  };

  const getStatusDisplay = (status) => {
    const statusMap = {
      'online': 'Online',
      'offline': 'Offline',
      'maintenance': 'Maintenance'
    };
    return statusMap[status] || 'Offline';
  };

  const getBatteryColor = (battery) => {
    if (battery >= 70) return "bg-green-500";
    if (battery >= 30) return "bg-yellow-500";
    return "bg-red-500";
  };

  const getBatteryTextColor = (battery) => {
    if (battery >= 70) return "text-green-600";
    if (battery >= 30) return "text-yellow-600";
    return "text-red-600";
  };

  // Count online AGVs
  const onlineCount = agvList.filter(agv => agv.status === "online").length;
  const totalCount = agvList.length;

  return (
    <header className="w-full bg-gradient-to-r from-sky-100 to-sky-700 text-white shadow px-4 py-3 flex flex-col sm:flex-row items-center sm:justify-between relative">
      {/* Left: Logo & Title */}
      <div className="flex items-center gap-3 flex-1 min-w-0">
        <button
          className="lg:hidden p-2 rounded-md border bg-sky-600 text-white flex-shrink-0"
          onClick={() => setMenuOpen(!menuOpen)}
        >
          ☰
        </button>

        <div className="flex flex-col sm:flex-row items-center gap-3">
          <img
            src={TaikishaLogo}
            alt="Taikisha Logo"
            className="h-10 sm:h-12 w-auto object-contain flex-shrink-0 select-none"
          />
          <div className="flex flex-col">
           
          </div>
        </div>
      </div>

      {/* Right: Actions */}
      <div className="flex items-center gap-2 sm:gap-3 mt-2 sm:mt-0">
        {/* Refresh Button */}
        <button
          onClick={handleRefresh}
          disabled={loading}
          className="px-2 py-2 bg-slate-100 text-black rounded-lg shadow flex items-center justify-center hover:bg-slate-200 transition disabled:opacity-50"
          title="Refresh Robot Status"
        >
          <FiRefreshCw className={`text-base ${loading ? 'animate-spin' : ''}`} />
        </button>

        {/* AGV Selection Dropdown */}
        <div className="relative" ref={agvDropdownRef}>
          <button
            onClick={() => setAgvDropdownOpen(!agvDropdownOpen)}
            className="px-2 sm:px-3 py-2 bg-gradient-to-r from-purple-500 to-purple-600 text-white rounded-lg shadow-lg flex items-center gap-1 sm:gap-2 hover:from-purple-600 hover:to-purple-700 transition-all duration-200 transform hover:scale-105 text-sm sm:text-base"
          >
            <FiActivity className="text-base sm:text-lg" />
            <span className="hidden xs:inline">AGVs</span>
            <span className={`text-xs font-bold px-1.5 sm:px-2 py-0.5 sm:py-1 rounded-full min-w-5 sm:min-w-6 ${
              onlineCount > 0 ? 'bg-white text-purple-600' : 'bg-red-500 text-white'
            }`}>
              {onlineCount}/{totalCount}
            </span>
          </button>

          {agvDropdownOpen && (
            <div className="absolute right-0 mt-2 w-screen max-w-xs sm:max-w-sm md:max-w-md lg:max-w-lg xl:max-w-xl bg-white text-black rounded-xl shadow-2xl p-3 sm:p-4 z-50 border border-purple-100 transform transition-all duration-200">
              {/* Header */}
              <div className="flex flex-col sm:flex-row sm:justify-between sm:items-center gap-3 mb-3 sm:mb-4 pb-3 border-b border-gray-200">
                <div>
                  <h3 className="font-bold text-lg text-gray-800">Robot Status</h3>
                  <div className="flex items-center gap-2 text-xs text-gray-500">
                    <span>Updated: {lastUpdate ? lastUpdate.toLocaleTimeString() : 'Never'}</span>
                    <div className={`flex items-center gap-1 ${wifiConnected ? 'text-green-600' : 'text-red-600'}`}>
                      {wifiConnected ? <FiWifi /> : <FiWifiOff />}
                      <span>{wifiConnected ? 'WiFi Connected' : 'WiFi Disconnected'}</span>
                    </div>
                  </div>
                </div>
                <div className="flex gap-2 justify-center sm:justify-start">
                  <button
                    onClick={selectAllAgvs}
                    className="px-2 sm:px-3 py-1.5 bg-green-500 text-white text-xs rounded-lg hover:bg-green-600 transition flex items-center gap-1"
                    disabled={onlineCount === 0}
                  >
                    <FiCheck className="text-xs" />
                    Select Online
                  </button>
                  <button
                    onClick={clearSelection}
                    className="px-2 sm:px-3 py-1.5 bg-red-500 text-white text-xs rounded-lg hover:bg-red-600 transition flex items-center gap-1"
                  >
                    <FiX className="text-xs" />
                    Clear
                  </button>
                </div>
              </div>

              {/* Search Bar */}
              <div className="relative mb-3 sm:mb-4">
                <FiSearch className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400 text-sm" />
                <input
                  type="text"
                  placeholder="Search AGVs..."
                  value={searchQuery}
                  onChange={(e) => setSearchQuery(e.target.value)}
                  className="w-full pl-10 pr-4 py-2.5 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-purple-500 focus:border-transparent text-sm"
                />
              </div>

              {/* AGV List */}
              <div className="max-h-48 sm:max-h-60 overflow-y-auto space-y-2 custom-scrollbar">
                {loading ? (
                  <div className="text-center py-4">
                    <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-purple-500 mx-auto"></div>
                    <p className="text-gray-500 text-sm mt-2">Checking robot connection...</p>
                  </div>
                ) : filteredAgvs.length === 0 ? (
                  <div className="text-center py-4 text-gray-500 text-sm">
                    No AGVs found matching your search
                  </div>
                ) : (
                  filteredAgvs.map((agv) => (
                    <div
                      key={agv.id}
                      onClick={() => toggleAgvSelection(agv.id)}
                      className={`p-2 sm:p-3 rounded-lg border-2 cursor-pointer transition-all duration-200 ${
                        selectedAgvs.includes(agv.id)
                          ? "border-purple-500 bg-purple-50"
                          : "border-gray-200 hover:border-purple-300 hover:bg-gray-50"
                      } ${agv.status === 'offline' ? 'opacity-60' : ''}`}
                    >
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2 sm:gap-3 flex-1 min-w-0">
                          <div className={`w-2 h-2 sm:w-3 sm:h-3 rounded-full flex-shrink-0 ${getStatusColor(agv.status)}`}></div>
                          <div className="flex flex-col">
                            <span className="font-semibold text-gray-800 text-sm sm:text-base truncate">{agv.name}</span>
                            <div className="flex items-center gap-2 mt-0.5">
                              {agv.wifiConnected ? (
                                <span className="flex items-center gap-1 text-xs text-green-600">
                                  <FiWifi className="text-xs" />
                                  WiFi
                                </span>
                              ) : (
                                <span className="flex items-center gap-1 text-xs text-red-600">
                                  <FiWifiOff className="text-xs" />
                                  No WiFi
                                </span>
                              )}
                              {agv.task && agv.task !== "N/A" && (
                                <span className="text-xs text-gray-500 truncate">
                                  {agv.task}
                                </span>
                              )}
                            </div>
                          </div>
                        </div>
                        
                        <div className="flex items-center gap-2 sm:gap-4 flex-shrink-0">
                          {/* Battery Indicator - Only show if battery > 0 and online */}
                          {agv.status === "online" && agv.battery > 0 && (
                            <div className="flex items-center gap-1">
                              <div className="w-8 sm:w-12 bg-gray-200 rounded-full h-1.5 sm:h-2 hidden xs:block">
                                <div 
                                  className={`h-1.5 sm:h-2 rounded-full ${getBatteryColor(agv.battery)}`}
                                  style={{ width: `${Math.min(agv.battery, 100)}%` }}
                                ></div>
                              </div>
                              <span className={`text-xs font-bold ${getBatteryTextColor(agv.battery)} hidden sm:block`}>
                                {agv.battery}%
                              </span>
                            </div>
                          )}
                          
                          {/* Selection Checkbox */}
                          <div className={`w-4 h-4 sm:w-5 sm:h-5 rounded border-2 flex items-center justify-center flex-shrink-0 ${
                            selectedAgvs.includes(agv.id)
                              ? "bg-purple-500 border-purple-500"
                              : "border-gray-400"
                          } ${agv.status === 'offline' ? 'opacity-50' : ''}`}>
                            {selectedAgvs.includes(agv.id) && (
                              <FiCheck className="text-white text-xs" />
                            )}
                          </div>
                        </div>
                      </div>
                      
                      {/* Status and Additional Info */}
                      <div className="flex justify-between items-center mt-1.5 sm:mt-2">
                        <span className={`text-xs px-1.5 sm:px-2 py-0.5 sm:py-1 rounded-full ${
                          agv.status === "online" ? "bg-green-100 text-green-800" :
                          agv.status === "offline" ? "bg-red-100 text-red-800" :
                          "bg-yellow-100 text-yellow-800"
                        }`}>
                          {getStatusDisplay(agv.status)}
                          {agv.error && agv.error !== "NONE" && agv.error !== "N/A" && " ⚠️"}
                        </span>
                        
                        {/* Mobile Battery Display */}
                        {agv.status === "online" && agv.battery > 0 && (
                          <span className={`text-xs font-bold sm:hidden ${getBatteryTextColor(agv.battery)}`}>
                            {agv.battery}%
                          </span>
                        )}
                        
                        {/* Additional info */}
                        <div className="flex items-center gap-1 text-xs text-gray-500">
                          {agv.camera && agv.camera !== "FAULT" && (
                            <span className="text-green-600">📷</span>
                          )}
                          {agv.lidar && agv.lidar !== "FAULT" && (
                            <span className="text-blue-600">📡</span>
                          )}
                        </div>
                      </div>

                      {/* Position info if available and robot is online */}
                      {agv.position && agv.position.x !== null && agv.status === "online" && (
                        <div className="text-xs text-gray-500 mt-1">
                          Position: ({agv.position.x.toFixed(2)}, {agv.position.y.toFixed(2)})
                        </div>
                      )}

                      {/* Last data received time */}
                      {agv.lastDataReceived && (
                        <div className="text-xs text-gray-400 mt-1">
                          Last data: {new Date(agv.lastDataReceived).toLocaleTimeString()}
                        </div>
                      )}
                    </div>
                  ))
                )}
              </div>

              {/* Footer */}
              <div className="mt-3 sm:mt-4 pt-3 border-t border-gray-200">
                <div className="flex flex-col sm:flex-row sm:justify-between sm:items-center gap-2 text-sm">
                  <div className="text-gray-600">
                    <span className="block text-center sm:text-left">
                      {selectedAgvs.length} AGV{selectedAgvs.length !== 1 ? 's' : ''} selected
                    </span>
                    <span className="text-xs text-gray-400">
                      WiFi: {wifiConnected ? 'Connected ✓' : 'Disconnected ✗'}
                    </span>
                  </div>


                  <div className="flex gap-2">
                    <button
                      onClick={() => {
                        setSearchQuery("");
                        setAgvDropdownOpen(false);
                      }}
                      className="px-3 sm:px-4 py-2 border border-gray-300 text-gray-700 rounded-lg hover:bg-gray-50 transition flex-1 sm:flex-none"
                    >
                      Cancel
                    </button>
                    <button
                      onClick={() => {
                        console.log("Selected AGVs:", selectedAgvs);
                        setAgvDropdownOpen(false);
                      }}
                      className="px-3 sm:px-4 py-2 bg-purple-500 text-white rounded-lg hover:bg-purple-600 transition flex-1 sm:flex-none"
                      disabled={selectedAgvs.length === 0}
                    >
                      Confirm ({selectedAgvs.length})
                    </button>
                  </div>
                </div>
              </div>
            </div>
          )}
        </div>

        {/* Notification Bell */}
        <button className="px-2 sm:px-3 py-2 bg-slate-100 text-black rounded-lg shadow flex items-center justify-center hover:bg-slate-200 transition relative">
          <FiBell className="text-base sm:text-lg" />
          <span className="absolute -top-1 -right-1 bg-red-500 text-white text-xs rounded-full w-4 h-4 sm:w-5 sm:h-5 flex items-center justify-center text-[10px] sm:text-xs">
            3
          </span>
        </button>

        {/* User Dropdown */}
        <div className="relative" ref={dropdownRef}>
          <button
            onClick={() => setUserDropdownOpen(!userDropdownOpen)}
            className="px-2 sm:px-3 py-2 bg-slate-100 text-black rounded-lg shadow flex items-center justify-center hover:bg-slate-200 transition"
          >
            <FiUser className="text-base sm:text-lg" />
          </button>

          {userDropdownOpen && (
            <div className="absolute right-0 mt-2 w-44 bg-white text-black rounded-lg shadow-lg p-2 z-50 border border-gray-200">
              <p className="py-2 px-3 text-sm font-medium border-b border-gray-100 truncate bg-gray-50 rounded-t">
                {loggedInUser}
              </p>
              <button
                onClick={handleLogout}
                className="w-full text-left py-2 px-3 text-sm hover:bg-red-50 text-red-600 rounded-b flex items-center gap-2 transition"
              >
                <FiLogOut /> Logout
              </button>
            </div>
          )}
        </div>
      </div>

      {/* Mobile Menu */}
      {menuOpen && (
        <div className="absolute top-24 left-0 w-full bg-white shadow-lg rounded-b-lg p-4 lg:hidden z-50">
          <nav className="space-y-2 text-gray-800">
            {[
              { to: "/", label: "Dashboard", icon: FiHome },
              { to: "/maps", label: "Maps", icon: FiBarChart2 },
              { to: "/realtime", label: "Real-Time Visualisation", icon: FiActivity },
              { to: "/tasks", label: "Task Allocation", icon: FiClipboard },
              { to: "/health", label: "Health Check", icon: FiHeart },
              { to: "/reports", label: "Reports", icon: FiBarChart2 },
              { to: "/settings", label: "Settings", icon: FiSettings },
            ].map(({ to, label, icon: Icon }) => (
              <Link
                key={to}
                to={to}
                className="flex items-center gap-3 py-3 px-4 rounded-lg hover:bg-sky-100 transition"
                onClick={() => setMenuOpen(false)}
              >
                <Icon className="text-base sm:text-lg" /> {label}
              </Link>
            ))}
          </nav>
        </div>
      )}

      {/* Custom Scrollbar Styles */}
      <style jsx>{`
        .custom-scrollbar::-webkit-scrollbar {
          width: 6px;
        }
        .custom-scrollbar::-webkit-scrollbar-track {
          background: #f1f1f1;
          border-radius: 3px;
        }
        .custom-scrollbar::-webkit-scrollbar-thumb {
          background: #c4b5fd;
          border-radius: 3px;
        }
        .custom-scrollbar::-webkit-scrollbar-thumb:hover {
          background: #a78bfa;
        }
      `}</style>
    </header>
  );
}