


// import React, { useEffect, useState, useRef } from "react";
// import ROSLIB from "roslib";

// import {
//   PieChart,
//   Pie,
//   Cell,
//   Tooltip,
//   Legend,
//   BarChart,
//   Bar,
//   XAxis,
//   YAxis,
//   ResponsiveContainer,
// } from "recharts";

// // Chart colors
// const COLORS = ["#0ea5e9", "#22c55e", "#ef4444"];

// // Status Badge
// const StatusBadge = ({ status }) => {
//   const map = {
//     RUNNING: "bg-green-100 text-green-700",
//     IDLE: "bg-yellow-100 text-yellow-700",
//     OFFLINE: "bg-gray-100 text-gray-700",
//     FAULT: "bg-red-100 text-red-700",
//   };

//   return (
//     <span
//       className={`px-2 py-1 rounded-full text-[10px] sm:text-xs font-medium ${
//         map[status] || "bg-gray-100 text-gray-700"
//       }`}
//     >
//       {status}
//     </span>
//   );
// };

// // Battery Badge
// const BatteryBadge = ({ level }) => {
//   const value = parseInt(level, 10);

//   if (isNaN(value)) return <span className="px-2 py-1 text-xs">🔋 N/A</span>;

//   let style = "bg-red-100 text-red-700";
//   if (value > 60) style = "bg-green-100 text-green-700";
//   else if (value > 30) style = "bg-yellow-100 text-yellow-700";

//   return (
//     <span className={`px-2 py-1 rounded-full text-xs font-medium ${style}`}>
//       🔋 {value}%
//     </span>
//   );
// };

// export default function Dashboard() {
//   const [agvs, setAgvs] = useState([
//     {
//       id: "AMR-01",
//       status: "OFFLINE",
//       battery: "N/A",
//       task: "Idle",
//       location: "Unknown",
//       camera: "FAULT",
//       lidar: "FAULT",
//       speed: 0,
//     },
//   ]);

//   const [rosConnected, setRosConnected] = useState(false);
//   const fetchIntervalRef = useRef(null);

//   // ----------------------------------------------------
//   // ROS Bridge Connection (Only for location updates)
//   // ----------------------------------------------------
//   useEffect(() => {
//     const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

//     ros.on("connection", () => setRosConnected(true));
//     ros.on("error", () => setRosConnected(false));
//     ros.on("close", () => setRosConnected(false));

//     // Subscribe to AMCL pose
//     const poseListener = new ROSLIB.Topic({
//       ros,
//       name: "/amcl_pose",
//       messageType: "geometry_msgs/PoseWithCovarianceStamped",
//     });

//     // Only update location (NO RUNNING/IDLE logic)
//     poseListener.subscribe((msg) => {
//       const x = Number(msg.pose.pose.position.x || 0);
//       const y = Number(msg.pose.pose.position.y || 0);

//       setAgvs((prev) =>
//         prev.map((a) => ({
//           ...a,
//           location: `X:${x.toFixed(2)}, Y:${y.toFixed(2)}`,
//         }))
//       );
//     });

//     return () => {
//       poseListener.unsubscribe();
//       ros.close();
//     };
//   }, []);

//   // ----------------------------------------------------
//   // Fetch backend robot status
//   // ----------------------------------------------------
//   useEffect(() => {
//     const fetchStatus = async () => {
//       try {
//         const res = await fetch("http://localhost:5000/status");
//         const data = await res.json();

//         setAgvs([
//           {
//             id: "AMR-01",
//             status: (data.robot_status || "OFFLINE").toUpperCase(),

//             battery:
//               typeof data.battery === "number"
//                 ? `${data.battery}%`
//                 : "N/A",

//             task: data.task || "Auto Mode",

//             location:
//               data.robot_position &&
//               typeof data.robot_position.x === "number"
//                 ? `X:${data.robot_position.x.toFixed(2)}, Y:${data.robot_position.y.toFixed(2)}`
//                 : "Unknown",

//             camera: (data.camera_status || "FAULT").toUpperCase(),
//             lidar: (data.lidar_status || "FAULT").toUpperCase(),
//           },
//         ]);
//       } catch {
//         // If backend down → everything offline
//         setAgvs([
//           {
//             id: "AMR-01",
//             status: "OFFLINE",
//             battery: "N/A",
//             task: "N/A",
//             location: "Unknown",
//             camera: "FAULT",
//             lidar: "FAULT",
//           },
//         ]);
//       }
//     };

//     fetchStatus();
//     fetchIntervalRef.current = setInterval(fetchStatus, 1000);

//     return () => clearInterval(fetchIntervalRef.current);
//   }, []);

//   // ----------------------------------------------------
//   // Charts
//   // ----------------------------------------------------
//   const pieData = [
//     { name: "Running", value: agvs.filter((a) => a.status === "RUNNING").length },
//     { name: "Idle", value: agvs.filter((a) => a.status === "IDLE").length },
//     { name: "Offline", value: agvs.filter((a) => a.status === "OFFLINE").length },
//   ];

//   const batteryData = agvs.map((a) => ({
//     name: a.id,
//     battery: parseInt(a.battery) || 0,
//   }));

//   return (
//     <div className="p-4 sm:p-6 space-y-8 bg-gray-50 dark:bg-gray-900 min-h-screen">
//       {/* TOP STATUS BAR */}
//       <div
//         className="rounded-2xl text-white shadow-md p-5"
//         style={{ background: "linear-gradient(135deg, #1c91df, #2c69c5)" }}
//       >
//         <h3 className="text-lg font-bold mb-3">📊 System Status</h3>

//         <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
//           <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
//             <p>ROS Bridge: {rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</p>
//           </div>

//           <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
//             <p>Robot Position: {agvs[0].location}</p>
//             <p>Total AMRs: {agvs.length}</p>
//           </div>
//         </div>
//       </div>

//       {/* CHARTS */}
//       <section className="grid grid-cols-1 md:grid-cols-2 gap-6">
//         <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
//           <h3 className="font-semibold mb-3">AMR Status Distribution</h3>
//           <ResponsiveContainer width="100%" height={260}>
//             <PieChart>
//               <Pie data={pieData} outerRadius={90} dataKey="value" label>
//                 {pieData.map((e, i) => (
//                   <Cell key={i} fill={COLORS[i % COLORS.length]} />
//                 ))}
//               </Pie>
//               <Tooltip />
//               <Legend />
//             </PieChart>
//           </ResponsiveContainer>
//         </div>

//         <div className="bg-white dark:bg-gray-900 p-4 rounded-2xl shadow-md">
//           <h3 className="font-semibold mb-3">Battery Level Overview</h3>
//           <ResponsiveContainer width="100%" height={260}>
//             <BarChart data={batteryData}>
//               <XAxis dataKey="name" />
//               <YAxis />
//               <Tooltip />
//               <Bar dataKey="battery" fill="#0ea5e9" />
//             </BarChart>
//           </ResponsiveContainer>
//         </div>
//       </section>

//      {/* AMR TABLE */}
// <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
//   <h3 className="font-semibold mb-4 text-lg">AMR Status</h3>

//   <table className="w-full text-sm">
//     <thead className="bg-sky-800 text-white">
//       <tr>
//         <th className="p-3 text-left">AMR ID</th>
//         <th className="p-3 text-left">Status</th>
//         <th className="p-3 text-left">Battery</th>
//         <th className="p-3 text-left">Current Location</th>
//         <th className="p-3 text-left">Target Location</th>
//         <th className="p-3 text-left">Error</th>
//       </tr>
//     </thead>

//     <tbody>
//       {agvs.map((agv) => (
//         <tr key={agv.id} className="border-t">
//           <td className="p-3">{agv.id}</td>

//           <td className="p-3">
//             <StatusBadge status={agv.status} />
//           </td>

//           <td className="p-3">
//             <BatteryBadge level={agv.battery} />
//           </td>

//           {/* Current location */}
//           <td className="p-3">{agv.location}</td>

//           {/* Target location */}
//           <td className="p-3">{agv.targetLocation}</td>

//           {/* Error */}
//           <td className="p-3">
//             <StatusBadge status={agv.error || "NONE"} />
//           </td>
//         </tr>
//       ))}
//     </tbody>
//   </table>
// </section>

//     </div>
//   );
// }

// src/pages/Dashboard.jsx
import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";
import {
  PieChart,
  Pie,
  Cell,
  Tooltip,
  Legend,
  BarChart,
  Bar,
  XAxis,
  YAxis,
  ResponsiveContainer,
} from "recharts";
import { useLanguage } from "../context/LanguageContext";

// Chart colors
const COLORS = ["#0ea5e9", "#22c55e", "#ef4444"];

// Mock station database
const STATION_DATABASE = [
  { id: "station_1", name: "Charging Station", type: "charging", x: 0.5, y: 0.5 },
  { id: "station_2", name: "Assembly Line A", type: "station", x: 2.0, y: 1.5 },
  { id: "station_3", name: "Storage Area", type: "station", x: 3.0, y: 2.0 },
  { id: "station_4", name: "Docking Point 1", type: "docking", x: 1.0, y: 2.5 },
  { id: "station_5", name: "Waypoint 1", type: "waypoint", x: 2.5, y: 3.0 },
];

// Function to find nearest station based on robot position
const findNearestStation = (x, y) => {
  if (x === undefined || y === undefined) return { name: "Unknown", distance: null };

  let nearest = null;
  let minDistance = Infinity;

  STATION_DATABASE.forEach(station => {
    const distance = Math.sqrt(Math.pow(station.x - x, 2) + Math.pow(station.y - y, 2));
    if (distance < minDistance && distance < 0.5) { // Within 0.5m radius
      minDistance = distance;
      nearest = station;
    }
  });

  return nearest ? { 
    name: nearest.name, 
    distance: minDistance.toFixed(2),
    type: nearest.type
  } : { name: "In Transit", distance: null };
};

// Status Badge
const StatusBadge = ({ status }) => {
  const map = {
    RUNNING: "bg-green-100 text-green-700",
    IDLE: "bg-yellow-100 text-yellow-700",
    OFFLINE: "bg-gray-100 text-gray-700",
    FAULT: "bg-red-100 text-red-700",
    NONE: "bg-gray-100 text-gray-700",
  };

  return (
    <span
      className={`px-2 py-1 rounded-full text-[10px] sm:text-xs font-medium ${
        map[status] || "bg-gray-100 text-gray-700"
      }`}
    >
      {status}
    </span>
  );
};

// Battery Badge
const BatteryBadge = ({ level }) => {
  const value = parseInt(level, 10);

  if (isNaN(value)) return <span className="px-2 py-1 text-xs">🔋 N/A</span>;

  let style = "bg-red-100 text-red-700";
  if (value > 60) style = "bg-green-100 text-green-700";
  else if (value > 30) style = "bg-yellow-100 text-yellow-700";

  return (
    <span className={`px-2 py-1 rounded-full text-xs font-medium ${style}`}>
      🔋 {value}%
    </span>
  );
};

// Station Icon based on type
const StationIcon = ({ type }) => {
  const icons = {
    station: "🏭",
    docking: "🛳️",
    waypoint: "📍",
    charging: "🔋",
    home: "🏠",
  };
  return <span className="mr-1">{icons[type] || "📍"}</span>;
};

export default function Dashboard() {
  const { t } = useLanguage();
  const [agvs, setAgvs] = useState([
    {
      id: "AMR-01",
      status: "OFFLINE",
      battery: "N/A",
      task: "Idle",
      location: t("unknown"),
      targetLocation: t("not_set"),
      error: "NONE",
      camera: "FAULT",
      lidar: "FAULT",
      speed: 0,
      position: { x: null, y: null },
    },
  ]);

  const [rosConnected, setRosConnected] = useState(false);
  const [stationData, setStationData] = useState([]);
  const fetchIntervalRef = useRef(null);
  const positionRef = useRef({ x: null, y: null });

  // ----------------------------------------------------
  // Fetch stations from backend
  // ----------------------------------------------------
  useEffect(() => {
    const fetchStations = async () => {
      try {
        const res = await fetch("http://localhost:5000/load-nodes?map_name=default");
        const data = await res.json();
        if (data.status === "success") {
          // Transform nodes to station format
          const stations = data.nodes.map(node => ({
            id: node.id,
            name: node.label || `Station ${node.dbId}`,
            type: node.type,
            x: node.rosX,
            y: node.rosY
          }));
          setStationData(stations);
        }
      } catch (error) {
        console.log("Using mock station data");
        setStationData(STATION_DATABASE);
      }
    };

    fetchStations();
  }, []);

  // ----------------------------------------------------
  // ROS Bridge Connection for position updates
  // ----------------------------------------------------
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    ros.on("connection", () => setRosConnected(true));
    ros.on("error", () => setRosConnected(false));
    ros.on("close", () => setRosConnected(false));

    // Subscribe to AMCL pose
    const poseListener = new ROSLIB.Topic({
      ros,
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    poseListener.subscribe((msg) => {
      const x = Number(msg.pose.pose.position.x || 0);
      const y = Number(msg.pose.pose.position.y || 0);
      
      // Store position in ref for nearest station calculation
      positionRef.current = { x, y };
    });

    return () => {
      poseListener.unsubscribe();
      ros.close();
    };
  }, []);

  // ----------------------------------------------------
  // Fetch backend robot status
  // ----------------------------------------------------
  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const res = await fetch("http://localhost:5000/status");
        const data = await res.json();

        // Get current position from ref
        const { x, y } = positionRef.current;
        
        // Find nearest station
        const currentLocationInfo = findNearestStation(x, y);
        
        // Get target location based on task
        let targetLocation = t("not_set");
        if (data.robot_status === "RUNNING") {
          targetLocation = data.task || t("auto_navigation");
        }

        setAgvs([
          {
            id: "AMR-01",
            status: (data.robot_status || "OFFLINE").toUpperCase(),
            battery: typeof data.battery === "number" ? `${data.battery}%` : "N/A",
            task: data.task || t("auto_mode"),
            location: currentLocationInfo.name,
            targetLocation: targetLocation,
            error: (data.error_status || "NONE").toUpperCase(),
            camera: (data.camera_status || "FAULT").toUpperCase(),
            lidar: (data.lidar_status || "FAULT").toUpperCase(),
            speed: 0,
            position: { x, y }
          },
        ]);
      } catch {
        // If backend down → everything offline
        setAgvs([
          {
            id: "AMR-01",
            status: "OFFLINE",
            battery: "N/A",
            task: "N/A",
            location: t("unknown"),
            targetLocation: t("not_set"),
            error: "NONE",
            camera: "FAULT",
            lidar: "FAULT",
            speed: 0,
            position: { x: null, y: null }
          },
        ]);
      }
    };

    fetchStatus();
    fetchIntervalRef.current = setInterval(fetchStatus, 1000);

    return () => clearInterval(fetchIntervalRef.current);
  }, [t]);

  // ----------------------------------------------------
  // Charts
  // ----------------------------------------------------
  const pieData = [
    { name: t("running"), value: agvs.filter((a) => a.status === "RUNNING").length },
    { name: t("idle"), value: agvs.filter((a) => a.status === "IDLE").length },
    { name: t("offline"), value: agvs.filter((a) => a.status === "OFFLINE").length },
  ];

  const batteryData = agvs.map((a) => ({
    name: a.id,
    battery: parseInt(a.battery) || 0,
  }));

  // Get station type for current location
  const currentAgv = agvs[0];
  const currentStation = stationData.find(s => s.name === currentAgv.location) || 
                        STATION_DATABASE.find(s => s.name === currentAgv.location);

  return (
    <div className="p-4 sm:p-6 space-y-8 bg-gray-50 dark:bg-gray-900 min-h-screen">
      {/* TOP STATUS BAR */}
      <div
        className="rounded-2xl text-white shadow-md p-5"
        style={{ background: "linear-gradient(135deg, #1c91df, #2c69c5)" }}
      >
        <h3 className="text-lg font-bold mb-3">📊 {t("system_status")}</h3>

        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p>{t("ros_bridge")}: {rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</p>
            <p>{t("stations_loaded")}: {stationData.length || STATION_DATABASE.length}</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p>{t("current_location")}: 
              {currentStation ? (
                <span className="ml-2">
                  <StationIcon type={currentStation.type} />
                  {currentAgv.location}
                </span>
              ) : ` ${t("unknown")}`}
            </p>
            <p>{t("target_location")}: {currentAgv.targetLocation}</p>
          </div>
        </div>
      </div>

      {/* CHARTS */}
      <section className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold mb-3">{t("amr_status_distribution")}</h3>
          <ResponsiveContainer width="100%" height={260}>
            <PieChart>
              <Pie data={pieData} outerRadius={90} dataKey="value" label>
                {pieData.map((e, i) => (
                  <Cell key={i} fill={COLORS[i % COLORS.length]} />
                ))}
              </Pie>
              <Tooltip />
              <Legend />
            </PieChart>
          </ResponsiveContainer>
        </div>

        <div className="bg-white dark:bg-gray-900 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold mb-3">{t("battery_level_overview")}</h3>
          <ResponsiveContainer width="100%" height={260}>
            <BarChart data={batteryData}>
              <XAxis dataKey="name" />
              <YAxis />
              <Tooltip />
              <Bar dataKey="battery" fill="#0ea5e9" />
            </BarChart>
          </ResponsiveContainer>
        </div>
      </section>

      {/* AMR TABLE */}
      <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
        <h3 className="font-semibold mb-4 text-lg">{t("amr_status")}</h3>

        <table className="w-full text-sm">
          <thead className="bg-sky-800 text-white">
            <tr>
              <th className="p-3 text-left">AMR ID</th>
              <th className="p-3 text-left">{t("status")}</th>
              <th className="p-3 text-left">{t("battery")}</th>
              <th className="p-3 text-left">{t("current_location")}</th>
              <th className="p-3 text-left">{t("target_location")}</th>
              <th className="p-3 text-left">{t("error")}</th>
            </tr>
          </thead>

          <tbody>
            {agvs.map((agv) => {
              const station = stationData.find(s => s.name === agv.location) || 
                             STATION_DATABASE.find(s => s.name === agv.location);
              
              return (
                <tr key={agv.id} className="border-t">
                  <td className="p-3 font-medium">{agv.id}</td>

                  <td className="p-3">
                    <StatusBadge status={agv.status} />
                  </td>

                  <td className="p-3">
                    <BatteryBadge level={agv.battery} />
                  </td>

                  {/* Current location with icon */}
                  <td className="p-3">
                    {station ? (
                      <div className="flex items-center">
                        <StationIcon type={station.type} />
                        {agv.location}
                      </div>
                    ) : (
                      agv.location
                    )}
                  </td>

                  {/* Target location */}
                  <td className="p-3">
                    {agv.targetLocation}
                    {agv.targetLocation !== t("not_set") && (
                      <div className="text-xs text-gray-500 mt-1">
                        {agv.status === "RUNNING" ? t("navigating") : t("ready")}
                      </div>
                    )}
                  </td>

                  {/* Error */}
                  <td className="p-3">
                    <StatusBadge status={agv.error} />
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      </section>

      {/* ADDITIONAL INFO CARD */}
      <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
        <h3 className="font-semibold mb-4 text-lg">📍 {t("station_information")}</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {(stationData.length > 0 ? stationData : STATION_DATABASE).map((station) => (
            <div 
              key={station.id} 
              className={`p-3 rounded-lg border ${
                station.name === currentAgv.location 
                  ? "border-blue-500 bg-blue-50 dark:bg-blue-900/20" 
                  : "border-gray-200 dark:border-gray-700"
              }`}
            >
              <div className="flex items-center mb-2">
                <StationIcon type={station.type} />
                <span className="font-medium">{station.name}</span>
                {station.name === currentAgv.location && (
                  <span className="ml-2 text-xs bg-blue-500 text-white px-2 py-1 rounded">{t("current")}</span>
                )}
              </div>
              <div className="text-sm text-gray-600 dark:text-gray-400">
                {t("type")}: {station.type}
                <br />
                {t("coordinates")}: ({station.x.toFixed(2)}, {station.y.toFixed(2)})
              </div>
            </div>
          ))}
        </div>
      </section>
    </div>
  );
}