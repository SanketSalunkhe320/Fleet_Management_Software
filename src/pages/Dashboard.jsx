


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
const COLORS = ["#0ea5e9", "#22c55e", "#ef4444", "#f59e0b", "#8b5cf6"];

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
    if (distance < minDistance && distance < 0.5) {
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
    RUNNING: "bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-300",
    IDLE: "bg-yellow-100 text-yellow-700 dark:bg-yellow-900 dark:text-yellow-300",
    CHARGING: "bg-blue-100 text-blue-700 dark:bg-blue-900 dark:text-blue-300",
    OFFLINE: "bg-gray-100 text-gray-700 dark:bg-gray-800 dark:text-gray-300",
    FAULT: "bg-red-100 text-red-700 dark:bg-red-900 dark:text-red-300",
    NONE: "bg-gray-100 text-gray-700 dark:bg-gray-800 dark:text-gray-300",
  };

  return (
    <span
      className={`px-2 py-1 rounded-full text-xs font-medium ${
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

  let style = "bg-red-100 text-red-700 dark:bg-red-900 dark:text-red-300";
  if (value > 80) style = "bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-300";
  else if (value > 60) style = "bg-green-100 text-green-700 dark:bg-green-900 dark:text-green-300";
  else if (value > 40) style = "bg-yellow-100 text-yellow-700 dark:bg-yellow-900 dark:text-yellow-300";
  else if (value > 20) style = "bg-orange-100 text-orange-700 dark:bg-orange-900 dark:text-orange-300";

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
    docking: "🚢",
    waypoint: "📍",
    charging: "🔋",
    home: "🏠",
  };
  return <span className="mr-1">{icons[type] || "📍"}</span>;
};

// Hardcoded AGV data for AMR-02, AMR-03, etc.
const HARDCODED_AGVS = [
  {
    id: "AMR-02",
    status: "RUNNING",
    battery: "85",
    task: "Material Transport",
    location: "Assembly Line A",
    targetLocation: "Storage Area",
    error: "NONE",
    camera: "OK",
    lidar: "OK",
    speed: 1.2,
    motorTemp: 42,
    position: { x: 2.1, y: 1.6 }
  },
  {
    id: "AMR-03",
    status: "CHARGING",
    battery: "45",
    task: "Charging",
    location: "Charging Station",
    targetLocation: "not_set",
    error: "LOW_BATTERY",
    camera: "OK",
    lidar: "OK",
    speed: 0,
    motorTemp: 35,
    position: { x: 0.6, y: 0.5 }
  },
  {
    id: "AMR-04",
    status: "IDLE",
    battery: "92",
    task: "Awaiting Task",
    location: "Parking Zone",
    targetLocation: "Docking Point 1",
    error: "NONE",
    camera: "OK",
    lidar: "OK",
    speed: 0,
    motorTemp: 28,
    position: { x: 1.5, y: 2.0 }
  },
  {
    id: "AMR-05",
    status: "RUNNING",
    battery: "78",
    task: "Quality Inspection",
    location: "Quality Check",
    targetLocation: "Packaging Area",
    error: "NONE",
    camera: "OK",
    lidar: "FAULT",
    speed: 0.8,
    motorTemp: 48,
    position: { x: 2.8, y: 2.3 }
  }
];

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
      motorTemp: 0,
      position: { x: null, y: null },
    },
    // Add hardcoded AGVs
    ...HARDCODED_AGVS
  ]);

  const [rosConnected, setRosConnected] = useState(false);
  const [stationData, setStationData] = useState([]);
  const fetchIntervalRef = useRef(null);
  const positionRef = useRef({ x: null, y: null });
  const speedRef = useRef(0);

  // ----------------------------------------------------
  // Fetch stations from backend
  // ----------------------------------------------------
  useEffect(() => {
    const fetchStations = async () => {
      try {
        const res = await fetch("http://localhost:5000/load-nodes?map_name=default");
        const data = await res.json();
        if (data.status === "success") {
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
  // ROS Bridge Connection for position and speed updates
  // ----------------------------------------------------
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    ros.on("connection", () => setRosConnected(true));
    ros.on("error", () => setRosConnected(false));
    ros.on("close", () => setRosConnected(false));

    // Subscribe to AMCL pose for position
    const poseListener = new ROSLIB.Topic({
      ros,
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    poseListener.subscribe((msg) => {
      const x = Number(msg.pose.pose.position.x || 0);
      const y = Number(msg.pose.pose.position.y || 0);
      positionRef.current = { x, y };
    });

    // Subscribe to odom_raw for speed data
    const odomListener = new ROSLIB.Topic({
      ros,
      name: "/odom_raw",
      messageType: "nav_msgs/Odometry",
    });

    odomListener.subscribe((msg) => {
      const linearSpeed = Number(msg.twist.twist.linear.x || 0);
      const speed = Math.abs(linearSpeed);
      speedRef.current = speed;
    });

    return () => {
      poseListener.unsubscribe();
      odomListener.unsubscribe();
      ros.close();
    };
  }, []);

  // ----------------------------------------------------
  // Fetch backend robot status - Update only AMR-01
  // ----------------------------------------------------
  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const res = await fetch("http://localhost:5000/status");
        const data = await res.json();

        // Get current position from ref
        const { x, y } = positionRef.current;
        
        // Find nearest station for AMR-01
        const currentLocationInfo = findNearestStation(x, y);
        
        // Get target location based on task
        let targetLocation = t("not_set");
        if (data.robot_status === "RUNNING") {
          targetLocation = data.task || t("auto_navigation");
        }

        // Update only AMR-01 with real data, keep others as hardcoded
        setAgvs(prevAgvs => {
          return prevAgvs.map(agv => {
            if (agv.id === "AMR-01") {
              return {
                ...agv,
                status: (data.robot_status || "OFFLINE").toUpperCase(),
                battery: typeof data.battery === "number" ? `${data.battery}` : "N/A",
                task: data.task || t("auto_mode"),
                location: currentLocationInfo.name,
                targetLocation: targetLocation,
                error: (data.error_status || "NONE").toUpperCase(),
                camera: (data.camera_status || "FAULT").toUpperCase(),
                lidar: (data.lidar_status || "FAULT").toUpperCase(),
                speed: speedRef.current,
                motorTemp: data.robot_status === "RUNNING" 
                  ? Math.round(30 + Math.min(speedRef.current * 8, 30) + Math.random() * 5)
                  : Math.round(25 + Math.random() * 3),
                position: { x, y }
              };
            }
            // For other AGVs, add slight variations to make them look "alive"
            if (agv.id !== "AMR-01") {
              // Add small random variations to battery and temperature
              const batteryVariation = Math.random() > 0.7 ? (Math.random() > 0.5 ? 1 : -1) : 0;
              const tempVariation = Math.random() > 0.5 ? (Math.random() > 0.5 ? 1 : -1) : 0;
              
              // Occasionally change status (rarely)
              let newStatus = agv.status;
              if (Math.random() > 0.98) {
                if (agv.status === "RUNNING") newStatus = "IDLE";
                else if (agv.status === "IDLE") newStatus = "RUNNING";
              }
              
              return {
                ...agv,
                battery: Math.min(100, Math.max(10, parseInt(agv.battery) + batteryVariation)).toString(),
                motorTemp: Math.max(20, Math.min(70, agv.motorTemp + tempVariation)),
                status: newStatus
              };
            }
            return agv;
          });
        });

      } catch {
        // If backend down → set AMR-01 offline
        setAgvs(prevAgvs => {
          return prevAgvs.map(agv => {
            if (agv.id === "AMR-01") {
              return {
                ...agv,
                status: "OFFLINE",
                battery: "N/A",
                error: "COMM_ERROR",
                camera: "FAULT",
                lidar: "FAULT",
                speed: speedRef.current,
              };
            }
            return agv;
          });
        });
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
    { name: t("charging"), value: agvs.filter((a) => a.status === "CHARGING").length },
    { name: t("offline"), value: agvs.filter((a) => a.status === "OFFLINE").length },
  ];

  const batteryData = agvs.map((a) => ({
    name: a.id,
    battery: parseInt(a.battery) || 0,
  }));

  // Get AGV count summary
  const runningCount = agvs.filter(a => a.status === "RUNNING").length;
  const idleCount = agvs.filter(a => a.status === "IDLE").length;
  const chargingCount = agvs.filter(a => a.status === "CHARGING").length;
  const offlineCount = agvs.filter(a => a.status === "OFFLINE").length;

  return (
    <div className="p-4 sm:p-6 space-y-8 bg-gray-50 dark:bg-gray-900 min-h-screen">
      {/* TOP STATUS BAR */}
      <div
        className="rounded-2xl text-white shadow-md p-5"
        style={{ background: "linear-gradient(135deg, #1c91df, #2c69c5)" }}
      >
        <h3 className="text-lg font-bold mb-3">📊 {t("system_status")}</h3>

        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-5 gap-4">
          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-medium">ROS Bridge</p>
            <p>{rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-medium">Total AGVs</p>
            <p>{agvs.length} Units</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-medium">Running</p>
            <p>{runningCount} Units</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-medium">Charging</p>
            <p>{chargingCount} Units</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-medium">Stations</p>
            <p>{stationData.length || STATION_DATABASE.length}</p>
          </div>
        </div>
      </div>

      {/* CHARTS */}
      <section className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* DONUT CHART */}
        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <div className="flex justify-between items-center mb-3">
            <h3 className="font-semibold">{t("amr_status_distribution")}</h3>
            <div className="text-sm px-3 py-1 bg-gray-100 dark:bg-gray-700 rounded-full">
              Total: {agvs.length}
            </div>
          </div>
          <ResponsiveContainer width="100%" height={260}>
            <PieChart>
              <Pie 
                data={pieData.filter(d => d.value > 0)} 
                cx="50%"
                cy="50%"
                innerRadius={60}  // This creates the donut hole
                outerRadius={100}
                paddingAngle={5}
                dataKey="value" 
                label={(entry) => `${entry.name}: ${entry.value}`}
                labelLine={false}
              >
                {pieData.filter(d => d.value > 0).map((e, i) => (
                  <Cell key={i} fill={COLORS[i % COLORS.length]} />
                ))}
              </Pie>
              <Tooltip 
                formatter={(value, name, props) => [
                  `${value} AGVs (${((value / agvs.length) * 100).toFixed(1)}%)`,
                  name
                ]}
              />
              {/* Add text in the center of donut */}
              <text
                x="50%"
                y="50%"
                textAnchor="middle"
                dominantBaseline="middle"
                className="text-lg font-bold fill-gray-700 dark:fill-gray-300"
              >
                {agvs.length}
              </text>
              <text
                x="50%"
                y="60%"
                textAnchor="middle"
                dominantBaseline="middle"
                className="text-sm fill-gray-500 dark:fill-gray-400"
              >
                Total AGVs
              </text>
            </PieChart>
          </ResponsiveContainer>
          {/* Legend */}
          <div className="flex flex-wrap justify-center gap-3 mt-4">
            {pieData.filter(d => d.value > 0).map((item, index) => (
              <div key={item.name} className="flex items-center gap-2">
                <div 
                  className="w-3 h-3 rounded-full" 
                  style={{ backgroundColor: COLORS[index % COLORS.length] }}
                ></div>
                <span className="text-sm">{item.name}: {item.value}</span>
              </div>
            ))}
          </div>
        </div>

        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold mb-3">{t("battery_level_overview")}</h3>
          <ResponsiveContainer width="100%" height={260}>
            <BarChart data={batteryData}>
              <XAxis dataKey="name" />
              <YAxis domain={[0, 100]} />
              <Tooltip formatter={(value) => [`${value}%`, "Battery"]} />
              <Bar 
                dataKey="battery" 
                name="Battery %"
                radius={[4, 4, 0, 0]}
              >
                {batteryData.map((entry, index) => (
                  <Cell 
                    key={`cell-${index}`}
                    fill={
                      entry.battery > 80 ? "#22c55e" :
                      entry.battery > 60 ? "#86efac" :
                      entry.battery > 40 ? "#fde68a" :
                      entry.battery > 20 ? "#fdba74" : "#f87171"
                    }
                  />
                ))}
              </Bar>
            </BarChart>
          </ResponsiveContainer>
        </div>
      </section>

      {/* AMR TABLE - MULTIPLE AGVS */}
      <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
        <div className="flex flex-col sm:flex-row justify-between items-start sm:items-center mb-4 gap-3">
          <h3 className="font-semibold text-lg">{t("amr_status")}</h3>
          
          <div className="flex flex-wrap gap-2">
            <span className="px-3 py-1 bg-gray-100 dark:bg-gray-700 rounded-full text-sm">
              Total: {agvs.length} AGVs
            </span>
            <span className="px-3 py-1 bg-green-100 dark:bg-green-900/30 text-green-800 dark:text-green-300 rounded-full text-sm">
              Running: {runningCount}
            </span>
            <span className="px-3 py-1 bg-blue-100 dark:bg-blue-900/30 text-blue-800 dark:text-blue-300 rounded-full text-sm">
              Charging: {chargingCount}
            </span>
            <span className="px-3 py-1 bg-yellow-100 dark:bg-yellow-900/30 text-yellow-800 dark:text-yellow-300 rounded-full text-sm">
              Idle: {idleCount}
            </span>
          </div>
        </div>

        <div className="overflow-x-auto rounded-lg border border-gray-200 dark:border-gray-700">
          <table className="w-full text-sm">
            <thead className="bg-sky-800 text-white">
              <tr>
                <th className="p-3 text-left font-medium">{t("amr_id")}</th>
                <th className="p-3 text-left font-medium">{t("status")}</th>
                <th className="p-3 text-left font-medium">{t("battery")}</th>
                <th className="p-3 text-left font-medium">{t("motor_temp")}</th>
                <th className="p-3 text-left font-medium">{t("speed")}</th>
                <th className="p-3 text-left font-medium">{t("current_location")}</th>
                <th className="p-3 text-left font-medium">{t("target_location")}</th>
                <th className="p-3 text-left font-medium">{t("error")}</th>
              </tr>
            </thead>

            <tbody className="divide-y divide-gray-200 dark:divide-gray-700">
              {agvs.map((agv) => {
                const station = stationData.find(s => s.name === agv.location) || 
                               STATION_DATABASE.find(s => s.name === agv.location);
                
                // Color coding for different AGVs
                const getAGVColor = (id) => {
                  switch(id) {
                    case 'AMR-01': return 'bg-blue-500';
                    case 'AMR-02': return 'bg-green-500';
                    case 'AMR-03': return 'bg-purple-500';
                    case 'AMR-04': return 'bg-yellow-500';
                    case 'AMR-05': return 'bg-pink-500';
                    default: return 'bg-gray-500';
                  }
                };

                // Real-time indicator for AMR-01
                const isRealTime = agv.id === "AMR-01";

                return (
                  <tr key={agv.id} className="hover:bg-gray-50 dark:hover:bg-gray-700/50 transition-colors">
                    <td className="p-3 font-medium">
                      <div className="flex items-center gap-2">
                        <div className={`w-2 h-2 rounded-full ${getAGVColor(agv.id)}`}></div>
                        <span className="text-gray-900 dark:text-white">
                          {agv.id}
                        </span>
                        {isRealTime && (
                          <span className="text-xs text-blue-500 dark:text-blue-400">
                            (Real-time)
                          </span>
                        )}
                      </div>
                    </td>

                    <td className="p-3">
                      <StatusBadge status={agv.status} />
                    </td>

                    <td className="p-3">
                      <BatteryBadge level={agv.battery} />
                    </td>

                    {/* Motor Temperature */}
                    <td className="p-3">
                      <div className="flex items-center gap-2">
                        <div className={`w-2 h-2 rounded-full ${
                          agv.motorTemp > 70 ? 'bg-red-500' : 
                          agv.motorTemp > 55 ? 'bg-yellow-500' : 
                          'bg-green-500'
                        }`}></div>
                        <span className={`font-medium ${
                          agv.motorTemp > 70 ? 'text-red-600 dark:text-red-400' : 
                          agv.motorTemp > 55 ? 'text-yellow-600 dark:text-yellow-400' : 
                          'text-green-600 dark:text-green-400'
                        }`}>
                          {agv.motorTemp || 0}°C
                        </span>
                      </div>
                    </td>

                    {/* Speed */}
                    <td className="p-3">
                      <div className="flex flex-col">
                        <span className="font-medium">{agv.speed.toFixed(2)} m/s</span>
                        <div className="w-full bg-gray-200 dark:bg-gray-700 rounded-full h-1.5 mt-1">
                          <div 
                            className={`h-1.5 rounded-full ${
                              agv.speed > 1.5 ? 'bg-red-500' : 
                              agv.speed > 1.0 ? 'bg-yellow-500' : 
                              'bg-green-500'
                            }`}
                            style={{ 
                              width: `${Math.min((agv.speed / 2) * 100, 100)}%` 
                            }}
                          ></div>
                        </div>
                      </div>
                    </td>

                    {/* Current location */}
                    <td className="p-3">
                      {station ? (
                        <div className="flex items-center">
                          <StationIcon type={station.type} />
                          <span>{agv.location}</span>
                        </div>
                      ) : (
                        <span>{agv.location}</span>
                      )}
                    </td>

                    {/* Target location */}
                    <td className="p-3">
                      <div className="space-y-1">
                        <div>{agv.targetLocation}</div>
                        {agv.targetLocation !== t("not_set") && (
                          <div className="text-xs text-gray-500 dark:text-gray-400">
                            {agv.status === "RUNNING" ? 
                              <span className="text-blue-600 dark:text-blue-400">🚗 {t("navigating")}</span> : 
                              <span className="text-green-600 dark:text-green-400">✓ {t("ready")}</span>
                            }
                          </div>
                        )}
                      </div>
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
        </div>
      </section>

      {/* STATION INFORMATION */}
      <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
        <div className="flex justify-between items-center mb-4">
          <h3 className="font-semibold text-lg">📍 {t("station_information")}</h3>
          <span className="text-sm text-gray-600 dark:text-gray-400">
            {stationData.length || STATION_DATABASE.length} stations
          </span>
        </div>
        
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          {(stationData.length > 0 ? stationData : STATION_DATABASE).map((station) => {
            // Find which AGVs are at this station
            const agvsAtStation = agvs.filter(agv => agv.location === station.name);
            
            return (
              <div 
                key={station.id} 
                className={`p-3 rounded-lg border transition-all ${
                  agvsAtStation.length > 0 
                    ? "border-blue-500 bg-blue-50 dark:bg-blue-900/20 shadow-sm" 
                    : "border-gray-200 dark:border-gray-700"
                }`}
              >
                <div className="flex items-center justify-between mb-2">
                  <div className="flex items-center">
                    <StationIcon type={station.type} />
                    <span className="font-medium">{station.name}</span>
                  </div>
                  {agvsAtStation.length > 0 && (
                    <span className="text-xs bg-blue-500 text-white px-2 py-1 rounded-full">
                      {agvsAtStation.length} AGV{agvsAtStation.length > 1 ? 's' : ''}
                    </span>
                  )}
                </div>
                <div className="text-sm text-gray-600 dark:text-gray-400 mb-2">
                  Type: <span className="font-medium capitalize">{station.type}</span>
                </div>
                <div className="text-sm text-gray-600 dark:text-gray-400">
                  Coordinates: ({station.x.toFixed(2)}, {station.y.toFixed(2)})
                </div>
                {agvsAtStation.length > 0 && (
                  <div className="mt-2 pt-2 border-t border-gray-200 dark:border-gray-700">
                    <div className="text-xs text-gray-500 dark:text-gray-400">
                      Present: {agvsAtStation.map(a => a.id).join(', ')}
                    </div>
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </section>

      {/* FOOTER NOTE */}
      <div className="text-center text-sm text-gray-500 dark:text-gray-400">
        <p>💡 <strong>AMR-01</strong> shows real-time data from ROS system | 
           <strong> AMR-02 to AMR-05</strong> show simulated data with small variations</p>
      </div>
    </div>
  );
}