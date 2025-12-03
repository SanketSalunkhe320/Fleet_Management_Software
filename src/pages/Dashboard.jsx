// import React, { useEffect, useState, useRef } from "react";
// import ROSLIB from "roslib";

// // Card component
// const Card = ({ title, value, footer }) => (
//   <div className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4 sm:p-6 w-full transition-transform hover:scale-105">
//     <div className="text-xs sm:text-sm text-slate-500 dark:text-gray-400">{title}</div>
//     <div className="text-2xl sm:text-3xl font-bold mt-2 text-sky-800 dark:text-sky-300">{value}</div>
//     <div className="mt-2 sm:mt-3 text-[10px] sm:text-xs text-slate-400 dark:text-gray-500">{footer}</div>
//   </div>
// );

// // Badge component
// const StatusBadge = ({ status }) => {
//   const colors = {
//     RUNNING: "bg-green-100 text-green-700",
//     FAULT: "bg-red-100 text-red-700",
//     IDLE: "bg-yellow-100 text-yellow-700",
//     Offline: "bg-gray-100 text-gray-700",
//   };
//   return (
//     <span
//       className={`px-2 py-1 rounded-full text-[10px] sm:text-xs font-medium ${
//         colors[status] || "bg-gray-100 text-gray-700"
//       }`}
//     >
//       {status}
//     </span>
//   );
// };

// export default function Dashboard() {
//   const [agvs, setAgvs] = useState([
//     { id: "AGV-01", status: "IDLE", battery: "85%", task: "Material Transport", location: "Zone A" },
//     { id: "AGV-02", status: "IDLE", battery: "72%", task: "Assembly Line", location: "Zone B" },
//     { id: "AGV-03", status: "IDLE", battery: "92%", task: "Idle", location: "Charging Station" },
//     { id: "AGV-04", status: "IDLE", battery: "65%", task: "Warehouse Pick", location: "Zone D" },
//     { id: "AGV-05", status: "Offline", battery: "10%", task: "Idle", location: "Maintenance" },
//   ]);

//   const [rosConnected, setRosConnected] = useState(false);
//   const [taskServerConnected, setTaskServerConnected] = useState(false);
//   const [cursorCoords, setCursorCoords] = useState(null);
//   const [zAxisValue, setZAxisValue] = useState(0);
//   const prevPositions = useRef({});

//   useEffect(() => {
//     const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

//     ros.on("connection", () => {
//       console.log("✅ Connected to ROS bridge");
//       setRosConnected(true);
//     });

//     ros.on("error", (err) => {
//       console.error("❌ ROS error:", err);
//       setRosConnected(false);
//     });

//     ros.on("close", () => {
//       console.log("🔌 ROS disconnected");
//       setRosConnected(false);
//     });

//     // Example of simulating Task Server connection
//     setTimeout(() => setTaskServerConnected(true), 1500);

//     const poseListener = new ROSLIB.Topic({
//       ros,
//       name: "/amcl_pose",
//       messageType: "geometry_msgs/PoseWithCovarianceStamped",
//     });

//     poseListener.subscribe((msg) => {
//       try {
//         const { x, y } = msg.pose.pose.position;
//         const prev = prevPositions.current["AGV-01"] || { x: 0, y: 0 };
//         const dx = Math.abs(x - prev.x);
//         const dy = Math.abs(y - prev.y);
//         const threshold = 0.001;
//         const status = dx > threshold || dy > threshold ? "RUNNING" : "IDLE";

//         setAgvs((prevAgvs) =>
//           prevAgvs.map((agv) =>
//             agv.id === "AGV-01" ? { ...agv, status, location: status } : agv
//           )
//         );

//         prevPositions.current["AGV-01"] = { x, y };
//         setCursorCoords({ rosX: x, rosY: y });
//       } catch (err) {
//         console.error("Failed to parse /amcl_pose:", err);
//       }
//     });

//     return () => poseListener.unsubscribe();
//   }, []);

//   const isMobile = window.innerWidth < 768;

//   return (
//     <div className="p-4 sm:p-6 space-y-6 sm:space-y-8 min-h-screen bg-gray-50 dark:bg-gray-900 select-none">

//       {/* 🌐 STATUS BAR SECTION (Your provided code integrated here) */}
//       <div
//         style={{
//           marginBottom: "20px",
//           padding: isMobile ? "12px" : "16px",
//           background:
//             "linear-gradient(135deg, var(--status-bg-start, #1c91dfff) 0%, var(--status-bg-end, #2c69c5ff) 100%)",
//           borderRadius: "12px",
//           border: "1px solid var(--status-border, rgba(255,255,255,0.2))",
//           boxShadow: "var(--status-shadow, 0 4px 15px rgba(0,0,0,0.1))",
//           color: "var(--status-text, white)",
//           fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
//         }}
//       >
//         {/* Header */}
//         <div
//           style={{
//             display: "flex",
//             alignItems: "center",
//             justifyContent: "space-between",
//             marginBottom: "12px",
//             paddingBottom: "8px",
//             borderBottom:
//               "1px solid var(--status-header-border, rgba(255,255,255,0.3))",
//           }}
//         >
//           <h3
//             style={{
//               margin: 0,
//               fontSize: isMobile ? "1rem" : "1.1rem",
//               fontWeight: "700",
//               display: "flex",
//               alignItems: "center",
//               gap: "8px",
//               color: "var(--status-heading, white)",
//             }}
//           >
//             📊 System Status
//           </h3>

//           {/* Overall Connection Indicator */}
//           <div
//             style={{
//               display: "flex",
//               alignItems: "center",
//               gap: "0.5rem",
//               padding: "0.5rem 0.75rem",
//               background:
//                 rosConnected && taskServerConnected
//                   ? "linear-gradient(135deg, #10b981, #059669)"
//                   : "linear-gradient(135deg, #ef4444, #dc2626)",
//               borderRadius: "8px",
//               border:
//                 rosConnected && taskServerConnected
//                   ? "1px solid rgba(74, 222, 128, 0.3)"
//                   : "1px solid rgba(248, 113, 113, 0.3)",
//               boxShadow:
//                 rosConnected && taskServerConnected
//                   ? "0 2px 8px rgba(16, 185, 129, 0.3)"
//                   : "0 2px 8px rgba(239, 68, 68, 0.3)",
//             }}
//           >
//             <div
//               style={{
//                 width: "12px",
//                 height: "12px",
//                 borderRadius: "50%",
//                 background:
//                   rosConnected && taskServerConnected ? "#4ade80" : "#f87171",
//                 boxShadow:
//                   rosConnected && taskServerConnected
//                     ? "0 0 12px rgba(74, 222, 128, 0.8)"
//                     : "0 0 12px rgba(248, 113, 113, 0.6)",
//               }}
//             ></div>
//             <span style={{ fontSize: "0.85rem", fontWeight: "600" }}>
//               {rosConnected && taskServerConnected
//                 ? "All Systems Connected"
//                 : "Connection Issues"}
//             </span>
//           </div>
//         </div>

//         {/* Coordinates & Connection Section */}
//         <div
//           style={{
//             display: "grid",
//             gridTemplateColumns: isMobile ? "1fr" : "1fr 1fr",
//             gap: "12px",
//           }}
//         >
//           {/* Connections */}
//           <div
//             style={{
//               background: "rgba(255,255,255,0.15)",
//               padding: "12px",
//               borderRadius: "8px",
//               backdropFilter: "blur(10px)",
//               border: "1px solid rgba(255,255,255,0.1)",
//             }}
//           >
//             <div
//               style={{
//                 fontSize: "0.8rem",
//                 fontWeight: "700",
//                 color: "white",
//                 marginBottom: "10px",
//               }}
//             >
//               🔗 CONNECTIONS
//             </div>

//             <div className="flex justify-between mb-2">
//               <span>ROS Bridge</span>
//               <span>{rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</span>
//             </div>

//             <div className="flex justify-between">
//               <span>Task Server</span>
//               <span>{taskServerConnected ? "🟢 Connected" : "🔴 Disconnected"}</span>
//             </div>
//           </div>

//           {/* Coordinates */}
//           <div
//             style={{
//               background: "rgba(255,255,255,0.15)",
//               padding: "12px",
//               borderRadius: "8px",
//               backdropFilter: "blur(10px)",
//               border: "1px solid rgba(255,255,255,0.1)",
//             }}
//           >
//             <div
//               style={{
//                 fontSize: "0.8rem",
//                 fontWeight: "700",
//                 color: "white",
//                 marginBottom: "10px",
//               }}
//             >
//               📍 COORDINATES
//             </div>
//             <div className="flex justify-between text-white">
//               <span>ROS (m)</span>
//               <span>
//                 {cursorCoords
//                   ? `${cursorCoords.rosX.toFixed(2)}, ${cursorCoords.rosY.toFixed(2)}`
//                   : "N/A"}
//               </span>
//             </div>
//             <div className="flex justify-between text-white mt-2">
//               <span>Z-Level</span>
//               <span
//                 style={{
//                   color:
//                     zAxisValue === 0
//                       ? "white"
//                       : zAxisValue > 0
//                       ? "#4ade80"
//                       : "#f87171",
//                 }}
//               >
//                 {zAxisValue}
//               </span>
//             </div>
//           </div>
//         </div>
//       </div>

//       {/* 🚚 Fleet Overview */}
//       <section>
//         <h2 className="text-lg sm:text-xl font-semibold mb-3 sm:mb-4 text-sky-900 dark:text-sky-300">
//           Fleet Overview
//         </h2>
//         <div className="grid grid-cols-1 sm:grid-cols-2 md:grid-cols-3 lg:grid-cols-4 gap-4 sm:gap-6">
//           <Card title="Total AGVs" value={agvs.length} footer="AGV Distribution Chart" />
//           <Card
//             title="Active AGVs"
//             value={agvs.filter((a) => a.status === "RUNNING").length}
//             footer="Active AGVs Trend"
//           />
//           <Card
//             title="Avg. Battery"
//             value={
//               Math.round(
//                 agvs.reduce((sum, a) => sum + parseInt(a.battery), 0) / agvs.length
//               ) + "%"
//             }
//             footer="Battery Levels"
//           />
//           <Card title="Avg. Uptime" value="6h 45m" footer="Uptime History" />
//         </div>
//       </section>

//       {/* 🧭 AGV Status Table */}
//       <section className="hidden sm:block bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4 sm:p-6 overflow-x-auto">
//         <h3 className="font-semibold mb-4 text-sky-900 dark:text-sky-300 text-lg">
//           AGV Status
//         </h3>
//         <table className="w-full text-sm border-collapse">
//           <thead className="bg-sky-800 text-white dark:bg-sky-700">
//             <tr>
//               <th className="p-3 text-left">AGV ID</th>
//               <th className="p-3 text-left">Status</th>
//               <th className="p-3 text-left">Battery</th>
//               <th className="p-3 text-left">Task</th>
//               <th className="p-3 text-left">Location</th>
//             </tr>
//           </thead>
//           <tbody className="text-gray-700 dark:text-gray-200">
//             {agvs.map((agv) => (
//               <tr key={agv.id} className="border-t dark:border-gray-700 hover:bg-sky-50 dark:hover:bg-gray-700">
//                 <td className="p-3 font-medium">{agv.id}</td>
//                 <td className="p-3"><StatusBadge status={agv.status} /></td>
//                 <td className="p-3">{agv.battery}</td>
//                 <td className="p-3">{agv.task}</td>
//                 <td className="p-3">{agv.location}</td>
//               </tr>
//             ))}
//           </tbody>
//         </table>
//       </section>
//     </div>
//   );
// }

import React, { useEffect, useState, useRef } from "react";
import ROSLIB from "roslib";

// Recharts
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

// Colors for charts
const COLORS = ["#0ea5e9", "#22c55e", "#eab308", "#ef4444", "#6366f1"];

// Card component
const Card = ({ title, value, footer }) => (
  <div className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4 sm:p-6 w-full transition-transform hover:scale-105">
    <div className="text-xs sm:text-sm text-slate-500 dark:text-gray-400">{title}</div>
    <div className="text-2xl sm:text-3xl font-bold mt-2 text-sky-800 dark:text-sky-300">{value}</div>
    <div className="mt-2 sm:mt-3 text-[10px] sm:text-xs text-slate-400 dark:text-gray-500">{footer}</div>
  </div>
);

// Badge Component
const StatusBadge = ({ status }) => {
  const colors = {
    RUNNING: "bg-green-100 text-green-700",
    FAULT: "bg-red-100 text-red-700",
    IDLE: "bg-yellow-100 text-yellow-700",
    Offline: "bg-gray-100 text-gray-700",
  };

  return (
    <span
      className={`px-2 py-1 rounded-full text-[10px] sm:text-xs font-medium ${
        colors[status] || "bg-gray-100 text-gray-700"
      }`}
    >
      {status}
    </span>
  );
};

export default function Dashboard() {
  const [agvs, setAgvs] = useState([
    { id: "AGV-01", status: "IDLE", battery: "85%", task: "Material Transport", location: "Zone A" },
    { id: "AGV-02", status: "IDLE", battery: "72%", task: "Assembly Line", location: "Zone B" },
    { id: "AGV-03", status: "IDLE", battery: "92%", task: "Idle", location: "Charging Station" },
    { id: "AGV-04", status: "IDLE", battery: "65%", task: "Warehouse Pick", location: "Zone D" },
    { id: "AGV-05", status: "Offline", battery: "10%", task: "Idle", location: "Maintenance" },
  ]);

  const [rosConnected, setRosConnected] = useState(false);
  const [taskServerConnected, setTaskServerConnected] = useState(false);
  const [cursorCoords, setCursorCoords] = useState(null);
  const [zAxisValue, setZAxisValue] = useState(0);
  const prevPositions = useRef({});

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

    ros.on("connection", () => setRosConnected(true));
    ros.on("error", () => setRosConnected(false));
    ros.on("close", () => setRosConnected(false));

    setTimeout(() => setTaskServerConnected(true), 1500);

    const poseListener = new ROSLIB.Topic({
      ros,
      name: "/amcl_pose",
      messageType: "geometry_msgs/PoseWithCovarianceStamped",
    });

    poseListener.subscribe((msg) => {
      try {
        const { x, y } = msg.pose.pose.position;
        const prev = prevPositions.current["AGV-01"] || { x: 0, y: 0 };
        const status =
          Math.abs(x - prev.x) > 0.001 || Math.abs(y - prev.y) > 0.001
            ? "RUNNING"
            : "IDLE";

        setAgvs((prevAgvs) =>
          prevAgvs.map((agv) =>
            agv.id === "AGV-01" ? { ...agv, status } : agv
          )
        );

        prevPositions.current["AGV-01"] = { x, y };
        setCursorCoords({ rosX: x, rosY: y });
      } catch {}
    });

    return () => poseListener.unsubscribe();
  }, []);

  const isMobile = window.innerWidth < 768;

  // -------------------- 📊 Pie Chart Data ---------------------
  const pieData = [
    { name: "Running", value: agvs.filter((a) => a.status === "RUNNING").length },
    { name: "Idle", value: agvs.filter((a) => a.status === "IDLE").length },
    { name: "Offline", value: agvs.filter((a) => a.status === "Offline").length },
  ];

  // -------------------- 📊 Bar Graph Data ---------------------
  const batteryData = agvs.map((a) => ({
    name: a.id,
    battery: parseInt(a.battery),
  }));

  return (
    <div className="p-4 sm:p-6 space-y-8 bg-gray-50 dark:bg-gray-900">

      {/* ================= 🌐 STATUS BAR ================= */}
      <div
        className="rounded-2xl text-white shadow-md"
        style={{
          padding: isMobile ? "12px" : "16px",
          background: "linear-gradient(135deg, #1c91df, #2c69c5)",
        }}
      >
        <h3 className="text-lg font-bold mb-3 flex items-center gap-2">
          📊 System Status
        </h3>

        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">

          {/* CONNECTIONS */}
          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-semibold mb-2 text-sm">🔗 CONNECTIONS</p>
            <div className="flex justify-between">
              <span>ROS Bridge</span>
              <span>{rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</span>
            </div>
            <div className="flex justify-between mt-1">
              <span>Task Server</span>
              <span>{taskServerConnected ? "🟢 Connected" : "🔴 Disconnected"}</span>
            </div>
          </div>

          {/* COORDINATES */}
          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p className="font-semibold mb-2 text-sm">📍 COORDINATES</p>
            <div className="flex justify-between">
              <span>ROS (m)</span>
              <span>
                {cursorCoords ? `${cursorCoords.rosX.toFixed(2)}, ${cursorCoords.rosY.toFixed(2)}` : "N/A"}
              </span>
            </div>
            <div className="flex justify-between mt-1">
              <span>Z Level</span>
              <span>{zAxisValue}</span>
            </div>
          </div>

        </div>
      </div>

      {/* ================= 🚚 FLEET OVERVIEW ================= */}
      <section>
        <h2 className="text-xl font-semibold mb-4 text-sky-900 dark:text-sky-300">Fleet Overview</h2>

        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
          <Card title="Total AGVs" value={agvs.length} footer="AGV Distribution" />
          <Card title="Active AGVs" value={pieData[0].value} footer="Running" />
          <Card
            title="Avg Battery"
            value={
              Math.round(
                agvs.reduce((s, a) => s + parseInt(a.battery), 0) / agvs.length
              ) + "%"
            }
            footer="Battery Stats"
          />
          <Card title="Avg Uptime" value="6h 45m" footer="Today’s Uptime" />
        </div>
      </section>

      {/* ================= 📈 CHARTS SECTION ================= */}
      <section className="grid grid-cols-1 md:grid-cols-2 gap-6">

        {/* PIE CHART */}
        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold text-sky-900 dark:text-sky-300 mb-3">
            AGV Status Distribution
          </h3>
          <ResponsiveContainer width="100%" height={260}>
            <PieChart>
              <Pie
                data={pieData}
                cx="50%"
                cy="50%"
                labelLine={false}
                outerRadius={90}
                fill="#8884d8"
                dataKey="value"
                label
              >
                {pieData.map((_, i) => (
                  <Cell key={i} fill={COLORS[i % COLORS.length]} />
                ))}
              </Pie>
              <Tooltip />
              <Legend />
            </PieChart>
          </ResponsiveContainer>
        </div>

        {/* BAR GRAPH */}
        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold text-sky-900 dark:text-sky-300 mb-3">
            Battery Level Overview
          </h3>
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

      {/* ================= 📋 TABLE ================= */}
      <section className="hidden sm:block bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
        <h3 className="font-semibold mb-4 text-sky-900 dark:text-sky-300 text-lg">
          AGV Status
        </h3>

        <table className="w-full text-sm">
          <thead className="bg-sky-800 text-white dark:bg-sky-700">
            <tr>
              <th className="p-3 text-left">AGV ID</th>
              <th className="p-3 text-left">Status</th>
              <th className="p-3 text-left">Battery</th>
              <th className="p-3 text-left">Task</th>
              <th className="p-3 text-left">Location</th>
            </tr>
          </thead>

          <tbody>
            {agvs.map((agv) => (
              <tr
                key={agv.id}
                className="border-t dark:border-gray-700 hover:bg-sky-50 dark:hover:bg-gray-700"
              >
                <td className="p-3">{agv.id}</td>
                <td className="p-3"><StatusBadge status={agv.status} /></td>
                <td className="p-3">{agv.battery}</td>
                <td className="p-3">{agv.task}</td>
                <td className="p-3">{agv.location}</td>
              </tr>
            ))}
          </tbody>
        </table>

      </section>
    </div>
  );
}
