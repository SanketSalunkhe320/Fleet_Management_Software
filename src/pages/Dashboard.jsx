// // ---------------------------------------------
// // CLEAN + FIXED DASHBOARD (ALL STATUS MATCHED)
// // ---------------------------------------------


// import React, { useEffect, useState, useRef } from "react";
// import ROSLIB from "roslib";

// // Recharts
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

// // Running, Idle, Offline colors
// const COLORS = ["#0ea5e9", "#22c55e", "#ef4444"];

// // Status badge color map — ALL UPPERCASE
// const StatusBadge = ({ status }) => {
//   const map = {
//     RUNNING: "bg-green-100 text-green-700",
//     IDLE: "bg-yellow-100 text-yellow-700",
//     OFFLINE: "bg-gray-100 text-gray-700",
//     FAULT: "bg-red-100 text-red-700",
//     OBSTACLE: "bg-red-100 text-red-700",
//     CLEAR: "bg-green-100 text-green-700",
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

// // Battery badge
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
//   const [taskServerConnected, setTaskServerConnected] = useState(false);
//   const prevPositions = useRef({});
//   const fetchIntervalRef = useRef(null);

//   // -------------------------------
//   // ROS Bridge Connection
//   // -------------------------------
//   useEffect(() => {
//     const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

//     ros.on("connection", () => setRosConnected(true));
//     ros.on("error", () => setRosConnected(false));
//     ros.on("close", () => setRosConnected(false));

//     // subscribe to pose
//     const poseListener = new ROSLIB.Topic({
//       ros,
//       name: "/amcl_pose",
//       messageType: "geometry_msgs/PoseWithCovarianceStamped",
//     });

//   poseListener.subscribe((msg) => {
//   const x = Number(msg.pose.pose.position.x || 0);
//   const y = Number(msg.pose.pose.position.y || 0);

//   setAgvs((prev) =>
//     prev.map((a) => ({
//       ...a,
//       location: `X:${x.toFixed(2)}, Y:${y.toFixed(2)}`
//     }))
//   );
//     });

//     setTimeout(() => setTaskServerConnected(true), 1200);

//     return () => {
//       poseListener.unsubscribe();
//       ros.close();
//     };
//   }, []);

//   // --------------------------------
//   // Fetch Backend Status
//   // --------------------------------
//   useEffect(() => {
//     const fetchStatus = async () => {
//       try {
//         const res = await fetch("http://localhost:5002/status");
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
//         setAgvs([
//           {
//             id: "AMR-01",
//             status: "OFFLINE",
//             battery: "N/A",
//             task: "N/A",
//             location: "N/A",
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

//   // --------------------------------
//   // Pie + Bar Chart Data
//   // --------------------------------
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
//       {/* STATUS BAR */}
//       <div className="rounded-2xl text-white shadow-md p-5"
//         style={{ background: "linear-gradient(135deg, #1c91df, #2c69c5)" }}
//       >
//         <h3 className="text-lg font-bold mb-3">📊 System Status</h3>

//         <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
//           <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
//             <p>ROS Bridge: {rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</p>
//             {/* <p>Task Server: {taskServerConnected ? "🟢 Connected" : "🔴 Disconnected"}</p> */}
//           </div>

//           <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
//             <p>ROS Position: {agvs[0].location}</p>
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

//       {/* AGV TABLE */}
//       <section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
//         <h3 className="font-semibold mb-4 text-lg">AMR Status</h3>
//         <table className="w-full text-sm">
//           <thead className="bg-sky-800 text-white">
//             <tr>
//               <th className="p-3 text-left">AMR ID</th>
//               <th className="p-3 text-left">Status</th>
//               <th className="p-3 text-left">Battery</th>
//               <th className="p-3 text-left">Location</th>
//               <th className="p-3 text-left">Camera</th>
//               <th className="p-3 text-left">Lidar</th>
//             </tr>
//           </thead>

//           <tbody>
//             {agvs.map((agv) => (
//               <tr key={agv.id} className="border-t">
//                 <td className="p-3">{agv.id}</td>
//                 <td className="p-3"><StatusBadge status={agv.status} /></td>
//                 <td className="p-3"><BatteryBadge level={agv.battery} /></td>
//                 <td className="p-3">{agv.location}</td>
//                 <td className="p-3"><StatusBadge status={agv.camera} /></td>
//                 <td className="p-3"><StatusBadge status={agv.lidar} /></td>
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

// Chart colors
const COLORS = ["#0ea5e9", "#22c55e", "#ef4444"];

// Status Badge
const StatusBadge = ({ status }) => {
  const map = {
    RUNNING: "bg-green-100 text-green-700",
    IDLE: "bg-yellow-100 text-yellow-700",
    OFFLINE: "bg-gray-100 text-gray-700",
    FAULT: "bg-red-100 text-red-700",
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

export default function Dashboard() {
  const [agvs, setAgvs] = useState([
    {
      id: "AMR-01",
      status: "OFFLINE",
      battery: "N/A",
      task: "Idle",
      location: "Unknown",
      camera: "FAULT",
      lidar: "FAULT",
      speed: 0,
    },
  ]);

  const [rosConnected, setRosConnected] = useState(false);
  const fetchIntervalRef = useRef(null);

  // ----------------------------------------------------
  // ROS Bridge Connection (Only for location updates)
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

    // Only update location (NO RUNNING/IDLE logic)
    poseListener.subscribe((msg) => {
      const x = Number(msg.pose.pose.position.x || 0);
      const y = Number(msg.pose.pose.position.y || 0);

      setAgvs((prev) =>
        prev.map((a) => ({
          ...a,
          location: `X:${x.toFixed(2)}, Y:${y.toFixed(2)}`,
        }))
      );
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
        const res = await fetch("http://localhost:5002/status");
        const data = await res.json();

        setAgvs([
          {
            id: "AMR-01",
            status: (data.robot_status || "OFFLINE").toUpperCase(),

            battery:
              typeof data.battery === "number"
                ? `${data.battery}%`
                : "N/A",

            task: data.task || "Auto Mode",

            location:
              data.robot_position &&
              typeof data.robot_position.x === "number"
                ? `X:${data.robot_position.x.toFixed(2)}, Y:${data.robot_position.y.toFixed(2)}`
                : "Unknown",

            camera: (data.camera_status || "FAULT").toUpperCase(),
            lidar: (data.lidar_status || "FAULT").toUpperCase(),
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
            location: "Unknown",
            camera: "FAULT",
            lidar: "FAULT",
          },
        ]);
      }
    };

    fetchStatus();
    fetchIntervalRef.current = setInterval(fetchStatus, 1000);

    return () => clearInterval(fetchIntervalRef.current);
  }, []);

  // ----------------------------------------------------
  // Charts
  // ----------------------------------------------------
  const pieData = [
    { name: "Running", value: agvs.filter((a) => a.status === "RUNNING").length },
    { name: "Idle", value: agvs.filter((a) => a.status === "IDLE").length },
    { name: "Offline", value: agvs.filter((a) => a.status === "OFFLINE").length },
  ];

  const batteryData = agvs.map((a) => ({
    name: a.id,
    battery: parseInt(a.battery) || 0,
  }));

  return (
    <div className="p-4 sm:p-6 space-y-8 bg-gray-50 dark:bg-gray-900 min-h-screen">
      {/* TOP STATUS BAR */}
      <div
        className="rounded-2xl text-white shadow-md p-5"
        style={{ background: "linear-gradient(135deg, #1c91df, #2c69c5)" }}
      >
        <h3 className="text-lg font-bold mb-3">📊 System Status</h3>

        <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p>ROS Bridge: {rosConnected ? "🟢 Connected" : "🔴 Disconnected"}</p>
          </div>

          <div className="bg-white/20 p-3 rounded-xl backdrop-blur-md">
            <p>Robot Position: {agvs[0].location}</p>
            <p>Total AMRs: {agvs.length}</p>
          </div>
        </div>
      </div>

      {/* CHARTS */}
      <section className="grid grid-cols-1 md:grid-cols-2 gap-6">
        <div className="bg-white dark:bg-gray-800 p-4 rounded-2xl shadow-md">
          <h3 className="font-semibold mb-3">AMR Status Distribution</h3>
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
          <h3 className="font-semibold mb-3">Battery Level Overview</h3>
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
     {/* AMR TABLE */}
<section className="bg-white dark:bg-gray-800 rounded-2xl shadow-md p-4">
  <h3 className="font-semibold mb-4 text-lg">AMR Status</h3>

  <table className="w-full text-sm">
    <thead className="bg-sky-800 text-white">
      <tr>
        <th className="p-3 text-left">AMR ID</th>
        <th className="p-3 text-left">Status</th>
        <th className="p-3 text-left">Battery</th>
        <th className="p-3 text-left">Current Location</th>
        <th className="p-3 text-left">Target Location</th>
        <th className="p-3 text-left">Error</th>
      </tr>
    </thead>

    <tbody>
      {agvs.map((agv) => (
        <tr key={agv.id} className="border-t">
          <td className="p-3">{agv.id}</td>

          <td className="p-3">
            <StatusBadge status={agv.status} />
          </td>

          <td className="p-3">
            <BatteryBadge level={agv.battery} />
          </td>

          {/* Current location */}
          <td className="p-3">{agv.location}</td>

          {/* Target location */}
          <td className="p-3">{agv.targetLocation}</td>

          {/* Error */}
          <td className="p-3">
            <StatusBadge status={agv.error || "NONE"} />
          </td>
        </tr>
      ))}
    </tbody>
  </table>
</section>

    </div>
  );
}
