// import React, { useEffect, useState } from "react";
// import { 
//   FaBatteryFull, 
//   FaThermometerHalf, 
//   FaSatelliteDish, 
//   FaCamera, 
//   FaCogs,
//   FaMapMarkerAlt
// } from "react-icons/fa";
// import { useLanguage } from "../context/LanguageContext";

// export default function HealthCheck() {
//   const { t } = useLanguage();
  
//   const [robotStatus, setRobotStatus] = useState("FAULT");
//   const [imuStatus, setImuStatus] = useState("FAULT");
//   const [lidarStatus, setLidarStatus] = useState("FAULT");
//   const [cameraStatus, setCameraStatus] = useState("FAULT");
//   const [motorStatus, setMotorStatus] = useState("FAULT");
//   const [robotPosition, setRobotPosition] = useState("UNKNOWN");

//   const fetchStatus = async () => {
//     try {
//       const res = await fetch("http://localhost:5000/status");
//       const data = await res.json();

//       setRobotStatus(data.robot_status || "FAULT");
//       setImuStatus(data.imu_status || "FAULT");
//       setLidarStatus(data.lidar_status || "FAULT");
//       setCameraStatus(data.camera_status || "FAULT");
//       setMotorStatus(data.motor_status || "FAULT");

//       // Position may be "x,y" string OR full object
//       if (data.robot_position) {
//         if (typeof data.robot_position === "string") {
//           setRobotPosition(data.robot_position);
//         } else if (data.robot_position.x && data.robot_position.y) {
//           setRobotPosition(`${data.robot_position.x.toFixed(2)}, ${data.robot_position.y.toFixed(2)}`);
//         } else {
//           setRobotPosition("UNKNOWN");
//         }
//       } else {
//         setRobotPosition("UNKNOWN");
//       }

//     } catch (e) {
//       setRobotStatus("FAULT");
//       setImuStatus("FAULT");
//       setLidarStatus("FAULT");
//       setCameraStatus("FAULT");
//       setMotorStatus("FAULT");
//       setRobotPosition("UNKNOWN");
//     }
//   };

//   useEffect(() => {
//     fetchStatus();
//     const interval = setInterval(fetchStatus, 500);
//     return () => clearInterval(interval);
//   }, []);

//   // Create metrics array with translated names
//   const metrics = [
//     { 
//       name: t("health.amr"), 
//       value: robotStatus, 
//       icon: <FaBatteryFull className="text-xl text-blue-500" /> 
//     },
//     { 
//       name: t("health.imu"), 
//       value: imuStatus, 
//       icon: <FaThermometerHalf className="text-xl text-green-500" /> 
//     },
//     { 
//       name: t("health.lidar"), 
//       value: lidarStatus, 
//       icon: <FaSatelliteDish className="text-xl text-orange-500" /> 
//     },
//     { 
//       name: t("health.camera"), 
//       value: cameraStatus, 
//       icon: <FaCamera className="text-xl text-purple-500" /> 
//     },
//     { 
//       name: t("health.motor"), 
//       value: motorStatus, 
//       icon: <FaCogs className="text-xl text-indigo-500" /> 
//     },
//     // { 
//     //   name: t("health.position"), 
//     //   value: robotPosition, 
//     //   icon: <FaMapMarkerAlt className="text-xl text-red-500" /> 
//     // },
//   ];

//   const getStatusColor = (status) => {
//     if (!status) return "bg-gray-100 text-gray-700";

//     const s = status.toUpperCase();

//     if (s === "OK" || s === "RUNNING") 
//       return "bg-green-100 dark:bg-green-900 text-green-700 dark:text-green-200";

//     if (s === "FAULT") 
//       return "bg-red-100 dark:bg-red-900 text-red-700 dark:text-red-200";

//     if (s === "IDLE") 
//       return "bg-yellow-100 dark:bg-yellow-900 text-yellow-700 dark:text-yellow-200";

//     return "bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-200";
//   };

//   return (
//     <div className="text-slate-900 dark:text-slate-100 select-none">
//       <h2 className="text-xl font-semibold mb-4">{t("health_check")}</h2>

//       <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
//         {metrics.map((m) => (
//           <div
//             key={m.name}
//             className="bg-white dark:bg-slate-800 rounded-xl shadow hover:shadow-lg transition p-4 flex items-center justify-between"
//           >
//             <div className="flex items-center gap-3">
//               <div className="p-2 rounded-full bg-slate-100 dark:bg-slate-700">
//                 {m.icon}
//               </div>
//               <span className="font-medium">{m.name}</span>
//             </div>

//             <span
//               className={`px-3 py-1 text-sm font-semibold rounded-full ${getStatusColor(m.value)}`}
//             >
//               {m.value}
//             </span>
//           </div>
//         ))}
//       </div>
//     </div>
//   );
// }

import React, { useEffect, useState } from "react";
import { 
  FaBatteryFull, 
  FaThermometerHalf, 
  FaSatelliteDish, 
  FaCamera, 
  FaCogs,
  FaMapMarkerAlt,
  FaMicrochip,
  FaHdd,
  FaMemory,
  FaRobot,
  FaNetworkWired,
  FaMapSigns,
  FaRoute,
  FaBrain,
  FaChartLine
} from "react-icons/fa";
import { useLanguage } from "../context/LanguageContext";

export default function HealthCheck() {
  const { t } = useLanguage();
  
  // Existing state
  const [robotStatus, setRobotStatus] = useState("FAULT");
  const [imuStatus, setImuStatus] = useState("FAULT");
  const [lidarStatus, setLidarStatus] = useState("FAULT");
  const [cameraStatus, setCameraStatus] = useState("FAULT");
  const [motorStatus, setMotorStatus] = useState("FAULT");
  const [batteryStatus, setBatteryStatus] = useState("FAULT");
  const [robotPosition, setRobotPosition] = useState("UNKNOWN");

  // New hardware state
  const [hardwareStatus, setHardwareStatus] = useState({
    encoder: "FAULT",
    brakes: "FAULT",
    cpuUsage: 0,
    diskUsage: 0,
    memoryUsage: 0
  });

  // New ROS2 state
  const [ros2Status, setRos2Status] = useState({
    nodes: 0,
    topics: 0,
    nav2Stack: "FAULT",
    localization: "FAULT",
    pathPlanner: "FAULT"
  });

  const fetchStatus = async () => {
    try {
      const res = await fetch("http://localhost:5000/status");
      const data = await res.json();

      // Update sensor status
      setRobotStatus(data.robot_status || "FAULT");
      setImuStatus(data.imu_status || "FAULT");
      setLidarStatus(data.lidar_status || "FAULT");
      setCameraStatus(data.camera_status || "FAULT");
      setMotorStatus(data.motor_status || "FAULT");

      // Update battery status - simple OK/FAULT logic
      const batteryValue = data.battery;
      if (batteryValue === "N/A" || batteryValue === undefined || batteryValue === null) {
        setBatteryStatus("FAULT");
      } else if (parseInt(batteryValue) > 10) { // If battery > 10%, it's OK
        setBatteryStatus("OK");
      } else {
        setBatteryStatus("LOW"); // Battery is very low
      }

      // Update position
      if (data.robot_position) {
        if (typeof data.robot_position === "string") {
          setRobotPosition(data.robot_position);
        } else if (data.robot_position.x && data.robot_position.y) {
          setRobotPosition(`${data.robot_position.x.toFixed(2)}, ${data.robot_position.y.toFixed(2)}`);
        } else {
          setRobotPosition("UNKNOWN");
        }
      } else {
        setRobotPosition("UNKNOWN");
      }

      // Simulate hardware status (replace with real API calls)
      setHardwareStatus({
        encoder: Math.random() > 0.7 ? "OK" : "FAULT",
        brakes: Math.random() > 0.8 ? "OK" : "FAULT",
        cpuUsage: Math.floor(Math.random() * 100),
        diskUsage: Math.floor(Math.random() * 60) + 20,
        memoryUsage: Math.floor(Math.random() * 85) + 15
      });

      // Simulate ROS2 status (replace with real API calls)
      setRos2Status({
        nodes: Math.floor(Math.random() * 10) + 5,
        topics: Math.floor(Math.random() * 50) + 20,
        nav2Stack: Math.random() > 0.3 ? "OK" : "FAULT",
        localization: Math.random() > 0.4 ? "OK" : "FAULT",
        pathPlanner: Math.random() > 0.5 ? "OK" : "FAULT"
      });

    } catch (e) {
      // Set all to fault state
      setRobotStatus("FAULT");
      setImuStatus("FAULT");
      setLidarStatus("FAULT");
      setCameraStatus("FAULT");
      setMotorStatus("FAULT");
      setBatteryStatus("FAULT");
      setRobotPosition("UNKNOWN");
      
      // Reset hardware and ROS2
      setHardwareStatus({
        encoder: "FAULT",
        brakes: "FAULT",
        cpuUsage: 0,
        diskUsage: 0,
        memoryUsage: 0
      });
      
      setRos2Status({
        nodes: 0,
        topics: 0,
        nav2Stack: "FAULT",
        localization: "FAULT",
        pathPlanner: "FAULT"
      });
    }
  };

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, []);

  const getStatusColor = (status, isProgressBar = false) => {
    if (!status) return isProgressBar ? "bg-gray-200" : "bg-gray-100 text-gray-700";

    const s = status.toString().toUpperCase();

    if (s === "OK" || s === "RUNNING") 
      return isProgressBar ? "bg-green-500" : "bg-green-100 dark:bg-green-900 text-green-700 dark:text-green-200";

    if (s === "FAULT" || s === "OFFLINE" || s === "ERROR") 
      return isProgressBar ? "bg-red-500" : "bg-red-100 dark:bg-red-900 text-red-700 dark:text-red-200";

    if (s === "IDLE" || s === "WARNING" || s === "LOW") 
      return isProgressBar ? "bg-yellow-500" : "bg-yellow-100 dark:bg-yellow-900 text-yellow-700 dark:text-yellow-200";

    return isProgressBar ? "bg-gray-200" : "bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-200";
  };

  const getUsageColor = (percentage) => {
    if (percentage > 80) return "bg-red-500";
    if (percentage > 60) return "bg-yellow-500";
    return "bg-green-500";
  };

  // Section 1: Sensor Health Check
  const sensorMetrics = [
    { 
      name: t("health.amr"), 
      value: robotStatus, 
      icon: <FaRobot className="text-xl text-blue-500" /> 
    },
    { 
      name: t("health.imu"), 
      value: imuStatus, 
      icon: <FaThermometerHalf className="text-xl text-green-500" /> 
    },
    { 
      name: t("health.lidar"), 
      value: lidarStatus, 
      icon: <FaSatelliteDish className="text-xl text-orange-500" /> 
    },
    { 
      name: t("health.camera"), 
      value: cameraStatus, 
      icon: <FaCamera className="text-xl text-purple-500" /> 
    },
    { 
      name: "Battery", 
      value: batteryStatus, 
      icon: <FaBatteryFull className="text-xl text-red-500" /> 
    },
  ];

  // Section 2: Hardware Health
  const hardwareMetrics = [
    { 
      name: "Motor Controller", 
      value: motorStatus, 
      icon: <FaCogs className="text-xl text-indigo-500" /> 
    },
    { 
      name: "Encoder", 
      value: hardwareStatus.encoder, 
      icon: <FaChartLine className="text-xl text-blue-500" /> 
    },
    { 
      name: "Brakes", 
      value: hardwareStatus.brakes, 
      icon: <FaCogs className="text-xl text-red-500" /> 
    },
    { 
      name: "CPU Usage", 
      value: `${hardwareStatus.cpuUsage}%`, 
      icon: <FaMicrochip className="text-xl text-green-500" />,
      isProgress: true,
      percentage: hardwareStatus.cpuUsage
    },
    { 
      name: "Memory Usage", 
      value: `${hardwareStatus.memoryUsage}%`, 
      icon: <FaMemory className="text-xl text-yellow-500" />,
      isProgress: true,
      percentage: hardwareStatus.memoryUsage
    },
    { 
      name: "Disk Usage", 
      value: `${hardwareStatus.diskUsage}%`, 
      icon: <FaHdd className="text-xl text-purple-500" />,
      isProgress: true,
      percentage: hardwareStatus.diskUsage
    },
  ];

  // Section 3: Software/ROS2
  const softwareMetrics = [
    { 
      name: "ROS2 Nodes", 
      value: ros2Status.nodes.toString(), 
      icon: <FaNetworkWired className="text-xl text-blue-500" /> 
    },
    { 
      name: "ROS2 Topics", 
      value: ros2Status.topics.toString(), 
      icon: <FaNetworkWired className="text-xl text-green-500" /> 
    },
    { 
      name: "Nav2 Stack", 
      value: ros2Status.nav2Stack, 
      icon: <FaMapSigns className="text-xl text-purple-500" /> 
    },
    { 
      name: "Localization", 
      value: ros2Status.localization, 
      icon: <FaMapMarkerAlt className="text-xl text-red-500" /> 
    },
    { 
      name: "Path Planner", 
      value: ros2Status.pathPlanner, 
      icon: <FaRoute className="text-xl text-indigo-500" /> 
    },
    { 
      name: "ROS Master", 
      value: "OK", 
      icon: <FaBrain className="text-xl text-orange-500" /> 
    },
  ];

  // Health card component
  const HealthCard = ({ metric }) => (
    <div
      className="bg-white dark:bg-slate-800 rounded-xl shadow hover:shadow-lg transition p-4 flex items-center justify-between"
    >
      <div className="flex items-center gap-3">
        <div className="p-2 rounded-full bg-slate-100 dark:bg-slate-700">
          {metric.icon}
        </div>
        <div>
          <span className="font-medium block">{metric.name}</span>
          {metric.isProgress && (
            <div className="w-24 h-1.5 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden mt-1">
              <div 
                className={`h-full ${getUsageColor(metric.percentage)}`}
                style={{ width: `${metric.percentage}%` }}
              ></div>
            </div>
          )}
        </div>
      </div>

      <span
        className={`px-3 py-1 text-sm font-semibold rounded-full ${getStatusColor(metric.value)}`}
      >
        {metric.value}
      </span>
    </div>
  );

  return (
    <div className="text-slate-900 dark:text-slate-100 select-none space-y-8">
      {/* Section 1: Sensor Health Check */}
      <div>
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-semibold">📡 Sensor Health Check</h2>
          <span className="text-xs text-gray-500">Real-time monitoring</span>
        </div>
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-5">
          {sensorMetrics.map((m) => (
            <HealthCard key={m.name} metric={m} />
          ))}
        </div>
      </div>

      {/* Section 2: Hardware Health */}
      <div>
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-semibold">⚙️ Hardware Health</h2>
          <span className="text-xs text-gray-500">Physical components</span>
        </div>
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {hardwareMetrics.map((m) => (
            <HealthCard key={m.name} metric={m} />
          ))}
        </div>
      </div>

      {/* Section 3: Software/ROS2 Health */}
      <div>
        <div className="flex items-center justify-between mb-4">
          <h2 className="text-xl font-semibold">🤖 Software & ROS2 Health</h2>
          <span className="text-xs text-gray-500">System software</span>
        </div>
        <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
          {softwareMetrics.map((m) => (
            <HealthCard key={m.name} metric={m} />
          ))}
        </div>
      </div>
    </div>
  );
}