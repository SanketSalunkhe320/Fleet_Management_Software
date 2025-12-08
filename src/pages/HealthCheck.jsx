import React, { useEffect, useState } from "react";
import { 
  FaBatteryFull, 
  FaThermometerHalf, 
  FaSatelliteDish, 
  FaCamera, 
  FaCogs,
  FaMapMarkerAlt
} from "react-icons/fa";

export default function HealthCheck() {
  const [robotStatus, setRobotStatus] = useState("FAULT");
  const [imuStatus, setImuStatus] = useState("FAULT");
  const [lidarStatus, setLidarStatus] = useState("FAULT");
  const [cameraStatus, setCameraStatus] = useState("FAULT");
  const [motorStatus, setMotorStatus] = useState("FAULT");
  const [robotPosition, setRobotPosition] = useState("UNKNOWN");

  const fetchStatus = async () => {
    try {
      const res = await fetch("http://localhost:5000/status");
      const data = await res.json();

      setRobotStatus(data.robot_status || "FAULT");
      setImuStatus(data.imu_status || "FAULT");
      setLidarStatus(data.lidar_status || "FAULT");
      setCameraStatus(data.camera_status || "FAULT");
      setMotorStatus(data.motor_status || "FAULT");

      // Position may be "x,y" string OR full object
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

    } catch (e) {
      setRobotStatus("FAULT");
      setImuStatus("FAULT");
      setLidarStatus("FAULT");
      setCameraStatus("FAULT");
      setMotorStatus("FAULT");
      setRobotPosition("UNKNOWN");
    }
  };

  useEffect(() => {
    fetchStatus();
    const interval = setInterval(fetchStatus, 500);
    return () => clearInterval(interval);
  }, []);

  const metrics = [
    { name: "AMR", value: robotStatus, icon: <FaBatteryFull className="text-xl text-blue-500" /> },
    { name: "IMU", value: imuStatus, icon: <FaThermometerHalf className="text-xl text-green-500" /> },
    { name: "LIDAR", value: lidarStatus, icon: <FaSatelliteDish className="text-xl text-orange-500" /> },
    { name: "Depth Camera", value: cameraStatus, icon: <FaCamera className="text-xl text-purple-500" /> },
    { name: "Motor Drive", value: motorStatus, icon: <FaCogs className="text-xl text-indigo-500" /> },
    // { name: "Position", value: robotPosition, icon: <FaMapMarkerAlt className="text-xl text-red-500" /> },
  ];

  const getStatusColor = (status) => {
    if (!status) return "bg-gray-100 text-gray-700";

    const s = status.toUpperCase();

    if (s === "OK" || s === "RUNNING") 
      return "bg-green-100 dark:bg-green-900 text-green-700 dark:text-green-200";

    if (s === "FAULT") 
      return "bg-red-100 dark:bg-red-900 text-red-700 dark:text-red-200";

    if (s === "IDLE") 
      return "bg-yellow-100 dark:bg-yellow-900 text-yellow-700 dark:text-yellow-200";

    return "bg-gray-100 dark:bg-gray-800 text-gray-700 dark:text-gray-200";
  };



  
  return (
    <div className="text-slate-900 dark:text-slate-100 select-none">
      <h2 className="text-xl font-semibold mb-4">Health Check</h2>

      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
        {metrics.map((m) => (
          <div
            key={m.name}
            className="bg-white dark:bg-slate-800 rounded-xl shadow hover:shadow-lg transition p-4 flex items-center justify-between"
          >
            <div className="flex items-center gap-3">
              <div className="p-2 rounded-full bg-slate-100 dark:bg-slate-700">
                {m.icon}
              </div>
              <span className="font-medium">{m.name}</span>
            </div>

            <span
              className={`px-3 py-1 text-sm font-semibold rounded-full ${getStatusColor(m.value)}`}
            >
              {m.value}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}
