import React from "react";
import { FaClock, FaBatteryFull, FaTasks, FaHeartbeat } from "react-icons/fa";

export default function Reports() {
  const reports = [
    {
      id: 1,
      name: "Uptime Report",
      description: "Summary of AGV operational hours and downtime.",
      icon: <FaClock className="w-6 h-6 text-sky-600 dark:text-sky-400" />,
    },
    {
      id: 2,
      name: "Battery Report",
      description: "Battery charge/discharge cycles and performance trends.",
      icon: <FaBatteryFull className="w-6 h-6 text-green-600 dark:text-green-400" />,
    },
    {
      id: 3,
      name: "Task Report",
      description: "Completed, ongoing, and failed task statistics.",
      icon: <FaTasks className="w-6 h-6 text-indigo-600 dark:text-indigo-400" />,
    },
    {
      id: 4,
      name: "Health Report",
      description: "Motor, sensor, and safety health check summary.",
      icon: <FaHeartbeat className="w-6 h-6 text-red-600 dark:text-red-400" />,
    },
  ];

  const handleDownload = (reportName) => {
    alert(`📥 Downloading ${reportName}... (connect backend API here)`);
  };

  return (
    <div className="text-slate-900 dark:text-slate-100 select-none">
      <h2 className="text-xl font-semibold mb-4">Fleet Reports</h2>

      <div className="grid gap-6 sm:grid-cols-2">
        {reports.map((report) => (
          <div
            key={report.id}
            className="bg-white dark:bg-slate-800 rounded-xl shadow p-6 flex flex-col justify-between hover:shadow-lg transition"
          >
            <div className="flex items-center gap-3 mb-3">
              {report.icon}
              <h3 className="text-lg font-semibold">{report.name}</h3>
            </div>

            <p className="text-slate-500 dark:text-slate-400 text-sm flex-1">
              {report.description}
            </p>

            <button
              onClick={() => handleDownload(report.name)}
              className="mt-4 px-4 py-2 bg-sky-700 dark:bg-sky-600 text-white rounded hover:bg-sky-800 dark:hover:bg-sky-500 transition"
            >
              Download
            </button>
          </div>
        ))}
      </div>
    </div>
  );
}
