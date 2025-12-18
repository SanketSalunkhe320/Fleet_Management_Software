// import React, { useState, useEffect } from "react";
// import { Plus, Trash2, Edit3, Save, Send, Settings } from "lucide-react";

// // ✅ Full event schema definition
// const eventSchema = {
//   Wait: {
//     displayName: "Wait",
//     attributes: {
//       duration: { type: "number", description: "Duration in milliseconds", readOnly: false },
//       trigger_service_name: {
//         type: "string",
//         description: "Name of the trigger service",
//         default: "wait_trigger",
//         readOnly: true,
//       },
//       wait_topic_info: { type: "string", description: "Info to be published on wait topic", readOnly: false },
//     },
//   },
//   GoToZone: {
//     displayName: "Go to Zone",
//     attributes: {
//       zone_to_go: { type: "string", description: "Zone name", readOnly: false },
//       direction: {
//         type: "enum",
//         options: ["forward", "reverse"],
//         description: "Direction",
//         readOnly: false,
//       },
//     },
//   },
//   LineFollow: {
//     displayName: "Line Follow",
//     attributes: {
//       direction: {
//         type: "enum",
//         options: ["forward", "reverse"],
//         description: "Direction",
//         readOnly: false,
//       },
//       end_tag: { type: "number", description: "End tag number", readOnly: false },
//       safety_tag: { type: "number", description: "Safety tag number", readOnly: false },
//       relative_distance_to_tag: {
//         type: "number",
//         description: "Relative distance to tag (-1 to 1)",
//         readOnly: false,
//       },
//     },
//   },
//   MoveRobot: {
//     displayName: "Move Robot",
//     attributes: {
//       direction: {
//         type: "enum",
//         options: ["forward", "reverse"],
//         description: "Direction",
//         readOnly: false,
//       },
//       movement_distance: { type: "number", description: "Movement distance in meters", readOnly: false },
//       movement_speed: { type: "number", description: "Movement speed in m/s", readOnly: false },
//       cmd_vel_topic: {
//         type: "string",
//         description: "Command velocity topic",
//         default: "controller_cmd_vel",
//         readOnly: true,
//       },
//     },
//   },
//   RotateOnSpot: {
//     displayName: "Rotate on Spot",
//     attributes: {
//       rotate_angle: { type: "number", description: "Rotate angle in degrees", readOnly: false },
//       direction: {
//         type: "enum",
//         options: ["clockwise", "anticlockwise"],
//         description: "Direction",
//         readOnly: false,
//       },
//       angular_speed: { type: "number", description: "Angular speed in r/s", readOnly: false },
//     },
//   },
//   MoveActuator: {
//     displayName: "Move Actuator",
//     attributes: {
//       actuator_type: {
//         type: "enum",
//         options: ["linear", "lifter"],
//         description: "Actuator Type",
//         readOnly: false,
//       },
//       actuator_number: {
//         type: "enum",
//         options: ["0", "1", "2", "3", "4"],
//         description: "Actuator Number",
//         readOnly: false,
//       },
//       move_time: { type: "number", description: "Move time in milliseconds", readOnly: false },
//       move_direction: {
//         type: "enum",
//         options: ["up", "down"],
//         description: "Move Direction",
//         readOnly: false,
//       },
//     },
//   },
//   ChangeSensorProfile: {
//     displayName: "Change Sensor Profile",
//     attributes: { profile_name: { type: "string", description: "Sensor profile", readOnly: false } },
//   },
//   TriggerMoveArm: {
//     displayName: "Trigger Move Arm",
//     attributes: {
//       machine_id: { type: "string", description: "Machine ID", readOnly: false },
//       operation: { type: "string", description: "Operation", readOnly: false },
//     },
//   },
// };

// export default function TaskAllocation() {
//   const [missions, setMissions] = useState([]);
//   const [selectedMission, setSelectedMission] = useState(null);
//   const [showTaskConfig, setShowTaskConfig] = useState(null);
//   const eventTypes = Object.keys(eventSchema);

//   // ✅ Load missions from localStorage on startup
//   useEffect(() => {
//     const stored = localStorage.getItem("missions");
//     if (stored) {
//       try {
//         setMissions(JSON.parse(stored));
//       } catch {
//         console.error("Invalid saved missions data");
//       }
//     }
//   }, []);

//   // ✅ Auto-save missions to localStorage on every change
//   useEffect(() => {
//     localStorage.setItem("missions", JSON.stringify(missions));
//   }, [missions]);

//   const createMission = () => {
//     const name = prompt("Enter Mission Name:");
//     if (!name) return;
//     const newMission = { id: Date.now(), name, tasks: [] };
//     setMissions([...missions, newMission]);
//   };

//   const addTask = (type) => {
//     const attrs = eventSchema[type]?.attributes || {};
//     const config = Object.fromEntries(Object.entries(attrs).map(([k, v]) => [k, v.default ?? ""]));
//     const newTask = { id: Date.now(), type, config };
//     const updated = { ...selectedMission, tasks: [...selectedMission.tasks, newTask] };
//     setSelectedMission(updated);
//     setMissions(missions.map((m) => (m.id === updated.id ? updated : m)));
//   };

//   const updateTaskConfig = (taskId, field, value) => {
//     const updatedTasks = selectedMission.tasks.map((t) =>
//       t.id === taskId ? { ...t, config: { ...t.config, [field]: value } } : t
//     );
//     const updatedMission = { ...selectedMission, tasks: updatedTasks };
//     setSelectedMission(updatedMission);
//     setMissions(missions.map((m) => (m.id === updatedMission.id ? updatedMission : m)));
//   };

//   const deleteTask = (taskId) => {
//     const updated = { ...selectedMission, tasks: selectedMission.tasks.filter((t) => t.id !== taskId) };
//     setSelectedMission(updated);
//     setMissions(missions.map((m) => (m.id === updated.id ? updated : m)));
//   };

//   const renderDynamicForm = (task) => {
//     const schema = eventSchema[task.type];
//     if (!schema) return <p>No configuration found.</p>;

//     return (
//       <div className="grid gap-3">
//         {Object.entries(schema.attributes).map(([key, attr]) => (
//           <div key={key}>
//             <label className="block text-sm font-medium mb-1">{attr.description}</label>
//             {attr.type === "enum" ? (
//               <select
//                 value={task.config[key] ?? ""}
//                 disabled={attr.readOnly}
//                 onChange={(e) => updateTaskConfig(task.id, key, e.target.value)}
//                 className="w-full p-2 border rounded bg-slate-50 dark:bg-slate-700"
//               >
//                 <option value="">Select...</option>
//                 {attr.options?.map((opt) => (
//                   <option key={opt} value={opt}>
//                     {opt}
//                   </option>
//                 ))}
//               </select>
//             ) : (
//               <input
//                 type={attr.type === "number" ? "number" : "text"}
//                 readOnly={attr.readOnly}
//                 value={task.config[key] ?? ""}
//                 onChange={(e) =>
//                   updateTaskConfig(
//                     task.id,
//                     key,
//                     attr.type === "number" ? +e.target.value : e.target.value
//                   )
//                 }
//                 className={`w-full p-2 border rounded bg-slate-50 dark:bg-slate-700 ${
//                   attr.readOnly ? "opacity-70 cursor-not-allowed" : ""
//                 }`}
//               />
//             )}
//           </div>
//         ))}
//       </div>
//     );
//   };

//   return (
//     <div className="p-6 bg-gray-50 dark:bg-gray-900 min-h-screen text-gray-900 dark:text-gray-100 space-y-6" style={{ userSelect: "none" }} >
      
//       <header className="flex justify-between items-center">
//         <h1 className="text-xl font-semibold flex items-center gap-2 text-sky-600">
//           <Settings size={20} /> Task Allocation (Mission Planner)
//         </h1>
//         <div className="flex gap-2">
//           <button
//             onClick={createMission}
//             className="flex items-center gap-2 bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded"
//           >
//             <Plus size={16} /> Create New Mission
//           </button>

//           {/* 📂 Load Mission Button */}
//           <button
//             onClick={() => {
//               const input = document.createElement("input");
//               input.type = "file";
//               input.accept = ".json";
//               input.onchange = (e) => {
//                 const file = e.target.files[0];
//                 if (!file) return;
//                 const reader = new FileReader();
//                 reader.onload = (event) => {
//                   try {
//                     const data = JSON.parse(event.target.result);
//                     setMissions((prev) => [...prev, data]);
//                     alert("✅ Mission loaded successfully!");
//                   } catch {
//                     alert("❌ Invalid JSON file.");
//                   }
//                 };
//                 reader.readAsText(file);
//               };
//               input.click();
//             }}
//             className="bg-yellow-600 hover:bg-yellow-700 text-white px-4 py-2 rounded"
//           >
//             📂 Load Mission from your computer
//           </button>
//         </div>
//       </header>

//       {!selectedMission && (
//         <div className="grid gap-3">
//           {missions.length === 0 ? (
//             <p className="text-gray-500">No missions yet. Click “New Mission”.</p>
//           ) : (
//             missions.map((m) => (
//               <div
//                 key={m.id}
//                 className="p-3 border rounded-lg flex justify-between items-center hover:bg-slate-50 dark:hover:bg-slate-800"
//               >
//                 <span>{m.name}</span>
//                 <button
//                   onClick={() => setSelectedMission(m)}
//                   className="bg-sky-600 hover:bg-sky-700 text-white px-3 py-1 rounded"
//                 >
//                   Edit
//                 </button>
//               </div>
//             ))
//           )}
//         </div>
//       )}

//       {selectedMission && (
//         <div className="bg-white dark:bg-slate-800 rounded-2xl p-6 shadow space-y-4">
//           <div className="flex justify-between items-center">
//             <h2 className="font-semibold text-sky-700">{selectedMission.name}</h2>
//             <button onClick={() => setSelectedMission(null)} className="text-red-600 hover:text-red-700">
//               Back
//             </button>
//           </div>

//           <div className="flex flex-wrap gap-2">
//             {eventTypes.map((t) => (
//               <button
//                 key={t}
//                 onClick={() => addTask(t)}
//                 className="px-3 py-2 rounded border text-sm bg-slate-100 hover:bg-slate-200 dark:bg-slate-700 dark:hover:bg-slate-600"
//               >
//                 + {eventSchema[t].displayName}
//               </button>
//             ))}
//           </div>

//           <div className="space-y-2 mt-4">
//             {selectedMission.tasks.length === 0 ? (
//               <p className="text-gray-500 text-sm">No tasks added yet.</p>
//             ) : (
//               selectedMission.tasks.map((task) => (
//                 <div
//                   key={task.id}
//                   className={`p-3 border rounded-lg ${
//                     showTaskConfig === task.id ? "border-sky-500 bg-sky-50 dark:bg-sky-900" : ""
//                   }`}
//                 >
//                   <div className="flex justify-between items-center">
//                     <span className="font-medium">
//                       {eventSchema[task.type]?.displayName || task.type}
//                     </span>
//                     <div className="flex gap-2">
//                       <button
//                         onClick={() =>
//                           setShowTaskConfig(showTaskConfig === task.id ? null : task.id)
//                         }
//                         className="text-sky-600 hover:text-sky-700"
//                       >
//                         <Edit3 size={16} />
//                       </button>
//                       <button
//                         onClick={() => deleteTask(task.id)}
//                         className="text-red-600 hover:text-red-700"
//                       >
//                         <Trash2 size={16} />
//                       </button>
//                     </div>
//                   </div>

//                   {showTaskConfig === task.id && (
//                     <div className="mt-3">{renderDynamicForm(task)}</div>
//                   )}
//                 </div>
//               ))
//             )}
//           </div>

//           <div className="flex gap-3 pt-4">
//             {/* 💾 Save */}
//             <button
//               onClick={() => {
//                 if (!selectedMission) return;
//                 const jsonData = JSON.stringify(selectedMission, null, 2);
//                 const blob = new Blob([jsonData], { type: "application/json" });
//                 const link = document.createElement("a");
//                 link.href = URL.createObjectURL(blob);
//                 link.download = `${selectedMission.name.replace(/\s+/g, "_")}_mission.json`;
//                 link.click();
//               }}
//               className="flex items-center gap-2 bg-green-600 hover:bg-green-700 text-white px-4 py-2 rounded"
//             >
//               <Save size={16} /> Save Mission
//             </button>

//             {/* 🚀 Send */}
//             <button
//               onClick={() => alert(`🚀 Mission "${selectedMission.name}" sent to AMR successfully!`)}
//               className="flex items-center gap-2 bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded"
//             >
//               <Send size={16} /> Send to AMR
//             </button>

//             {/* 🗑️ Delete */}
//             <button
//               onClick={() => {
//                 if (window.confirm(`Delete mission "${selectedMission.name}"?`)) {
//                   setMissions((prev) => prev.filter((m) => m.id !== selectedMission.id));
//                   setSelectedMission(null);
//                 }
//               }}
//               className="flex items-center gap-2 bg-red-600 hover:bg-red-700 text-white px-4 py-2 rounded"
//             >
//               <Trash2 size={16} /> Delete
//             </button>
//           </div>
//         </div>
//       )}
//     </div>
//   );
// }


import React, { useEffect, useState } from "react";
import axios from "axios";
import { Plus, Trash2, Send } from "lucide-react";

const api = axios.create({
  baseURL: "http://localhost:3002/api",
});

// Define task schema
const taskSchema = {
  MoveForward: { label: "Move Forward", config: { speed: 0.5, distance: 1 } },
  MoveReverse: { label: "Move Reverse", config: { speed: 0.5, distance: 1 } },
  RotateClockwise: { label: "Rotate Clockwise", config: { angle: 90 } },
  RotateAntiClockwise: { label: "Rotate Anti-Clockwise", config: { angle: 90 } },
  Wait: { label: "Wait", config: { duration: 1000 } },
};

export default function TaskAllocationPage() {
  const [robots, setRobots] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState("");
  const [missions, setMissions] = useState([]);
  const [selectedMission, setSelectedMission] = useState(null);
  const [backendStatus, setBackendStatus] = useState("Disconnected");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState("");

  // Fetch robots
  const fetchRobots = async () => {
    try {
      const res = await api.get("/robots");
      if (res.data.success) {
        setRobots(res.data.robots);
        setBackendStatus("Connected");
      } else setBackendStatus("Disconnected");
    } catch {
      setBackendStatus("Disconnected");
    }
  };

  useEffect(() => {
    fetchRobots();
    const interval = setInterval(fetchRobots, 3000);
    return () => clearInterval(interval);
  }, []);

  // Create a new mission
  const createMission = () => {
    const name = prompt("Enter mission name:");
    if (!name) return;
    const mission = { id: Date.now(), name, tasks: [] };
    setMissions([...missions, mission]);
    setSelectedMission(mission);
  };

  // Add a task
  const addTask = (type) => {
    if (!selectedMission) return alert("Select a mission first");
    const task = {
      id: Date.now(),
      type,
      config: { ...taskSchema[type].config },
    };
    const updated = { ...selectedMission, tasks: [...selectedMission.tasks, task] };
    setSelectedMission(updated);
    setMissions(missions.map(m => m.id === updated.id ? updated : m));
  };

  // Update task config
  const updateTaskConfig = (taskId, field, value) => {
    const updatedTasks = selectedMission.tasks.map(task =>
      task.id === taskId ? { ...task, config: { ...task.config, [field]: value } } : task
    );
    const updatedMission = { ...selectedMission, tasks: updatedTasks };
    setSelectedMission(updatedMission);
    setMissions(missions.map(m => m.id === updatedMission.id ? updatedMission : m));
  };

  // Delete task
  const deleteTask = (taskId) => {
    const updatedTasks = selectedMission.tasks.filter(t => t.id !== taskId);
    const updatedMission = { ...selectedMission, tasks: updatedTasks };
    setSelectedMission(updatedMission);
    setMissions(missions.map(m => m.id === updatedMission.id ? updatedMission : m));
  };

  // Assign mission to robot
  const assignMission = async () => {
    if (!selectedRobot || !selectedMission || selectedMission.tasks.length === 0)
      return alert("Select a robot and add tasks to the mission");

    try {
      setLoading(true);
      const res = await api.post("/assign-mission", {
        robotId: selectedRobot,
        mission: selectedMission,
      });
      if (res.data.success) alert("Mission assigned successfully!");
      else alert(res.data.error || "Failed to assign mission");
    } catch {
      alert("Backend not reachable");
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="p-6 space-y-6">
      {/* Header */}
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-bold">Task Allocation</h1>
        <span
          className={`px-3 py-1 rounded-full text-sm ${
            backendStatus === "Connected"
              ? "bg-green-600 text-white"
              : "bg-red-600 text-white"
          }`}
        >
          Backend: {backendStatus}
        </span>
      </div>

      {/* Robot Selection */}
      <div className="space-y-2">
        <label>Select Robot:</label>
        <select
          value={selectedRobot}
          onChange={(e) => setSelectedRobot(e.target.value)}
          className="p-2 border rounded"
        >
          <option value="">-- Select Robot --</option>
          {robots.map(r => (
            <option key={r.id} value={r.id}>{r.id} ({r.status})</option>
          ))}
        </select>
      </div>

      {/* Missions */}
      <div className="space-y-2">
        <button onClick={createMission} className="flex items-center gap-2 px-4 py-2 bg-blue-600 text-white rounded">
          <Plus size={16} /> New Mission
        </button>

        {missions.map(m => (
          <div
            key={m.id}
            onClick={() => setSelectedMission(m)}
            className={`p-2 border rounded cursor-pointer ${selectedMission?.id === m.id ? 'border-blue-500 bg-blue-50' : 'border-gray-300'}`}
          >
            {m.name} ({m.tasks.length} tasks)
          </div>
        ))}
      </div>

      {/* Task Editor */}
      {selectedMission && (
        <div className="p-4 border rounded space-y-4">
          <h2 className="font-bold">{selectedMission.name} - Tasks</h2>

          {/* Add Task Buttons */}
          <div className="flex flex-wrap gap-2">
            {Object.keys(taskSchema).map(type => (
              <button key={type} onClick={() => addTask(type)} className="px-3 py-1 border rounded hover:bg-gray-100">
                + {taskSchema[type].label}
              </button>
            ))}
          </div>

          {/* Task List */}
          {selectedMission.tasks.map((task, idx) => (
            <div key={task.id} className="p-2 border rounded space-y-1">
              <div className="flex justify-between items-center">
                <strong>{idx + 1}. {taskSchema[task.type]?.label || task.type}</strong>
                <button onClick={() => deleteTask(task.id)} className="text-red-500">
                  <Trash2 size={16} />
                </button>
              </div>

              {/* Task Config */}
              <div className="space-y-1">
                {Object.entries(task.config).map(([key, value]) => (
                  <div key={key} className="flex items-center gap-2">
                    <label className="w-24">{key}:</label>
                    <input
                      type="number"
                      value={value}
                      onChange={e => updateTaskConfig(task.id, key, parseFloat(e.target.value))}
                      className="p-1 border rounded w-24"
                    />
                  </div>
                ))}
              </div>
            </div>
          ))}

          {/* Assign Mission Button */}
          <button
            onClick={assignMission}
            disabled={loading || selectedMission.tasks.length === 0}
            className="flex items-center gap-2 bg-purple-600 hover:bg-purple-700 text-white px-4 py-2 rounded mt-2"
          >
            <Send size={16} /> Assign Mission
          </button>
        </div>
      )}
    </div>
  );
}
