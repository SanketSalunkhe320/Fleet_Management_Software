


// import React, { useEffect, useState } from "react";
// import axios from "axios";
// import { Plus, Trash2, Edit3, Send, Settings, Check, X } from "lucide-react";

// const api = axios.create({
//   baseURL: "http://localhost:3002/api",
// });

// /* ================= EVENT SCHEMA ================= */

// const eventSchema = {
//   Wait: {
//     displayName: "Wait",
//     attributes: {
//       duration: { type: "number", label: "Duration (ms)", default: 1000 },
//     },
//   },
//   MoveForward: {
//     displayName: "Move Forward",
//     attributes: {
//       distance: { type: "number", label: "Distance (m)", default: 1 },
//       speed: { type: "number", label: "Speed (m/s)", default: 0.5 },
//     },
//   },
//   MoveReverse: {
//     displayName: "Move Reverse",
//     attributes: {
//       distance: { type: "number", label: "Distance (m)", default: 1 },
//       speed: { type: "number", label: "Speed (m/s)", default: 0.5 },
//     },
//   },
//   RotateClockwise: {
//     displayName: "Rotate Clockwise",
//     attributes: {
//       angle: { type: "number", label: "Angle (deg)", default: 90 },
//     },
//   },
//   RotateAntiClockwise: {
//     displayName: "Rotate Anti-Clockwise",
//     attributes: {
//       angle: { type: "number", label: "Angle (deg)", default: 90 },
//     },
//   },
// };

// /* ================= COMPONENT ================= */

// export default function TaskAllocationPage() {
//   const [robots, setRobots] = useState([]);
//   const [missions, setMissions] = useState([]);
//   const [selectedMissionId, setSelectedMissionId] = useState(null);
//   const [selectedRobot, setSelectedRobot] = useState("");
//   const [editingMissionName, setEditingMissionName] = useState(false);
//   const [backendStatus, setBackendStatus] = useState("Disconnected");
//   const [loading, setLoading] = useState(false);

//   const selectedMission = missions.find((m) => m.id === selectedMissionId);
//   const eventTypes = Object.keys(eventSchema);

//   /* ================= LOAD / SAVE MISSIONS ================= */

//   useEffect(() => {
//     const saved = localStorage.getItem("missions");
//     if (saved) setMissions(JSON.parse(saved));
//   }, []);

//   useEffect(() => {
//     localStorage.setItem("missions", JSON.stringify(missions));
//   }, [missions]);

//   /* ================= FETCH ROBOTS ================= */

//   useEffect(() => {
//     const fetchRobots = async () => {
//       try {
//         const res = await api.get("/robots");
//         if (res.data.success) {
//           setRobots(res.data.robots);
//           setBackendStatus("Connected");
//         }
//       } catch {
//         setBackendStatus("Disconnected");
//       }
//     };

//     fetchRobots();
//     const i = setInterval(fetchRobots, 3000);
//     return () => clearInterval(i);
//   }, []);

//   /* ================= MISSION ACTIONS ================= */

//   const createMission = () => {
//     const name = prompt("Mission name:");
//     if (!name) return;

//     const mission = { id: Date.now(), name, tasks: [] };
//     setMissions((prev) => [...prev, mission]);
//     setSelectedMissionId(mission.id);
//   };

//   const updateMissionName = (name) => {
//     setMissions((prev) =>
//       prev.map((m) =>
//         m.id === selectedMissionId ? { ...m, name } : m
//       )
//     );
//     setEditingMissionName(false);
//   };

//   const deleteMission = () => {
//     if (!selectedMission) return;

//     if (selectedMission.tasks.length > 0) {
//       alert("Delete all tasks before deleting mission");
//       return;
//     }

//     if (!window.confirm("Delete this mission?")) return;

//     setMissions((prev) => prev.filter((m) => m.id !== selectedMissionId));
//     setSelectedMissionId(null);
//   };

//   const deleteAllTasks = () => {
//     if (!selectedMission) return;
//     if (!window.confirm("Delete all tasks?")) return;

//     setMissions((prev) =>
//       prev.map((m) =>
//         m.id === selectedMissionId ? { ...m, tasks: [] } : m
//       )
//     );
//   };

//   /* ================= TASK ACTIONS ================= */

//   const addTask = (type) => {
//     if (!selectedMission) return;

//     const schema = eventSchema[type].attributes;
//     const config = Object.fromEntries(
//       Object.entries(schema).map(([k, v]) => [k, v.default])
//     );

//     const task = { id: Date.now(), type, config };

//     setMissions((prev) =>
//       prev.map((m) =>
//         m.id === selectedMissionId
//           ? { ...m, tasks: [...m.tasks, task] }
//           : m
//       )
//     );
//   };

//   const updateTask = (taskId, key, value) => {
//     setMissions((prev) =>
//       prev.map((m) =>
//         m.id === selectedMissionId
//           ? {
//               ...m,
//               tasks: m.tasks.map((t) =>
//                 t.id === taskId
//                   ? { ...t, config: { ...t.config, [key]: value } }
//                   : t
//               ),
//             }
//           : m
//       )
//     );
//   };

//   const deleteTask = (taskId) => {
//     setMissions((prev) =>
//       prev.map((m) =>
//         m.id === selectedMissionId
//           ? { ...m, tasks: m.tasks.filter((t) => t.id !== taskId) }
//           : m
//       )
//     );
//   };

//   /* ================= SEND MISSION ================= */

//   const assignMission = async () => {
//     if (!selectedMission || !selectedRobot || selectedMission.tasks.length === 0)
//       return alert("Select robot and add tasks");

//     try {
//       setLoading(true);
//       await api.post("/assign-mission", {
//         robotId: selectedRobot,
//         mission: selectedMission,
//       });
//       alert("Mission sent to AMR");
//     } catch {
//       alert("Backend not reachable");
//     } finally {
//       setLoading(false);
//     }
//   };

//   /* ================= UI ================= */

//   return (
//     <div className="p-6 space-y-6">
//       <header className="flex justify-between">
//         <h1 className="text-2xl font-bold flex gap-2">
//           <Settings size={20} /> Task Allocation
//         </h1>
//         <span
//           className={`px-3 py-1 rounded text-white ${
//             backendStatus === "Connected" ? "bg-green-600" : "bg-red-600"
//           }`}
//         >
//           Backend: {backendStatus}
//         </span>
//       </header>

//       {/* ROBOT SELECT */}
//       <select
//         value={selectedRobot}
//         onChange={(e) => setSelectedRobot(e.target.value)}
//         className="border p-2 rounded"
//       >
//         <option value="">Select Robot</option>
//         {robots.map((r) => (
//           <option key={r.id} value={r.id}>
//             {r.id}
//           </option>
//         ))}
//       </select>

//       {/* MISSIONS */}
//       <button onClick={createMission} className="bg-blue-600 text-white px-4 py-2 rounded flex gap-2">
//         <Plus size={16} /> New Mission
//       </button>

//       {missions.map((m) => (
//         <div
//           key={m.id}
//           onClick={() => setSelectedMissionId(m.id)}
//           className={`p-2 border rounded cursor-pointer ${
//             m.id === selectedMissionId ? "bg-blue-50 border-blue-500" : ""
//           }`}
//         >
//           {m.name} ({m.tasks.length})
//         </div>
//       ))}

//       {/* EDITOR */}
//       {selectedMission && (
//         <div className="border p-4 rounded space-y-4">
//           {/* Mission name edit */}
//           <div className="flex items-center gap-2">
//             {editingMissionName ? (
//               <>
//                 <input
//                   defaultValue={selectedMission.name}
//                   onBlur={(e) => updateMissionName(e.target.value)}
//                   className="border p-1 rounded"
//                 />
//                 <Check size={18} />
//               </>
//             ) : (
//               <>
//                 <h2 className="font-bold text-lg">{selectedMission.name}</h2>
//                 <Edit3 size={18} onClick={() => setEditingMissionName(true)} />
//               </>
//             )}
//             <Trash2
//               className="text-red-600 cursor-pointer ml-auto"
//               onClick={deleteMission}
//             />
//           </div>

//           {/* TASK BUTTONS */}
//           <div className="flex gap-2 flex-wrap">
//             {eventTypes.map((t) => (
//               <button key={t} onClick={() => addTask(t)} className="border px-3 py-1 rounded">
//                 + {eventSchema[t].displayName}
//               </button>
//             ))}
//             <button onClick={deleteAllTasks} className="text-red-600 border px-3 py-1 rounded">
//               Delete All Tasks
//             </button>
//           </div>

//           {/* TASKS */}
//           {selectedMission.tasks.map((task) => (
//             <div key={task.id} className="border p-2 rounded">
//               <div className="flex justify-between">
//                 <strong>{eventSchema[task.type].displayName}</strong>
//                 <Trash2 className="text-red-600 cursor-pointer" onClick={() => deleteTask(task.id)} />
//               </div>

//               {Object.entries(eventSchema[task.type].attributes).map(([k, v]) => (
//                 <div key={k} className="flex gap-2 mt-1">
//                   <label className="w-32">{v.label}</label>
//                   <input
//                     type="number"
//                     value={task.config[k]}
//                     onChange={(e) => updateTask(task.id, k, +e.target.value)}
//                     className="border p-1 rounded w-32"
//                   />
//                 </div>
//               ))}
//             </div>
//           ))}

//           <button
//             onClick={assignMission}
//             disabled={loading}
//             className="bg-purple-600 text-white px-4 py-2 rounded flex gap-2"
//           >
//             <Send size={16} /> Send to AMR
//           </button>
//         </div>
//       )}
//     </div>
//   );
// }



import React, { useEffect, useState } from "react";
import axios from "axios";
import { Plus, Trash2, Edit3, Send, Settings, Check } from "lucide-react";

const api = axios.create({
  baseURL: "http://localhost:3002/api",
});

/* ================= EVENT SCHEMA ================= */

const eventSchema = {
  Wait: {
    displayName: "Wait",
    attributes: {
      duration: { type: "number", label: "Duration (ms)", default: 1000 },
    },
  },
  MoveForward: {
    displayName: "Move Forward",
    attributes: {
      distance: { type: "number", label: "Distance (m)", default: 1 },
      speed: { type: "number", label: "Speed (m/s)", default: 0.5 },
    },
  },
  MoveReverse: {
    displayName: "Move Reverse",
    attributes: {
      distance: { type: "number", label: "Distance (m)", default: 1 },
      speed: { type: "number", label: "Speed (m/s)", default: 0.5 },
    },
  },
  RotateClockwise: {
    displayName: "Rotate Clockwise",
    attributes: {
      angle: { type: "number", label: "Angle (deg)", default: 90 },
    },
  },
  RotateAntiClockwise: {
    displayName: "Rotate Anti-Clockwise",
    attributes: {
      angle: { type: "number", label: "Angle (deg)", default: 90 },
    },
  },
};

/* ================= COMPONENT ================= */

export default function TaskAllocationPage() {
  const [robots, setRobots] = useState([]);
  const [missions, setMissions] = useState([]);
  const [selectedMissionId, setSelectedMissionId] = useState(null);
  const [selectedRobot, setSelectedRobot] = useState("");
  const [editingMissionName, setEditingMissionName] = useState(false);
  const [backendStatus, setBackendStatus] = useState("Disconnected");
  const [loading, setLoading] = useState(false);

  // ✅ NEW
  const [loopMission, setLoopMission] = useState(false);

  const selectedMission = missions.find((m) => m.id === selectedMissionId);
  const eventTypes = Object.keys(eventSchema);

  /* ================= LOAD / SAVE MISSIONS ================= */

  useEffect(() => {
    const saved = localStorage.getItem("missions");
    if (saved) setMissions(JSON.parse(saved));
  }, []);

  useEffect(() => {
    localStorage.setItem("missions", JSON.stringify(missions));
  }, [missions]);

  /* ================= FETCH ROBOTS ================= */

  useEffect(() => {
    const fetchRobots = async () => {
      try {
        const res = await api.get("/robots");
        if (res.data.success) {
          setRobots(res.data.robots);
          setBackendStatus("Connected");
        }
      } catch {
        setBackendStatus("Disconnected");
      }
    };

    fetchRobots();
    const i = setInterval(fetchRobots, 3000);
    return () => clearInterval(i);
  }, []);

  /* ================= MISSION ACTIONS ================= */

  const createMission = () => {
    const name = prompt("Mission name:");
    if (!name) return;

    const mission = { id: Date.now(), name, tasks: [] };
    setMissions((prev) => [...prev, mission]);
    setSelectedMissionId(mission.id);
  };

  const updateMissionName = (name) => {
    setMissions((prev) =>
      prev.map((m) =>
        m.id === selectedMissionId ? { ...m, name } : m
      )
    );
    setEditingMissionName(false);
  };

  const deleteMission = () => {
    if (!selectedMission) return;

    if (selectedMission.tasks.length > 0) {
      alert("Delete all tasks before deleting mission");
      return;
    }

    if (!window.confirm("Delete this mission?")) return;

    setMissions((prev) => prev.filter((m) => m.id !== selectedMissionId));
    setSelectedMissionId(null);
  };

  const deleteAllTasks = () => {
    if (!selectedMission) return;
    if (!window.confirm("Delete all tasks?")) return;

    setMissions((prev) =>
      prev.map((m) =>
        m.id === selectedMissionId ? { ...m, tasks: [] } : m
      )
    );
  };

  /* ================= TASK ACTIONS ================= */

  const addTask = (type) => {
    if (!selectedMission) return;

    const schema = eventSchema[type].attributes;
    const config = Object.fromEntries(
      Object.entries(schema).map(([k, v]) => [k, v.default])
    );

    const task = { id: Date.now(), type, config };

    setMissions((prev) =>
      prev.map((m) =>
        m.id === selectedMissionId
          ? { ...m, tasks: [...m.tasks, task] }
          : m
      )
    );
  };

  const updateTask = (taskId, key, value) => {
    setMissions((prev) =>
      prev.map((m) =>
        m.id === selectedMissionId
          ? {
              ...m,
              tasks: m.tasks.map((t) =>
                t.id === taskId
                  ? { ...t, config: { ...t.config, [key]: value } }
                  : t
              ),
            }
          : m
      )
    );
  };

  const deleteTask = (taskId) => {
    setMissions((prev) =>
      prev.map((m) =>
        m.id === selectedMissionId
          ? { ...m, tasks: m.tasks.filter((t) => t.id !== taskId) }
          : m
      )
    );
  };

  /* ================= SEND MISSION ================= */

  const assignMission = async () => {
    if (!selectedMission || !selectedRobot || selectedMission.tasks.length === 0)
      return alert("Select robot and add tasks");

    try {
      setLoading(true);
      await api.post("/assign-mission", {
        robotId: selectedRobot,
        mission: selectedMission,
        loop: loopMission, // ✅ NEW
      });
      alert(loopMission ? "Loop mission sent to AMR" : "Mission sent to AMR");
    } catch {
      alert("Backend not reachable");
    } finally {
      setLoading(false);
    }
  };

  /* ================= UI ================= */

  return (
    <div className="p-6 space-y-6">
      <header className="flex justify-between">
        <h1 className="text-2xl font-bold flex gap-2">
          <Settings size={20} /> Task Allocation
        </h1>
        <span
          className={`px-3 py-1 rounded text-white ${
            backendStatus === "Connected" ? "bg-green-600" : "bg-red-600"
          }`}
        >
          Backend: {backendStatus}
        </span>
      </header>

      {/* ROBOT SELECT */}
      <select
        value={selectedRobot}
        onChange={(e) => setSelectedRobot(e.target.value)}
        className="border p-2 rounded"
      >
        <option value="">Select Robot</option>
        {robots.map((r) => (
          <option key={r.id} value={r.id}>
            {r.id}
          </option>
        ))}
      </select>

      {/* MISSIONS */}
      <button
        onClick={createMission}
        className="bg-blue-600 text-white px-4 py-2 rounded flex gap-2"
      >
        <Plus size={16} /> New Mission
      </button>

      {missions.map((m) => (
        <div
          key={m.id}
          onClick={() => setSelectedMissionId(m.id)}
          className={`p-2 border rounded cursor-pointer ${
            m.id === selectedMissionId ? "bg-blue-50 border-blue-500" : ""
          }`}
        >
          {m.name} ({m.tasks.length})
        </div>
      ))}

      {/* EDITOR */}
      {selectedMission && (
        <div className="border p-4 rounded space-y-4">
          {/* Mission name */}
          <div className="flex items-center gap-2">
            {editingMissionName ? (
              <>
                <input
                  defaultValue={selectedMission.name}
                  onBlur={(e) => updateMissionName(e.target.value)}
                  className="border p-1 rounded"
                />
                <Check size={18} />
              </>
            ) : (
              <>
                <h2 className="font-bold text-lg">{selectedMission.name}</h2>
                <Edit3 size={18} onClick={() => setEditingMissionName(true)} />
              </>
            )}
            <Trash2
              className="text-red-600 cursor-pointer ml-auto"
              onClick={deleteMission}
            />
          </div>

          {/* TASK BUTTONS */}
          <div className="flex gap-2 flex-wrap">
            {eventTypes.map((t) => (
              <button
                key={t}
                onClick={() => addTask(t)}
                className="border px-3 py-1 rounded"
              >
                + {eventSchema[t].displayName}
              </button>
            ))}
            <button
              onClick={deleteAllTasks}
              className="text-red-600 border px-3 py-1 rounded"
            >
              Delete All Tasks
            </button>
          </div>

          {/* TASKS */}
          {selectedMission.tasks.map((task) => (
            <div key={task.id} className="border p-2 rounded">
              <div className="flex justify-between">
                <strong>{eventSchema[task.type].displayName}</strong>
                <Trash2
                  className="text-red-600 cursor-pointer"
                  onClick={() => deleteTask(task.id)}
                />
              </div>

              {Object.entries(eventSchema[task.type].attributes).map(
                ([k, v]) => (
                  <div key={k} className="flex gap-2 mt-1">
                    <label className="w-32">{v.label}</label>
                    <input
                      type="number"
                      value={task.config[k]}
                      onChange={(e) =>
                        updateTask(task.id, k, +e.target.value)
                      }
                      className="border p-1 rounded w-32"
                    />
                  </div>
                )
              )}
            </div>
          ))}

          {/* LOOP TOGGLE */}
          <button
            onClick={() => setLoopMission((prev) => !prev)}
            className={`px-4 py-2 rounded border ${
              loopMission
                ? "bg-green-600 text-white"
                : "bg-gray-200 text-black"
            }`}
          >
            🔁 Loop Mission: {loopMission ? "ON" : "OFF"}
          </button>

          {/* SEND */}
          <button
            onClick={assignMission}
            disabled={loading}
            className="bg-purple-600 text-white px-4 py-2 rounded flex gap-2"
          >
            <Send size={16} /> Send to AMR
          </button>
        </div>
      )}
    </div>
  );
}
