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
import { FiBell, FiUser, FiLogOut, FiHome, FiActivity, FiClipboard, FiHeart, FiBarChart2, FiSettings, FiCheck, FiX, FiSearch } from "react-icons/fi";
import TaikishaLogo from "../assets/Taikishaimage1.png";

export default function Header() {
  const [menuOpen, setMenuOpen] = useState(false);
  const [userDropdownOpen, setUserDropdownOpen] = useState(false);
  const [agvDropdownOpen, setAgvDropdownOpen] = useState(false);
  const [selectedAgvs, setSelectedAgvs] = useState([]);
  const [searchQuery, setSearchQuery] = useState("");
  const navigate = useNavigate();
  const dropdownRef = useRef(null);
  const agvDropdownRef = useRef(null);

  const loggedInUser = localStorage.getItem("loggedInUser") || "Guest";

  // Sample AGV data - replace with your actual AGV data
  const agvList = [
    { id: "agv1", name: "AGV-001", status: "online", battery: 85 },
    { id: "agv2", name: "AGV-002", status: "online", battery: 72 },
    { id: "agv3", name: "AGV-003", status: "offline", battery: 0 },
    { id: "agv4", name: "AGV-004", status: "online", battery: 91 },
    { id: "agv5", name: "AGV-005", status: "maintenance", battery: 45 },
    { id: "agv6", name: "AGV-006", status: "online", battery: 63 },
    { id: "agv7", name: "AGV-007", status: "online", battery: 88 },
  ];

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

  const getBatteryColor = (battery) => {
    if (battery >= 70) return "bg-green-500";
    if (battery >= 30) return "bg-yellow-500";
    return "bg-red-500";
  };

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

        <div className="flex flex-col sm:flex-row items-center gap-2">
          <img
            src={TaikishaLogo}
            alt="Taikisha Logo"
            className="h-10 sm:h-12 w-auto object-contain flex-shrink-0 select-none"
          />
        </div>
      </div>

      {/* Right: Actions */}
      <div className="flex items-center gap-2 sm:gap-3 mt-2 sm:mt-0">
        {/* AGV Selection Dropdown */}
        <div className="relative" ref={agvDropdownRef}>
          <button
            onClick={() => setAgvDropdownOpen(!agvDropdownOpen)}
            className="px-2 sm:px-3 py-2 bg-gradient-to-r from-purple-500 to-purple-600 text-white rounded-lg shadow-lg flex items-center gap-1 sm:gap-2 hover:from-purple-600 hover:to-purple-700 transition-all duration-200 transform hover:scale-105 text-sm sm:text-base"
          >
            <FiActivity className="text-base sm:text-lg" />
            <span className="hidden xs:inline">AGVs</span>
            {selectedAgvs.length > 0 && (
              <span className="bg-white text-purple-600 text-xs font-bold px-1.5 sm:px-2 py-0.5 sm:py-1 rounded-full min-w-5 sm:min-w-6">
                {selectedAgvs.length}
              </span>
            )}
          </button>

          {agvDropdownOpen && (
            <div className="absolute right-0 mt-2 w-screen max-w-xs sm:max-w-sm md:max-w-md lg:max-w-lg xl:max-w-xl bg-white text-black rounded-xl shadow-2xl p-3 sm:p-4 z-50 border border-purple-100 transform transition-all duration-200">
              {/* Header */}
              <div className="flex flex-col sm:flex-row sm:justify-between sm:items-center gap-3 mb-3 sm:mb-4 pb-3 border-b border-gray-200">
                <h3 className="font-bold text-lg text-gray-800 text-center sm:text-left">Select AGVs</h3>
                <div className="flex gap-2 justify-center sm:justify-start">
                  <button
                    onClick={selectAllAgvs}
                    className="px-2 sm:px-3 py-1.5 bg-green-500 text-white text-xs rounded-lg hover:bg-green-600 transition flex items-center gap-1"
                  >
                    <FiCheck className="text-xs" />
                    All Online
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
                {filteredAgvs.length === 0 ? (
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
                      }`}
                    >
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-2 sm:gap-3 flex-1 min-w-0">
                          <div className={`w-2 h-2 sm:w-3 sm:h-3 rounded-full flex-shrink-0 ${getStatusColor(agv.status)}`}></div>
                          <span className="font-semibold text-gray-800 text-sm sm:text-base truncate">{agv.name}</span>
                        </div>
                        
                        <div className="flex items-center gap-2 sm:gap-4 flex-shrink-0">
                          {/* Battery Indicator */}
                          {agv.status === "online" && (
                            <div className="flex items-center gap-1">
                              <div className="w-8 sm:w-12 bg-gray-200 rounded-full h-1.5 sm:h-2 hidden xs:block">
                                <div 
                                  className={`h-1.5 sm:h-2 rounded-full ${getBatteryColor(agv.battery)}`}
                                  style={{ width: `${agv.battery}%` }}
                                ></div>
                              </div>
                              <span className={`text-xs font-bold ${getBatteryColor(agv.battery).replace('bg-', 'text-')} hidden sm:block`}>
                                {agv.battery}%
                              </span>
                            </div>
                          )}
                          
                          {/* Selection Checkbox */}
                          <div className={`w-4 h-4 sm:w-5 sm:h-5 rounded border-2 flex items-center justify-center flex-shrink-0 ${
                            selectedAgvs.includes(agv.id)
                              ? "bg-purple-500 border-purple-500"
                              : "border-gray-400"
                          }`}>
                            {selectedAgvs.includes(agv.id) && (
                              <FiCheck className="text-white text-xs" />
                            )}
                          </div>
                        </div>
                      </div>
                      
                      {/* Status and Battery Info */}
                      <div className="flex justify-between items-center mt-1.5 sm:mt-2">
                        <span className={`text-xs px-1.5 sm:px-2 py-0.5 sm:py-1 rounded-full ${
                          agv.status === "online" ? "bg-green-100 text-green-800" :
                          agv.status === "offline" ? "bg-red-100 text-red-800" :
                          "bg-yellow-100 text-yellow-800"
                        }`}>
                          {agv.status.charAt(0).toUpperCase() + agv.status.slice(1)}
                        </span>
                        
                        {/* Mobile Battery Display */}
                        {agv.status === "online" && (
                          <span className={`text-xs font-bold sm:hidden ${getBatteryColor(agv.battery).replace('bg-', 'text-')}`}>
                            {agv.battery}%
                          </span>
                        )}
                        
                        {agv.status !== "online" && (
                          <span className="text-xs text-gray-500 truncate ml-2">
                            {agv.status === "maintenance" ? "Maintenance" : "Offline"}
                          </span>
                        )}
                      </div>
                    </div>
                  ))
                )}
              </div>

              {/* Footer */}
              <div className="mt-3 sm:mt-4 pt-3 border-t border-gray-200">
                <div className="flex flex-col sm:flex-row sm:justify-between sm:items-center gap-2 text-sm">
                  <span className="text-gray-600 text-center sm:text-left">
                    {selectedAgvs.length} AGV{selectedAgvs.length !== 1 ? 's' : ''} selected
                  </span>
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
                      onClick={() => setAgvDropdownOpen(false)}
                      className="px-3 sm:px-4 py-2 bg-purple-500 text-white rounded-lg hover:bg-purple-600 transition flex-1 sm:flex-none"
                    >
                      Confirm
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