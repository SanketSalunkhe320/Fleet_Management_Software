// import React from "react";
// import { Link, useLocation } from "react-router-dom";
// import {
//   FiHome,
//   FiActivity,
//   FiClipboard,
//   FiHeart,
//   FiBarChart2,
//   FiSettings,
//   FiMap, // ✅ Added map icon
// } from "react-icons/fi";

// export default function Sidebar() {
//   const location = useLocation();

//   const links = [
//     { to: "/", label: "Dashboard", icon: FiHome },
//     { to: "/maps", label: "Maps", icon: FiMap }, // ✅ Added Maps page
//     { to: "/realtime", label: "Real-Time Visualisation", icon: FiActivity },
//     { to: "/tasks", label: "Task Allocation", icon: FiClipboard },
//     { to: "/health", label: "Health Check", icon: FiHeart },
//     { to: "/reports", label: "Reports", icon: FiBarChart2 },
//     { to: "/settings", label: "Settings", icon: FiSettings },
//   ];

//   return (
//     <aside className="hidden lg:block w-64 bg-gradient-to-b from-sky-700 to-sky-800 text-white p-7 space-y-6 opacity-90 select-none">
//       <nav className="space-y-4">
//         {links.map(({ to, label, icon: Icon }) => (
//           <Link
//             key={to}
//             to={to}
//             className={`flex items-center gap-3 py-2 px-3 rounded-md transition-all duration-200 ${
//               location.pathname === to
//                 ? "bg-sky-600 text-white font-semibold shadow"
//                 : "text-gray-200 hover:bg-sky-500"
//             }`}
//           >
//             <Icon className="text-lg select-none" />
//             <span className="font-bold">{label}</span>
//           </Link>
//         ))}
//       </nav>

//       <div className="border-t border-sky-400 mt-6"></div>

//       <div className="mt-4 text-sm opacity-90 flex items-center justify-center">
//         Version 1.0
//       </div>
//     </aside>
//   );
// }

import React from "react";
import { Link, useLocation } from "react-router-dom";
import {
  FiHome,
  FiActivity,
  FiClipboard,
  FiHeart,
  FiBarChart2,
  FiSettings,
  FiMap,
} from "react-icons/fi";
import { useLanguage } from "../context/LanguageContext";

export default function Sidebar() {
  const location = useLocation();
  const { t } = useLanguage();

  const links = [
    { to: "/", label: t("dashboard"), icon: FiHome },
    { to: "/maps", label: t("maps"), icon: FiMap },
    { to: "/realtime", label: t("realtime_visualisation"), icon: FiActivity },
    { to: "/tasks", label: t("task_allocation"), icon: FiClipboard },
    { to: "/health", label: t("health_check"), icon: FiHeart },
    { to: "/reports", label: t("reports"), icon: FiBarChart2 },
    { to: "/settings", label: t("settings"), icon: FiSettings },
  ];

  return (
    <aside className="hidden lg:block w-64 bg-gradient-to-b from-sky-700 to-sky-800 text-white p-7 space-y-6 opacity-90 select-none">
      <nav className="space-y-4">
        {links.map(({ to, label, icon: Icon }) => (
          <Link
            key={to}
            to={to}
            className={`flex items-center gap-3 py-2 px-3 rounded-md transition-all duration-200 ${
              location.pathname === to
                ? "bg-sky-600 text-white font-semibold shadow"
                : "text-gray-200 hover:bg-sky-500"
            }`}
          >
            <Icon className="text-lg select-none" />
            <span className="font-bold">{label}</span>
          </Link>
        ))}
      </nav>

      <div className="border-t border-sky-400 mt-6"></div>

      <div className="mt-4 text-sm opacity-90 flex items-center justify-center">
        Version 1.0
      </div>
    </aside>
  );
}
