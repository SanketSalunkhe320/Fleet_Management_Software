// // import React, { useState, useEffect } from "react";
// // import { BrowserRouter as Router, Routes, Route, Navigate } from "react-router-dom";

// // // Pages
// // import Dashboard from "./pages/Dashboard";
// // import Maps from "./pages/Maps";
// // import RealTime from "./pages/RealTime";
// // import TaskAllocation from "./pages/TaskAllocation";
// // import HealthCheck from "./pages/HealthCheck";
// // import Reports from "./pages/Reports";
// // import Settings from "./pages/Settings";
// // import Animation from "./pages/Animation";
// // import LoginForm from "./pages/LoginForm";
// // import SignupForm from "./pages/SignupForm";

// // // Layout
// // import DashboardLayout from "./components/DashboardLayout";
// // import ProtectedRoute from "./components/ProtectedRoute";

// // export default function App() {
// //   // 🌙 Dark Mode state
// //   const [darkMode, setDarkMode] = useState(
// //     localStorage.getItem("darkMode") === "true"
// //   );

// //   // 🌐 Language state (EN by default)
// //   const [language, setLanguage] = useState(
// //     localStorage.getItem("language") || "en"
// //   );

// //   // Update HTML class based on dark mode
// //   useEffect(() => {
// //     if (darkMode) document.documentElement.classList.add("dark");
// //     else document.documentElement.classList.remove("dark");
// //   }, [darkMode]);

// //   return (
// //     <Router>
// //       <Routes>

        
// //         {/* Public routes */}
// //         <Route path="/login" element={<LoginForm language={language} />} />
// //         <Route path="/signup" element={<SignupForm language={language} />} />
// //         <Route path="/animation" element={<Animation />} />

// //         {/* Protected Routes */}
// //         <Route
// //           path="/"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <Dashboard language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/maps"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <Maps language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/realtime"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <RealTime language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/tasks"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <TaskAllocation language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/health"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <HealthCheck language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/reports"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <Reports language={language} />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         <Route
// //           path="/settings"
// //           element={
// //             <ProtectedRoute>
// //               <DashboardLayout
// //                 darkMode={darkMode}
// //                 setDarkMode={setDarkMode}
// //                 language={language}
// //               >
// //                 <Settings
// //                   darkMode={darkMode}
// //                   setDarkMode={setDarkMode}
// //                   language={language}
// //                   setLanguage={setLanguage}
// //                 />
// //               </DashboardLayout>
// //             </ProtectedRoute>
// //           }
// //         />

// //         {/* Redirect everything else */}
// //         <Route path="*" element={<Navigate to="/login" replace />} />
// //       </Routes>
// //     </Router>
// //   );
// // }

// import React, { useState, useEffect } from "react";
// import { BrowserRouter as Router, Routes, Route, Navigate } from "react-router-dom";

// // Pages
// import Dashboard from "./pages/Dashboard";
// import Maps from "./pages/Maps";
// import RealTime from "./pages/RealTime";
// import TaskAllocation from "./pages/TaskAllocation";
// import HealthCheck from "./pages/HealthCheck";
// import Reports from "./pages/Reports";
// import Settings from "./pages/Settings";
// import Animation from "./pages/Animation";
// import LoginForm from "./pages/LoginForm";
// import SignupForm from "./pages/SignupForm";

// // Layout
// import DashboardLayout from "./components/DashboardLayout";
// import ProtectedRoute from "./components/ProtectedRoute";

// export default function App() {
//   // 🌙 Dark Mode state
//   const [darkMode, setDarkMode] = useState(
//     localStorage.getItem("darkMode") === "true"
//   );

//   // Update HTML class based on dark mode
//   useEffect(() => {
//     if (darkMode) document.documentElement.classList.add("dark");
//     else document.documentElement.classList.remove("dark");
//   }, [darkMode]);

//   return (
//     <Router>
//       <Routes>
//         {/* Public routes */}
//         <Route path="/login" element={<LoginForm />} />
//         <Route path="/signup" element={<SignupForm />} />
//         <Route path="/animation" element={<Animation />} />

//         {/* Protected Routes */}
//         <Route
//           path="/"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <Dashboard />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/maps"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <Maps />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/realtime"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <RealTime />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/tasks"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <TaskAllocation />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/health"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <HealthCheck />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/reports"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <Reports />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         <Route
//           path="/settings"
//           element={
//             <ProtectedRoute>
//               <DashboardLayout
//                 darkMode={darkMode}
//                 setDarkMode={setDarkMode}
//               >
//                 <Settings
//                   darkMode={darkMode}
//                   setDarkMode={setDarkMode}
//                 />
//               </DashboardLayout>
//             </ProtectedRoute>
//           }
//         />

//         {/* Redirect everything else */}
//         <Route path="*" element={<Navigate to="/login" replace />} />
//       </Routes>
//     </Router>
//   );
// }

import React, { useState, useEffect } from "react";
import { BrowserRouter as Router, Routes, Route, Navigate } from "react-router-dom";

// Pages
import Dashboard from "./pages/Dashboard";
import Maps from "./pages/Maps";
import RealTime from "./pages/RealTime";
import TaskAllocation from "./pages/TaskAllocation";
import HealthCheck from "./pages/HealthCheck";
import Reports from "./pages/Reports";
import Settings from "./pages/Settings";
import Animation from "./pages/Animation";
import LoginForm from "./pages/LoginForm";
import SignupForm from "./pages/SignupForm";

// Layout
import DashboardLayout from "./components/DashboardLayout";
import ProtectedRoute from "./components/ProtectedRoute";

export default function App() {
  // 🌙 Dark Mode state
  const [darkMode, setDarkMode] = useState(
    localStorage.getItem("darkMode") === "true"
  );

  // Update HTML class based on dark mode
  useEffect(() => {
    if (darkMode) document.documentElement.classList.add("dark");
    else document.documentElement.classList.remove("dark");
  }, [darkMode]);

  return (
    <Router>
      <Routes>
        {/* Public routes */}
        <Route path="/login" element={<LoginForm />} />
        <Route path="/signup" element={<SignupForm />} />
        <Route path="/animation" element={<Animation />} />

        {/* Protected Routes */}
        <Route
          path="/"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <Dashboard />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/maps"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <Maps />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/realtime"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <RealTime />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/tasks"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <TaskAllocation />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/health"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <HealthCheck />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/reports"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <Reports />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        <Route
          path="/settings"
          element={
            <ProtectedRoute>
              <DashboardLayout
                darkMode={darkMode}
                setDarkMode={setDarkMode}
              >
                <Settings
                  darkMode={darkMode}
                  setDarkMode={setDarkMode}
                />
              </DashboardLayout>
            </ProtectedRoute>
          }
        />

        {/* Redirect everything else */}
        <Route path="*" element={<Navigate to="/login" replace />} />
      </Routes>
    </Router>
  );
}