// import React, { useState, useEffect } from "react";

// export default function Settings({ darkMode, setDarkMode, language, setLanguage }) {
//   const [localDarkMode, setLocalDarkMode] = useState(darkMode);
//   const [notifications, setNotifications] = useState(false);
//   const [localLanguage, setLocalLanguage] = useState(language);

//   // Sync when global darkMode changes
//   useEffect(() => {
//     setLocalDarkMode(darkMode);
//   }, [darkMode]);

//   // Sync when global language changes
//   useEffect(() => {
//     setLocalLanguage(language);
//   }, [language]);

//   const handleSave = () => {
//     // Save dark mode
//     setDarkMode(localDarkMode);
//     localStorage.setItem("darkMode", localDarkMode);

//     // Save language
//     setLanguage(localLanguage);
//     localStorage.setItem("language", localLanguage);

//     alert("✅ Settings saved successfully!");
//   };

//   const handleReset = () => {
//     // Reset UI
//     setLocalDarkMode(false);
//     setNotifications(false);
//     setLocalLanguage("en");

//     // Reset app-wide states
//     setDarkMode(false);
//     setLanguage("en");

//     // Reset localStorage
//     localStorage.setItem("darkMode", false);
//     localStorage.setItem("language", "en");
//   };

//   return (
//     <div className="p-4 min-h-screen bg-gray-100 dark:bg-gray-900 transition-colors select-none">
//       <h2 className="text-xl font-semibold mb-4 text-gray-800 dark:text-white">
//         Settings
//       </h2>

//       <div className="bg-white dark:bg-gray-800 rounded shadow p-6 space-y-6 transition-colors">

//         {/* Dark Mode Switch */}
//         <div className="flex items-center justify-between">
//           <span className="text-gray-700 dark:text-gray-200">Dark Mode</span>
//           <input
//             type="checkbox"
//             checked={localDarkMode}
//             onChange={(e) => setLocalDarkMode(e.target.checked)}
//             className="w-5 h-5 accent-sky-700 dark:accent-sky-400"
//           />
//         </div>

//         {/* Notifications Switch */}
//         <div className="flex items-center justify-between">
//           <span className="text-gray-700 dark:text-gray-200">Enable Notifications</span>
//           <input
//             type="checkbox"
//             checked={notifications}
//             onChange={(e) => setNotifications(e.target.checked)}
//             className="w-5 h-5 accent-sky-700 dark:accent-sky-400"
//           />
//         </div>

//         {/* Language Selector */}
//         <div>
//           <label className="block mb-2 text-gray-700 dark:text-gray-200">
//             Language
//           </label>

//           <select
//             value={localLanguage}
//             onChange={(e) => setLocalLanguage(e.target.value)}
//             className="border rounded px-3 py-2 w-full bg-white dark:bg-gray-700 text-black dark:text-white"
//           >
//             <option value="mr">मराठी</option>
//             <option value="en">English</option>
//             <option value="hi">हिन्दी</option>
//           </select>
//         </div>

//         {/* Buttons */}
//         <div className="flex justify-end gap-3">
//           <button
//             onClick={handleReset}
//             className="px-4 py-2 bg-gray-200 dark:bg-gray-600 dark:text-white rounded hover:bg-gray-300 dark:hover:bg-gray-500 transition-colors"
//           >
//             Reset
//           </button>

//           <button
//             onClick={handleSave}
//             className="px-4 py-2 bg-sky-700 text-white rounded hover:bg-sky-800 transition-colors"
//           >
//             Save
//           </button>
//         </div>
//       </div>
//     </div>
//   );
// }

import React, { useState, useEffect } from "react";
import { useLanguage } from "../context/LanguageContext";

export default function Settings({ darkMode, setDarkMode }) {
  const { language, setLanguage: changeLanguage, t } = useLanguage();
  const [localDarkMode, setLocalDarkMode] = useState(darkMode);
  const [notifications, setNotifications] = useState(false);
  const [localLanguage, setLocalLanguage] = useState(language);

  // Sync when global darkMode changes
  useEffect(() => {
    setLocalDarkMode(darkMode);
  }, [darkMode]);

  // Sync when global language changes
  useEffect(() => {
    setLocalLanguage(language);
  }, [language]);

  const handleSave = () => {
    // Save dark mode
    setDarkMode(localDarkMode);
    localStorage.setItem("darkMode", localDarkMode);

    // Save language
    changeLanguage(localLanguage);

    alert(t("settings_saved"));
  };

  const handleReset = () => {
    // Reset UI
    setLocalDarkMode(false);
    setNotifications(false);
    setLocalLanguage("en");

    // Reset app-wide states
    setDarkMode(false);
    changeLanguage("en");

    // Reset localStorage
    localStorage.setItem("darkMode", false);
  };

  return (
    <div className="p-4 min-h-screen bg-gray-100 dark:bg-gray-900 transition-colors select-none">
      <h2 className="text-xl font-semibold mb-4 text-gray-800 dark:text-white">
        {t("settings")}
      </h2>

      <div className="bg-white dark:bg-gray-800 rounded shadow p-6 space-y-6 transition-colors">

        {/* Dark Mode Switch */}
        <div className="flex items-center justify-between">
          <span className="text-gray-700 dark:text-gray-200">{t("dark_mode")}</span>
          <input
            type="checkbox"
            checked={localDarkMode}
            onChange={(e) => setLocalDarkMode(e.target.checked)}
            className="w-5 h-5 accent-sky-700 dark:accent-sky-400"
          />
        </div>

        {/* Notifications Switch */}
        <div className="flex items-center justify-between">
          <span className="text-gray-700 dark:text-gray-200">{t("enable_notifications")}</span>
          <input
            type="checkbox"
            checked={notifications}
            onChange={(e) => setNotifications(e.target.checked)}
            className="w-5 h-5 accent-sky-700 dark:accent-sky-400"
          />
        </div>

        {/* Language Selector */}
        <div>
          <label className="block mb-2 text-gray-700 dark:text-gray-200">
            {t("language")}
          </label>

          <select
            value={localLanguage}
            onChange={(e) => setLocalLanguage(e.target.value)}
            className="border rounded px-3 py-2 w-full bg-white dark:bg-gray-700 text-black dark:text-white"
          >
            <option value="mr">मराठी</option>
            <option value="en">English</option>
            <option value="hi">हिन्दी</option>
          </select>
        </div>

        {/* Buttons */}
        <div className="flex justify-end gap-3">
          <button
            onClick={handleReset}
            className="px-4 py-2 bg-gray-200 dark:bg-gray-600 dark:text-white rounded hover:bg-gray-300 dark:hover:bg-gray-500 transition-colors"
          >
            {t("reset")}
          </button>

          <button
            onClick={handleSave}
            className="px-4 py-2 bg-sky-700 text-white rounded hover:bg-sky-800 transition-colors"
          >
            {t("save")}
          </button>
        </div>
      </div>
    </div>
  );
}