import React from "react";
import Header from "./Header";
import Sidebar from "./Sidebar";
import Footer from "./Footer";

export default function DashboardLayout({ children, darkMode, setDarkMode }) {
  return (
    <div className="flex min-h-screen bg-gray-100 dark:bg-gray-900 transition-colors">
      <Sidebar darkMode={darkMode} />
      <div className="flex-1 flex flex-col">
        <Header darkMode={darkMode} setDarkMode={setDarkMode} />
        <main className="p-4 flex-1 text-gray-800 dark:text-gray-100 transition-colors">
          {children}
        </main>
        <Footer />
      </div>
    </div>
  );
}
