// context/LanguageContext.js
import React, { createContext, useState, useContext, useEffect } from 'react';

// Default translations for English
const defaultTranslations = {
  settings: "Settings",
  dark_mode: "Dark Mode",
  enable_notifications: "Enable Notifications",
  language: "Language",
  save: "Save",
  reset: "Reset",
  settings_saved: "Settings saved successfully!",
  logout: "Logout",
  notifications: "Notifications",
  
  // Header translations
  agvs: "AGVs",
  robot_status: "Robot Status",
  updated: "Updated",
  never: "Never",
  loading: "Loading...",
  wifi_connected: "WiFi Connected",
  wifi_disconnected: "WiFi Disconnected",
  select_online: "Select Online",
  clear: "Clear",
  search_agvs: "Search AGVs...",
  agvs_selected: "AGVs selected",
  cancel: "Cancel",
  confirm: "Confirm",
  
  // Sidebar translations
  dashboard: "Dashboard",
  maps: "Maps",
  realtime_visualisation: "Real-Time Visualisation",
  task_allocation: "Task Allocation",
  health_check: "Health Check",
  reports: "Reports",
  
  // Footer translation
  copyright: "© 2025 Taikisha India | All Rights Reserved.",
  
  // Dashboard translations
  system_status: "System Status",
  ros_bridge: "ROS Bridge",
  stations_loaded: "Stations Loaded",
  current_location: "Current Location",
  target_location: "Target Location",
  amr_status_distribution: "AMR Status Distribution",
  battery_level_overview: "Battery Level Overview",
  amr_status: "AMR Status",
  station_information: "Station Information",
  
  // Status translations
  online: "Online",
  offline: "Offline",
  running: "Running",
  idle: "Idle",
  maintenance: "Maintenance",

  // Health translations
  health_check: "Health Check",
  health: {
    amr: "AMR",
    imu: "IMU",
    lidar: "LIDAR",
    camera: "Depth Camera",
    motor: "Motor Drive",
    position: "Position",
    status: {
      ok: "OK",
      fault: "FAULT",
      idle: "IDLE",
      unknown: "UNKNOWN"
    }
  }
};

// Marathi translations
const mrTranslations = {
  settings: "सेटिंग्ज",
  dark_mode: "डार्क मोड",
  enable_notifications: "सूचना सक्षम करा",
  language: "भाषा",
  save: "जतन करा",
  reset: "रीसेट करा",
  settings_saved: "सेटिंग्ज यशस्वीरित्या जतन केल्या!",
  logout: "लॉग आउट",
  notifications: "सूचना",
  
  agvs: "एजीव्ही",
  robot_status: "रोबोट स्थिती",
  updated: "अपडेट केले",
  never: "कधीही नाही",
  loading: "लोड होत आहे...",
  wifi_connected: "WiFi कनेक्ट केले",
  wifi_disconnected: "WiFi डिस्कनेक्ट केले",
  select_online: "ऑनलाइन निवडा",
  clear: "साफ करा",
  search_agvs: "एजीव्ही शोधा...",
  agvs_selected: "एजीव्ही निवडले",
  cancel: "रद्द करा",
  confirm: "पुष्टी करा",
  
  dashboard: "डॅशबोर्ड",
  maps: "नकाशे",
  realtime_visualisation: "रीयल-टाइम व्हिज्युअलायझेशन",
  task_allocation: "कार्य वाटप",
  health_check: "आरोग्य तपासणी",
  reports: "अहवाल",
  
  copyright: "© २०२५ ताइकिशा इंडिया | सर्व हक्क राखीव.",
  
  system_status: "सिस्टम स्थिती",
  ros_bridge: "ROS ब्रिज",
  stations_loaded: "स्टेशने लोड केली",
  current_location: "सध्याचे स्थान",
  target_location: "लक्ष्य स्थान",
  amr_status_distribution: "एएमआर स्थिती वितरण",
  battery_level_overview: "बॅटरी पातळी अवलोकन",
  amr_status: "एएमआर स्थिती",
  station_information: "स्टेशन माहिती",
  
  online: "ऑनलाइन",
  offline: "ऑफलाइन",
  running: "चालू",
  idle: "निष्क्रिय",
  maintenance: "देखभाल",

  // Health translations
  health_check: "आरोग्य तपासणी",
  health: {
    amr: "एएमआर",
    imu: "आयएमयू",
    lidar: "लिडार",
    camera: "खोली कॅमेरा",
    motor: "मोटर ड्राइव्ह",
    position: "स्थान",
    status: {
      ok: "ठीक",
      fault: "दोष",
      idle: "निष्क्रिय",
      unknown: "अज्ञात"
    }
  }
};

// Hindi translations
const hiTranslations = {
  settings: "सेटिंग्स",
  dark_mode: "डार्क मोड",
  enable_notifications: "सूचनाएं सक्षम करें",
  language: "भाषा",
  save: "सहेजें",
  reset: "रीसेट करें",
  settings_saved: "सेटिंग्स सफलतापूर्वक सहेजी गईं!",
  logout: "लॉग आउट",
  notifications: "सूचनाएं",
  
  agvs: "एजीवी",
  robot_status: "रोबोट स्थिति",
  updated: "अपडेट किया गया",
  never: "कभी नहीं",
  loading: "लोड हो रहा है...",
  wifi_connected: "WiFi जुड़ा हुआ",
  wifi_disconnected: "WiFi डिस्कनेक्ट किया गया",
  select_online: "ऑनलाइन चुनें",
  clear: "साफ करें",
  search_agvs: "एजीवी खोजें...",
  agvs_selected: "एजीवी चुने गए",
  cancel: "रद्द करें",
  confirm: "पुष्टि करें",
  
  dashboard: "डैशबोर्ड",
  maps: "मानचित्र",
  realtime_visualisation: "रियल-टाइम विज़ुअलाइज़ेशन",
  task_allocation: "कार्य आवंटन",
  health_check: "स्वास्थ्य जांच",
  reports: "रिपोर्ट",
  
  copyright: "© २०२५ ताइकिशा इंडिया | सर्व अधिकार सुरक्षित।",
  
  system_status: "सिस्टम स्थिति",
  ros_bridge: "ROS ब्रिज",
  stations_loaded: "स्टेशन लोड किए गए",
  current_location: "वर्तमान स्थान",
  target_location: "लक्ष्य स्थान",
  amr_status_distribution: "एएमआर स्थिति वितरण",
  battery_level_overview: "बैटरी स्तर अवलोकन",
  amr_status: "एएमआर स्थिति",
  station_information: "स्टेशन सूचना",
  
  online: "ऑनलाइन",
  offline: "ऑफलाइन",
  running: "चल रहा है",
  idle: "निष्क्रिय",
  maintenance: "रखरखाव",

  // Health translations
  health_check: "स्वास्थ्य जांच",
  health: {
    amr: "एएमआर",
    imu: "आईएमयू",
    lidar: "लिडार",
    camera: "डेप्थ कैमरा",
    motor: "मोटर ड्राइव",
    position: "स्थिति",
    status: {
      ok: "ठीक",
      fault: "दोष",
      idle: "निष्क्रिय",
      unknown: "अज्ञात"
    }
  }
};

const translations = {
  en: defaultTranslations,
  mr: mrTranslations,
  hi: hiTranslations
};

const LanguageContext = createContext();

export const LanguageProvider = ({ children }) => {
  const [language, setLanguage] = useState(() => {
    return localStorage.getItem('language') || 'en';
  });

  const [translationsData, setTranslationsData] = useState(translations[language]);

  useEffect(() => {
    setTranslationsData(translations[language]);
    localStorage.setItem('language', language);
  }, [language]);

  // Updated t() function that supports nested keys with dot notation
  const t = (key) => {
    if (!key) return '';
    
    // Split by dots to handle nested keys
    const keys = key.split('.');
    let result = translationsData;
    
    // Traverse through the nested object
    for (const k of keys) {
      if (result && typeof result === 'object' && k in result) {
        result = result[k];
      } else {
        // If key not found, return the original key
        console.warn(`Translation not found for key: "${key}"`);
        return key;
      }
    }
    
    // Return the final value or the key if value is undefined
    return result || key;
  };

  return (
    <LanguageContext.Provider value={{ language, setLanguage, t }}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = () => useContext(LanguageContext);