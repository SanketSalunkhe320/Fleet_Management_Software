// import React from "react";

// export default function Footer() {
//   return (
//     <footer className="w-full bg-slate-100 border-t mt-0 select-none">
//       <div className="max-w-8xl mx-auto px-7 py-6 flex items-center justify-center text-sm text-slate-800">
//         © 2025 Taikisha India | All Rights Reserved.
//       </div>
//     </footer>
//   );
// }

import React from "react";
import { useLanguage } from "../context/LanguageContext";

export default function Footer() {
  const { t } = useLanguage();

  return (
    <footer className="w-full bg-slate-100 border-t mt-0 select-none">
      <div className="max-w-8xl mx-auto px-7 py-3 flex items-center justify-center text-xs text-slate-800">
        {t("copyright")}
      </div>
    </footer>
  );
}