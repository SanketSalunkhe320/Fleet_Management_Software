



import React, { useState, useEffect } from "react";
import { useNavigate } from "react-router-dom";
import { FiEye, FiEyeOff } from "react-icons/fi";

export default function LoginForm() {
  const [username, setUsername] = useState("");
  const [password, setPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const navigate = useNavigate();

  // Prevent browser back button
  useEffect(() => {
    window.history.pushState(null, "", window.location.href);
    const handlePopState = () => {
      window.history.pushState(null, "", window.location.href);
    };
    window.addEventListener("popstate", handlePopState);
    return () => window.removeEventListener("popstate", handlePopState);
  }, []);

  const handleLogin = async (e) => {
    e.preventDefault();

    if (!username || !password) {
      alert("Please fill all the fields.");
      return;
    }

    setIsLoading(true);

    try {
      // Try backend authentication first
      const response = await fetch("http://localhost:5000/api/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          username: username.trim(),
          password: password.trim(),
        }),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Store login status and user info
        localStorage.setItem("isLoggedIn", "true");
        localStorage.setItem("currentUser", JSON.stringify(data.user));
        localStorage.setItem("authToken", data.token);
        localStorage.setItem("loggedInUser", data.user.orgName);
        
        alert("Login successful!");
        navigate("/", { replace: true });
        return;
      }

      // Fallback to localStorage check if backend fails
      const savedUser = JSON.parse(localStorage.getItem("registeredUser") || "null");

      if (!savedUser) {
        alert("No registered account found! Please sign up first.");
        navigate("/signup", { replace: true });
        return;
      }

      if (
        (username.trim().toLowerCase() === savedUser.orgName.toLowerCase() ||
          username.trim().toLowerCase() === savedUser.contact.toLowerCase()) &&
        password.trim() === savedUser.password
      ) {
        localStorage.setItem("isLoggedIn", "true");
        localStorage.setItem("currentUser", JSON.stringify(savedUser));
        localStorage.setItem("loggedInUser", savedUser.orgName);
        navigate("/", { replace: true });
      } else {
        alert(data.message || "Invalid credentials! Please try again.");
      }
    } catch (error) {
      console.error("Login error:", error);
      // Fallback to localStorage authentication
      const savedUser = JSON.parse(localStorage.getItem("registeredUser") || "null");

      if (!savedUser) {
        alert("No registered account found! Please sign up first.");
        navigate("/signup", { replace: true });
        return;
      }

      if (
        (username.trim().toLowerCase() === savedUser.orgName.toLowerCase() ||
          username.trim().toLowerCase() === savedUser.contact.toLowerCase()) &&
        password.trim() === savedUser.password
      ) {
        localStorage.setItem("isLoggedIn", "true");
        localStorage.setItem("currentUser", JSON.stringify(savedUser));
        localStorage.setItem("loggedInUser", savedUser.orgName);
        navigate("/", { replace: true });
      } else {
        alert("Invalid credentials! Please try again.");
      }
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="flex items-center justify-center min-h-screen bg-gradient-to-tr from-sky-700 to-sky-400 px-4">
      <div className="bg-white rounded-xl shadow-lg p-6 sm:p-10 w-full max-w-sm sm:max-w-md md:max-w-lg backdrop-blur-sm select-none">
        <h2 className="text-2xl sm:text-3xl font-bold text-sky-900 mb-6 text-center">
          Login
        </h2>

        <form className="flex flex-col gap-4" onSubmit={handleLogin}>
          <input
            type="text"
            placeholder="Organization Name or Email / Phone"
            value={username}
            onChange={(e) => setUsername(e.target.value)}
            className="px-4 py-2 border rounded-md text-sm sm:text-base focus:outline-none focus:ring-2 focus:ring-sky-500"
            required
            disabled={isLoading}
          />

          <div className="relative">
            <input
              type={showPassword ? "text" : "password"}
              placeholder="Password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              className="w-full px-4 py-2 border rounded-md text-sm sm:text-base focus:outline-none focus:ring-2 focus:ring-sky-500 pr-10"
              required
              disabled={isLoading}
            />
            <span
              className="absolute right-3 top-1/2 transform -translate-y-1/2 cursor-pointer text-gray-600 hover:text-gray-800"
              onClick={() => setShowPassword(!showPassword)}
            >
              {showPassword ? <FiEyeOff /> : <FiEye />}
            </span>
          </div>

          <button
            type="submit"
            disabled={isLoading}
            className="bg-sky-700 text-white py-2 sm:py-3 rounded-md hover:bg-sky-800 transition-colors text-sm sm:text-base font-medium disabled:opacity-50 disabled:cursor-not-allowed"
          >
            {isLoading ? "Logging in..." : "Login"}
          </button>
        </form>

        <p className="text-xs sm:text-sm text-center text-gray-600 mt-4">
          Don't have an account?{" "}
          <span
            className="text-sky-700 font-semibold hover:underline cursor-pointer"
            onClick={() => navigate("/signup", { replace: true })}
          >
            Sign up
          </span>
        </p>
      </div>
    </div>
  );
}


//JWT AUthentication

// import React, { useState, useEffect } from "react";
// import { useNavigate } from "react-router-dom";
// import { FiEye, FiEyeOff } from "react-icons/fi";

// export default function LoginForm() {
//   const [username, setUsername] = useState("");
//   const [password, setPassword] = useState("");
//   const [showPassword, setShowPassword] = useState(false);
//   const [isLoading, setIsLoading] = useState(false);
//   const navigate = useNavigate();

//   useEffect(() => {
//     window.history.pushState(null, "", window.location.href);
//     const handlePopState = () => {
//       window.history.pushState(null, "", window.location.href);
//     };
//     window.addEventListener("popstate", handlePopState);
//     return () => window.removeEventListener("popstate", handlePopState);
//   }, []);

//   const handleLogin = async (e) => {
//     e.preventDefault();

//     if (!username || !password) {
//       alert("Please fill all the fields.");
//       return;
//     }

//     setIsLoading(true);

//     try {
//       const response = await fetch("http://localhost:5000/api/login", {
//         method: "POST",
//         headers: {
//           "Content-Type": "application/json",
//         },
//         body: JSON.stringify({
//           username: username.trim(),
//           password: password.trim(),
//         }),
//       });

//       const data = await response.json();

//       // JWT login success
//       if (response.ok && data.success) {
//         localStorage.setItem("authToken", data.token); // <-- JWT saved
//         localStorage.setItem("isLoggedIn", "true");
//         localStorage.setItem("currentUser", JSON.stringify(data.user));
//         localStorage.setItem("loggedInUser", data.user.orgName);

//         alert("Login successful!");
//         navigate("/", { replace: true });
//         return;
//       }

//       alert(data.message || "Invalid credentials!");
//     } catch (error) {
//       console.error("Login error:", error);
//       alert("Server unreachable! Try again.");
//     } finally {
//       setIsLoading(false);
//     }
//   };

//   return (
//     <div className="flex items-center justify-center min-h-screen bg-gradient-to-tr from-sky-700 to-sky-400 px-4">
//       <div className="bg-white rounded-xl shadow-lg p-6 sm:p-10 w-full max-w-sm sm:max-w-md md:max-w-lg backdrop-blur-sm">
        
//         <h2 className="text-2xl sm:text-3xl font-bold text-sky-900 mb-6 text-center">
//           Login
//         </h2>

//         <form className="flex flex-col gap-4" onSubmit={handleLogin}>
          
//           <input
//             type="text"
//             placeholder="Organization Name / Email / Phone"
//             value={username}
//             onChange={(e) => setUsername(e.target.value)}
//             className="px-4 py-2 border rounded-md"
//             required
//           />

//           <div className="relative">
//             <input
//               type={showPassword ? "text" : "password"}
//               placeholder="Password"
//               value={password}
//               onChange={(e) => setPassword(e.target.value)}
//               className="w-full px-4 py-2 border rounded-md pr-10"
//               required
//             />
//             <span
//               onClick={() => setShowPassword(!showPassword)}
//               className="absolute right-3 top-1/2 -translate-y-1/2 cursor-pointer"
//             >
//               {showPassword ? <FiEyeOff /> : <FiEye />}
//             </span>
//           </div>

//           <button
//             type="submit"
//             disabled={isLoading}
//             className="bg-sky-700 text-white py-3 rounded-md"
//           >
//             {isLoading ? "Logging in..." : "Login"}
//           </button>

//         </form>

//         <p className="text-sm text-center mt-4">
//           Don't have an account?
//           <span
//             onClick={() => navigate("/signup")}
//             className="text-sky-700 font-semibold hover:underline cursor-pointer ml-1"
//           >
//             Sign up
//           </span>
//         </p>
//       </div>
//     </div>
//   );
// }
