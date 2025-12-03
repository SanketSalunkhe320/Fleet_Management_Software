
import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { FiEye, FiEyeOff } from "react-icons/fi";

export default function SignupForm() {
  const [orgName, setOrgName] = useState("");
  const [orgAddress, setOrgAddress] = useState("");
  const [contact, setContact] = useState("");
  const [password, setPassword] = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const navigate = useNavigate();

  const handleSignup = async (e) => {
    e.preventDefault();

    if (!orgName || !orgAddress || !contact || !password) {
      alert("Please fill all the fields.");
      return;
    }

    const isPhone = /^[0-9]{10}$/.test(contact.trim());
    const isEmail = /^[^\s@]+@[^\s@]+\.[^\s@]+$/.test(contact.trim());

    if (!isPhone && !isEmail) {
      alert("Please enter a valid 10-digit phone number or email ID.");
      return;
    }

    const passwordRegex = /^(?=.*[a-z])(?=.*[A-Z])(?=.*\d)(?=.*[!@#$%^&*])[A-Za-z\d!@#$%^&*]{8,}$/;

    if (!passwordRegex.test(password)) {
      alert(
        "Password must be at least 8 characters long and include uppercase, lowercase, number, and special character."
      );
      return;
    }

    setIsLoading(true);

    try {
      const userData = {
        orgName: orgName.trim(),
        orgAddress: orgAddress.trim(),
        contact: contact.trim(),
        password: password.trim(),
      };

      // Send signup data to backend
      const response = await fetch("http://localhost:5000/api/signup", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify(userData),
      });

      const data = await response.json();

      if (response.ok && data.success) {
        // Also save to localStorage for backward compatibility
        localStorage.setItem("registeredUser", JSON.stringify(userData));
        alert("Signup successful! Please login to continue.");
        navigate("/login", { replace: true });
      } else {
        alert(data.message || "Signup failed. Please try again.");
      }
    } catch (error) {
      console.error("Signup error:", error);
      // Fallback to localStorage if backend is unavailable
      localStorage.setItem("registeredUser", JSON.stringify({
        orgName: orgName.trim(),
        orgAddress: orgAddress.trim(),
        contact: contact.trim(),
        password: password.trim(),
      }));
      alert("Signup successful! Please login to continue.");
      navigate("/login", { replace: true });
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="flex items-center justify-center min-h-screen bg-gradient-to-tr from-sky-700 to-sky-400 px-4">
      <div className="bg-white rounded-xl shadow-lg p-6 sm:p-10 w-full max-w-sm sm:max-w-md md:max-w-lg backdrop-blur-sm">
        <h2 className="text-2xl sm:text-3xl font-bold text-sky-900 mb-6 text-center">
          Sign Up
        </h2>

        <form className="flex flex-col gap-4" onSubmit={handleSignup}>
          <input
            type="text"
            placeholder="Organization Name"
            value={orgName}
            onChange={(e) => setOrgName(e.target.value)}
            className="px-4 py-2 border rounded-md text-sm sm:text-base focus:outline-none focus:ring-2 focus:ring-sky-500"
            required
            disabled={isLoading}
          />

          <input
            type="text"
            placeholder="Organization Address"
            value={orgAddress}
            onChange={(e) => setOrgAddress(e.target.value)}
            className="px-4 py-2 border rounded-md text-sm sm:text-base focus:outline-none focus:ring-2 focus:ring-sky-500"
            required
            disabled={isLoading}
          />

          <input
            type="text"
            placeholder="Phone No / Email ID"
            value={contact}
            onChange={(e) => setContact(e.target.value)}
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
            {isLoading ? "Signing Up..." : "Sign Up"}
          </button>
        </form>

        <p className="text-xs sm:text-sm text-center text-gray-600 mt-4">
          Already have an account?{" "}
          <span
            className="text-sky-700 font-semibold hover:underline cursor-pointer"
            onClick={() => navigate("/login")}
          >
            Login
          </span>
        </p>
      </div>
    </div>
  );
}

//JWT authentication

// import React, { useState } from "react";
// import { useNavigate } from "react-router-dom";
// import { FiEye, FiEyeOff } from "react-icons/fi";

// export default function SignupForm() {
//   const [orgName, setOrgName] = useState("");
//   const [orgAddress, setOrgAddress] = useState("");
//   const [contact, setContact] = useState("");
//   const [password, setPassword] = useState("");
//   const [showPassword, setShowPassword] = useState(false);
//   const [isLoading, setIsLoading] = useState(false);
//   const navigate = useNavigate();

//   const handleSignup = async (e) => {
//     e.preventDefault();

//     if (!orgName || !orgAddress || !contact || !password) {
//       alert("Please fill all fields.");
//       return;
//     }

//     setIsLoading(true);

//     try {
//       const response = await fetch("http://localhost:5000/api/signup", {
//         method: "POST",
//         headers: { "Content-Type": "application/json" },
//         body: JSON.stringify({
//           orgName: orgName.trim(),
//           orgAddress: orgAddress.trim(),
//           contact: contact.trim(),
//           password: password.trim(),
//         }),
//       });

//       const data = await response.json();

//       if (response.ok && data.success) {
//         alert("Signup successful! Please login.");
//         navigate("/login");
//         return;
//       }

//       alert(data.message || "Signup failed");
//     } catch (error) {
//       console.error(error);
//       alert("Server unreachable! Try again.");
//     } finally {
//       setIsLoading(false);
//     }
//   };

//   return (
//     <div className="flex items-center justify-center min-h-screen bg-gradient-to-tr from-sky-700 to-sky-400 px-4">
//       <div className="bg-white p-10 rounded-xl shadow-lg max-w-md w-full">

//         <h2 className="text-3xl font-bold text-center text-sky-900 mb-6">
//           Sign Up
//         </h2>

//         <form className="flex flex-col gap-4" onSubmit={handleSignup}>

//           <input
//             type="text"
//             placeholder="Organization Name"
//             value={orgName}
//             onChange={(e) => setOrgName(e.target.value)}
//             className="px-4 py-2 border rounded-md"
//             required
//           />

//           <input
//             type="text"
//             placeholder="Organization Address"
//             value={orgAddress}
//             onChange={(e) => setOrgAddress(e.target.value)}
//             className="px-4 py-2 border rounded-md"
//             required
//           />

//           <input
//             type="text"
//             placeholder="Phone / Email"
//             value={contact}
//             onChange={(e) => setContact(e.target.value)}
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
//             {isLoading ? "Signing Up..." : "Sign Up"}
//           </button>
//         </form>

//         <p className="text-sm text-center mt-4">
//           Already have an account?
//           <span
//             onClick={() => navigate("/login")}
//             className="text-sky-700 font-semibold hover:underline cursor-pointer ml-1"
//           >
//             Login
//           </span>
//         </p>

//       </div>
//     </div>
//   );
// }
