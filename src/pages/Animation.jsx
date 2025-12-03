// import React, { useEffect } from "react";
// import { useNavigate } from "react-router-dom";
// import TaikishaLogo from "../assets/Taikishaimage1.png";

// export default function Animation() {
//   const navigate = useNavigate();

//   useEffect(() => {
//     const timer = setTimeout(() => {
//       navigate("/login", {replace : true});
//     }, 5000);

//     return () => clearTimeout(timer);
//   }, [navigate]);

//   return (
//     <div
//       style={{
//         fontFamily: "Arial, sans-serif",
//         background: "url('black.jpeg') no-repeat center center/cover",
//         color: "#fff",
//         display: "flex",
//         flexDirection: "column",
//         alignItems: "center",
//         justifyContent: "center",
//         height: "100vh",
//         width: "100vw",
//         textAlign: "center",
//         position: "relative",
//         overflow: "hidden",
//       }}
//     >
//       <style>
//         {`
//           @keyframes slideFromTop {
//             0% { transform: translateY(-100%); opacity: 0; }
//             100% { transform: translateY(0); opacity: 1; }
//           }
//           .logo-slide {
//             width: 80%;
//             max-width: 650px;
//             height: auto;
//             margin-bottom: 40px;
//             animation: slideFromTop 2s ease-out forwards;
//           }
//         `}
//       </style>

//       <img src={TaikishaLogo} alt="Taikisha Logo" className="logo-slide" />

//       <div
//         style={{
//           position: "absolute",
//           bottom: "10%",
//           left: "5%",
//           fontSize: "clamp(12px, 2.5vw, 20px)", // Min, Preferred, Max
//           color: "#181616ff",
//           textAlign: "left",
//           width: "90%",
//         }}
//       >
//         Design and Developed by Taikisha India.
//       </div>
//     </div>
//   );
// }




import React, { useEffect } from "react";
import { useNavigate } from "react-router-dom";
import TaikishaLogo from "../assets/Taikishaimage1.png";

export default function Animation() {
  const navigate = useNavigate();

  useEffect(() => {
    const timer = setTimeout(() => {
      navigate("/login", {replace: true});
    }, 5000);

    return () => clearTimeout(timer);
  }, [navigate]);

  return (
    <div
      style={{
        fontFamily: "'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
        background: "linear-gradient(135deg, #667eea 0%, #4ca6daff 100%)",
        color: "#fff",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        justifyContent: "center",
        height: "100vh",
        width: "100vw",
        textAlign: "center",
        position: "relative",
        overflow: "hidden",
      }}
    >
      <style>
        {`
          @keyframes slideFromTop {
            0% { transform: translateY(-100px); opacity: 0; }
            100% { transform: translateY(0); opacity: 1; }
          }
          
          @keyframes fadeInUp {
            0% { transform: translateY(30px); opacity: 0; }
            100% { transform: translateY(0); opacity: 1; }
          }
          
          @keyframes pulse {
            0% { transform: scale(1); }
            50% { transform: scale(1.05); }
            100% { transform: scale(1); }
          }
          
          @keyframes typing {
            from { width: 0 }
            to { width: 100% }
          }
          
          @keyframes blink {
            0%, 100% { border-color: transparent }
            50% { border-color: white }
          }
          
          .logo-slide {
            width: 70%;
            max-width: 500px;
            height: auto;
            margin-bottom: 30px;
            animation: slideFromTop 1.5s ease-out forwards, pulse 3s ease-in-out 2s infinite;
          }
          
          .main-title {
            font-size: clamp(24px, 4vw, 48px);
            font-weight: 700;
            margin-bottom: 15px;
            background: linear-gradient(45deg, #fff, #f0f0f0);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            animation: fadeInUp 1s ease-out 0.5s both;
          }
          
          .subtitle {
            font-size: clamp(16px, 2vw, 24px);
            font-weight: 500;
            margin-bottom: 30px;
            opacity: 0.9;
            animation: fadeInUp 1s ease-out 1s both;
          }
          
          .tagline {
            font-size: clamp(14px, 1.5vw, 18px);
            font-weight: 400;
            margin-bottom: 40px;
            opacity: 0.8;
            font-style: italic;
            animation: fadeInUp 1s ease-out 1.5s both;
          }
          
          .loading-text {
            font-size: clamp(14px, 1.5vw, 18px);
            margin-bottom: 20px;
            animation: fadeInUp 1s ease-out 2s both;
          }
          
          .loading-bar {
            width: 200px;
            height: 4px;
            background: rgba(255,255,255,0.3);
            border-radius: 2px;
            overflow: hidden;
            margin-bottom: 30px;
            animation: fadeInUp 1s ease-out 2s both;
          }
          
          .loading-progress {
            width: 0%;
            height: 100%;
            background: linear-gradient(90deg, #fff, #ffd700);
            border-radius: 2px;
            animation: loading 5s ease-in-out forwards;
          }
          
          @keyframes loading {
            0% { width: 0%; }
            100% { width: 100%; }
          }
          
          .feature-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin: 30px 0;
            max-width: 800px;
            animation: fadeInUp 1s ease-out 2.5s both;
          }
          
          .feature-item {
            background: rgba(255,255,255,0.1);
            padding: 20px;
            border-radius: 10px;
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.2);
            transition: transform 0.3s ease;
          }
          
          .feature-item:hover {
            transform: translateY(-5px);
          }
          
          .feature-icon {
            font-size: 2em;
            margin-bottom: 10px;
          }
          
          .feature-text {
            font-size: clamp(12px, 1.2vw, 16px);
            opacity: 0.9;
          }
          
         .footer-text {
      position: fixed;
      bottom: 20px;
      left: 0;
      right: 0;
      text-align: center;
      font-size: clamp(16px, 2vw, 24px);
      color: rgba(255,255,255,0.8);
      padding: 0 20px;
      animation: fadeInUp 1s ease-out 3s both;
      z-index: 10;
    }

    /* Mobile adjustments */
    @media (max-width: 768px) {
      .footer-text {
        bottom: 15px;
        font-size: 14px;
      }
    }

    @media (max-width: 480px) {
      .footer-text {
        bottom: 10px;
        font-size: 12px;
        padding: 0 15px;
      }
    }

    /* Very small screens */
    @media (max-width: 320px) {
      .footer-text {
        bottom: 5px;
        font-size: 11px;
      }

          /* Mobile responsiveness */
          @media (max-width: 768px) {
            .feature-grid {
              grid-template-columns: 1fr;
              gap: 15px;
              margin: 20px 0;
            }
            
            .feature-item {
              padding: 15px;
            }
            
            .logo-slide {
              width: 85%;
              margin-bottom: 20px;
            }
          }
        `}
      </style>

      {/* Main Logo */}
      <img src={TaikishaLogo} alt="Taikisha Logo" className="logo-slide" />

      {/* Main Title */}
      <h1 className="main-title">
        Welcome to Taikisha India Fleet management Dashboard
      </h1>

      {/* Subtitle */}
      <p className="subtitle">
        Customer's First
      </p>

      {/* Tagline */}
      {/* <p className="tagline">
        "Innovating for a Sustainable Future"
      </p> */}

      {/* Features Grid */}
      {/* <div className="feature-grid">
        <div className="feature-item">
          <div className="feature-icon">🏭</div>
          <div className="feature-text">Industrial Solutions</div>
        </div>
        <div className="feature-item">
          <div className="feature-icon">🔧</div>
          <div className="feature-text">Engineering Expertise</div>
        </div>
        <div className="feature-item">
          <div className="feature-icon">🌱</div>
          <div className="feature-text">Sustainable Practices</div>
        </div>
        <div className="feature-item">
          <div className="feature-icon">🚀</div>
          <div className="feature-text">Innovation Driven</div>
        </div>
      </div> */}

      {/* Loading Indicator */}
      <div className="loading-text">
        Preparing your experience...
      </div>
      
      <div className="loading-bar">
        <div className="loading-progress"></div>
      </div>

      {/* Footer Text */}
      <div className="footer-text">
        Design and Developed by Taikisha India 
      </div>

      {/* Background Decorative Elements */}
      <div style={{
        position: "absolute",
        top: "10%",
        right: "10%",
        width: "100px",
        height: "100px",
        border: "2px solid rgba(255,255,255,0.1)",
        borderRadius: "50%",
        animation: "pulse 4s ease-in-out infinite"
      }}></div>
      
      <div style={{
        position: "absolute",
        bottom: "20%",
        left: "5%",
        width: "50px",
        height: "50px",
        border: "2px solid rgba(255,255,255,0.1)",
        borderRadius: "50%",
        animation: "pulse 3s ease-in-out infinite 1s"
      }}></div>
    </div>
  );
}