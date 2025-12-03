import React from "react";
import { Navigate } from "react-router-dom";

export default function ProtectedRoute({ children }) {
  const loggedInUser = localStorage.getItem("loggedInUser");
  if (!loggedInUser) {
    // User not logged in, redirect to login
    return <Navigate to="/login" replace />;
  }
  return children;
}
