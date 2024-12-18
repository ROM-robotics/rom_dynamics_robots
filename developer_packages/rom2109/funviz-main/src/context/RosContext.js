// src/RosContext.js
import React, { createContext, useEffect, useState } from "react";
import ROSLIB from "roslib";

export const RosContext = createContext();

export const RosProvider = ({ children }) => {
  const [ros, setRos] = useState(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({
      url: "ws://localhost:9090", // WebSocket server address
    });

    rosInstance.on("connection", () => {
      console.log("Connected to ROS WebSocket server.");
    });

    rosInstance.on("error", (error) => {
      console.error("Error connecting to ROS:", error);
    });

    rosInstance.on("close", () => {
      console.log("Connection to ROS closed.");
    });

    setRos(rosInstance);
    
    return () => {
      rosInstance.close();
    };
  }, []);

  return <RosContext.Provider value={ros}>{children}</RosContext.Provider>;
};
