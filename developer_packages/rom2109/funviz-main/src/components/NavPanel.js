// src/components/NavPanel.js
import React, { useEffect, useState } from "react";
import ROSLIB from "roslib";
import "./NavPanel.css";

const NavPanel = () => {
  const [ros, setRos] = useState(null);

  useEffect(() => {
    // Connect to the ROS WebSocket server
    const rosConnection = new ROSLIB.Ros({
      url: "ws://localhost:9090", // Update with your ROS WebSocket URL
    });

    rosConnection.on("connection", () => {
      console.log("Connected to ROS!");
    });

    rosConnection.on("error", (error) => {
      console.error("Error connecting to ROS:", error);
    });

    rosConnection.on("close", () => {
      console.log("Disconnected from ROS!");
    });

    setRos(rosConnection);

    // Cleanup on component unmount
    return () => rosConnection.close();
  }, []);

  const handleClick = (buttonName) => {
    if (!ros) {
      alert("Not connected to ROS!");
      return;
    }

    // Create a topic object
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: "/change_mode",
      messageType: "std_msgs/String",
    });

    // Publish a message
    const message = new ROSLIB.Message({
      data: buttonName,
    });

    topic.publish(message);

    alert(`${buttonName} command published!`);
  };

  return (
    <nav className="nav-panel">
      <ul>
        <li>
          <button onClick={() => handleClick("Navigation")}>Navigation</button>
        </li>
        <li>
          <button onClick={() => handleClick("Mapping")}>Mapping</button>
        </li>
        <li>
          <button onClick={() => handleClick("Remapping")}>Remapping</button>
        </li>
      </ul>
    </nav>
  );
};

export default NavPanel;
