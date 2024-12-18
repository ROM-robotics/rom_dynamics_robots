import React, { createContext, useEffect, useState, useContext } from "react";
import ROSLIB from "roslib";
import { RosContext } from "./RosContext";

export const LidarContext = createContext();

export const LidarProvider = ({ children }) => {
  const ros = useContext(RosContext);
  const [lidarData, setLidarData] = useState([]);

  useEffect(() => {
    if (!ros) return;

    const lidarTopic = new ROSLIB.Topic({
      ros,
      name: "/scan", // Replace with your LIDAR topic name
      messageType: "sensor_msgs/msg/LaserScan",
    });

    const handleLidarMessage = (message) => {
      const { angle_min, angle_increment, ranges } = message;
      const points = [];

      // Convert polar coordinates to Cartesian
      ranges.forEach((range, index) => {
        if (range > 0) {
          const angle = angle_min + index * angle_increment;
          const x = range * Math.cos(angle);
          const y = range * Math.sin(angle);
          points.push({ x, y, z: 0 }); // Z-coordinate is 0 for 2D LIDAR
        }
      });

      setLidarData(points);
    };

    lidarTopic.subscribe(handleLidarMessage);

    return () => {
      lidarTopic.unsubscribe();
    };
  }, [ros]);

  return (
    <LidarContext.Provider value={{ lidarData }}>
      {children}
    </LidarContext.Provider>
  );
};
