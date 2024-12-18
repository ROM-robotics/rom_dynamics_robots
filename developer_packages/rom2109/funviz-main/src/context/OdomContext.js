import React, { createContext, useContext, useState, useEffect } from "react";

import ROSLIB from "roslib";
import { RosContext } from "./RosContext";

export const OdomContext = createContext();


export const OdomProvider = ({children }) => {
    const ros = useContext(RosContext);
  const [odomData, setOdomData] = useState(null);

  useEffect(() => {
    if (ros) {
      const odomListener = new ROSLIB.Topic({
        ros: ros,
        name: "/odom",
        messageType: "nav_msgs/Odometry",
      });

      odomListener.subscribe((message) => {
        const { position, orientation } = message.pose.pose;
        setOdomData({
          position: {
            x: position.x,
            y: position.y,
            z: position.z,
          },
          orientation: {
            x: orientation.x,
            y: orientation.y,
            z: orientation.z,
            w: orientation.w,
          },
        });
      });

      return () => odomListener.unsubscribe();
    }
  }, [ros]);

  return <OdomContext.Provider value={{ odomData }}>{children}</OdomContext.Provider>;
};
