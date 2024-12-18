import React, { createContext, useEffect, useRef, useCallback } from "react";
import ROSLIB from "roslib";
import * as THREE from "three";

export const TFContext = createContext();

export const TFProvider = ({ ros, sceneRef, children }) => {
  const tfGroupsRef = useRef({});

  useEffect(() => {
    if (!ros) return;

    // Dynamic transform listener
    const tfListener = new ROSLIB.Topic({
      ros,
      name: "/tf",
      messageType: "tf2_msgs/TFMessage",
    });

    tfListener.subscribe((message) => {
      message.transforms.forEach((transform) => {
        const { translation, rotation } = transform.transform;
        const { child_frame_id } = transform;

        // Manage TF groups in the scene
        if (!tfGroupsRef.current[child_frame_id]) {
          const group = new THREE.Group();
          tfGroupsRef.current[child_frame_id] = group;

          // Add AxesHelper for visualizing the orientation
          const axesHelper = new THREE.AxesHelper(0.3); // Length of the axes
          group.add(axesHelper);

          if (sceneRef.current) {
            sceneRef.current.add(group);
          }
        }

        // Update TF group position and rotation
        const group = tfGroupsRef.current[child_frame_id];
        if (group) {
          group.position.set(translation.x, translation.y, translation.z);
          group.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
        }
      });
    });

    // Static transform listener
    const tfStaticListener = new ROSLIB.Topic({
      ros,
      name: "/tf_static",
      messageType: "tf2_msgs/TFMessage",
    });

    tfStaticListener.subscribe((message) => {
      message.transforms.forEach((transform) => {
        const { translation, rotation } = transform.transform;
        const { child_frame_id } = transform;

        // Manage TF groups in the scene
        if (!tfGroupsRef.current[child_frame_id]) {
          const group = new THREE.Group();
          tfGroupsRef.current[child_frame_id] = group;

          // Add AxesHelper for visualizing the orientation
          const axesHelper = new THREE.AxesHelper(0.3); // Length of the axes
          group.add(axesHelper);

          if (sceneRef.current) {
            sceneRef.current.add(group);
          }
        }

        // Update TF group position and rotation
        const group = tfGroupsRef.current[child_frame_id];
        if (group) {
          group.position.set(translation.x, translation.y, translation.z);
          group.quaternion.set(rotation.x, rotation.y, rotation.z, rotation.w);
        }
      });
    });

    return () => {
      tfListener.unsubscribe();
      tfStaticListener.unsubscribe();
    };
  }, [ros]);

  // Function to get specific TF frame data
  const getTFFrameData = useCallback(
    (frameId) => {
      const frame = tfGroupsRef.current[frameId];
      if (frame) {
        return {
          position: frame.position,
          quaternion: frame.quaternion,
        };
      }
      return null;
    },
    [tfGroupsRef]
  );

  return (
    <TFContext.Provider value={{ tfGroupsRef, getTFFrameData }}>
      {children}
    </TFContext.Provider>
  );
};
