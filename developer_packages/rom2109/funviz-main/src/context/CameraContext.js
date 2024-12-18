import React, { createContext, useState, useRef, useEffect } from "react";

export const CameraContext = createContext();

export const CameraProvider = ({ children }) => {
  const [cameraPosition, setCameraPosition] = useState({ x: 0, y: 0, z: 2 });
  const [cameraRotation, setCameraRotation] = useState({ x: 0, y: 0 });
  const isDragging = useRef(false);
  const lastMousePosition = useRef({ x: 0, y: 0 });
  const dragMode = useRef(null); // Tracks left/right drag modes

  const startDrag = (event) => {
    if (event.button === 0) {
      // Left-click for camera position dragging
      dragMode.current = "position";
    } else if (event.button === 2) {
      // Right-click for camera rotation
      dragMode.current = "rotation";
    }
    isDragging.current = true;
    lastMousePosition.current = { x: event.clientX, y: event.clientY };
    event.preventDefault(); // Prevent context menu on right-click
  };

  const updateDrag = (event) => {
    if (!isDragging.current) return;

    const deltaX = event.clientX - lastMousePosition.current.x;
    const deltaY = event.clientY - lastMousePosition.current.y;

    lastMousePosition.current = { x: event.clientX, y: event.clientY };

    if (dragMode.current === "position") {
      // Update camera position (move)
      setCameraPosition((prev) => ({
        x: prev.x - deltaX * 0.01, // Reverse X for more intuitive dragging
        y: prev.y + deltaY * 0.01,
        z: prev.z,
      }));
    } else if (dragMode.current === "rotation") {
      // Update camera rotation (rotate)
      setCameraRotation((prev) => ({
        x: prev.x + deltaX * 0.01 //rotate x
      }));
    }
  };

  const endDrag = () => {
    isDragging.current = false;
    dragMode.current = null;
  };

  // Add event listeners to the window for drag events
  useEffect(() => {
    const handleMouseMove = (event) => updateDrag(event);
    const handleMouseUp = () => endDrag();
    const handleMouseDown = (event) => startDrag(event);

    window.addEventListener("mousemove", handleMouseMove);
    window.addEventListener("mouseup", handleMouseUp);
    window.addEventListener("mousedown", handleMouseDown);

    // Prevent the default context menu from appearing on right-click
    window.addEventListener("contextmenu", (e) => e.preventDefault());

    return () => {
      window.removeEventListener("mousemove", handleMouseMove);
      window.removeEventListener("mouseup", handleMouseUp);
      window.removeEventListener("mousedown", handleMouseDown);
      window.removeEventListener("contextmenu", (e) => e.preventDefault());
    };
  }, []);

  return (
    <CameraContext.Provider
      value={{
        cameraPosition,
        setCameraPosition,
        cameraRotation,
        setCameraRotation,
      }}
    >
      {children}
    </CameraContext.Provider>
  );
};
