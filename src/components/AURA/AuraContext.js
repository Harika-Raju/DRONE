import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from '../../AuthContext';

const AuraContext = createContext();

export const useAura = () => {
  return useContext(AuraContext);
};

export const AuraProvider = ({ children }) => {
  const { currentUser } = useAuth();
  const [droneStatus, setDroneStatus] = useState({
    battery: 100,
    signalStrength: 95,
    altitude: "50m",
    status: "Ready",
  });

  const [missionData, setMissionData] = useState({
    areaCovered: "78%",
    objectsDetected: 12,
    weatherCondition: "Clear",
    windSpeed: "5km/h",
  });

  // Update drone status periodically (simulate real-time updates)
  useEffect(() => {
    const interval = setInterval(() => {
      setDroneStatus(prev => ({
        ...prev,
        battery: Math.max(0, prev.battery - 0.1),
        signalStrength: 85 + Math.random() * 10,
      }));
    }, 5000);

    return () => clearInterval(interval);
  }, []);

  const value = {
    droneStatus,
    setDroneStatus,
    missionData,
    setMissionData,
  };

  return (
    <AuraContext.Provider value={value}>
      {children}
    </AuraContext.Provider>
  );
};