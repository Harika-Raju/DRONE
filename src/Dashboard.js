import React, { useState, useRef } from 'react';
import { NavLink } from 'react-router-dom';
import './Dashboard.css';
import { useAuth } from './AuthContext';
import MapComponent from './components/MapComponent';
import ChatInterface from './components/AURA/ChatInterface';
import { useAura } from './components/AURA/AuraContext';

function Dashboard() {
  const [missionStatus, setMissionStatus] = useState('');
  const { logout } = useAuth();
  const [isSidebarOpen, setIsSidebarOpen] = useState(false);
  const { droneStatus, missionData } = useAura();
  const mapComponentRef = useRef(null);

  const handleLogout = async () => {
    try {
      await logout();
    } catch (error) {
      alert('Logout failed: ' + error.message);
    }
  };

  const toggleSidebar = () => {
    setIsSidebarOpen(prevState => !prevState);
  };

  const handleDispatchDrone = () => {
    if (mapComponentRef.current) {
      mapComponentRef.current.handleDispatchDrone();
    } else {
      setMissionStatus('Please select an area on the map first');
    }
  };

  return (
    <div className="dashboard-container">
      <aside className={`sidebar ${isSidebarOpen ? 'open' : ''}`}>
        <button className="close-btn" onClick={toggleSidebar}>✖</button>
        <h2>AI Drone Rescue Command Center</h2>
        <ul>
          <li>
            <NavLink to="/missioncontrol" className={({ isActive }) => (isActive ? 'active' : '')}>
              Mission Control
            </NavLink>
          </li>
          <li>
            <NavLink to="/history" className={({ isActive }) => (isActive ? 'active' : '')}>
              History
            </NavLink>
          </li>
          <li>
            <NavLink to="/alerts" className={({ isActive }) => (isActive ? 'active' : '')}>
              Alerts
            </NavLink>
          </li>
          <li>
            <NavLink to="/payloadmanagement" className={({ isActive }) => (isActive ? 'active' : '')}>
              Payload Management
            </NavLink>
          </li>
          <li>
            <NavLink to="/trainingsimulation" className={({ isActive }) => (isActive ? 'active' : '')}>
              Training & Simulation
            </NavLink>
          </li>
          <li>
            <NavLink to="/settings" className={({ isActive }) => (isActive ? 'active' : '')}>
              Settings
            </NavLink>
          </li>
        </ul>
      </aside>

      <main className={`main-content ${isSidebarOpen ? 'expanded' : ''}`}>
        <header className="header">
          <button className="menu-icon" onClick={toggleSidebar}>
            ☰
          </button>
          <span>Welcome, Rescue Operator</span>
          <button className="logout-button" onClick={handleLogout}>
            Logout
          </button>
        </header>

        <section className="dashboard">
          <div className="mission-map">
            <h3>Mission Map</h3>
            <div className="map-container">
              <MapComponent 
                ref={mapComponentRef}
                onDispatchDrone={setMissionStatus} 
              />
            </div>
            <div className="map-info">
              <span>Area Covered: {missionData.areaCovered}</span>
              <button 
                className="dispatch-button" 
                onClick={handleDispatchDrone}
              >
                Dispatch Drone
              </button>
              <span>Objects Detected: {missionData.objectsDetected}</span>
            </div>
            {missionStatus && (
              <div className="mission-status">
                <p>{missionStatus}</p>
              </div>
            )}
          </div>
        </section>

        <ChatInterface 
          droneStatus={droneStatus}
          missionData={missionData}
        />
      </main>
    </div>
  );
}

export default Dashboard;
