import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Dashboard from './Dashboard';
import MissionControl from './components/MissionControl';
import ObjectRecognition from './components/ObjectRecognition';
import Alerts from './components/Alerts';
import PayloadManagement from './components/PayloadManagement';
import TrainingSimulation from './components/TrainingSimulation';
import Settings from './components/Settings';
import Login from './components/Login';
import { AuthProvider, useAuth } from './AuthContext';
import Register from "./components/Register";
import { AuraProvider } from './components/AURA/AuraContext'; // Add this import

// Protected Route Component
const ProtectedRoute = ({ element }) => {
  const { currentUser } = useAuth();
  return currentUser ? element : <Login />;
};

function App() {
  return (
    <AuthProvider>
      <AuraProvider> {/* Add this wrapper */}
        <Router>
          <Routes>
            <Route path="/login" element={<Login />} />
            <Route path="/register" element={<Register/>} />
            
            {/* Protected routes */}
            <Route path="/" element={<ProtectedRoute element={<Dashboard />} />} />
            <Route path="/objectrecognition" element={<ProtectedRoute element={<ObjectRecognition />} />} />
            <Route path="/alerts" element={<ProtectedRoute element={<Alerts />} />} />
            <Route path="/payloadmanagement" element={<ProtectedRoute element={<PayloadManagement />} />} />
            <Route path="/trainingsimulation" element={<ProtectedRoute element={<TrainingSimulation />} />} />
            <Route path="/settings" element={<ProtectedRoute element={<Settings />} />} />
            <Route path="/missioncontrol" element={<ProtectedRoute element={<MissionControl />} />} />
          </Routes>
        </Router>
      </AuraProvider>
    </AuthProvider>
  );
}

export default App;