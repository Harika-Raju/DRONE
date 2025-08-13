import React, { useState, useCallback, useRef, forwardRef, useImperativeHandle } from 'react';
import { GoogleMap, Polyline, useLoadScript } from '@react-google-maps/api';
import axios from 'axios';
import './MapComponent.css';

const libraries = ['drawing', 'places'];

const MapComponent = forwardRef(({ onDispatchDrone }, ref) => {
  const [selectedArea, setSelectedArea] = useState(null);
  const [drawingManager, setDrawingManager] = useState(null);
  const [gridLines, setGridLines] = useState([]);
  const [selectedLocation, setSelectedLocation] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');

  const center = {
    lat: 17.39716,
    lng: 78.49040,
  };

  const { isLoaded, loadError } = useLoadScript({
    googleMapsApiKey: 'AIzaSyB-6NuWru71NBpaRmAaiEZjRmTJnUfQBbQ',
    libraries,
  });

  const searchBoxRef = useRef(null);
  const mapRef = useRef(null);
  const autocompleteRef = useRef(null);

  useImperativeHandle(ref, () => ({
    handleDispatchDrone: () => {
      if (selectedArea) {
        dispatchDroneToArea(selectedArea);
      } else {
        onDispatchDrone('Please select an area on the map first');
      }
    }
  }));

  const mapContainerStyle = {
    width: '100%',
    height: '100%',
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
  };

  const mapOptions = {
    zoom: 18,
    center: center,
    mapTypeId: 'satellite',
    mapTypeControl: false,
    streetViewControl: false,
    fullscreenControl: true,
    zoomControl: true,
  };

  const onLoad = useCallback((map) => {
    if (typeof window !== 'undefined' && window.google && window.google.maps) {
      const drawingManagerInstance = new window.google.maps.drawing.DrawingManager({
        drawingMode: null,
        drawingControl: false, // Disable default drawing controls
      });
  
      drawingManagerInstance.setMap(map);
      setDrawingManager(drawingManagerInstance);
  
      // Create custom buttons for "Draw Rectangle" and "Stop Drawing"
      const drawRectangleButton = document.createElement('button');
      drawRectangleButton.innerHTML = `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="black">
        <rect x="4" y="4" width="16" height="16" />
      </svg>`; // SVG for black rectangle
      drawRectangleButton.className = 'custom-drawing-button'; // Use custom styling
      
      const stopDrawingButton = document.createElement('button');
      stopDrawingButton.innerHTML = '&#9995;'; // Unicode for raised hand symbol (Palm)
      stopDrawingButton.className = 'custom-drawing-button'; // Use custom styling
  
      // Add event listeners to the buttons
      drawRectangleButton.addEventListener('click', () => {
        drawingManagerInstance.setDrawingMode(window.google.maps.drawing.OverlayType.RECTANGLE);
      });
  
      stopDrawingButton.addEventListener('click', () => {
        drawingManagerInstance.setDrawingMode(null);
      });
  
      // Create a container for the buttons
      const buttonContainer = document.createElement('div');
      buttonContainer.className = 'custom-drawing-controls'; // Use custom styling
      buttonContainer.appendChild(drawRectangleButton);
      buttonContainer.appendChild(stopDrawingButton);
  
      // Position the container directly below the fullscreen button
      map.controls[window.google.maps.ControlPosition.RIGHT_TOP].push(buttonContainer);
  
      // Add listener for rectangle completion
      window.google.maps.event.addListener(drawingManagerInstance, 'rectanglecomplete', (rectangle) => {
        const bounds = rectangle.getBounds();
        const selectedBounds = bounds.toJSON();
        setSelectedArea(selectedBounds);
  
        // Call function to draw grid
        drawGrid(selectedBounds);
      });
    }
  }, []);
  
  
  
  

  const onUnmount = useCallback(() => {
    if (drawingManager) {
      drawingManager.setMap(null);
    }
  }, [drawingManager]);

  const drawGrid = (area) => {
    const gridLinesArray = [];
    const latStep = (area.north - area.south) / 10;  // Adjust the grid density here (still keeping it for reference, though unused for vertical only)
    const lngStep = (area.east - area.west) / 50;  // Increase the number to reduce the spacing between vertical grid lines
  
    // Only Vertical grid lines
    for (let i = 0; i <= 50; i++) {  // Increase the iteration for more lines (adjust the number for spacing)
      const lng = area.west + lngStep * i;
      const latStart = area.south;
      const latEnd = area.north;
  
      // Draw vertical grid lines only
      gridLinesArray.push([
        { lat: latStart, lng },
        { lat: latEnd, lng }
      ]);
    }
  
    setGridLines(gridLinesArray);
  };
  

  const handleSearch = useCallback(() => {
    if (typeof window !== 'undefined' && window.google && window.google.maps) {
      if (searchBoxRef.current && !autocompleteRef.current) {
        autocompleteRef.current = new window.google.maps.places.Autocomplete(
          searchBoxRef.current,
          { types: ['geocode'] }
        );

        autocompleteRef.current.addListener('place_changed', () => {
          const place = autocompleteRef.current.getPlace();
          if (place.geometry) {
            const { lat, lng } = place.geometry.location;
            setSelectedLocation({ lat: lat(), lng: lng() });
            mapRef.current.panTo(place.geometry.location);
            setSearchTerm('');
          } else {
            alert("No details available for the selected place.");
          }
        });
      }
    }
  }, []);

  const dispatchDroneToArea = async (area) => {
    try {
      const requestData = {
        top_left: {
          latitude: parseFloat(area.north).toFixed(6),
          longitude: parseFloat(area.west).toFixed(6),
        },
        bottom_right: {
          latitude: parseFloat(area.south).toFixed(6),
          longitude: parseFloat(area.east).toFixed(6),
        },
      };

      const response = await axios.post('http://172.168.35.117:3001/drone/dispatch/rectangle', requestData);
      console.log('Drone dispatched:', response.data);
      onDispatchDrone('Drone successfully dispatched to the selected area');
    } catch (error) {
      console.error('Error dispatching drone:', error);
      onDispatchDrone('Failed to dispatch drone: ' + error.message);
    }
  };

  if (loadError) {
    return <div className="map-error">Error loading maps</div>;
  }

  if (!isLoaded) {
    return <div className="map-loading">Loading maps...</div>;
  }

  return (
    <div className="map-wrapper">
      <input
        ref={searchBoxRef}
        type="text"
        placeholder="Search a place"
        className="search-box"
        value={searchTerm}
        onChange={(e) => setSearchTerm(e.target.value)}
      />

      <GoogleMap
        ref={mapRef}
        mapContainerStyle={mapContainerStyle}
        options={mapOptions}
        onLoad={onLoad}
        onUnmount={onUnmount}
        center={selectedLocation || center}
      >
        {gridLines.map((line, index) => (
          <Polyline
            key={index}
            path={line}
            options={{
              strokeColor: '#00FF00', // Grid line color
              strokeOpacity: 0.6,
              strokeWeight: 2,
            }}
          />
        ))}
      </GoogleMap>
    </div>
  );
});

export default MapComponent;