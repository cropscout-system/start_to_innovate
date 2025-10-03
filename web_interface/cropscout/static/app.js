// Global variables
let map, droneMarker, currentRoute = {waypoints: []};
let routePolyline, waypointMarkers = [];
let token = localStorage.getItem('auth_token');
let selectedRouteId = null;
let isDroneInMotion = false;
let mouseDownTimer = null;
let mouseDownPos = null;
let homeLocation = null;
let defaultZoom = 17;

const goida = true;

// Initialize the application
document.addEventListener('DOMContentLoaded', () => {
    // Check if user is logged in
    if (token) {
        document.getElementById('loginContainer').style.display = 'none';
        document.getElementById('appContainer').style.display = 'grid';
        initializeMap();
        loadSavedRoutes();
        setupEventListeners();
    } else {
        setupLoginForm();
    }
});

// Initialize Leaflet map
function initializeMap() {
    // Load saved home location
    const savedHome = localStorage.getItem('map_home');
    if (savedHome) {
        homeLocation = JSON.parse(savedHome);
    }

    // Load saved map state
    // Yandex has 0.17918 lat offset – since this is their private API
    const savedMapState = localStorage.getItem('map_state');
    let initialLat = 55.54691;
    let initialLng = 37.91791;
    let initialZoom = defaultZoom;

    if (savedMapState) {
        const state = JSON.parse(savedMapState);
        initialLat = state.lat;
        initialLng = state.lng;
        initialZoom = state.zoom;
    } else if (homeLocation) {
        initialLat = homeLocation.lat;
        initialLng = homeLocation.lng;
    }

    map = L.map('map', {attributionControl: false}).setView([initialLat, initialLng], initialZoom);

    // L.layerGroup([
    //     L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    //         attribution: '&copy; <a href="https://www.esri.com/en-us/home">Esri</a>',
    //         maxZoom: 19
    //     }),
    //     L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}', {
    //         attribution: '&copy; <a href="https://www.esri.com/en-us/home">Esri</a>',
    //         maxZoom: 19
    //     }),
    //     L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}', {
    //         attribution: '&copy; <a href="https://www.esri.com/en-us/home">Esri</a>',
    //         maxZoom: 19
    //     })
    // ]).addTo(map);

    L.layerGroup([
        L.tileLayer('https://sat{s}.maps.yandex.net/tiles?l=sat&v=3.1942.0&x={x}&y={y}&z={z}&lang=ru_RU', {
            maxZoom: 19,
            subdomains: ['01', '02', '03', '04']
        }),
        L.tileLayer('https://core-renderer-tiles.maps.yandex.ru/tiles?l=skl&v=25.10.02-1~b:250924101600~ib:25.10.03-0&x={x}&y={y}&z={z}&lang=ru_RU', {
            maxZoom: 19,
        })
    ], {
        attribution: 'Yandex'
    }).addTo(map);

    L.control.attribution({
        prefix: 'Leaflet'
    }).addTo(map);

    // Add click listener for adding waypoints
    map.on('contextmenu', function (e) {
        function isValidFloat(str) {
            return !isNaN(str) && parseFloat(str).toString() === str.trim();
        }

        let alt = NaN;

        if (goida) {
            while (!isValidFloat(alt)) {
                alt = prompt('Enter desired waypoint photo altitude AGL (above ground level) in m:');
            }
            e.latlng.alt = alt;
        }
        addWaypoint(e.latlng);
    });

    // Add mousedown handler for setting home
    map.on('mousedown', function (e) {
        mouseDownPos = e.latlng;
        mouseDownTimer = setTimeout(function () {
            if (mouseDownPos) {
                const lat = mouseDownPos.lat.toFixed(6);
                const lng = mouseDownPos.lng.toFixed(6);
                if (confirm(`Would you like to set home at ${lat}, ${lng}?`)) {
                    homeLocation = {lat: mouseDownPos.lat, lng: mouseDownPos.lng};
                    localStorage.setItem('map_home', JSON.stringify(homeLocation));
                    map.setView([homeLocation.lat, homeLocation.lng], defaultZoom);
                }
            }
        }, 1000);
    });

    // Add mouseup handler to cancel hold timer
    map.on('mouseup', function () {
        if (mouseDownTimer) {
            clearTimeout(mouseDownTimer);
            mouseDownTimer = null;
        }
        mouseDownPos = null;
    });

    // Save map state on move/zoom
    map.on('moveend zoomend', function () {
        const center = map.getCenter();
        const zoom = map.getZoom();
        localStorage.setItem('map_state', JSON.stringify({
            lat: center.lat,
            lng: center.lng,
            zoom: zoom
        }));
    });
}

// Setup all event listeners for UI interactions
function setupEventListeners() {
    // User menu dropdown toggle
    document.getElementById('userMenuBtn').addEventListener('click', () => {
        document.getElementById('userDropdown').classList.toggle('show');
    });

    // Close dropdown when clicking outside
    window.addEventListener('click', (e) => {
        if (!e.target.matches('.user-menu-button') && !e.target.matches('#userMenuBtn i') && !e.target.matches('#userMenuBtn span')) {
            const dropdown = document.getElementById('userDropdown');
            if (dropdown.classList.contains('show')) {
                dropdown.classList.remove('show');
            }
        }
    });

    // Save route button
    document.getElementById('saveRouteBtn').addEventListener('click', saveCurrentRoute);

    // Clear route button
    document.getElementById('clearRouteBtn').addEventListener('click', clearCurrentRoute);

    // Start mission button
    document.getElementById('startMissionBtn').addEventListener('click', startMission);

    // Change password button
    document.getElementById('changePasswordBtn').addEventListener('click', () => {
        document.getElementById('changePasswordModal').classList.add('show');
    });

    // Cancel change password button
    document.getElementById('cancelChangePassword').addEventListener('click', () => {
        document.getElementById('changePasswordModal').classList.remove('show');
        document.getElementById('changePasswordForm').reset();
    });

    // Change password form submission
    document.getElementById('changePasswordForm').addEventListener('submit', changePassword);

    // Logout button
    document.getElementById('logoutBtn').addEventListener('click', logout);

    // Home button
    document.getElementById('homeBtn').addEventListener('click', () => {
        if (homeLocation) {
            map.setView([homeLocation.lat, homeLocation.lng], defaultZoom);
        } else {
            map.setView([55.54691, 37.91791], defaultZoom);
        }
    });

    // Follow drone button
    document.getElementById('followDroneBtn').addEventListener('click', () => {
        if (droneMarker && isDroneInMotion) {
            const pos = droneMarker.getLatLng();
            map.setView([pos.lat, pos.lng], defaultZoom);
        }
    });

    // Display username
    const payload = parseJwt(token);
    if (payload && payload.sub) {
        document.getElementById('usernameDisplay').textContent = payload.sub;
    }
}

// Setup login form functionality
function setupLoginForm() {
    document.getElementById('loginForm').addEventListener('submit', async (e) => {
        e.preventDefault();

        const username = document.getElementById('username').value;
        const password = document.getElementById('password').value;

        try {
            const response = await fetch('/api/auth/login', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({username, password})
            });

            if (response.ok) {
                const data = await response.json();
                token = data.access_token;
                localStorage.setItem('auth_token', token);

                document.getElementById('loginContainer').style.display = 'none';
                document.getElementById('appContainer').style.display = 'grid';

                initializeMap();
                loadSavedRoutes();
                setupEventListeners();
            } else {
                alert('Login failed. Please check your credentials.');
            }
        } catch (error) {
            console.error('Login error:', error);
            alert('Login failed due to a server error.');
        }
    });
}

// Load all saved routes from the server
async function loadSavedRoutes() {
    try {
        const response = await fetch('/api/routes', {
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });

        if (response.ok) {
            const routes = await response.json();
            const routeList = document.getElementById('routeList');
            routeList.innerHTML = '';

            if (routes.length === 0) {
                routeList.innerHTML = '<li>No saved routes</li>';
                return;
            }

            routes.forEach(route => {
                const li = document.createElement('li');
                li.className = 'route-item';
                li.dataset.id = route.id;
                li.innerHTML = `
                    <div class="route-content">
                        <div>${route.name}</div>
                        <div>${route.waypoints.length} waypoints</div>
                    </div>
                    <div class="route-delete" data-route-id="${route.id}" title="Delete route">
                        <i class="fas fa-times"></i>
                    </div>
                `;

                // Add click listener for route selection (only on route content, not delete button)
                li.querySelector('.route-content').addEventListener('click', () => {
                    // Deselect previous
                    document.querySelectorAll('.route-item').forEach(item => {
                        item.classList.remove('active');
                    });

                    // Select this route
                    li.classList.add('active');
                    loadRoute(route.id);
                });

                // Add click listener for delete button
                li.querySelector('.route-delete').addEventListener('click', (e) => {
                    e.stopPropagation(); // Prevent route selection
                    deleteRoute(route.id, route.name);
                });

                routeList.appendChild(li);
            });
        } else {
            console.error('Failed to load routes');
        }
    } catch (error) {
        console.error('Error loading routes:', error);
    }
}

// Delete route function
async function deleteRoute(routeId, routeName) {
    if (!confirm(`Are you sure you want to delete the route "${routeName}"?`)) {
        return;
    }

    try {
        const response = await fetch(`/api/routes/${routeId}`, {
            method: 'DELETE',
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });

        if (response.ok) {
            // If the deleted route was currently selected, clear it
            if (selectedRouteId === routeId) {
                clearCurrentRoute();
            }

            // Reload the routes list
            loadSavedRoutes();

            alert('Route deleted successfully');
        } else {
            const error = await response.json();
            alert(error.detail || 'Failed to delete route');
        }
    } catch (error) {
        console.error('Error deleting route:', error);
        alert('Error deleting route');
    }
}

// Load a specific route by ID
async function loadRoute(routeId) {
    try {
        const response = await fetch(`/api/routes/${routeId}`, {
            headers: {
                'Authorization': `Bearer ${token}`
            }
        });

        if (response.ok) {
            const route = await response.json();
            clearCurrentRoute();

            // Set as current route
            currentRoute = route;
            selectedRouteId = route.id;

            // Add waypoints to map
            route.waypoints.forEach(wp => {
                const latlng = {lat: wp.lat, lng: wp.lng};
                addWaypointToMap(latlng, wp.id);
            });

            updateRoutePolyline();

            // Show mission start button
            document.getElementById('startMissionBtn').style.display = 'flex';

            // Show current route details
            document.getElementById('currentRouteInfo').style.display = 'block';
            document.getElementById('currentRouteDetails').innerHTML = `
                <p>Route name: ${route.name}</p>
                <p>Waypoints: ${route.waypoints.length}</p>
            `;
            document.getElementById('saveRouteContainer').style.display = 'none';
        } else {
            console.error('Failed to load route');
        }
    } catch (error) {
        console.error('Error loading route:', error);
    }
}

// Add a new waypoint from map click
function addWaypoint(latlng) {
    // If a saved route is selected, deselect it
    if (selectedRouteId) {
        document.querySelectorAll('.route-item').forEach(item => {
            item.classList.remove('active');
        });
        selectedRouteId = null;
    }

    // Generate waypoint ID
    const waypointId = currentRoute.waypoints.length;

    // Add waypoint to the current route
    currentRoute.waypoints.push({
        id: waypointId,
        lat: latlng.lat,
        lng: latlng.lng,
        alt: latlng.alt
    });

    // Add the marker to the map
    addWaypointToMap(latlng, waypointId);

    // Update polyline
    updateRoutePolyline();

    // Show save container and start mission button
    document.getElementById('currentRouteInfo').style.display = 'block';
    document.getElementById('saveRouteContainer').style.display = 'block';
    document.getElementById('startMissionBtn').style.display = 'flex';
}

// Add a waypoint marker to the map
function addWaypointToMap(latlng, id) {
    // Create custom icon with waypoint number
    const icon = L.divIcon({
        className: 'waypoint-icon',
        html: `${id + 1}`,
        iconSize: [30, 30]
    });

    // Create marker
    const marker = L.marker([latlng.lat, latlng.lng], {icon: icon}).addTo(map);

    // Add right-click event to remove waypoint
    marker.on('contextmenu', function () {
        removeWaypoint(id);
    });

    // Store marker
    waypointMarkers.push({id, marker});
}

// Remove a waypoint by ID
function removeWaypoint(id) {
    // Remove waypoint from currentRoute
    currentRoute.waypoints = currentRoute.waypoints.filter(wp => wp.id !== id);

    // Remove marker from map
    waypointMarkers.forEach(wpm => {
        if (wpm.id === id) {
            map.removeLayer(wpm.marker);
        }
    });

    // Remove from markers array
    waypointMarkers = waypointMarkers.filter(wpm => wpm.id !== id);

    // Renumber waypoints
    currentRoute.waypoints.forEach((wp, index) => {
        wp.id = index;
    });

    // Replace all markers with updated numbering
    refreshWaypointMarkers();

    // Update polyline
    updateRoutePolyline();

    // Hide mission start button if no waypoints
    if (currentRoute.waypoints.length === 0) {
        document.getElementById('startMissionBtn').style.display = 'none';
        document.getElementById('currentRouteInfo').style.display = 'none';
    }
}

// Refresh all waypoint markers (for renumbering)
function refreshWaypointMarkers() {
    // Remove all existing markers
    waypointMarkers.forEach(wpm => {
        map.removeLayer(wpm.marker);
    });

    waypointMarkers = [];

    // Add all waypoints back with correct numbering
    currentRoute.waypoints.forEach(wp => {
        const latlng = {lat: wp.lat, lng: wp.lng};
        addWaypointToMap(latlng, wp.id);
    });
}

// Update the polyline connecting waypoints
function updateRoutePolyline() {
    // Remove existing polyline
    if (routePolyline) {
        map.removeLayer(routePolyline);
    }

    // Create coordinates array for polyline
    const coordinates = currentRoute.waypoints.map(wp => [wp.lat, wp.lng]);

    // Add closing segment back to first waypoint if we have points
    if (coordinates.length > 0) {
        coordinates.push(coordinates[0]);
    }

    // Create new polyline
    if (coordinates.length > 1) {
        routePolyline = L.polyline(coordinates, {
            color: '#00ffff',
            weight: 3,
            opacity: 0.7,
            dashArray: '5, 10'
        }).addTo(map);
    }
}

// Save the current route to the server
async function saveCurrentRoute(e) {
    e.preventDefault();

    const routeName = document.getElementById('routeName').value.trim();
    if (!routeName) {
        alert('Please enter a name for the route');
        return;
    }

    if (currentRoute.waypoints.length === 0) {
        alert('Cannot save an empty route');
        return;
    }

    const routeData = {
        id: selectedRouteId || '',
        name: routeName,
        waypoints: currentRoute.waypoints
    };

    try {
        const method = selectedRouteId ? 'PUT' : 'POST';
        const url = selectedRouteId ? `/api/routes/${selectedRouteId}` : '/api/routes';

        const response = await fetch(url, {
            method: method,
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify(routeData)
        });

        if (response.ok) {
            const savedRoute = await response.json();
            selectedRouteId = savedRoute.id;

            // Update the route display
            document.getElementById('currentRouteDetails').innerHTML = `
                <p>Route name: ${savedRoute.name}</p>
                <p>Waypoints: ${savedRoute.waypoints.length}</p>
            `;

            // Reload saved routes
            loadSavedRoutes();

            // Clear route name input
            document.getElementById('routeName').value = '';

            alert('Route saved successfully');
        } else {
            console.error('Failed to save route');
            alert('Failed to save route');
        }
    } catch (error) {
        console.error('Error saving route:', error);
        alert('Error saving route');
    }
}

// Clear the current route
function clearCurrentRoute() {
    // Clear waypoints
    currentRoute = {waypoints: []};
    selectedRouteId = null;

    // Remove existing markers
    waypointMarkers.forEach(wpm => {
        map.removeLayer(wpm.marker);
    });
    waypointMarkers = [];

    // Remove polyline
    if (routePolyline) {
        map.removeLayer(routePolyline);
        routePolyline = null;
    }

    // Hide mission start button and current route info
    document.getElementById('startMissionBtn').style.display = 'none';
    document.getElementById('currentRouteInfo').style.display = 'none';
}

// Start the drone mission
async function startMission() {
    // Prevent starting if already in motion
    if (isDroneInMotion) {
        return;
    }

    if (currentRoute.waypoints.length === 0) {
        alert('No waypoints defined for mission');
        return;
    }

    // If this is a saved route, tell the server to start the mission
    if (selectedRouteId) {
        try {
            const response = await fetch(`/api/routes/${selectedRouteId}/start`, {
                method: 'POST',
                headers: {
                    'Authorization': `Bearer ${token}`
                }
            });

            if (!response.ok) {
                alert('Failed to start mission on the server');
                return;
            }
        } catch (error) {
            console.error('Error starting mission:', error);
            alert('Error starting mission');
            return;
        }
    }

    // Simulate drone movement on the map
    simulateDroneMission();
}

// Simulate drone movement along the route
function simulateDroneMission() {
    isDroneInMotion = true;
    document.getElementById('startMissionBtn').style.display = 'none';

    // Create drone icon
    const droneIcon = L.divIcon({
        className: 'drone-icon',
        html: '<i class="fas fa-drone-alt"></i>',
        iconSize: [30, 30]
    });

    // Create drone marker at first waypoint
    const firstWaypoint = currentRoute.waypoints[0];
    droneMarker = L.marker([firstWaypoint.lat, firstWaypoint.lng], {
        icon: droneIcon,
        zIndexOffset: 1000 // Place above waypoint markers
    }).addTo(map);

    // Create array of waypoints (plus return to start)
    const waypoints = [...currentRoute.waypoints];
    if (waypoints.length > 0) {
        waypoints.push(waypoints[0]); // Return to start
    }

    let currentWaypointIndex = 0;

    // Update drone status information
    updateDroneStatus(true);

    function moveToNextWaypoint() {
        if (currentWaypointIndex >= waypoints.length - 1) {
            // Mission complete
            setTimeout(() => {
                map.removeLayer(droneMarker);
                droneMarker = null;
                isDroneInMotion = false;
                document.getElementById('startMissionBtn').style.display = 'flex';
                updateDroneStatus(false);
            }, 2000);
            return;
        }

        const currentWP = waypoints[currentWaypointIndex];
        const nextWP = waypoints[currentWaypointIndex + 1];

        // Animate drone movement
        animateDroneMovement(
            [currentWP.lat, currentWP.lng],
            [nextWP.lat, nextWP.lng],
            5000, // 5 seconds per segment
            () => {
                // Wait at waypoint for 2 seconds
                setTimeout(() => {
                    currentWaypointIndex++;
                    moveToNextWaypoint();
                }, 2000);
            }
        );
    }

    // Start movement
    moveToNextWaypoint();
}

// Animate drone movement between two points
function animateDroneMovement(startLatLng, endLatLng, duration, callback) {
    const startTime = Date.now();
    const startLat = startLatLng[0];
    const startLng = startLatLng[1];
    const latDiff = endLatLng[0] - startLat;
    const lngDiff = endLatLng[1] - startLng;

    // Calculate distance for speed display
    const distance = calculateDistance(startLatLng, endLatLng);
    const speed = distance / (duration / 1000); // meters per second

    function animate() {
        const elapsed = Date.now() - startTime;
        const progress = Math.min(elapsed / duration, 1);

        const currentLat = startLat + (latDiff * progress);
        const currentLng = startLng + (lngDiff * progress);

        // Update drone position
        droneMarker.setLatLng([currentLat, currentLng]);

        // Update speed display
        if (progress < 1) {
            document.getElementById('speed').textContent = `${speed.toFixed(1)} m/s`;
        } else {
            document.getElementById('speed').textContent = '0 m/s';
        }

        if (progress < 1) {
            requestAnimationFrame(animate);
        } else if (callback) {
            callback();
        }
    }

    animate();
}

// Calculate distance between two points in meters
function calculateDistance(latlng1, latlng2) {
    const lat1 = latlng1[0];
    const lon1 = latlng1[1];
    const lat2 = latlng2[0];
    const lon2 = latlng2[1];

    const R = 6371e3; // Earth radius in meters
    const φ1 = lat1 * Math.PI / 180;
    const φ2 = lat2 * Math.PI / 180;
    const Δφ = (lat2 - lat1) * Math.PI / 180;
    const Δλ = (lon2 - lon1) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
        Math.cos(φ1) * Math.cos(φ2) *
        Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    return R * c; // Distance in meters
}

// Update drone status display
function updateDroneStatus(isActive) {
    if (isActive) {
        // Random battery level between 75% and 95%
        const batteryLevel = Math.floor(75 + Math.random() * 20);
        document.getElementById('battery-level').textContent = `${batteryLevel}%`;

        // Signal strength
        document.getElementById('signal-strength').textContent = 'Strong';

        // Speed starts at 0
        document.getElementById('speed').textContent = '0 m/s';
    } else {
        // Reset to default values
        document.getElementById('battery-level').textContent = '87%';
        document.getElementById('signal-strength').textContent = 'Strong';
        document.getElementById('speed').textContent = '0 m/s';
    }
}

// Change password function
async function changePassword(e) {
    e.preventDefault();

    const currentPassword = document.getElementById('currentPassword').value;
    const newPassword = document.getElementById('newPassword').value;

    try {
        const response = await fetch('/api/auth/change-password', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify({
                current_password: currentPassword,
                new_password: newPassword
            })
        });

        if (response.ok) {
            // Close modal and reset form
            document.getElementById('changePasswordModal').classList.remove('show');
            document.getElementById('changePasswordForm').reset();
            alert('Password updated successfully');
        } else {
            const error = await response.json();
            alert(error.detail || 'Failed to update password');
        }
    } catch (error) {
        console.error('Error updating password:', error);
        alert('Error updating password');
    }
}

// Logout function
function logout() {
    localStorage.removeItem('auth_token');
    window.location.reload();
}

// Parse JWT token
function parseJwt(token) {
    try {
        const base64Url = token.split('.')[1];
        const base64 = base64Url.replace(/-/g, '+').replace(/_/g, '/');
        const jsonPayload = decodeURIComponent(atob(base64).split('').map(function (c) {
            return '%' + ('00' + c.charCodeAt(0).toString(16)).slice(-2);
        }).join(''));

        return JSON.parse(jsonPayload);
    } catch (e) {
        return null;
    }
}