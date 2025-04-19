document.addEventListener('DOMContentLoaded', () => {
    // Server connection    
    let socket = null;    
    let isConnected = false;    
    const serverAddress = 'ws://' + window.location.hostname + ':8765';
    
    // UI Elements    
    const statusMessage = document.getElementById('status-message');    
    const statusIndicator = document.getElementById('status-indicator');    
    const timestampDisplay = document.getElementById('timestamp');
    
    // Speed control elements    
    const linearSpeedSlider = document.getElementById('linear-speed-slider');    
    const angularSpeedSlider = document.getElementById('angular-speed-slider');    
    const linearSpeedValue = document.getElementById('linear-speed-value');    
    const angularSpeedValue = document.getElementById('angular-speed-value');
    
    // Direction control buttons    
    const forwardBtn = document.getElementById('btn-forward');    
    const backwardBtn = document.getElementById('btn-backward');   
    const leftBtn = document.getElementById('btn-left');    
    const rightBtn = document.getElementById('btn-right');    
    const stopBtn = document.getElementById('btn-stop');    
    const emergencyStopBtn = document.getElementById('emergency-stop');
    
    // Visualization elements and contexts    
    const mapCanvas = document.getElementById('map-canvas');    
    const lidarCanvas = document.getElementById('lidar-canvas');    
    const odomCanvas = document.getElementById('odom-canvas');
    
    // Initialize canvas contexts    
    const mapCtx = mapCanvas ? mapCanvas.getContext('2d') : null;    
    const lidarCtx = lidarCanvas ? lidarCanvas.getContext('2d') : null;    
    const odomCtx = odomCanvas ? odomCanvas.getContext('2d') : null;
    
    // Raw data display elements    
    const rawDataContent = document.getElementById('raw-data-content');
    
    // Visualization data    
    let odomData = null;    
    let mapData = null;    
    let scanData = null;    
    let robotPath = [];
    const MAX_PATH_LENGTH = 1000;
    
    // Visualization settings    
    let mapViewScale = 50; // pixels per meter    
    let mapOffsetX = mapCanvas ? mapCanvas.width / 2 : 0;    
    let mapOffsetY = mapCanvas ? mapCanvas.height / 2 : 0;        
    
    let lidarViewScale = 50; // pixels per meter    
    let lidarOffsetX = lidarCanvas ? lidarCanvas.width / 2 : 0;    
    let lidarOffsetY = lidarCanvas ? lidarCanvas.height / 2 : 0;        
    
    let odomViewScale = 50; // pixels per meter    
    let odomOffsetX = odomCanvas ? odomCanvas.width / 2 : 0;    
    let odomOffsetY = odomCanvas ? odomCanvas.height / 2 : 0;
    
    // Tab handling    
    const tabElements = document.querySelectorAll('.viz-tab');    
    const contentElements = document.querySelectorAll('.viz-content');
    
    // Setup tab switching    
    tabElements.forEach(tab => {
        tab.addEventListener('click', () => {
            const tabId = tab.getAttribute('data-tab');
            
            tabElements.forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            
            contentElements.forEach(content => {
                if (content.id === tabId) {
                    content.classList.add('active');
                    
                    if (tabId === 'lidar-view' && scanData) {
                        redrawLidarView();
                    } else if (tabId === 'map-view' && mapData) {
                        redrawMapView();
                    } else if (tabId === 'odom-view' && odomData) {
                        redrawOdomView();
                    } else if (tabId === 'raw-data') {
                        updateRawDataView();
                    }
                } else {
                    content.classList.remove('active');
                }
            });
        });
    });
    
    const dataTypeRadios = document.querySelectorAll('input[name="data-type"]');
    dataTypeRadios.forEach(radio => {
        radio.addEventListener('change', updateRawDataView);
    });
    
    // Map visualization controls
    document.getElementById('show-map')?.addEventListener('change', redrawMapView);
    document.getElementById('show-scan-on-map')?.addEventListener('change', redrawMapView);
    document.getElementById('show-path')?.addEventListener('change', redrawMapView);
    document.getElementById('map-zoom-in')?.addEventListener('click', () => {
        mapViewScale *= 1.2;
        redrawMapView();
    });
    document.getElementById('map-zoom-out')?.addEventListener('click', () => {
        mapViewScale /= 1.2;
        redrawMapView();
    });
    document.getElementById('map-reset')?.addEventListener('click', () => {
        mapViewScale = 50;
        mapOffsetX = mapCanvas.width / 2;
        mapOffsetY = mapCanvas.height / 2;
        redrawMapView();
    });
    
    // LIDAR visualization controls
    document.getElementById('show-lidar-rays')?.addEventListener('change', redrawLidarView);
    document.getElementById('lidar-color-mode')?.addEventListener('change', redrawLidarView);
    document.getElementById('lidar-zoom-in')?.addEventListener('click', () => {
        lidarViewScale *= 1.2;
        redrawLidarView();
    });
    document.getElementById('lidar-zoom-out')?.addEventListener('click', () => {
        lidarViewScale /= 1.2;
        redrawLidarView();
    });
    document.getElementById('lidar-reset')?.addEventListener('click', () => {
        lidarViewScale = 50;
        lidarOffsetX = lidarCanvas.width / 2;
        lidarOffsetY = lidarCanvas.height / 2;
        redrawLidarView();
    });
    
    // Canvas panning with mouse for map
    let isMapPanning = false;
    let mapLastX = 0;
    let mapLastY = 0;
    
    if (mapCanvas) {
        mapCanvas.addEventListener('mousedown', (e) => {
            isMapPanning = true;
            mapLastX = e.clientX;
            mapLastY = e.clientY;
            mapCanvas.style.cursor = 'grabbing';
        });
        
        mapCanvas.addEventListener('mousemove', (e) => {
            if (isMapPanning) {
                const dx = e.clientX - mapLastX;
                const dy = e.clientY - mapLastY;
                mapOffsetX += dx;
                mapOffsetY += dy;
                mapLastX = e.clientX;
                mapLastY = e.clientY;
                redrawMapView();
            }
        });
        
        mapCanvas.addEventListener('mouseup', () => {
            isMapPanning = false;
            mapCanvas.style.cursor = 'grab';
        });
        
        mapCanvas.addEventListener('mouseleave', () => {
            isMapPanning = false;
            mapCanvas.style.cursor = 'grab';
        });
    }
    
    // Canvas panning with mouse for LIDAR
    let isLidarPanning = false;
    let lidarLastX = 0;
    let lidarLastY = 0;
    
    if (lidarCanvas) {
        lidarCanvas.addEventListener('mousedown', (e) => {
            isLidarPanning = true;
            lidarLastX = e.clientX;
            lidarLastY = e.clientY;
            lidarCanvas.style.cursor = 'grabbing';
        });
        
        lidarCanvas.addEventListener('mousemove', (e) => {
            if (isLidarPanning) {
                const dx = e.clientX - lidarLastX;
                const dy = e.clientY - lidarLastY;
                lidarOffsetX += dx;
                lidarOffsetY += dy;
                lidarLastX = e.clientX;
                lidarLastY = e.clientY;
                redrawLidarView();
            }
        });
        
        lidarCanvas.addEventListener('mouseup', () => {
            isLidarPanning = false;
            lidarCanvas.style.cursor = 'grab';
        });
        
        lidarCanvas.addEventListener('mouseleave', () => {
            isLidarPanning = false;
            lidarCanvas.style.cursor = 'grab';
        });
    }
    
    // Initialize speed display
    function updateSpeedDisplay() {
        if (linearSpeedSlider && linearSpeedValue) {
            linearSpeedValue.textContent = `${linearSpeedSlider.value} m/s`;
        }
        if (angularSpeedSlider && angularSpeedValue) {
            angularSpeedValue.textContent = `${angularSpeedSlider.value} rad/s`;
        }
    }
    
    // Event listeners for sliders
    linearSpeedSlider?.addEventListener('input', updateSpeedDisplay);
    angularSpeedSlider?.addEventListener('input', updateSpeedDisplay);
    
    // Movement button event listeners
    forwardBtn?.addEventListener('click', () => {
        sendCommand('forward');
        highlightButton(forwardBtn);
    });
    
    backwardBtn?.addEventListener('click', () => {
        sendCommand('backward');
        highlightButton(backwardBtn);
    });
    
    leftBtn?.addEventListener('click', () => {
        sendCommand('left');
        highlightButton(leftBtn);
    });
    
    rightBtn?.addEventListener('click', () => {
        sendCommand('right');
        highlightButton(rightBtn);
    });
    
    stopBtn?.addEventListener('click', () => {
        sendCommand('stop');
        highlightButton(stopBtn);
    });
    
    emergencyStopBtn?.addEventListener('click', () => {
        sendCommand('emergency_stop');
        highlightButton(emergencyStopBtn);
    });
    
    // Function to provide visual feedback when button is pressed
    function highlightButton(button) {
        button.classList.add('active');
        setTimeout(() => {
            button.classList.remove('active');
        }, 200);
    }
    
    // Handle key presses for movement
    document.addEventListener('keydown', (event) => {
        // Don't trigger if user is typing in an input field
        if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
            return;
        }
        
        switch(event.key) {
            case 'ArrowUp':
                event.preventDefault();
                forwardBtn?.click();
                break;
            case 'ArrowDown':
                event.preventDefault();
                backwardBtn?.click();
                break;
            case 'ArrowLeft':
                event.preventDefault();
                leftBtn?.click();
                break;
            case 'ArrowRight':
                event.preventDefault();
                rightBtn?.click();
                break;
            case ' ': // Space bar
                event.preventDefault();
                stopBtn?.click();
                break;
        }
    });
    
    // Connect to server
    function connectToServer() {
        try {
            updateStatus('Connecting to robot server...', 'connecting');
            
            socket = new WebSocket(serverAddress);
            
            socket.onopen = () => {
                isConnected = true;
                updateStatus('Connected to robot', 'connected');
                updateTimestamp();
                // Start timestamp update interval
                setInterval(updateTimestamp, 1000);
            };
            
            socket.onclose = () => {
                isConnected = false;
                updateStatus('Disconnected. Trying to reconnect...', 'disconnected');
                
                // Try to reconnect after 2 seconds
                setTimeout(connectToServer, 2000);
            };
            
            socket.onerror = (error) => {
                updateStatus('Connection error', 'error');
                console.error('WebSocket error:', error);
            };
            
            socket.onmessage = (event) => {
                try {
                    const response = JSON.parse(event.data);
                    
                    if (response.type === 'odom_data') {
                        handleOdomData(response.data);
                    } else if (response.type === 'map_data') {
                        handleMapData(response.data);
                    } else if (response.type === 'scan_data') {
                        handleScanData(response.data);
                    } else if (response.type === 'camera_feed') {
                        // Handle camera feed
                        const cameraImg = document.getElementById('camera-stream');
                        if (cameraImg) {
                            cameraImg.src = `data:image/jpeg;base64,${response.data}`;
                        }
                    } else if (response.type === 'status') {
                        updateStatus(response.message, response.status || 'info');
                    } else {
                        updateStatus(`Server response: ${response.message || JSON.stringify(response)}`, 'info');
                    }
                } catch (e) {
                    console.error('Error parsing server message:', e);
                    console.log('Raw message:', event.data);
                }
            };
        } catch (error) {
            updateStatus('Failed to connect to server', 'error');
            console.error('Connection error:', error);
            
            // Try to reconnect after 2 seconds
            setTimeout(connectToServer, 2000);
        }
    }
    
    // Update timestamp
    function updateTimestamp() {
        if (timestampDisplay) {
            const now = new Date();
            const hours = now.getHours().toString().padStart(2, '0');
            const minutes = now.getMinutes().toString().padStart(2, '0');
            const seconds = now.getSeconds().toString().padStart(2, '0');
            timestampDisplay.textContent = `${hours}:${minutes}:${seconds}`;
        }
    }
    
    // Update status message and indicator
    function updateStatus(message, status = 'info') {
        if (statusMessage) {
            statusMessage.textContent = message;
        }
        
        if (statusIndicator) {
            statusIndicator.className = 'status-indicator';
            statusIndicator.classList.add(status);
        }
        
        console.log(`[${status.toUpperCase()}] ${message}`);
    }
    
    // Send command to server
    function sendCommand(command, params = {}) {
        if (!isConnected) {
            updateStatus('Not connected to server', 'warning');
            return;
        }
        
        const message = {
            command: command,
            params: params
        };
        
        try {
            socket.send(JSON.stringify(message));
        } catch (error) {
            updateStatus('Failed to send command', 'error');
            console.error('Send error:', error);
        }
    }
    
    // Handle incoming data
    function handleOdomData(data) {
        odomData = data;
        
        // Add point to path history if position data exists
        if (data.position) {
            robotPath.push({
                x: data.position.x,
                y: data.position.y
            });
            
            // Limit path length
            if (robotPath.length > MAX_PATH_LENGTH) {
                robotPath.shift();
            }
        }
        
        // Update the current speed display
        const currentSpeed = document.getElementById('current-speed');
        if (currentSpeed && data.twist && data.twist.linear) {
            const speed = Math.sqrt(
                Math.pow(data.twist.linear.x, 2) + 
                Math.pow(data.twist.linear.y, 2)
            ).toFixed(2);
            currentSpeed.textContent = `${speed} m/s`;
        }
        
        // Update visualizations if they're active
        if (document.getElementById('map-view').classList.contains('active')) {
            redrawMapView();
        }
        if (document.getElementById('odom-view').classList.contains('active')) {
            redrawOdomView();
        }
        
        // Update raw data view if it's active
        if (document.getElementById('raw-data').classList.contains('active')) {
            updateRawDataView();
        }
    }
    
    // Handle telemetry data (battery, motor RPMs)
    function handleTelemetryData(data) {
        // Update battery display
        const batteryLevel = document.getElementById('battery-level');
        if (batteryLevel && data.battery_level !== undefined) {
            batteryLevel.textContent = `${data.battery_level}%`;
        }
        
        // Update ping/connection display with motor RPMs
        const pingDisplay = document.getElementById('ping');
        if (pingDisplay && data.left_motor_rpm !== undefined && data.right_motor_rpm !== undefined) {
            pingDisplay.textContent = `L: ${data.left_motor_rpm.toFixed(0)} R: ${data.right_motor_rpm.toFixed(0)} RPM`;
        }
    }
    
    function handleMapData(data) {
        mapData = data;
        
        // Update visualization if map view is active
        if (document.getElementById('map-view').classList.contains('active')) {
            redrawMapView();
        }
        
        // Update raw data view if it's active
        if (document.getElementById('raw-data').classList.contains('active')) {
            updateRawDataView();
        }
    }
    
    function handleScanData(data) {
        scanData = data;
        
        // Update visualizations if they're active
        if (document.getElementById('map-view').classList.contains('active')) {
            redrawMapView();
        }
        if (document.getElementById('lidar-view').classList.contains('active')) {
            redrawLidarView();
        }
        
        // Update raw data view if it's active
        if (document.getElementById('raw-data').classList.contains('active')) {
            updateRawDataView();
        }
    }
    
    // Map view rendering
    function redrawMapView() {
        if (!mapCtx) return;
        
        const canvas = mapCtx.canvas;
        const showMap = document.getElementById('show-map')?.checked ?? true;
        const showScan = document.getElementById('show-scan-on-map')?.checked ?? true;
        const showPath = document.getElementById('show-path')?.checked ?? true;
        
        // Clear the canvas
        mapCtx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw coordinate grid
        drawGrid(mapCtx, canvas, mapViewScale, mapOffsetX, mapOffsetY);
        
        // Draw map if available and enabled
        if (mapData && showMap) {
            drawMap(mapCtx, mapViewScale, mapOffsetX, mapOffsetY);
        }
        
        // Draw robot path if enabled
        if (robotPath.length > 0 && showPath) {
            drawPath(mapCtx, mapViewScale, mapOffsetX, mapOffsetY);
        }
        
        // Draw robot position and orientation
        if (odomData) {
            drawRobot(mapCtx, mapViewScale, mapOffsetX, mapOffsetY);
        }
        
        // Draw LIDAR scan if available and enabled
        if (scanData && odomData && showScan) {
            drawScan(mapCtx, mapViewScale, mapOffsetX, mapOffsetY);
        }
    }
    
    // LIDAR view rendering
    function redrawLidarView() {
        if (!lidarCtx || !scanData || !odomData) {
            return;
        }
        
        const canvas = lidarCtx.canvas;
        const showRays = document.getElementById('show-lidar-rays')?.checked ?? true;
        const colorMode = document.getElementById('lidar-color-mode')?.value || 'intensity';
        
        // Clear the canvas
        lidarCtx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw coordinate grid
        drawGrid(lidarCtx, canvas, lidarViewScale, lidarOffsetX, lidarOffsetY);
        
        // Draw robot at the center
        drawRobot(lidarCtx, lidarViewScale, lidarOffsetX, lidarOffsetY, true);
        
        // Draw LIDAR scan with selected options
        if (scanData && showRays) {
            drawLidarScan(lidarCtx, lidarViewScale, lidarOffsetX, lidarOffsetY, colorMode);
        }
    }
    
    // Odometry view rendering
    function redrawOdomView() {
        if (!odomCtx || !odomData) {
            return;
        }
        
        const canvas = odomCtx.canvas;
        const showVelocity = document.getElementById('show-velocity')?.checked ?? true;
        const showPose = document.getElementById('show-pose')?.checked ?? true;
        
        // Clear the canvas
        odomCtx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw coordinate grid
        drawGrid(odomCtx, canvas, odomViewScale, odomOffsetX, odomOffsetY);
        
        // Draw robot
        drawRobot(odomCtx, odomViewScale, odomOffsetX, odomOffsetY);
        
        // Draw path
        drawPath(odomCtx, odomViewScale, odomOffsetX, odomOffsetY);
        
        // Draw velocity vector if enabled
        if (showVelocity && odomData.twist) {
            drawVelocityVector(odomCtx, odomViewScale, odomOffsetX, odomOffsetY);
        }
        
        // Draw pose information if enabled
        if (showPose && odomData.position) {
            drawPoseInfo(odomCtx, odomViewScale, odomOffsetX, odomOffsetY);
        }
    }
    
    // Update raw data view based on selected data type
    function updateRawDataView() {
        if (!rawDataContent) return;
        
        const selectedType = document.querySelector('input[name="data-type"]:checked')?.value || 'odom';
        
        switch (selectedType) {
            case 'odom':
                if (odomData) {
                    rawDataContent.textContent = JSON.stringify(odomData, null, 2);
                } else {
                    rawDataContent.textContent = 'No odometry data available.';
                }
                break;
            case 'lidar':
                if (scanData) {
                    // Format LIDAR data for display - show summary and sample of ranges
                    const summary = {
                        angle_min: scanData.angle_min,
                        angle_max: scanData.angle_max,
                        angle_increment: scanData.angle_increment,
                        range_min: scanData.range_min,
                        range_max: scanData.range_max,
                        scan_time: scanData.scan_time,
                        total_points: scanData.ranges ? scanData.ranges.length : 0,
                        sample_ranges: scanData.ranges ? 
                            scanData.ranges.slice(0, 10).map(r => r.toFixed(3)) : []
                    };
                    rawDataContent.textContent = JSON.stringify(summary, null, 2);
                } else {
                    rawDataContent.textContent = 'No LIDAR data available.';
                }
                break;
            case 'map':
                if (mapData) {
                    // Show map metadata without the full data array
                    const mapSummary = {
                        width: mapData.width,
                        height: mapData.height,
                        resolution: mapData.resolution,
                        origin: mapData.origin
                    };
                    rawDataContent.textContent = JSON.stringify(mapSummary, null, 2);
                } else {
                    rawDataContent.textContent = 'No map data available.';
                }
                break;
            default:
                rawDataContent.textContent = 'No data available for the selected type.';
        }
    }
    
    // Drawing functions
    function drawGrid(ctx, canvas, scale, offsetX, offsetY) {
        const gridSize = 1.0; // 1 meter grid
        const gridPixels = gridSize * scale;
        
        ctx.strokeStyle = '#EEEEEE';
        ctx.lineWidth = 1;
        
        // Calculate grid boundaries
        const startX = offsetX % gridPixels;
        const startY = offsetY % gridPixels;
        
        // Draw vertical grid lines
        for (let x = startX; x < canvas.width; x += gridPixels) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, canvas.height);
            ctx.stroke();
        }
        
        // Draw horizontal grid lines
        for (let y = startY; y < canvas.height; y += gridPixels) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(canvas.width, y);
            ctx.stroke();
        }
        
        // Draw coordinate system origin
        ctx.strokeStyle = '#000000';
        ctx.lineWidth = 2;
        const originX = worldToCanvasX(0, scale, offsetX);
        const originY = worldToCanvasY(0, scale, offsetY);
        
        // X-axis
        ctx.beginPath();
        ctx.moveTo(originX, originY);
        ctx.lineTo(originX + 50, originY);
        ctx.stroke();
        
        // Y-axis
        ctx.beginPath();
        ctx.moveTo(originX, originY);
        ctx.lineTo(originX, originY - 50);
        ctx.stroke();
        
        // Labels
        ctx.fillStyle = '#000000';
        ctx.font = '12px Arial';
        ctx.fillText('X', originX + 55, originY + 5);
        ctx.fillText('Y', originX - 5, originY - 55);
    }
    
    function drawMap(ctx, scale, offsetX, offsetY) {
        if (!mapData || !mapData.data) return;
        
        const width = mapData.width;
        const height = mapData.height;
        const resolution = mapData.resolution; // meters per cell
        const originX = mapData.origin.x;
        const originY = mapData.origin.y;
        
        // Calculate cell size in pixels
        const cellSize = Math.max(1, scale * resolution);
        
        for (let y = 0; y < height; y++) {
            for (let x = 0; x < width; x++) {
                const index = y * width + x;
                const value = mapData.data[index];
                
                // Skip unknown cells (value = -1)
                if (value === -1) continue;
                
                // Convert occupancy grid value to grayscale (0-100 to 255-0)
                // 0 = free space (white), 100 = occupied (black)
                const intensity = 255 - Math.floor(value * 2.55);
                
                // Draw a single pixel for each cell
                const canvasX = worldToCanvasX(originX + x * resolution, scale, offsetX);
                const canvasY = worldToCanvasY(originY + y * resolution, scale, offsetY);
                
                ctx.fillStyle = `rgb(${intensity}, ${intensity}, ${intensity})`;
                ctx.fillRect(canvasX, canvasY, cellSize, cellSize);
            }
        }
    }
    
    function drawPath(ctx, scale, offsetX, offsetY) {
        if (!robotPath || robotPath.length < 2) return;
        
        ctx.strokeStyle = 'rgba(255, 100, 100, 0.7)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        for (let i = 0; i < robotPath.length; i++) {
            const x = worldToCanvasX(robotPath[i].x, scale, offsetX);
            const y = worldToCanvasY(robotPath[i].y, scale, offsetY);
            
            if (i === 0) {
                ctx.moveTo(x, y);
            } else {
                ctx.lineTo(x, y);
            }
        }
        
        ctx.stroke();
    }
    
    function drawRobot(ctx, scale, offsetX, offsetY, center = false) {
        // If center is true, draw robot at center of view, otherwise use odometry position
        const robotX = center ? 0 : (odomData.position ? odomData.position.x : 0);
        const robotY = center ? 0 : (odomData.position ? odomData.position.y : 0);
        
        const x = worldToCanvasX(robotX, scale, offsetX);
        const y = worldToCanvasY(robotY, scale, offsetY);
        const radius = 15;
        
        // Calculate robot orientation from quaternion
        let yaw = 0;
        if (odomData && odomData.orientation) {
            const q = odomData.orientation;
            // Convert quaternion to yaw (simplified)
            yaw = Math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            );
        }
        
        // Robot body (circle)
        ctx.fillStyle = 'rgba(0, 150, 255, 0.7)';
        ctx.beginPath();
        ctx.arc(x, y, radius, 0, 2 * Math.PI);
        ctx.fill();
        
        // Direction indicator
        ctx.strokeStyle = '#FFFFFF';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(
            x + Math.cos(yaw) * radius * 1.5,
            y - Math.sin(yaw) * radius * 1.5
        );
        ctx.stroke();
    }
    
    function drawScan(ctx, scale, offsetX, offsetY) {
        if (!scanData || !scanData.ranges || !odomData || !odomData.position) return;
        
        const ranges = scanData.ranges;
        const angleMin = scanData.angle_min;
        const angleIncrement = scanData.angle_increment;
        const robotX = odomData.position.x;
        const robotY = odomData.position.y;
        
        // Calculate robot orientation from quaternion
        let robotYaw = 0;
        if (odomData.orientation) {
            const q = odomData.orientation;
            robotYaw = Math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            );
        }
        
        ctx.fillStyle = 'rgba(255, 0, 0, 0.5)';
        
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];
            
            // Skip invalid readings
            if (range < scanData.range_min || range > scanData.range_max || isNaN(range)) {
                continue;
            }
            
            // Calculate the angle of this reading
            const angle = angleMin + (i * angleIncrement) + robotYaw;
            
            // Calculate the endpoint of the ray
            const endX = robotX + range * Math.cos(angle);
            const endY = robotY + range * Math.sin(angle);
            
            // Convert to canvas coordinates
            const canvasX = worldToCanvasX(endX, scale, offsetX);
            const canvasY = worldToCanvasY(endY, scale, offsetY);
            
            // Draw the point
            ctx.beginPath();
            ctx.arc(canvasX, canvasY, 2, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
    
    function drawLidarScan(ctx, scale, offsetX, offsetY, colorMode) {
        if (!scanData || !scanData.ranges) return;
        
        const ranges = scanData.ranges;
        const angleMin = scanData.angle_min;
        const angleIncrement = scanData.angle_increment;
        const robotX = 0;  // In LIDAR view, robot is at center
        const robotY = 0;
        
        for (let i = 0; i < ranges.length; i++) {
            const range = ranges[i];
            
            // Skip invalid readings
            if (range < scanData.range_min || range > scanData.range_max || isNaN(range)) {
                continue;
            }
            
            // Calculate the angle of this reading
            const angle = angleMin + (i * angleIncrement);
            
            // Calculate the endpoint of the ray
            const endX = robotX + range * Math.cos(angle);
            const endY = robotY + range * Math.sin(angle);
            
            // Convert to canvas coordinates
            const canvasX = worldToCanvasX(endX, scale, offsetX);
            const canvasY = worldToCanvasY(endY, scale, offsetY);
            const robotCenterX = worldToCanvasX(robotX, scale, offsetX);
            const robotCenterY = worldToCanvasY(robotY, scale, offsetY);
            
            // Choose color based on selected mode
            let color;
            switch (colorMode) {
                case 'intensity':
                    // Color based on intensity - assuming normalized between 0 and 1
                    const intensity = ranges[i] / scanData.range_max;
                    color = `rgba(255, ${Math.floor(255 * intensity)}, 0, 0.8)`;
                    break;
                case 'distance':
                    // Color gradient based on distance (red = close, blue = far)
                    const normalized = Math.min(1, range / scanData.range_max);
                    const r = Math.floor(255 * (1 - normalized));
                    const b = Math.floor(255 * normalized);
                    color = `rgba(${r}, 0, ${b}, 0.8)`;
                    break;
                case 'rainbow':
                    // Rainbow color based on angle
                    const hue = ((i / ranges.length) * 360) % 360;
                    color = `hsla(${hue}, 100%, 50%, 0.8)`;
                    break;
                default:
                    color = 'rgba(255, 0, 0, 0.8)';
            }
            
            // Draw the ray
            ctx.strokeStyle = `rgba(255, 255, 255, 0.2)`;
            ctx.beginPath();
            ctx.moveTo(robotCenterX, robotCenterY);
            ctx.lineTo(canvasX, canvasY);
            ctx.stroke();
            
            // Draw the point
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(canvasX, canvasY, 3, 0, 2 * Math.PI);
            ctx.fill();
        }
    }
    
    function drawVelocityVector(ctx, scale, offsetX, offsetY) {
        if (!odomData || !odomData.twist || !odomData.position) return;
        
        const twist = odomData.twist;
        const position = odomData.position;
        
        const robotX = position.x;
        const robotY = position.y;
        const linearX = twist.linear ? twist.linear.x : 0;
        const linearY = twist.linear ? twist.linear.y : 0;
        const angularZ = twist.angular ? twist.angular.z : 0;
        
        const canvasRobotX = worldToCanvasX(robotX, scale, offsetX);
        const canvasRobotY = worldToCanvasY(robotY, scale, offsetY);
        
        // Draw linear velocity vector
        const velocityMagnitude = Math.sqrt(linearX * linearX + linearY * linearY);
        if (velocityMagnitude > 0.01) {  // Only draw if significant
            const velocityAngle = Math.atan2(linearY, linearX);
            const velocityLength = Math.min(50, velocityMagnitude * 20);  // Scale but cap length
            
            const endX = canvasRobotX + velocityLength * Math.cos(velocityAngle);
            const endY = canvasRobotY - velocityLength * Math.sin(velocityAngle);
            
            // Draw the arrow
            ctx.strokeStyle = '#00FF00';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(canvasRobotX, canvasRobotY);
            ctx.lineTo(endX, endY);
            ctx.stroke();
            
            // Draw arrowhead
            const arrowSize = 8;
            const arrowAngle = Math.PI / 6;  // 30 degrees
            ctx.beginPath();
            ctx.moveTo(endX, endY);
            ctx.lineTo(
                endX - arrowSize * Math.cos(velocityAngle - arrowAngle),
                endY + arrowSize * Math.sin(velocityAngle - arrowAngle)
            );
            ctx.moveTo(endX, endY);
            ctx.lineTo(
                endX - arrowSize * Math.cos(velocityAngle + arrowAngle),
                endY + arrowSize * Math.sin(velocityAngle + arrowAngle)
            );
            ctx.stroke();
        }
        
        // Draw angular velocity indicator (curved arrow)
        if (Math.abs(angularZ) > 0.01) {
            const radius = 25; // Slightly larger radius for the indicator
            const arcAngle = Math.PI / 3; // Draw a 60-degree arc segment
            const startAngle = -Math.PI / 2 - arcAngle / 2; // Center the arc segment at the top
            const endAngle = startAngle + arcAngle;
            const antiClockwise = angularZ < 0; // ROS REP 103: positive Z is CCW

            ctx.strokeStyle = '#FFAA00';
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.arc(canvasRobotX, canvasRobotY, radius, startAngle, endAngle, antiClockwise);
            ctx.stroke();

            // Draw arrowhead at the end of the arc
            const arrowSize = 6;
            const headAngle = Math.PI / 6; // 30 degrees arrowhead angle
            const arrowTipAngle = antiClockwise ? startAngle : endAngle; // Arrow points in direction of rotation

            const arrowTipX = canvasRobotX + radius * Math.cos(arrowTipAngle);
            const arrowTipY = canvasRobotY + radius * Math.sin(arrowTipAngle);

            // Calculate angles for the two sides of the arrowhead
            // Tangent angle is arrowTipAngle +/- PI/2
            const tangentAngle = arrowTipAngle + (antiClockwise ? Math.PI / 2 : -Math.PI / 2);
            const angle1 = tangentAngle - headAngle;
            const angle2 = tangentAngle + headAngle;

            ctx.beginPath();
            ctx.moveTo(arrowTipX, arrowTipY);
            ctx.lineTo(arrowTipX + arrowSize * Math.cos(angle1), arrowTipY + arrowSize * Math.sin(angle1));
            ctx.moveTo(arrowTipX, arrowTipY);
            ctx.lineTo(arrowTipX + arrowSize * Math.cos(angle2), arrowTipY + arrowSize * Math.sin(angle2));
            ctx.stroke();
        }
    }

    function drawPoseInfo(ctx, scale, offsetX, offsetY) {
        if (!odomData || !odomData.position) return;
        
        const position = odomData.position;
        const canvasX = worldToCanvasX(position.x, scale, offsetX);
        const canvasY = worldToCanvasY(position.y, scale, offsetY);
        
        // Calculate yaw from quaternion
        let yaw = 0;
        if (odomData.orientation) {
            const q = odomData.orientation;
            yaw = Math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            ) * (180 / Math.PI);  // Convert to degrees
        }
        
        // Display position and orientation text
        ctx.fillStyle = '#000000';
        ctx.font = '12px Arial';
        ctx.fillText(`(${position.x.toFixed(2)}, ${position.y.toFixed(2)})`, canvasX + 20, canvasY);
        ctx.fillText(`θ: ${yaw.toFixed(1)}°`, canvasX + 20, canvasY + 15);
    }
    
    // Coordinate conversion helpers
    function worldToCanvasX(x, scale, offsetX) {
        return offsetX + x * scale;
    }
    
    function worldToCanvasY(y, scale, offsetY) {
        return offsetY - y * scale; // Y is flipped in canvas
    }
    
    // Initialize
    updateSpeedDisplay();
    updateTimestamp();
    connectToServer();
});
