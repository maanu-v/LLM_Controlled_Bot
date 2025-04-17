document.addEventListener('DOMContentLoaded', () => {
    // Server connection
    let socket = null;
    let isConnected = false;
    const serverAddress = 'ws://' + window.location.hostname + ':8765';
    
    // UI Elements
    const statusMessage = document.getElementById('status-message');
    const connectionStatus = document.getElementById('connection-status');
    
    // Speed control elements
    const linearSpeedSlider = document.getElementById('linear-speed');
    const angularSpeedSlider = document.getElementById('angular-speed');
    const linearSpeedValue = document.getElementById('linear-speed-value');
    const angularSpeedValue = document.getElementById('angular-speed-value');
    const updateSpeedBtn = document.getElementById('update-speed-btn');
    
    // Direction control buttons
    const forwardBtn = document.getElementById('forward-btn');
    const backwardBtn = document.getElementById('backward-btn');
    const leftBtn = document.getElementById('left-btn');
    const rightBtn = document.getElementById('right-btn');
    const stopBtn = document.getElementById('stop-btn');
    
    // Initialize UI
    function updateSpeedDisplay() {
        linearSpeedValue.textContent = `${linearSpeedSlider.value} m/s`;
        angularSpeedValue.textContent = `${angularSpeedSlider.value} rad/s`;
    }
    
    // Event listeners for sliders
    linearSpeedSlider.addEventListener('input', updateSpeedDisplay);
    angularSpeedSlider.addEventListener('input', updateSpeedDisplay);
    
    // Update speed on server
    updateSpeedBtn.addEventListener('click', () => {
        sendCommand('speed', {
            linear: parseFloat(linearSpeedSlider.value),
            angular: parseFloat(angularSpeedSlider.value)
        });
        updateStatus(`Speed updated to linear: ${linearSpeedSlider.value} m/s, angular: ${angularSpeedSlider.value} rad/s`);
    });
    
    // Movement button event listeners
    forwardBtn.addEventListener('click', () => {
        sendCommand('forward');
        highlightButton(forwardBtn);
    });
    backwardBtn.addEventListener('click', () => {
        sendCommand('backward');
        highlightButton(backwardBtn);
    });
    leftBtn.addEventListener('click', () => {
        sendCommand('left');
        highlightButton(leftBtn);
    });
    rightBtn.addEventListener('click', () => {
        sendCommand('right');
        highlightButton(rightBtn);
    });
    stopBtn.addEventListener('click', () => {
        sendCommand('stop');
        highlightButton(stopBtn);
    });
    
    // Function to provide visual feedback when button is pressed
    function highlightButton(button) {
        button.style.transform = 'scale(0.95)';
        setTimeout(() => {
            button.style.transform = '';
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
                forwardBtn.click();
                break;
            case 'ArrowDown':
                event.preventDefault();
                backwardBtn.click();
                break;
            case 'ArrowLeft':
                event.preventDefault();
                leftBtn.click();
                break;
            case 'ArrowRight':
                event.preventDefault();
                rightBtn.click();
                break;
            case ' ': // Space bar
                event.preventDefault();
                stopBtn.click();
                break;
        }
    });
    
    // Connect to server
    function connectToServer() {
        try {
            // Using a direct TCP connection via the WebBridge
            socket = new WebSocket(serverAddress);
            
            socket.onopen = () => {
                isConnected = true;
                connectionStatus.textContent = 'Connected';
                connectionStatus.className = 'connected';
                updateStatus('Connected to robot');
                enableControls(true);
            };
            
            socket.onclose = () => {
                isConnected = false;
                connectionStatus.textContent = 'Disconnected';
                connectionStatus.className = 'disconnected';
                updateStatus('Connection lost. Attempting to reconnect...');
                enableControls(false);
                
                // Try to reconnect after 2 seconds
                setTimeout(connectToServer, 2000);
            };
            
            socket.onerror = (error) => {
                updateStatus('Connection error');
                console.error('WebSocket error:', error);
                enableControls(false);
            };
            
            socket.onmessage = (event) => {
                try {
                    const response = JSON.parse(event.data);
                    updateStatus(`Server response: ${response.message}`);
                } catch (e) {
                    updateStatus(`Received: ${event.data}`);
                }
            };
        } catch (error) {
            updateStatus('Failed to connect to server');
            console.error('Connection error:', error);
            enableControls(false);
            
            // Try to reconnect after 2 seconds
            setTimeout(connectToServer, 2000);
        }
    }
    
    // Enable/disable controls based on connection status
    function enableControls(enabled) {
        const controls = [
            forwardBtn, backwardBtn, leftBtn, rightBtn, stopBtn, 
            updateSpeedBtn, linearSpeedSlider, angularSpeedSlider
        ];
        
        controls.forEach(control => {
            control.disabled = !enabled;
            if (enabled) {
                control.classList.remove('disabled');
            } else {
                control.classList.add('disabled');
            }
        });
    }
    
    // Send command to server
    function sendCommand(command, params = {}) {
        if (!isConnected) {
            updateStatus('Not connected to server');
            return;
        }
        
        const message = {
            command: command,
            params: params
        };
        
        try {
            socket.send(JSON.stringify(message) + '\n');
            updateStatus(`Sent command: ${command}`);
        } catch (error) {
            updateStatus('Failed to send command');
            console.error('Send error:', error);
        }
    }
    
    // Update status message
    function updateStatus(message) {
        statusMessage.textContent = message;
        console.log(message);
    }
    
    // Initialize connection and UI
    updateSpeedDisplay();
    enableControls(false); // Start with controls disabled until connected
    connectToServer();
});
