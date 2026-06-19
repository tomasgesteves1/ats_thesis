// State management
let currentTelemetry = null;
let selectedMission = null; 
let loadedMissions = []; 
let activeStatus = {
    simulation_running: false,
    mission_running: false,
    active_mission: 'None'
};

// DOM elements
const statusSim = document.getElementById('status-sim');
const statusMission = document.getElementById('status-mission');
const btnStartSim = document.getElementById('btn-start-sim');
const btnStopSim = document.getElementById('btn-stop-sim');
const btnLaunchMission = document.getElementById('btn-launch-mission');
const btnStopMission = document.getElementById('btn-stop-mission');
const btnEmergency = document.getElementById('btn-emergency');
const simTether = document.getElementById('sim-tether');
const connectionStatus = document.getElementById('connection-status');
const logsConsole = document.getElementById('logs-console');
const btnClearLogs = document.getElementById('btn-clear-logs');
const missionSelector = document.getElementById('mission-selector');
const missionParamsContainer = document.getElementById('mission-params-container');
const dynamicParams = document.getElementById('dynamic-params');
const simType = document.getElementById('sim-type');
const tetherGroup = document.getElementById('tether-group');

// Radar canvas setup
const canvas = document.getElementById('radar-canvas');
const ctx = canvas.getContext('2d');

// Configure canvas responsiveness
function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * window.devicePixelRatio;
    canvas.height = rect.height * window.devicePixelRatio;
}
window.addEventListener('resize', resizeCanvas);
resizeCanvas();

// Add a log message to console
function addLog(message, type = 'info') {
    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    const timestamp = new Date().toLocaleTimeString();
    entry.textContent = `[${timestamp}] ${message}`;
    logsConsole.appendChild(entry);
    logsConsole.scrollTop = logsConsole.scrollHeight;
}

// Clear log helper
btnClearLogs.addEventListener('click', () => {
    if (activeLogType === 'gcs') {
        logsConsole.innerHTML = '';
        addLog('Logs cleared.', 'info');
    } else {
        // Clear log file on server
        fetch('/api/clear_logs', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ type: activeLogType })
        })
        .then(res => res.json())
        .then(data => {
            if (data.success) {
                const el = document.getElementById(`logs-${activeLogType}`);
                if (el) {
                    el.textContent = '';
                }
                addLog(`${activeLogType === 'control' ? 'Control' : 'Simulation'} console cleared.`, 'info');
            }
        })
        .catch(err => {
            console.error("Error clearing logs:", err);
        });
    }
});

// Periodic API status updates
async function updateStatus() {
    try {
        const response = await fetch('/api/status');
        if (!response.ok) throw new Error('Status fetch failed');
        activeStatus = await response.json();
        
        // Update simulation UI
        if (activeStatus.simulation_running) {
            statusSim.className = 'badge badge-active';
            statusSim.textContent = 'Active';
            btnStartSim.disabled = true;
            btnStopSim.disabled = false;
        } else {
            statusSim.className = 'badge badge-inactive';
            statusSim.textContent = 'Inactive';
            btnStartSim.disabled = false;
            btnStopSim.disabled = true;
        }

        // Update mission UI
        if (activeStatus.mission_running) {
            statusMission.className = 'badge badge-active';
            statusMission.textContent = activeStatus.active_mission.toUpperCase();
            btnLaunchMission.disabled = true;
            btnStopMission.disabled = false;
        } else {
            statusMission.className = 'badge badge-inactive';
            statusMission.textContent = 'None';
            btnLaunchMission.disabled = !activeStatus.simulation_running || !selectedMission;
            btnStopMission.disabled = true;
        }
    } catch (error) {
        // Silent error since SSE connection status reflects disconnect
    }
}

// REST API calls
async function postAPI(endpoint, data = {}) {
    try {
        const response = await fetch(endpoint, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data)
        });
        const result = await response.json();
        if (result.success) {
            addLog(result.message, 'success');
        } else {
            addLog(result.message, 'error');
        }
        await updateStatus();
    } catch (error) {
        addLog(`Erro ao enviar comando para ${endpoint}: ${error.message}`, 'error');
    }
}

// Event listeners for launcher controls
btnStartSim.addEventListener('click', () => {
    if (!selectedMission) {
        addLog('Select a mission first to start the simulation.', 'warning');
        return;
    }
    
    // Collect parameters
    const paramInputs = dynamicParams.querySelectorAll('input, select');
    const params = {};
    paramInputs.forEach(input => {
        if (input.type === 'checkbox') {
            params[input.dataset.key] = input.checked;
        } else {
            params[input.dataset.key] = input.value;
        }
    });

    addLog(`Starting simulation for mission ${selectedMission.toUpperCase()} with parameters...`, 'info');
    postAPI('/api/launch', { type: 'simulation', mission_id: selectedMission, params: params });
});

btnStopSim.addEventListener('click', () => {
    addLog('Stopping simulation...', 'info');
    postAPI('/api/stop', { type: 'simulation' });
});

btnLaunchMission.addEventListener('click', () => {
    if (!selectedMission) {
        addLog('Select a mission first.', 'warning');
        return;
    }
    
    // Collect parameters
    const paramInputs = dynamicParams.querySelectorAll('input, select');
    const params = {};
    paramInputs.forEach(input => {
        if (input.type === 'checkbox') {
            params[input.dataset.key] = input.checked;
        } else {
            params[input.dataset.key] = input.value;
        }
    });

    addLog(`Launching mission: ${selectedMission.toUpperCase()} with parameters: ${JSON.stringify(params)}...`, 'info');
    postAPI('/api/launch', { type: 'mission', name: selectedMission, params: params });
});

btnStopMission.addEventListener('click', () => {
    addLog('Stopping active mission...', 'info');
    postAPI('/api/stop', { type: 'mission' });
});

btnEmergency.addEventListener('click', () => {
    addLog('!!! EMERGENCY STOP REQUESTED !!!', 'error');
    postAPI('/api/emergency_stop');
});

// Render dynamic missions
async function fetchAndRenderMissions() {
    try {
        const response = await fetch('/api/missions');
        if (!response.ok) throw new Error('Failed to fetch missions');
        loadedMissions = await response.json();
        
        missionSelector.innerHTML = '';
        if (loadedMissions.length === 0) {
            missionSelector.innerHTML = '<p style="color: rgba(255,255,255,0.4); text-align: center; padding: 10px;">No missions found.</p>';
            return;
        }
        
        const iconMap = {
            'circle': 'fa-circle-notch',
            'link': 'fa-link',
            'anchor': 'fa-anchor',
            'radar': 'fa-bullseye'
        };

        loadedMissions.forEach(m => {
            const btn = document.createElement('button');
            btn.className = 'mission-btn';
            btn.dataset.id = m.id;
            
            const iconClass = iconMap[m.icon] || 'fa-route';
            
            btn.innerHTML = `
                <div class="mission-icon"><i class="fa-solid ${iconClass}"></i></div>
                <div class="mission-info">
                    <h3>${m.name}</h3>
                    <p>${m.description}</p>
                </div>
            `;
            
            btn.addEventListener('click', () => {
                if (activeStatus.mission_running) return;
                selectMission(m.id);
            });
            
            missionSelector.appendChild(btn);
        });
        
        if (loadedMissions.length > 0 && !selectedMission) {
            selectMission(loadedMissions[0].id);
        }
    } catch (err) {
        addLog(`Error loading missions: ${err.message}`, 'error');
        missionSelector.innerHTML = '<p style="color: #ef4444; text-align: center; padding: 10px;">Error loading missions.</p>';
    }
}

function selectMission(missionId) {
    selectedMission = missionId;
    
    const buttons = missionSelector.querySelectorAll('.mission-btn');
    buttons.forEach(btn => {
        if (btn.dataset.id === missionId) {
            btn.classList.add('selected');
        } else {
            btn.classList.remove('selected');
        }
    });
    
    const mission = loadedMissions.find(m => m.id === missionId);
    if (!mission) return;
    
    dynamicParams.innerHTML = '';
    
    if (mission.user_params && mission.user_params.length > 0) {
        missionParamsContainer.style.display = 'block';
        
        mission.user_params.forEach(p => {
            const group = document.createElement('div');
            group.className = 'form-group';
            group.style.marginBottom = '12px';
            
            const label = document.createElement('label');
            label.className = 'form-label';
            label.style.display = 'block';
            label.style.marginBottom = '4px';
            label.style.fontSize = '0.8rem';
            label.innerHTML = `${p.label} ${p.unit ? `(<small>${p.unit}</small>)` : ''}:`;
            group.appendChild(label);
            
            let input;
            if (p.type === 'bool') {
                const switchContainer = document.createElement('label');
                switchContainer.className = 'switch-container';
                
                input = document.createElement('input');
                input.type = 'checkbox';
                input.dataset.key = p.key;
                input.checked = p.default === 'true' || p.default === true;
                
                const slider = document.createElement('span');
                slider.className = 'slider';
                
                const labelText = document.createElement('span');
                labelText.className = 'switch-label';
                labelText.textContent = p.label;
                
                switchContainer.appendChild(input);
                switchContainer.appendChild(slider);
                switchContainer.appendChild(labelText);
                
                label.style.display = 'none';
                group.appendChild(switchContainer);
            } else if (p.type === 'float' || p.type === 'int') {
                const sliderWrapper = document.createElement('div');
                sliderWrapper.style.display = 'flex';
                sliderWrapper.style.alignItems = 'center';
                sliderWrapper.style.gap = '10px';
                
                input = document.createElement('input');
                input.type = 'range';
                input.className = 'form-range';
                input.style.flex = '1';
                input.dataset.key = p.key;
                input.min = p.min;
                input.max = p.max;
                input.step = p.type === 'float' ? '0.1' : '1';
                input.value = p.default;
                
                const valBubble = document.createElement('span');
                valBubble.style.minWidth = '45px';
                valBubble.style.textAlign = 'right';
                valBubble.style.fontSize = '0.85rem';
                valBubble.style.fontFamily = 'monospace';
                valBubble.style.color = '#38bdf8';
                valBubble.textContent = `${p.default}${p.unit ? p.unit : ''}`;
                
                input.addEventListener('input', () => {
                    valBubble.textContent = `${input.value}${p.unit ? p.unit : ''}`;
                });
                
                sliderWrapper.appendChild(input);
                sliderWrapper.appendChild(valBubble);
                group.appendChild(sliderWrapper);
            } else if (p.type === 'enum') {
                input = document.createElement('select');
                input.className = 'form-select';
                input.dataset.key = p.key;
                
                p.options.forEach(opt => {
                    const o = document.createElement('option');
                    o.value = opt;
                    o.text = opt;
                    if (opt === p.default) o.selected = true;
                    input.appendChild(o);
                });
                group.appendChild(input);
            } else {
                input = document.createElement('input');
                input.type = 'text';
                input.className = 'form-input';
                input.style.width = '100%';
                input.dataset.key = p.key;
                input.value = p.default;
                group.appendChild(input);
            }
            
            dynamicParams.appendChild(group);
        });
    } else {
        missionParamsContainer.style.display = 'none';
    }
    
    btnLaunchMission.disabled = !activeStatus.simulation_running;
    addLog(`Mission selected: ${mission.name}.`, 'info');
}

// SSE connection for telemetry
let eventSource = null;

function connectTelemetry() {
    if (eventSource) {
        eventSource.close();
    }

    eventSource = new EventSource('/api/telemetry');

    eventSource.onopen = () => {
        connectionStatus.querySelector('.status-dot').className = 'status-dot connected';
        connectionStatus.querySelector('.status-label').textContent = 'Connected';
        addLog('GCS Web Server connected.', 'success');
        updateStatus();
    };

    eventSource.onerror = (err) => {
        connectionStatus.querySelector('.status-dot').className = 'status-dot disconnected';
        connectionStatus.querySelector('.status-label').textContent = 'Disconnected';
        addLog('GCS Web Server disconnected. Reconnecting...', 'warning');
        eventSource.close();
        
        // Reconnect after 2 seconds
        setTimeout(connectTelemetry, 2000);
    };

    eventSource.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            currentTelemetry = data;
            updateTelemetryUI(data);
        } catch (e) {
            console.error('Failed to parse telemetry', e);
        }
    };
}

// Update telemetry UI cards
function updateTelemetryUI(data) {
    // UAV
    document.getElementById('uav-x').innerHTML = `${data.uav.x.toFixed(2)} <small>m</small>`;
    document.getElementById('uav-y').innerHTML = `${data.uav.y.toFixed(2)} <small>m</small>`;
    document.getElementById('uav-z').innerHTML = `${data.uav.z.toFixed(2)} <small>m</small>`;
    document.getElementById('uav-vz').innerHTML = `${data.uav.vz.toFixed(2)} <small>m/s</small>`;
    document.getElementById('uav-yaw').innerHTML = `${data.uav.yaw.toFixed(1)} <small>°</small>`;
    
    // UAV Status Badges
    const armedBadge = document.getElementById('uav-armed');
    if (data.uav.armed) {
        armedBadge.className = 'badge badge-uav-arm armed';
        armedBadge.textContent = 'ARMED';
    } else {
        armedBadge.className = 'badge badge-uav-arm disarmed';
        armedBadge.textContent = 'DISARMED';
    }
    document.getElementById('uav-mode').textContent = data.uav.nav_state;

    // UAV Battery
    const batPercent = data.uav.battery_percent;
    const batVoltage = data.uav.battery_voltage;
    document.getElementById('uav-bat-val').textContent = `${batPercent.toFixed(0)}% (${batVoltage.toFixed(1)}V)`;
    
    const batBar = document.getElementById('uav-bat-bar');
    batBar.style.width = `${batPercent}%`;
    if (batPercent < 20) {
        batBar.style.background = '#ef4444'; // Red
    } else if (batPercent < 50) {
        batBar.style.background = '#f59e0b'; // Amber
    } else {
        batBar.style.background = 'linear-gradient(90deg, #f59e0b 0%, #10b981 100%)'; // Green gradient
    }

    // USV
    document.getElementById('usv-x').innerHTML = `${data.usv.x.toFixed(2)} <small>m</small>`;
    document.getElementById('usv-y').innerHTML = `${data.usv.y.toFixed(2)} <small>m</small>`;
    document.getElementById('usv-z').innerHTML = `${data.usv.z.toFixed(2)} <small>m</small>`;
    document.getElementById('usv-vx').innerHTML = `${data.usv.vx.toFixed(2)} <small>m/s</small>`;
    document.getElementById('usv-yaw').innerHTML = `${data.usv.yaw.toFixed(1)} <small>°</small>`;

    // Tether
    document.getElementById('tether-length').innerHTML = `${data.tether.length.toFixed(2)} <small>m</small>`;
    document.getElementById('tether-dist').innerHTML = `${data.tether.distance.toFixed(2)} <small>m</small>`;
    document.getElementById('tension-drone').innerHTML = `${data.tether.tension_drone.toFixed(2)} <small>N</small>`;
    document.getElementById('tension-boat').innerHTML = `${data.tether.tension_boat.toFixed(2)} <small>N</small>`;
}

// 2D Radar Canvas Plotting Loop
function drawRadar() {
    requestAnimationFrame(drawRadar);

    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);

    // Grid center point
    const cx = w / 2;
    const cy = h / 2;

    // Draw coordinate rings (Radar look)
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.1)';
    ctx.lineWidth = 1;
    const rings = 4;
    const maxRadius = Math.min(w, h) * 0.45;
    for (let i = 1; i <= rings; i++) {
        ctx.beginPath();
        ctx.arc(cx, cy, (maxRadius / rings) * i, 0, 2 * Math.PI);
        ctx.stroke();
    }

    // Draw crosshair axes
    ctx.beginPath();
    ctx.moveTo(cx - maxRadius, cy);
    ctx.lineTo(cx + maxRadius, cy);
    ctx.moveTo(cx, cy - maxRadius);
    ctx.lineTo(cx, cy + maxRadius);
    ctx.stroke();

    // Radar labels
    ctx.fillStyle = 'rgba(56, 189, 248, 0.4)';
    ctx.font = `${Math.round(w * 0.025)}px monospace`;
    ctx.textAlign = 'center';
    ctx.fillText('N', cx, cy - maxRadius + 15);
    ctx.fillText('S', cx, cy + maxRadius - 5);
    ctx.fillText('W', cx - maxRadius + 10, cy + 4);
    ctx.fillText('E', cx + maxRadius - 10, cy + 4);

    if (!currentTelemetry) {
        // Draw standard offline radar message
        ctx.fillStyle = 'rgba(255, 255, 255, 0.2)';
        ctx.font = `${Math.round(w * 0.04)}px sans-serif`;
        ctx.textAlign = 'center';
        ctx.fillText('WAITING FOR TELEMETRY', cx, cy);
        return;
    }

    // Extract Positions
    const uav = currentTelemetry.uav;
    const usv = currentTelemetry.usv;

    // Scale calculation: Dynamic scale to fit both vehicles in the canvas
    // We want the maximum coordinate deviation to map to 80% of maxRadius.
    const maxCoord = Math.max(
        Math.abs(uav.x), Math.abs(uav.y),
        Math.abs(usv.x), Math.abs(usv.y),
        5.0 // Minimum 5m scale
    );
    const scale = (maxRadius * 0.8) / maxCoord;

    // Coordinate conversion helper (Gazebo X points North/Up, Y points West/Left. 
    // Wait: ROS coordinate convention: X is forward (Up), Y is left.
    // Let's map ROS X -> canvas screen negative Y (up), ROS Y -> canvas screen negative X (left).
    function toScreen(x, y) {
        return {
            x: cx - y * scale, // Y positive is left, so subtract
            y: cy - x * scale  // X positive is forward (up), so subtract
        };
    }

    const posBoat = toScreen(usv.x, usv.y);
    const posDrone = toScreen(uav.x, uav.y);

    // 1. Draw Tether Line
    ctx.beginPath();
    ctx.strokeStyle = '#f59e0b';
    ctx.lineWidth = 3;
    ctx.shadowBlur = 10;
    ctx.shadowColor = '#f59e0b';
    ctx.moveTo(posBoat.x, posBoat.y);
    ctx.lineTo(posDrone.x, posDrone.y);
    ctx.stroke();
    // Reset shadow
    ctx.shadowBlur = 0;

    // 2. Draw WAM-V Boat
    ctx.save();
    ctx.translate(posBoat.x, posBoat.y);
    ctx.rotate(-usv.yaw * Math.PI / 180.0); // Convert degree to radian, negative because screen Y is inverted
    
    // Boat drawing (triangle/arrow shape pointing forward)
    ctx.fillStyle = '#a855f7';
    ctx.strokeStyle = '#c084fc';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, -16); // bow (front)
    ctx.lineTo(8, 8);   // stern right
    ctx.lineTo(-8, 8);  // stern left
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // 3. Draw Drone (x500)
    ctx.save();
    ctx.translate(posDrone.x, posDrone.y);
    ctx.rotate(-uav.yaw * Math.PI / 180.0);

    // Quadrotor drawing (X-shape and center circle)
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 3;
    ctx.beginPath();
    // Arms
    ctx.moveTo(-10, -10);
    ctx.lineTo(10, 10);
    ctx.moveTo(10, -10);
    ctx.lineTo(-10, 10);
    ctx.stroke();

    // Rotors
    ctx.fillStyle = '#0ea5e9';
    ctx.beginPath();
    ctx.arc(-10, -10, 4, 0, 2*Math.PI);
    ctx.arc(10, -10, 4, 0, 2*Math.PI);
    ctx.arc(10, 10, 4, 0, 2*Math.PI);
    ctx.arc(-10, 10, 4, 0, 2*Math.PI);
    ctx.fill();

    // Center hub
    ctx.fillStyle = 'white';
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, 2*Math.PI);
    ctx.fill();
    ctx.restore();

    // Draw scale indicator at bottom-left
    const indicatorLengthMeters = maxCoord <= 5 ? 1 : (maxCoord <= 15 ? 5 : (maxCoord <= 35 ? 10 : 25));
    const indicatorLengthPx = indicatorLengthMeters * scale;
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(20, h - 30);
    ctx.lineTo(20 + indicatorLengthPx, h - 30);
    ctx.moveTo(20, h - 35);
    ctx.lineTo(20, h - 25);
    ctx.moveTo(20 + indicatorLengthPx, h - 35);
    ctx.lineTo(20 + indicatorLengthPx, h - 25);
    ctx.stroke();

    ctx.fillStyle = 'rgba(255, 255, 255, 0.4)';
    ctx.font = '10px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`${indicatorLengthMeters}m`, 25, h - 38);
}

// Start connections
connectTelemetry();
fetchAndRenderMissions();
setInterval(updateStatus, 1000); // Check status every second
drawRadar(); // Start animation loop

// Log tabs switching and polling logic
const logTabs = document.querySelectorAll('.log-tab');
let activeLogType = 'gcs';

logTabs.forEach(tab => {
    tab.addEventListener('click', () => {
        logTabs.forEach(t => t.classList.remove('active'));
        tab.classList.add('active');
        activeLogType = tab.dataset.type;

        // Hide all log content sections
        document.getElementById('logs-console').style.display = 'none';
        document.getElementById('logs-control-container').style.display = 'none';
        document.getElementById('logs-simulation-container').style.display = 'none';

        // Show selected section
        if (activeLogType === 'gcs') {
            document.getElementById('logs-console').style.display = 'block';
        } else if (activeLogType === 'control') {
            document.getElementById('logs-control-container').style.display = 'block';
            updateConsoleLogs('control', 'logs-control');
        } else if (activeLogType === 'simulation') {
            document.getElementById('logs-simulation-container').style.display = 'block';
            updateConsoleLogs('simulation', 'logs-simulation');
        }
    });
});

async function updateConsoleLogs(type, elementId) {
    if (activeLogType !== type) return;
    try {
        const response = await fetch(`/api/logs?type=${type}`);
        if (response.ok) {
            const text = await response.text();
            const el = document.getElementById(elementId);
            if (el) {
                el.textContent = text;
                // Auto scroll to bottom
                el.parentNode.scrollTop = el.parentNode.scrollHeight;
            }
        }
    } catch (e) {
        console.error("Error fetching logs:", e);
    }
}

// Poll active console log tab every second
setInterval(() => {
    if (activeLogType === 'control') {
        updateConsoleLogs('control', 'logs-control');
    } else if (activeLogType === 'simulation') {
        updateConsoleLogs('simulation', 'logs-simulation');
    }
}, 1000);

// Maximize/minimize button toggle logic
const btnToggleExpand = document.getElementById('btn-toggle-expand');
const systemLogsPanel = document.getElementById('system-logs-panel');
const expandIcon = document.getElementById('expand-icon');

btnToggleExpand.addEventListener('click', () => {
    systemLogsPanel.classList.toggle('expanded');
    if (systemLogsPanel.classList.contains('expanded')) {
        expandIcon.className = 'fa-solid fa-compress';
    } else {
        expandIcon.className = 'fa-solid fa-expand';
    }
});
