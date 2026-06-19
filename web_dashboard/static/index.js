// State management
let currentTelemetry = null;
let selectedMission = null; // 'circle' or 'marsupial'
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
const missionCircle = document.getElementById('mission-circle');
const missionMarsupial = document.getElementById('mission-marsupial');
const missionBoatMpc = document.getElementById('mission-boat-mpc');
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
    logsConsole.innerHTML = '';
    addLog('Logs limpos.', 'info');
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
            statusSim.textContent = 'Ativa';
            btnStartSim.disabled = true;
            btnStopSim.disabled = false;
        } else {
            statusSim.className = 'badge badge-inactive';
            statusSim.textContent = 'Inativa';
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
            statusMission.textContent = 'Nenhuma';
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
    const typeSim = simType.value;
    const useTether = simTether.checked;
    
    if (typeSim === 'cooperative') {
        addLog(`A iniciar simulação cooperativa (Cabo Tether: ${useTether ? 'Sim' : 'Não'})...`, 'info');
        postAPI('/api/launch', { type: 'simulation', sim_type: 'cooperative', use_tether: useTether });
    } else {
        addLog('A iniciar simulação individual do barco (sem drone)...', 'info');
        postAPI('/api/launch', { type: 'simulation', sim_type: 'individual' });
    }
});

// Show/Hide tether options based on simulation type
simType.addEventListener('change', () => {
    if (simType.value === 'individual') {
        tetherGroup.style.display = 'none';
    } else {
        tetherGroup.style.display = 'block';
    }
});

btnStopSim.addEventListener('click', () => {
    addLog('A parar simulação...', 'info');
    postAPI('/api/stop', { type: 'simulation' });
});

btnLaunchMission.addEventListener('click', () => {
    if (!selectedMission) {
        addLog('Escolha uma missão primeiro.', 'warning');
        return;
    }
    addLog(`A lançar missão: ${selectedMission.toUpperCase()}...`, 'info');
    postAPI('/api/launch', { type: 'mission', name: selectedMission });
});

btnStopMission.addEventListener('click', () => {
    addLog('A parar missão ativa...', 'info');
    postAPI('/api/stop', { type: 'mission' });
});

btnEmergency.addEventListener('click', () => {
    addLog('!!! PARAGEM DE EMERGÊNCIA SOLICITADA !!!', 'error');
    postAPI('/api/emergency_stop');
});

// Mission selector logic
missionCircle.addEventListener('click', () => {
    if (activeStatus.mission_running) return;
    selectedMission = 'circle';
    missionCircle.classList.add('selected');
    missionMarsupial.classList.remove('selected');
    btnLaunchMission.disabled = !activeStatus.simulation_running;
    addLog('Missão selecionada: Trajetória Circular.', 'info');
});

missionMarsupial.addEventListener('click', () => {
    if (activeStatus.mission_running) return;
    selectedMission = 'marsupial';
    missionMarsupial.classList.add('selected');
    missionCircle.classList.remove('selected');
    missionBoatMpc.classList.remove('selected');
    btnLaunchMission.disabled = !activeStatus.simulation_running;
    addLog('Missão selecionada: Seguimento de Barco.', 'info');
});

missionBoatMpc.addEventListener('click', () => {
    if (activeStatus.mission_running) return;
    selectedMission = 'boat_mpc';
    missionBoatMpc.classList.add('selected');
    missionCircle.classList.remove('selected');
    missionMarsupial.classList.remove('selected');
    btnLaunchMission.disabled = !activeStatus.simulation_running;
    addLog('Missão selecionada: MPC Individual Barco.', 'info');
});

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
        addLog('GCS Web Server conectado.', 'success');
        updateStatus();
    };

    eventSource.onerror = (err) => {
        connectionStatus.querySelector('.status-dot').className = 'status-dot disconnected';
        connectionStatus.querySelector('.status-label').textContent = 'Disconnected';
        addLog('GCS Web Server desconectado. A tentar reconectar...', 'warning');
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
        ctx.fillText('AGUARDANDO TELEMETRIA', cx, cy);
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
setInterval(updateStatus, 1000); // Check status every second
drawRadar(); // Start animation loop
