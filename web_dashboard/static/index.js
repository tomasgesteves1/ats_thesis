// Dashboard of the tethered UAV-USV system: polls /api/state and draws a top view of the vehicles.
const STRUCTURES = [{ name: 'radio tower', x: -25, y: 0, r: 6.5 }];   // static objects of the world (m)
const $ = (id) => document.getElementById(id);
let missions = [];
let selected = null;
let view = null;          // current map window {cx, cy, span}
let last = null;

async function api(path, body) {
    const opts = body === undefined ? {} : { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                             body: JSON.stringify(body) };
    const r = await fetch(path, opts);
    const data = await r.json();
    if (!r.ok || data.ok === false) { alert(data.error || `request failed: ${path}`); }
    return data;
}

// ------------------------------------------------------------------------------------------------- missions
async function loadMissions() {
    missions = await api('/api/missions');
    const list = $('mission-list');
    list.innerHTML = '';
    for (const m of missions) {
        const li = document.createElement('li');
        li.innerHTML = `${m.name || m.id}<small>${m.id}</small>`;
        li.onclick = () => selectMission(m.id);
        li.dataset.id = m.id;
        list.appendChild(li);
    }
    if (missions.length && !selected) { selectMission(missions[0].id); }
}

function selectMission(id) {
    selected = missions.find((m) => m.id === id);
    document.querySelectorAll('#mission-list li').forEach((li) => li.classList.toggle('selected', li.dataset.id === id));
    $('mission-detail').classList.remove('hidden');
    $('md-name').textContent = selected.name;
    $('md-desc').textContent = selected.description;
    const chips = (steps) => steps.map((s) => `<span class="step">${s.name}</span>`).join('') || '<span class="muted small">none</span>';
    $('md-uav').innerHTML = chips(selected.uav);
    $('md-usv').innerHTML = chips(selected.usv);
    const opts = Object.entries(selected.launch).map(([k, v]) => `${k}=${v}`).join(', ');
    $('md-launch').textContent = opts ? `launch options: ${opts}` : '';
}

// ------------------------------------------------------------------------------------------------- state
const fmt = (v, d = 1, unit = '') => (v === undefined || v === null || Number.isNaN(v)) ? '–' : `${v.toFixed(d)}${unit}`;
// a value is fresh if it is less than 2 s older than the server clock (the browser clock may differ)
const fresh = (o) => o && o.stamp && last && (last.now - o.stamp) < 2.0;

function fillDl(id, rows) {
    $(id).innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

function render(s) {
    const t = s.telemetry;
    const sim = s.processes.sim;
    const simLive = fresh(t.sim);
    const chip = $('sim-chip');
    chip.className = 'chip ' + (simLive ? 'on' : sim.running ? 'starting' : 'off');
    chip.textContent = simLive ? 'simulation running' : sim.running ? 'starting…' : 'simulation off';
    $('sim-time').textContent = simLive ? `t = ${fmt(t.sim.time, 0, ' s')} · RTF ${fmt(t.sim.rtf, 2)}` : '';
    $('sim-start').disabled = sim.running;
    $('sim-stop').disabled = !sim.running;

    const missionRunning = s.processes.mission.running;
    $('mission-start').disabled = !simLive || missionRunning;
    $('mission-abort').disabled = !missionRunning || s.mission_state !== 'running';
    $('mission-stop').disabled = !missionRunning;
    const st = s.mission_state;
    $('mission-status').innerHTML = s.mission ? `
        <div>Mission <b>${s.mission}</b>: <span class="state-${st.split(' ')[0]}">${st}</span></div>
        ${s.bag ? `<div class="muted small">bag: ${s.bag}${s.processes.bag.running ? ' (recording)' : ''}</div>` : ''}` : '';

    const u = t.uav, b = t.usv, th = t.tether, ix = t.intruder;
    fillDl('t-uav', [['mode', `${u.mode || '–'} ${u.armed ? '· armed' : ''}`], ['position', `${fmt(u.x)}, ${fmt(u.y)}`],
                     ['height', fmt(u.z, 2, ' m')], ['speed', fmt(u.speed, 2, ' m/s')], ['climb', fmt(u.vz, 2, ' m/s')],
                     ['battery', fmt(u.battery, 0, ' %')]]);
    fillDl('t-usv', [['position', `${fmt(b.x)}, ${fmt(b.y)}`], ['heading', fmt(b.yaw, 0, '°')],
                     ['speed', fmt(b.speed, 2, ' m/s')], ['surge', fmt(b.surge, 2, ' m/s')]]);
    fillDl('t-tether', [['length', fmt(th.length, 2, ' m')], ['distance', fmt(th.distance, 2, ' m')],
                        ['force UAV', fmt(th.force_uav, 2, ' N')], ['winch pull', th.pull ? 'on' : 'off']]);
    fillDl('t-intruder', fresh(ix) ? [['position', `${fmt(ix.x)}, ${fmt(ix.y)}`], ['speed', fmt(ix.speed, 2, ' m/s')],
                                      ['distance USV', fresh(b) ? fmt(Math.hypot(ix.x - b.x, ix.y - b.y), 1, ' m') : '–']]
                                    : [['', 'not present']]);

    renderProgress(s);
    drawMap(t);
}

// ------------------------------------------------------------------------------------------------- progress
// Maneuvers of the running mission as blocks: done, current and next
function renderProgress(s) {
    const m = missions.find((x) => x.id === s.mission);
    if (!m) { $('progress').innerHTML = '<div class="muted small">no mission</div>'; return; }
    const complete = s.mission_state === 'complete';
    const row = (who, label) => {
        const current = s.steps[who] || '';
        const abort = current.endsWith('(abort)');
        // the blocks before the current one are done (the first events can be missed while the topic is discovered)
        const at = m[who].findIndex((b) => b.ids.includes(current));
        let blocks = m[who].map((b, i) => {
            let cls = 'next';
            if (complete || (at >= 0 && i < at) || b.ids.every((id) => s.done[who].includes(id))) { cls = 'done'; }
            else if (!abort && i === at) { cls = 'current'; }
            return `<span class="block ${cls}">${b.name}</span>`;
        });
        if (abort) { blocks.push(`<span class="block current abort">${who === 'uav' ? 'landing' : 'hold'} (abort)</span>`); }
        return `<div class="progress-row"><span class="tag">${label}</span>${blocks.join('<span class="arrow">›</span>')}</div>`;
    };
    $('progress').innerHTML = row('uav', 'UAV') + row('usv', 'USV');
}

// ------------------------------------------------------------------------------------------------- map
function drawMap(t) {
    const cv = $('map');
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
    const g = cv.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);

    // window: fits the vehicles (and the paths), at least 40 m, moved smoothly
    const pts = [];
    for (const o of [t.usv, t.uav, t.intruder]) { if (fresh(o)) pts.push([o.x, o.y]); }
    for (const k of ['uav', 'usv']) { (t.paths[k] || []).forEach((p) => pts.push(p)); }
    if (!pts.length) { pts.push([0, 0]); }
    const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
    const target = { cx: (Math.min(...xs) + Math.max(...xs)) / 2, cy: (Math.min(...ys) + Math.max(...ys)) / 2,
                     span: Math.max(40, 1.4 * Math.max(Math.max(...xs) - Math.min(...xs), Math.max(...ys) - Math.min(...ys))) };
    view = view ? { cx: view.cx + 0.15 * (target.cx - view.cx), cy: view.cy + 0.15 * (target.cy - view.cy),
                    span: view.span + 0.1 * (target.span - view.span) } : target;
    const k = Math.min(w, h) / view.span;
    const X = (x) => w / 2 + (x - view.cx) * k;   // world x to the right
    const Y = (y) => h / 2 - (y - view.cy) * k;   // world y up

    // grid every 10 m
    g.strokeStyle = '#1b2430'; g.lineWidth = 1; g.font = '10px system-ui'; g.fillStyle = '#4c596b';
    const x0 = Math.floor((view.cx - w / 2 / k) / 10) * 10, y0 = Math.floor((view.cy - h / 2 / k) / 10) * 10;
    for (let x = x0; x < view.cx + w / 2 / k; x += 10) { g.beginPath(); g.moveTo(X(x), 0); g.lineTo(X(x), h); g.stroke(); g.fillText(x, X(x) + 2, h - 4); }
    for (let y = y0; y < view.cy + h / 2 / k; y += 10) { g.beginPath(); g.moveTo(0, Y(y)); g.lineTo(w, Y(y)); g.stroke(); g.fillText(y, 2, Y(y) - 2); }

    const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
    for (const s of STRUCTURES) {
        g.fillStyle = css('--tower') + '55'; g.strokeStyle = css('--tower');
        g.beginPath(); g.arc(X(s.x), Y(s.y), s.r * k, 0, 2 * Math.PI); g.fill(); g.stroke();
    }
    // reference paths
    g.setLineDash([4, 4]); g.lineWidth = 1.5;
    for (const [key, color] of [['usv', '--usv'], ['uav', '--uav']]) {
        const p = t.paths[key] || [];
        if (p.length < 2) continue;
        g.strokeStyle = css(color); g.beginPath(); g.moveTo(X(p[0][0]), Y(p[0][1]));
        p.forEach((q) => g.lineTo(X(q[0]), Y(q[1]))); g.stroke();
    }
    g.setLineDash([]);

    const boat = (o, color, len) => {
        const a = -o.yaw * Math.PI / 180, L = Math.max(len * k, 8), W = L * 0.45;
        g.save(); g.translate(X(o.x), Y(o.y)); g.rotate(a); g.fillStyle = color;
        g.beginPath(); g.moveTo(L / 2, 0); g.lineTo(-L / 2, W / 2); g.lineTo(-L / 2, -W / 2); g.closePath(); g.fill();
        g.restore();
    };
    if (fresh(t.usv) && fresh(t.uav)) {   // tether: winch (boat centre) to the UAV
        g.strokeStyle = css('--tether'); g.lineWidth = 1.5;
        g.beginPath(); g.moveTo(X(t.usv.x), Y(t.usv.y)); g.lineTo(X(t.uav.x), Y(t.uav.y)); g.stroke();
    }
    if (fresh(t.intruder)) { boat(t.intruder, css('--intruder'), 5); }
    if (fresh(t.usv)) { boat(t.usv, css('--usv'), 5); }
    if (fresh(t.uav)) {
        g.fillStyle = css('--uav'); g.beginPath(); g.arc(X(t.uav.x), Y(t.uav.y), Math.max(0.5 * k, 5), 0, 2 * Math.PI); g.fill();
        g.fillStyle = css('--text'); g.font = '11px system-ui';
        g.fillText(`${t.uav.z.toFixed(1)} m`, X(t.uav.x) + 8, Y(t.uav.y) - 8);
    }
}

// ------------------------------------------------------------------------------------------------- polling
async function poll() {
    try {
        last = await (await fetch('/api/state')).json();
        render(last);
    } catch (e) { $('sim-chip').textContent = 'dashboard offline'; }
}

async function pollLogs() {
    if (!$('logs').open) return;
    const r = await (await fetch(`/api/logs?name=${$('log-name').value}`)).json();
    const pre = $('log-text');
    const atBottom = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 5;
    pre.textContent = r.text;
    if (atBottom) pre.scrollTop = pre.scrollHeight;
}

$('sim-start').onclick = () => api('/api/sim/start', { gui: $('sim-gui').checked });
$('sim-stop').onclick = () => confirm('Stop the simulation (and the mission)?') && api('/api/sim/stop', {});
$('mission-start').onclick = () => selected && api('/api/mission/start', { id: selected.id, record: $('md-record').checked });
$('mission-abort').onclick = () => api('/api/mission/abort', {});
$('mission-stop').onclick = () => api('/api/mission/stop', {});

loadMissions();
setInterval(poll, 500);
setInterval(pollLogs, 2000);
poll();
