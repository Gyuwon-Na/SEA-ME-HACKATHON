const BATTERY_THRESHOLDS = {
  good: 70,
  caution: 35,
};

const STORAGE_THRESHOLDS = {
  good: 70,
  caution: 85,
};

// Voltage droop (max-min over the rolling window) that flags a weak/sagging
// supply even when the absolute brownout line has not been crossed.
const POWER_DROOP_CAUTION = 0.8;

const CARD_CLASSES = [
  'is-good',
  'is-live',
  'is-caution',
  'is-low',
  'is-waiting',
  'is-stale',
  'is-offline',
];

const config = window.MONITOR_CONFIG || {
  statusEndpoint: '/api/status',
  graphEndpoint: '/api/graph',
  frameEndpoint: '/api/frame',
  debugFrameGrayscaleEndpoint: '/api/frame/grayscale',
  debugFrameBlurEndpoint: '/api/frame/blur',
  debugFrameEdgeEndpoint: '/api/frame/edge',
  debugImageEnabled: false,
  placeholderUrl: '/api/frame/placeholder',
  refreshIntervalMs: 1000,
  imageRefreshIntervalMs: 150,
};

const elements = {
  batteryCard: document.getElementById('battery-card'),
  batteryChip: document.getElementById('battery-chip'),
  batteryValue: document.getElementById('battery-value'),
  batteryUpdated: document.getElementById('battery-updated'),
  batteryMeterFill: document.getElementById('battery-meter-fill'),
  imageCard: document.getElementById('image-card'),
  imageChip: document.getElementById('image-chip'),
  imageUpdated: document.getElementById('image-updated'),
  imageResolution: document.getElementById('image-resolution'),
  cameraFrame: document.getElementById('camera-frame'),
  debugFrameGrayscale: document.getElementById('debug-frame-grayscale'),
  debugFrameBlur: document.getElementById('debug-frame-blur'),
  debugFrameEdge: document.getElementById('debug-frame-edge'),
  recordBadge: document.getElementById('record-badge'),
  recordBadgeLabel: document.getElementById('record-badge-label'),
  controlCard: document.getElementById('control-card'),
  controlChip: document.getElementById('control-chip'),
  controlUpdated: document.getElementById('control-updated'),
  throttleBar: document.getElementById('throttle-bar'),
  throttleBarFill: document.getElementById('throttle-bar-fill'),
  throttleValue: document.getElementById('throttle-value'),
  steeringBar: document.getElementById('steering-bar'),
  steeringBarFill: document.getElementById('steering-bar-fill'),
  steeringValue: document.getElementById('steering-value'),
  storageCard: document.getElementById('storage-card'),
  storageChip: document.getElementById('storage-chip'),
  storageValue: document.getElementById('storage-value'),
  storageUpdated: document.getElementById('storage-updated'),
  storageMeterFill: document.getElementById('storage-meter-fill'),
  storageDetail: document.getElementById('storage-detail'),
  graphCard: document.getElementById('graph-card'),
  graphChip: document.getElementById('graph-chip'),
  graphCanvas: document.getElementById('ros-graph-canvas'),
  graphUpdated: document.getElementById('graph-updated'),
  graphSummary: document.getElementById('graph-summary'),
  powerCard: document.getElementById('power-card'),
  powerChip: document.getElementById('power-chip'),
  powerVoltage: document.getElementById('power-voltage'),
  powerWatt: document.getElementById('power-watt'),
  powerCurrent: document.getElementById('power-current'),
  powerChart: document.getElementById('power-chart'),
  powerBaseline: document.getElementById('power-baseline'),
  powerMotor: document.getElementById('power-motor'),
  powerSplitBaseline: document.getElementById('power-split-baseline'),
  powerSplitMotor: document.getElementById('power-split-motor'),
  powerDroop: document.getElementById('power-droop'),
  powerUpdated: document.getElementById('power-updated'),
};

let imageRequestInFlight = false;
let debugImageRequestInFlight = false;

function clampPercent(value) {
  return Math.max(0, Math.min(100, Number(value) || 0));
}

function clampControl(value) {
  return Math.max(-1, Math.min(1, Number(value) || 0));
}

function formatUpdatedAt(updatedAt) {
  if (!updatedAt) {
    return 'Waiting for message';
  }

  const date = new Date(updatedAt);

  if (Number.isNaN(date.getTime())) {
    return 'Invalid timestamp';
  }

  return date.toLocaleString();
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>"']/g, (character) => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;',
  }[character]));
}

function getGraphLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function renderGraphNodeCard(node, role) {
  if (!node) {
    return '<span class="ros-graph__node-card ros-graph__node-card--placeholder">--</span>';
  }

  return `
    <span class="ros-graph__node-card ros-graph__node-card--${role}" title="${escapeHtml(node.label)}">
      ${escapeHtml(node.label || node.id)}
    </span>
  `;
}

function renderGraphEdge(edge, topicNode) {
  const topicLabel = topicNode?.label || edge.topic || 'topic';
  return `
    <div class="ros-graph__row">
      <div class="ros-graph__column ros-graph__column--source">
        ${renderGraphNodeCard(edge.source_node, 'source')}
      </div>
      <div class="ros-graph__edge" aria-label="${escapeHtml(topicLabel)}">
        <span class="ros-graph__edge-label" title="${escapeHtml(topicLabel)}">
          ${escapeHtml(topicLabel)}
        </span>
      </div>
      <div class="ros-graph__column ros-graph__column--target">
        ${renderGraphNodeCard(edge.target_node, 'target')}
      </div>
    </div>
  `;
}

function renderGraphPublisherTopicRow(edge) {
  const topicLabel = edge.topic_node?.label || edge.topic || 'topic';
  return `
    <div class="ros-graph__publisher-topic-row">
      <div class="ros-graph__edge" aria-label="${escapeHtml(topicLabel)}">
        <span class="ros-graph__edge-label" title="${escapeHtml(topicLabel)}">
          ${escapeHtml(topicLabel)}
        </span>
      </div>
      <div class="ros-graph__column ros-graph__column--target">
        ${renderGraphNodeCard(edge.target_node, 'target')}
      </div>
    </div>
  `;
}

function renderGraphPublisherGroup(group) {
  return `
    <div class="ros-graph__row ros-graph__publisher-group">
      <div class="ros-graph__column ros-graph__column--source">
        ${renderGraphNodeCard(group.source_node, 'source')}
      </div>
      <div class="ros-graph__publisher-topic-list">
        ${group.edges.map(renderGraphPublisherTopicRow).join('')}
      </div>
    </div>
  `;
}

function buildGraphRenderItems(rows) {
  const items = [];
  let currentPublisherGroup = null;

  rows.forEach((edge) => {
    if (edge.direction === 'publishes') {
      if (!currentPublisherGroup || currentPublisherGroup.source !== edge.source) {
        currentPublisherGroup = {
          kind: 'publisherGroup',
          source: edge.source,
          source_node: edge.source_node,
          edges: [],
        };
        items.push(currentPublisherGroup);
      }

      currentPublisherGroup.edges.push(edge);
      return;
    }

    currentPublisherGroup = null;
    items.push({ kind: 'edge', edge });
  });

  return items;
}

function renderGraph(payload) {
  if (!elements.graphCard || !elements.graphCanvas) {
    return;
  }

  const nodes = Array.isArray(payload.nodes) ? payload.nodes : [];
  const edges = Array.isArray(payload.edges) ? payload.edges : [];
  const nodeById = new Map(nodes.map((node) => [node.id, node]));
  const rows = edges.map((edge) => ({
    ...edge,
    source_node: nodeById.get(edge.source),
    target_node: nodeById.get(edge.target),
    topic_node: nodeById.get(`topic:${edge.topic}`),
  }));
  const graphState = rows.length > 0 ? 'live' : 'waiting';
  const renderItems = buildGraphRenderItems(rows);
  const rowMarkup = renderItems.map((item) => (
    item.kind === 'publisherGroup'
      ? renderGraphPublisherGroup(item)
      : renderGraphEdge(item.edge, item.edge.topic_node)
  )).join('');

  setCardState(elements.graphCard, graphState);
  elements.graphChip.textContent = getGraphLabel(graphState);
  elements.graphUpdated.textContent = formatUpdatedAt(payload.updated_at);
  elements.graphSummary.textContent = `${nodes.length} nodes / ${edges.length} edges`;
  elements.graphCanvas.innerHTML = rowMarkup || '<p class="ros-graph__placeholder">Waiting for ROS graph</p>';
}

function setCardState(cardElement, state) {
  cardElement.classList.remove(...CARD_CLASSES);
  cardElement.classList.add(`is-${state}`);
}

function getBatteryState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const battery = clampPercent(data.battery_status);

  if (battery >= BATTERY_THRESHOLDS.good) {
    return 'good';
  }

  if (battery >= BATTERY_THRESHOLDS.caution) {
    return 'caution';
  }

  return 'low';
}

function getBatteryLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'LOW';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getImageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  return 'live';
}

function getImageLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getControlState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  return 'live';
}

function getControlLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function getStorageState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  const storageUsed = clampPercent(data.used_percentage);

  if (storageUsed >= STORAGE_THRESHOLDS.caution) {
    return 'low';
  }

  if (storageUsed >= STORAGE_THRESHOLDS.good) {
    return 'caution';
  }

  return 'good';
}

function getStorageLabel(state) {
  switch (state) {
    case 'good':
      return 'GOOD';
    case 'caution':
      return 'CAUTION';
    case 'low':
      return 'HIGH';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function formatControlValue(value, hasData) {
  if (!hasData || value === null || value === undefined) {
    return '--.--';
  }

  return Number(value).toFixed(2);
}

function setControlBar(barElement, fillElement, value, hasData) {
  barElement.classList.remove('is-positive', 'is-negative', 'is-neutral');

  if (!hasData || value === null || value === undefined) {
    barElement.classList.add('is-neutral');
    fillElement.style.left = '50%';
    fillElement.style.width = '0%';
    return;
  }

  const clampedValue = clampControl(value);
  const magnitude = Math.abs(clampedValue) * 50;

  if (magnitude < 0.5) {
    barElement.classList.add('is-neutral');
    fillElement.style.left = '50%';
    fillElement.style.width = '0%';
    return;
  }

  fillElement.style.left = clampedValue >= 0 ? '50%' : `${50 - magnitude}%`;
  fillElement.style.width = `${magnitude}%`;
  barElement.classList.add(clampedValue >= 0 ? 'is-positive' : 'is-negative');
}

function renderBattery(data) {
  const state = getBatteryState(data);
  const batteryPercent = data.has_data ? clampPercent(data.battery_status) : 0;

  setCardState(elements.batteryCard, state);
  elements.batteryChip.textContent = getBatteryLabel(state);
  elements.batteryValue.textContent = data.battery_display || '--.-%';
  elements.batteryUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.batteryMeterFill.style.width = `${batteryPercent}%`;
}

function renderImage(data) {
  const state = getImageState(data);

  setCardState(elements.imageCard, state);
  elements.imageChip.textContent = getImageLabel(state);
  elements.imageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.imageResolution.textContent = data.resolution_display || 'Waiting for frame';
}

function renderControl(data) {
  const state = getControlState(data);
  const hasData = Boolean(data.has_data);

  setCardState(elements.controlCard, state);
  elements.controlChip.textContent = getControlLabel(state);
  elements.controlUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.throttleValue.textContent = formatControlValue(data.throttle, hasData);
  elements.steeringValue.textContent = formatControlValue(data.steering, hasData);

  setControlBar(elements.throttleBar, elements.throttleBarFill, data.throttle, hasData);
  setControlBar(elements.steeringBar, elements.steeringBarFill, data.steering, hasData);
}

function renderRecording(data) {
  const isRecording = Boolean(data.is_recording);

  elements.recordBadge.classList.toggle('is-recording', isRecording);
  elements.recordBadgeLabel.textContent = isRecording ? 'REC ON' : 'REC OFF';
}

function renderStorage(data) {
  const state = getStorageState(data);
  const usedPercent = data.has_data ? clampPercent(data.used_percentage) : 0;

  setCardState(elements.storageCard, state);
  elements.storageChip.textContent = getStorageLabel(state);
  elements.storageValue.textContent = data.used_display || '--.-%';
  elements.storageUpdated.textContent = formatUpdatedAt(data.updated_at);
  elements.storageMeterFill.style.width = `${usedPercent}%`;
  elements.storageDetail.textContent = `${data.used_space_display || '--'} / ${data.total_space_display || '--'} used`;
}

function getPowerState(data) {
  if (!data.has_data) {
    return 'waiting';
  }

  if (data.is_stale) {
    return 'stale';
  }

  if (data.sag_active) {
    return 'low';
  }

  if (data.droop !== null && data.droop !== undefined && data.droop >= POWER_DROOP_CAUTION) {
    return 'caution';
  }

  return 'live';
}

function getPowerLabel(state) {
  switch (state) {
    case 'live':
      return 'LIVE';
    case 'caution':
      return 'SAG';
    case 'low':
      return 'BROWNOUT';
    case 'stale':
      return 'STALE';
    default:
      return 'WAITING';
  }
}

function buildPowerSparkline(history, brownoutVoltage) {
  const voltages = history && Array.isArray(history.v) ? history.v : [];
  const throttles = history && Array.isArray(history.thr) ? history.thr : [];

  if (voltages.length < 2) {
    return '<p class="power-chart__placeholder">Waiting for samples</p>';
  }

  const width = 300;
  const height = 64;
  const pad = 4;
  const hasThreshold = brownoutVoltage !== null && brownoutVoltage !== undefined && Number.isFinite(Number(brownoutVoltage));

  let vMin = Math.min(...voltages);
  let vMax = Math.max(...voltages);
  if (hasThreshold) {
    vMin = Math.min(vMin, Number(brownoutVoltage));
    vMax = Math.max(vMax, Number(brownoutVoltage));
  }
  if (vMax - vMin < 0.1) {
    vMax += 0.05;
    vMin -= 0.05;
  }

  const count = voltages.length;
  const xAt = (index) => pad + (index / (count - 1)) * (width - 2 * pad);
  const yAt = (value) => pad + (1 - (value - vMin) / (vMax - vMin)) * (height - 2 * pad);

  let throttleBars = '';
  const barWidth = (width - 2 * pad) / count;
  for (let i = 0; i < throttles.length; i += 1) {
    const magnitude = Math.min(1, Math.abs(Number(throttles[i]) || 0));
    if (magnitude < 0.02) {
      continue;
    }
    const barHeight = magnitude * (height - 2 * pad);
    throttleBars += `<rect x="${xAt(i).toFixed(1)}" y="${(height - pad - barHeight).toFixed(1)}" width="${barWidth.toFixed(1)}" height="${barHeight.toFixed(1)}" fill="rgba(220,220,170,0.20)"></rect>`;
  }

  const points = voltages
    .map((value, index) => `${xAt(index).toFixed(1)},${yAt(value).toFixed(1)}`)
    .join(' ');

  let thresholdLine = '';
  if (hasThreshold && Number(brownoutVoltage) >= vMin && Number(brownoutVoltage) <= vMax) {
    const yThreshold = yAt(Number(brownoutVoltage)).toFixed(1);
    thresholdLine = `<line x1="${pad}" y1="${yThreshold}" x2="${width - pad}" y2="${yThreshold}" stroke="rgba(244,135,113,0.9)" stroke-width="1" stroke-dasharray="4 3"></line>`;
  }

  return `<svg viewBox="0 0 ${width} ${height}" preserveAspectRatio="none" class="power-chart__svg">${throttleBars}${thresholdLine}<polyline points="${points}" fill="none" stroke="#4ec9b0" stroke-width="1.5"></polyline></svg>`;
}

function renderPower(data) {
  if (!elements.powerCard) {
    return;
  }

  const state = getPowerState(data);
  setCardState(elements.powerCard, state);
  elements.powerChip.textContent = getPowerLabel(state);
  elements.powerVoltage.textContent = data.voltage_display || '--.- V';
  elements.powerWatt.textContent = data.watt_display || '--.- W';
  elements.powerCurrent.textContent = data.current_display || '--- mA';
  elements.powerDroop.textContent = data.droop_display || '-- V';
  elements.powerUpdated.textContent = formatUpdatedAt(data.updated_at);

  const baseline = data.baseline_w === null || data.baseline_w === undefined ? null : Number(data.baseline_w);
  const motor = data.motor_w === null || data.motor_w === undefined ? null : Number(data.motor_w);
  elements.powerBaseline.textContent = baseline === null ? '-- W' : `${baseline.toFixed(2)} W`;
  elements.powerMotor.textContent = motor === null ? '-- W' : `${motor.toFixed(2)} W`;

  const total = (baseline || 0) + (motor || 0);
  if (elements.powerSplitBaseline && elements.powerSplitMotor) {
    if (total > 0) {
      elements.powerSplitBaseline.style.width = `${(100 * (baseline || 0) / total).toFixed(1)}%`;
      elements.powerSplitMotor.style.width = `${(100 * (motor || 0) / total).toFixed(1)}%`;
    } else {
      elements.powerSplitBaseline.style.width = '0%';
      elements.powerSplitMotor.style.width = '0%';
    }
  }

  if (elements.powerChart) {
    elements.powerChart.innerHTML = buildPowerSparkline(data.history, data.brownout_voltage);
  }
}

function renderOffline() {
  setCardState(elements.batteryCard, 'offline');
  setCardState(elements.imageCard, 'offline');
  setCardState(elements.controlCard, 'offline');
  setCardState(elements.storageCard, 'offline');
  if (elements.graphCard) {
    setCardState(elements.graphCard, 'offline');
  }
  if (elements.powerCard) {
    setCardState(elements.powerCard, 'offline');
    elements.powerChip.textContent = 'OFFLINE';
    elements.powerUpdated.textContent = 'Unable to reach monitor server';
  }
  elements.batteryChip.textContent = 'OFFLINE';
  elements.imageChip.textContent = 'OFFLINE';
  elements.controlChip.textContent = 'OFFLINE';
  elements.storageChip.textContent = 'OFFLINE';
  if (elements.graphChip) {
    elements.graphChip.textContent = 'OFFLINE';
  }
  renderRecording({ is_recording: false });
  elements.batteryUpdated.textContent = 'Unable to reach monitor server';
  elements.imageUpdated.textContent = 'Unable to reach monitor server';
  elements.controlUpdated.textContent = 'Unable to reach monitor server';
  elements.storageUpdated.textContent = 'Unable to reach monitor server';
  if (elements.graphUpdated) {
    elements.graphUpdated.textContent = 'Unable to reach monitor server';
  }
}

async function fetchGraph() {
  if (!config.graphEndpoint) {
    return;
  }

  try {
    const response = await fetch(config.graphEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    renderGraph(payload || {});
  } catch (error) {
    console.error('Failed to fetch ROS graph', error);
    if (elements.graphCard) {
      setCardState(elements.graphCard, 'stale');
      elements.graphChip.textContent = 'STALE';
      elements.graphUpdated.textContent = 'Unable to reach graph endpoint';
    }
  }
}

async function fetchStatus() {
  try {
    const response = await fetch(config.statusEndpoint, { cache: 'no-store' });

    if (!response.ok) {
      throw new Error(`Unexpected response: ${response.status}`);
    }

    const payload = await response.json();
    renderBattery(payload.battery || {});
    renderImage(payload.image || {});
    renderControl(payload.control || {});
    renderRecording(payload.recording || {});
    renderStorage(payload.storage || {});
    try {
      renderPower(payload.power || {});
    } catch (powerError) {
      console.error('Failed to render power panel', powerError);
    }
  } catch (error) {
    console.error('Failed to fetch monitor status', error);
    renderOffline();
  }
}

function refreshCameraFrame() {
  if (imageRequestInFlight) {
    return;
  }

  imageRequestInFlight = true;

  const image = new Image();
  image.onload = () => {
    elements.cameraFrame.src = image.src;
    imageRequestInFlight = false;
  };
  image.onerror = () => {
    elements.cameraFrame.src = config.placeholderUrl;
    imageRequestInFlight = false;
  };
  image.src = `${config.frameEndpoint}?t=${Date.now()}`;
}

function refreshImageByEndpoint(targetElement, endpoint) {
  if (!targetElement) {
    return;
  }

  const image = new Image();
  image.onload = () => {
    targetElement.src = image.src;
  };
  image.onerror = () => {
    targetElement.src = config.placeholderUrl;
  };
  image.src = `${endpoint}?t=${Date.now()}`;
}

function refreshDebugFrames() {
  if (!config.debugImageEnabled || debugImageRequestInFlight) {
    return;
  }

  debugImageRequestInFlight = true;
  refreshImageByEndpoint(elements.debugFrameGrayscale, config.debugFrameGrayscaleEndpoint);
  refreshImageByEndpoint(elements.debugFrameBlur, config.debugFrameBlurEndpoint);
  refreshImageByEndpoint(elements.debugFrameEdge, config.debugFrameEdgeEndpoint);
  debugImageRequestInFlight = false;
}

function startPolling() {
  fetchStatus();
  fetchGraph();
  refreshCameraFrame();
  if (config.debugImageEnabled) {
    refreshDebugFrames();
  }
  window.setInterval(fetchStatus, config.refreshIntervalMs);
  window.setInterval(fetchGraph, config.refreshIntervalMs);
  window.setInterval(refreshCameraFrame, config.imageRefreshIntervalMs);
  if (config.debugImageEnabled) {
    window.setInterval(refreshDebugFrames, config.imageRefreshIntervalMs);
  }
}

document.addEventListener('DOMContentLoaded', startPolling);
