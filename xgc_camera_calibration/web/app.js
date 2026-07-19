"use strict";

const ui = {
  canvas: document.getElementById("camera-canvas"),
  placeholder: document.getElementById("camera-placeholder"),
  modeChip: document.getElementById("mode-chip"),
  inputChip: document.getElementById("input-chip"),
  error: document.getElementById("error-banner"),
  success: document.getElementById("success-banner"),
  frameMeta: document.getElementById("frame-meta"),
  coordinateHint: document.getElementById("coordinate-hint"),
  markerSelect: document.getElementById("marker-select"),
  pointsBody: document.getElementById("points-body"),
  result: document.getElementById("result-box"),
  freeze: document.getElementById("freeze-button"),
  live: document.getElementById("live-button"),
  remove: document.getElementById("remove-button"),
  clear: document.getElementById("clear-button"),
  solve: document.getElementById("solve-button"),
  imageTopic: document.getElementById("image-topic"),
  infoTopic: document.getElementById("info-topic"),
  posePrefix: document.getElementById("pose-prefix"),
  outputFile: document.getElementById("output-file"),
};

const context = ui.canvas.getContext("2d");
const state = {
  server: null,
  frameImage: null,
  points: [],
  projections: [],
  busy: false,
  frameLoading: false,
  stateLoading: false,
  liveTimer: null,
  stateTimer: null,
  restoredResultGeneration: null,
};

function showBanner(element, message) {
  element.textContent = message || "";
  element.classList.toggle("hidden", !message);
}

function clearMessages() {
  showBanner(ui.error, "");
  showBanner(ui.success, "");
}

async function api(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options });
  const contentType = response.headers.get("content-type") || "";
  const payload = contentType.includes("application/json") ? await response.json() : null;
  if (!response.ok) {
    throw new Error(payload && payload.error ? payload.error : `Request failed (${response.status})`);
  }
  return payload;
}

async function post(path, body = {}) {
  return api(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

function setBusy(busy) {
  state.busy = busy;
  renderControls();
}

function availableMarkers() {
  if (!state.server || state.server.mode !== "frozen") return [];
  const used = new Set(state.points.map((point) => point.marker));
  return state.server.markers.map((marker) => marker.name).filter((name) => !used.has(name));
}

function renderMarkerSelect() {
  const previous = ui.markerSelect.value;
  const names = availableMarkers();
  ui.markerSelect.replaceChildren();
  if (!names.length) {
    const option = document.createElement("option");
    option.textContent = state.server && state.server.mode === "frozen" ? "All markers selected" : "Freeze a frame first";
    option.value = "";
    ui.markerSelect.appendChild(option);
  } else {
    for (const name of names) {
      const option = document.createElement("option");
      option.value = name;
      option.textContent = name;
      ui.markerSelect.appendChild(option);
    }
    if (names.includes(previous)) ui.markerSelect.value = previous;
  }
  ui.markerSelect.disabled = state.busy || !names.length;
}

function renderTable() {
  ui.pointsBody.replaceChildren();
  if (!state.points.length) {
    const row = document.createElement("tr");
    const cell = document.createElement("td");
    cell.colSpan = 4;
    cell.className = "empty";
    cell.textContent = "No correspondences";
    row.appendChild(cell);
    ui.pointsBody.appendChild(row);
    return;
  }
  for (const point of state.points) {
    const row = document.createElement("tr");
    const error = point.error == null ? "—" : `${point.error.toFixed(2)} px`;
    for (const value of [point.marker, point.pixel[0].toFixed(1), point.pixel[1].toFixed(1), error]) {
      const cell = document.createElement("td");
      cell.textContent = value;
      row.appendChild(cell);
    }
    ui.pointsBody.appendChild(row);
  }
}

function renderControls() {
  const frozen = state.server && state.server.mode === "frozen";
  ui.freeze.disabled = state.busy || !state.server || !state.server.source.image_ready;
  ui.live.disabled = state.busy || !frozen;
  ui.remove.disabled = state.busy || !state.points.length;
  ui.clear.disabled = state.busy || !state.points.length;
  ui.solve.disabled = state.busy || !frozen || state.points.length < 4;
  renderMarkerSelect();
  renderTable();
}

function fitCanvas() {
  const image = state.frameImage;
  if (!image) return;
  const bounds = ui.canvas.parentElement.getBoundingClientRect();
  const ratio = Math.min(bounds.width / image.naturalWidth, bounds.height / image.naturalHeight);
  const cssWidth = Math.max(1, Math.floor(image.naturalWidth * ratio));
  const cssHeight = Math.max(1, Math.floor(image.naturalHeight * ratio));
  const deviceRatio = window.devicePixelRatio || 1;
  ui.canvas.width = Math.floor(cssWidth * deviceRatio);
  ui.canvas.height = Math.floor(cssHeight * deviceRatio);
  ui.canvas.style.width = `${cssWidth}px`;
  ui.canvas.style.height = `${cssHeight}px`;
  context.setTransform(deviceRatio, 0, 0, deviceRatio, 0, 0);
  context.clearRect(0, 0, cssWidth, cssHeight);
  context.drawImage(image, 0, 0, cssWidth, cssHeight);

  const scaleX = cssWidth / image.naturalWidth;
  const scaleY = cssHeight / image.naturalHeight;
  context.font = "12px ui-monospace, monospace";
  context.lineWidth = 2;
  for (const projection of state.projections) {
    drawPoint(projection.marker, projection.pixel[0] * scaleX, projection.pixel[1] * scaleY, "#4ce691", 6);
  }
  for (const point of state.points) {
    drawPoint(point.marker, point.pixel[0] * scaleX, point.pixel[1] * scaleY, "#ff5d67", 7);
  }
}

function drawPoint(label, x, y, color, radius) {
  context.strokeStyle = color;
  context.fillStyle = color;
  context.beginPath();
  context.arc(x, y, radius, 0, Math.PI * 2);
  context.stroke();
  context.fillText(label, x + radius + 4, y - radius - 2);
}

async function loadFrame() {
  if (state.frameLoading) return;
  state.frameLoading = true;
  const image = new Image();
  image.decoding = "async";
  image.src = `/api/v1/image.jpg?t=${Date.now()}`;
  try {
    await image.decode();
  } catch (_error) {
    state.frameLoading = false;
    return;
  }
  state.frameImage = image;
  ui.placeholder.classList.add("hidden");
  fitCanvas();
  state.frameLoading = false;
}

function renderState(serverState) {
  state.server = serverState;
  if (
    serverState.mode === "frozen" &&
    serverState.result &&
    state.restoredResultGeneration !== serverState.generation
  ) {
    state.points = (serverState.result.points || []).map((point) => ({
      marker: point.marker,
      pixel: point.pixel,
      error: point.reprojection_error_px,
    }));
    state.projections = serverState.result.projections || [];
    ui.result.textContent = formatResult(serverState.result, serverState);
    state.restoredResultGeneration = serverState.generation;
  }
  const source = serverState.source;
  const ready = source.image_ready && source.camera_info_ready && source.marker_count > 0;
  ui.modeChip.textContent = serverState.mode === "frozen" ? "Frozen" : "Live";
  ui.modeChip.classList.toggle("muted", serverState.mode !== "frozen");
  ui.inputChip.textContent = ready ? `${source.marker_count} pose markers` : "Waiting for ROS inputs";
  ui.inputChip.classList.toggle("muted", !ready);
  ui.imageTopic.textContent = source.image_topic;
  ui.infoTopic.textContent = source.camera_info_topic;
  ui.posePrefix.textContent = source.pose_prefix;
  ui.outputFile.textContent = serverState.output_file;
  if (serverState.frame) {
    ui.frameMeta.textContent = `${serverState.frame.width}×${serverState.frame.height} · t=${serverState.frame.stamp_sec.toFixed(3)}`;
    ui.coordinateHint.textContent = `${serverState.markers.length} synchronized markers; select a marker and click its center.`;
  } else {
    ui.frameMeta.textContent = source.image_ready ? "Live camera" : "No frame";
    ui.coordinateHint.textContent = "Freeze a synchronized frame before selecting points.";
  }
  renderControls();
}

async function refreshState() {
  if (state.stateLoading) return;
  state.stateLoading = true;
  try {
    const serverState = await api("/api/v1/state");
    const needsFrozenFrame = serverState.mode === "frozen" && (
      !state.frameImage ||
      !state.server ||
      state.server.mode !== "frozen" ||
      state.server.generation !== serverState.generation
    );
    renderState(serverState);
    if (needsFrozenFrame) await loadFrame();
  } catch (error) {
    showBanner(ui.error, error.message);
    ui.modeChip.textContent = "Disconnected";
    ui.inputChip.textContent = "Backend unavailable";
  } finally {
    state.stateLoading = false;
  }
}

async function freezeFrame() {
  clearMessages();
  setBusy(true);
  try {
    const serverState = await post("/api/v1/freeze");
    state.points = [];
    state.projections = [];
    state.restoredResultGeneration = serverState.generation;
    renderState(serverState);
    await loadFrame();
  } catch (error) {
    showBanner(ui.error, error.message);
  } finally {
    setBusy(false);
  }
}

async function liveFrame() {
  clearMessages();
  setBusy(true);
  try {
    renderState(await post("/api/v1/live"));
    state.points = [];
    state.projections = [];
    state.restoredResultGeneration = null;
    ui.result.textContent = "Select at least four markers.";
  } catch (error) {
    showBanner(ui.error, error.message);
  } finally {
    setBusy(false);
  }
}

function clearPoints() {
  state.points = [];
  state.projections = [];
  state.restoredResultGeneration = state.server ? state.server.generation : null;
  ui.result.textContent = "Select at least four markers.";
  renderControls();
  fitCanvas();
}

async function solve() {
  clearMessages();
  setBusy(true);
  try {
    const result = await post("/api/v1/solve", {
      generation: state.server.generation,
      points: state.points.map((point) => ({ marker: point.marker, pixel: point.pixel })),
    });
    state.projections = result.projections || [];
    const pointByName = new Map((result.points || []).map((point) => [point.marker, point]));
    for (const point of state.points) {
      const solved = pointByName.get(point.marker);
      point.error = solved ? solved.reprojection_error_px : null;
    }
    ui.result.textContent = formatResult(result, state.server);
    state.restoredResultGeneration = state.server.generation;
    showBanner(ui.success, `Calibration saved to ${result.output_file}`);
    renderControls();
    fitCanvas();
  } catch (error) {
    showBanner(ui.error, error.message);
  } finally {
    setBusy(false);
  }
}

function formatResult(result, serverState) {
  return [
    `${serverState.parent_frame} → ${serverState.child_frame}`,
    `xyz [${result.translation.map((value) => value.toFixed(6)).join(", ")}]`,
    `q_xyzw [${result.quaternion_xyzw.map((value) => value.toFixed(6)).join(", ")}]`,
    `mean ${result.mean_reprojection_error_px.toFixed(3)} px, max ${result.max_reprojection_error_px.toFixed(3)} px`,
    ...(result.warnings || []),
  ].join("\n");
}

ui.canvas.addEventListener("click", (event) => {
  if (state.busy || !state.server || state.server.mode !== "frozen" || !state.frameImage) return;
  const marker = ui.markerSelect.value;
  if (!marker) return;
  const bounds = ui.canvas.getBoundingClientRect();
  const x = (event.clientX - bounds.left) * state.frameImage.naturalWidth / bounds.width;
  const y = (event.clientY - bounds.top) * state.frameImage.naturalHeight / bounds.height;
  state.points.push({ marker, pixel: [x, y], error: null });
  state.projections = [];
  renderControls();
  fitCanvas();
});

ui.freeze.addEventListener("click", freezeFrame);
ui.live.addEventListener("click", liveFrame);
ui.remove.addEventListener("click", () => {
  state.points.pop();
  state.projections = [];
  renderControls();
  fitCanvas();
});
ui.clear.addEventListener("click", clearPoints);
ui.solve.addEventListener("click", solve);
window.addEventListener("resize", fitCanvas);

async function tickLiveFrame() {
  if (state.server && state.server.mode === "live" && !state.busy) await loadFrame();
}

refreshState();
state.stateTimer = window.setInterval(refreshState, 1000);

/* ---------------- Intrinsic mode ---------------- */
const intrinsic = {
  view: document.getElementById("intrinsic-view"),
  extView: document.getElementById("extrinsic-view"),
  image: document.getElementById("intrinsic-image"),
  bars: document.getElementById("intrinsic-bars"),
  samples: document.getElementById("intrinsic-samples"),
  calibrate: document.getElementById("intrinsic-calibrate"),
  reset: document.getElementById("intrinsic-reset"),
  result: document.getElementById("intrinsic-result"),
  status: document.getElementById("intrinsic-status"),
};
let activeMode = "intrinsic";

function setMode(next) {
  activeMode = next;
  document.querySelectorAll(".tab[data-mode]").forEach((tab) =>
    tab.classList.toggle("active", tab.dataset.mode === next));
  intrinsic.view.classList.toggle("hidden", next !== "intrinsic");
  intrinsic.extView.classList.toggle("hidden", next !== "extrinsic");
  if (next === "intrinsic") { pollIntrinsic(); refreshIntrinsicImage(); }
}

function renderIntrinsicBars(bars) {
  if (intrinsic.bars.childElementCount !== bars.length) {
    intrinsic.bars.innerHTML = bars.map((bar) =>
      `<div class="bar"><div class="bar-label"><span>${bar.label}</span><span class="pct"></span></div>` +
      `<div class="bar-track"><div class="bar-fill"></div></div></div>`).join("");
  }
  bars.forEach((bar, index) => {
    const wrap = intrinsic.bars.children[index];
    const pct = Math.round(bar.progress * 100);
    const fill = wrap.querySelector(".bar-fill");
    fill.style.width = Math.min(100, pct) + "%";
    fill.classList.toggle("full", bar.progress >= 1);
    wrap.querySelector(".pct").textContent = pct + "%";
  });
}

async function pollIntrinsic() {
  if (activeMode !== "intrinsic") return;
  let snapshot;
  try {
    snapshot = await api("/api/v1/intrinsic/state");
  } catch (error) {
    intrinsic.status.textContent = "Intrinsic calibration unavailable";
    return;
  }
  intrinsic.samples.textContent = `${snapshot.samples} samples`;
  renderIntrinsicBars(snapshot.coverage || []);
  intrinsic.calibrate.disabled = !(snapshot.goodenough && !snapshot.calibrated);
  if (snapshot.result) {
    const r = snapshot.result;
    intrinsic.result.hidden = false;
    intrinsic.result.textContent =
      `fx = ${r.fx.toFixed(2)}   fy = ${r.fy.toFixed(2)}\n` +
      `cx = ${r.cx.toFixed(2)}   cy = ${r.cy.toFixed(2)}\n` +
      `rms = ${r.rms_reprojection_error_px.toFixed(3)} px   (${r.sample_count} samples)\n` +
      `saved: ${r.output_file}`;
  } else {
    intrinsic.result.hidden = true;
  }
}

function refreshIntrinsicImage() {
  if (activeMode !== "intrinsic") return;
  intrinsic.image.src = "/api/v1/intrinsic/image.jpg?t=" + Date.now();
}

intrinsic.calibrate.addEventListener("click", async () => {
  intrinsic.status.textContent = "Calibrating…";
  try {
    await post("/api/v1/intrinsic/calibrate");
    intrinsic.status.textContent = "Calibrated and saved.";
  } catch (error) {
    intrinsic.status.textContent = error.message;
  }
  pollIntrinsic();
});
intrinsic.reset.addEventListener("click", async () => {
  try {
    await post("/api/v1/intrinsic/reset");
    intrinsic.status.textContent = "Coverage reset.";
  } catch (error) {
    intrinsic.status.textContent = error.message;
  }
  pollIntrinsic();
});
document.querySelectorAll(".tab[data-mode]").forEach((tab) =>
  tab.addEventListener("click", () => setMode(tab.dataset.mode)));

window.setInterval(pollIntrinsic, 700);
window.setInterval(refreshIntrinsicImage, 500);
setMode("intrinsic");
state.liveTimer = window.setInterval(tickLiveFrame, 500);
