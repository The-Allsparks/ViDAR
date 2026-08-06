import { loadTuning, tuningFromUi } from "./config.js";
import { detectBlobs, bestByCategory, computeRoi, maskToCanvas } from "./detection.js";
import {
  createTagState,
  updateTagPipeline,
  backdateFieldPose,
} from "./tag-pipeline.js";
import {
  applyTemporalFilter,
  confirmedDetections,
  createTemporalFilterState,
  TEMPORAL_REQUIRED,
  TEMPORAL_WINDOW,
} from "./temporal-filter.js";
import { drawMockScene } from "./mock-scene.js";
import { renderFrame, describeLogic } from "./renderer.js";
import { saveCapture, canvasToDataUrl, downloadDataUrl } from "./capture.js";
import { drawImageNative, setElementAspectRatio, toGrayscaleImageData } from "./canvas-util.js";

const display = /** @type {HTMLCanvasElement} */ (document.getElementById("display"));
const capture = /** @type {HTMLCanvasElement} */ (document.getElementById("capture"));
const processCanvas = /** @type {HTMLCanvasElement} */ (document.getElementById("process"));
const webcam = /** @type {HTMLVideoElement} */ (document.getElementById("webcam"));

const fpsEl = document.getElementById("fps");
const modeLabel = document.getElementById("mode-label");
const detectionList = document.getElementById("detection-list");
const logicState = document.getElementById("logic-state");
const elementErrorEl = document.getElementById("element-error");
const robotDistEl = document.getElementById("robot-dist");
const elementRangeEl = document.getElementById("element-range");
const elementSizeRangeEl = document.getElementById("element-size-range");
const elementFloorRangeEl = document.getElementById("element-floor-range");
const elementConfidenceEl = document.getElementById("element-confidence");
const elementRobotXYEl = document.getElementById("element-robot-xy");
const tagScoutEl = document.getElementById("tag-scout");
const tagFixEl = document.getElementById("tag-fix");
const tagPoseNowEl = document.getElementById("tag-pose-now");
const tagSampleBtn = /** @type {HTMLButtonElement | null} */ (document.getElementById("tag-sample-btn"));
const captureStatusEl = document.getElementById("capture-status");
const processSizeLabel = document.getElementById("process-size-label");

const sourceSelect = /** @type {HTMLSelectElement} */ (document.getElementById("source-select"));
const displayViewSelect = /** @type {HTMLSelectElement} */ (document.getElementById("display-view"));
const overlaySelect = /** @type {HTMLSelectElement} */ (document.getElementById("overlay-select"));
const showProcessPipCheck = /** @type {HTMLInputElement} */ (document.getElementById("show-process-pip"));
const showMaskCheck = /** @type {HTMLInputElement} */ (document.getElementById("show-mask"));
const downscaleInput = /** @type {HTMLInputElement} */ (document.getElementById("downscale-ratio"));
const cropHeightInput = /** @type {HTMLInputElement} */ (document.getElementById("crop-height"));
const cropOffsetInput = /** @type {HTMLInputElement} */ (document.getElementById("crop-offset"));
const startBtn = /** @type {HTMLButtonElement} */ (document.getElementById("start-btn"));
const stopBtn = /** @type {HTMLButtonElement} */ (document.getElementById("stop-btn"));
const captureBtn = /** @type {HTMLButtonElement} */ (document.getElementById("capture-btn"));
const detectElementCheck = /** @type {HTMLInputElement} */ (document.getElementById("detect-element"));
const detectRedCheck = /** @type {HTMLInputElement} */ (document.getElementById("detect-red"));
const detectBlueCheck = /** @type {HTMLInputElement} */ (document.getElementById("detect-blue"));
const grayscaleProcessCheck = /** @type {HTMLInputElement} */ (document.getElementById("grayscale-process"));
const temporalFilterCheck = /** @type {HTMLInputElement} */ (document.getElementById("temporal-filter"));
const detectorActiveLabel = document.getElementById("detector-active-label");
const showMaskLabel = document.getElementById("show-mask-label");

const hsvBrightnessMinInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-brightness-min"));
const hsvBrightnessSpreadInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-brightness-spread"));
const hsvMaxSaturationInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-max-saturation"));
const hsvMinValueInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-min-value"));
const hsvMinAreaInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-min-area"));
const hsvMorphPassesInput = /** @type {HTMLInputElement} */ (document.getElementById("hsv-morph-passes"));

const displayCtx = display.getContext("2d", { willReadFrequently: true });
const captureCtx = capture.getContext("2d", { willReadFrequently: true });
const processCtx = processCanvas.getContext("2d", { willReadFrequently: true });

/** @type {import('./config.js').VidarTuning | null} */
let baseTuning = null;
let running = false;
let tick = 0;
let rafId = 0;
let lastFpsTime = performance.now();
let frames = 0;

/** @type {import('./tag-pipeline.js').TagPipelineState} */
let tagState = createTagState();

/** @type {import('./temporal-filter.js').TemporalFilterState} */
let temporalState = createTemporalFilterState();

/** @type {{ detections: import('./temporal-filter.js').StableDetection[], rawDetections: import('./detection.js').Detection[], roi: {x:number,y:number,w:number,h:number}, elementMask: Uint8Array | null, maskPixels: number, tuning: import('./config.js').VidarTuning, detectOpts: ReturnType<typeof readDetectOptions>, activeDetectors: string } | null} */
let lastFrame = null;

function readDetectOptions() {
  return {
    detectElement: detectElementCheck.checked,
    detectRedPlate: detectRedCheck.checked,
    detectBluePlate: detectBlueCheck.checked,
    grayscaleProcess: grayscaleProcessCheck.checked,
  };
}

function syncDetectorUiLabels() {
  const opts = readDetectOptions();
  if (detectorActiveLabel) {
    const targets = [
      opts.detectElement ? "element (contour)" : null,
      opts.detectRedPlate ? "red plate" : null,
      opts.detectBluePlate ? "blue plate" : null,
    ].filter(Boolean);
    detectorActiveLabel.textContent =
      targets.length ? `Active: ${targets.join(" · ")}` : "Active: none — enable a detector";
  }
  if (showMaskLabel) {
    showMaskLabel.textContent = opts.detectElement
      ? "Show element debug overlay (contour mask)"
      : "Show element debug overlay (element off)";
  }
}

function readElementMetricsUi() {
  return {
    brightnessMin: Number(hsvBrightnessMinInput.value),
    brightnessSpread: Number(hsvBrightnessSpreadInput.value),
    maxSaturation: Number(hsvMaxSaturationInput.value),
    hsvMinValue: Number(hsvMinValueInput.value),
    minArea: Number(hsvMinAreaInput.value),
    morphClosePasses: Number(hsvMorphPassesInput.value),
  };
}

/** @param {import('./config.js').ColorTarget} target */
function applyElementMetricsToUi(target) {
  hsvBrightnessMinInput.value = String(target.brightnessMin ?? target.interiorBright ?? 58);
  hsvBrightnessSpreadInput.value = String(target.brightnessSpread ?? target.interiorSpread ?? 65);
  hsvMaxSaturationInput.value = String(target.maxSaturation ?? 85);
  hsvMinValueInput.value = String(target.hsvLow?.[2] ?? 55);
  hsvMinAreaInput.value = String(target.minArea ?? 12);
  hsvMorphPassesInput.value = String(target.morphClosePasses ?? 8);
}

function readUiTuning() {
  if (!baseTuning) return null;
  return tuningFromUi(
    { ...baseTuning, captureWidth: capture.width, captureHeight: capture.height },
    {
      downscaleRatio: Number(downscaleInput.value),
      verticalCropOffset: Number(cropOffsetInput.value),
      verticalCropHeight: Number(cropHeightInput.value),
      element: readElementMetricsUi(),
    },
  );
}

function syncCaptureDimensions(w, h) {
  capture.width = w;
  capture.height = h;
  display.width = w;
  display.height = h;
  setElementAspectRatio(display, w, h);
  cropHeightInput.max = String(h);
  cropOffsetInput.max = String(Math.max(0, h - 1));
  if (Number(cropHeightInput.value) > h) cropHeightInput.value = String(h);
  if (Number(cropOffsetInput.value) >= h) cropOffsetInput.value = String(Math.max(0, h - 1));
}

/** Scale crop UI from tuning reference (640×480) to actual camera size. */
function applyProportionalCrop(capW, capH) {
  if (!baseTuning) return;
  const refW = baseTuning.captureWidth;
  const refH = baseTuning.captureHeight;
  const sx = capW / refW;
  const sy = capH / refH;
  cropOffsetInput.value = String(Math.round(baseTuning.verticalCropOffset * sy));
  cropHeightInput.value = String(Math.round(baseTuning.verticalCropHeight * sy));
}

function syncProcessCanvasSize(tuning) {
  if (processCanvas.width !== tuning.processWidth || processCanvas.height !== tuning.processHeight) {
    processCanvas.width = tuning.processWidth;
    processCanvas.height = tuning.processHeight;
  }
  if (processSizeLabel) {
    const cam = `${capture.width}×${capture.height}`;
    processSizeLabel.textContent =
      `Camera ${cam} · process ${tuning.processWidth}×${tuning.processHeight} · crop y=${tuning.crop?.y} h=${tuning.crop?.h}`;
  }
}

/** @param {HTMLVideoElement} video */
function waitForVideoDimensions(video) {
  if (video.videoWidth > 0 && video.videoHeight > 0) return Promise.resolve();
  return new Promise((resolve) => {
    const done = () => {
      if (video.videoWidth > 0 && video.videoHeight > 0) {
        video.removeEventListener("loadedmetadata", done);
        resolve(undefined);
      }
    };
    video.addEventListener("loadedmetadata", done);
    done();
  });
}

async function init() {
  try {
    baseTuning = await loadTuning();
    syncCaptureDimensions(baseTuning.captureWidth, baseTuning.captureHeight);

    downscaleInput.value = String(baseTuning.downscaleRatio);
    cropHeightInput.value = String(baseTuning.verticalCropHeight);
    cropOffsetInput.value = String(baseTuning.verticalCropOffset);

    const elementEl = baseTuning.elements[0];
    if (elementEl) applyElementMetricsToUi(elementEl);
    syncDetectorUiLabels();

    const t = readUiTuning();
    if (t) syncProcessCanvasSize(t);
  } catch (err) {
    detectionList.innerHTML = `<li class="empty">${err.message}. Serve via scripts/serve_sim.ps1</li>`;
    startBtn.disabled = true;
  }
}

downscaleInput.addEventListener("input", () => {
  const t = readUiTuning();
  if (t) syncProcessCanvasSize(t);
});
cropHeightInput.addEventListener("input", () => {
  const t = readUiTuning();
  if (t) syncProcessCanvasSize(t);
});
cropOffsetInput.addEventListener("input", () => {
  const t = readUiTuning();
  if (t) syncProcessCanvasSize(t);
});

[detectElementCheck, detectRedCheck, detectBlueCheck].forEach((el) => {
  el.addEventListener("change", syncDetectorUiLabels);
});
temporalFilterCheck.addEventListener("change", () => {
  temporalState = createTemporalFilterState();
});

startBtn.addEventListener("click", () => start());
stopBtn.addEventListener("click", () => stop());
captureBtn.addEventListener("click", () => captureStill());

async function start() {
  const tuning = readUiTuning();
  if (!tuning || running) return;
  running = true;
  temporalState = createTemporalFilterState();
  startBtn.disabled = true;
  stopBtn.disabled = false;
  captureBtn.disabled = false;
  tick = 0;
  lastFpsTime = performance.now();
  frames = 0;

  if (sourceSelect.value === "webcam") {
    try {
      const stream = await navigator.mediaDevices.getUserMedia({
        video: {
          width: { ideal: baseTuning?.captureWidth ?? 640 },
          height: { ideal: baseTuning?.captureHeight ?? 480 },
          facingMode: "environment",
        },
        audio: false,
      });
      webcam.srcObject = stream;
      await webcam.play();
      await waitForVideoDimensions(webcam);
      syncCaptureDimensions(webcam.videoWidth, webcam.videoHeight);
      applyProportionalCrop(webcam.videoWidth, webcam.videoHeight);
      if (processSizeLabel) {
        processSizeLabel.textContent =
          `Camera: ${webcam.videoWidth}×${webcam.videoHeight} (native aspect)`;
      }
    } catch (err) {
      running = false;
      startBtn.disabled = false;
      stopBtn.disabled = true;
      captureBtn.disabled = true;
      detectionList.innerHTML = `<li class="empty">Webcam error: ${err.message}</li>`;
      return;
    }
  } else if (baseTuning) {
    syncCaptureDimensions(baseTuning.captureWidth, baseTuning.captureHeight);
  }

  modeLabel.textContent = sourceSelect.value === "webcam" ? "Webcam" : "Mock";
  syncDetectorUiLabels();
  loop();
}

function stop() {
  running = false;
  temporalState = createTemporalFilterState();
  cancelAnimationFrame(rafId);
  startBtn.disabled = false;
  stopBtn.disabled = true;
  captureBtn.disabled = true;

  if (webcam.srcObject) {
    for (const track of /** @type {MediaStream} */ (webcam.srcObject).getTracks()) {
      track.stop();
    }
    webcam.srcObject = null;
  }
}

function loop() {
  const tuning = readUiTuning();
  if (!running || !tuning) return;
  rafId = requestAnimationFrame(loop);

  if (sourceSelect.value === "webcam") {
    if (webcam.readyState >= HTMLMediaElement.HAVE_CURRENT_DATA) {
      drawImageNative(captureCtx, webcam);
    }
  } else {
    drawMockScene(captureCtx, capture.width, capture.height, tick++);
  }

  syncProcessCanvasSize(tuning);
  const crop = tuning.crop ?? { x: 0, y: 0, w: tuning.captureWidth, h: tuning.captureHeight };

  processCtx.drawImage(
    capture,
    crop.x, crop.y, crop.w, crop.h,
    0, 0, tuning.processWidth, tuning.processHeight,
  );

  const roi = computeRoi(tuning, tuning.processWidth, tuning.processHeight);
  const colorImageData = processCtx.getImageData(0, 0, tuning.processWidth, tuning.processHeight);
  const detectOpts = readDetectOptions();

  if (detectOpts.grayscaleProcess) {
    processCtx.putImageData(toGrayscaleImageData(colorImageData), 0, 0);
  }

  const { detections: rawDetections, elementMask, activeDetectors } = detectBlobs(
    colorImageData, tuning, roi, detectOpts,
  );

  tagState = updateTagPipeline(
    tagState,
    colorImageData,
    tuning.tag ?? { enabled: false },
    performance.now(),
    {
      expectedBearingDeg: tuning.tag?.expectedBearingDeg ?? undefined,
      cameraBearingDeg: tuning.geometry?.cameras?.[tuning.geometry?.activeCameraIndex ?? 0]?.bearingDeg ?? 0,
    },
  );

  const filtered = applyTemporalFilter(temporalState, rawDetections, {
    enabled: temporalFilterCheck.checked,
    windowSize: TEMPORAL_WINDOW,
    required: TEMPORAL_REQUIRED,
  });
  temporalState = filtered.state;
  const detections = filtered.stable;
  const logicDetections = confirmedDetections(detections);

  const best = bestByCategory(logicDetections, tuning.processHeight, tuning.geometry);
  const logic = describeLogic(best, tuning);

  const maskPixels = elementMask ? elementMask.reduce((n, v) => n + v, 0) : 0;
  lastFrame = {
    detections,
    rawDetections,
    roi,
    elementMask,
    maskPixels,
    tuning,
    detectOpts,
    activeDetectors,
  };

  const maskCanvas = elementMask
    ? maskToCanvas(elementMask, tuning.processWidth, tuning.processHeight)
    : null;

  renderFrame(displayCtx, capture, detections, roi, tuning, {
    overlay: overlaySelect.value,
    viewMode: displayViewSelect.value,
    showProcessPip: showProcessPipCheck.checked,
    showMask: showMaskCheck.checked,
    showCrop: true,
    grayscaleProcess: detectOpts.grayscaleProcess,
    processCanvas,
    maskCanvas,
    tagRegion: tagState.lastRegion,
  });

  updateSidebar(detections, rawDetections, logic, maskPixels, tuning, activeDetectors);
  syncDetectorUiLabels();
  updateFps();
}

async function captureStill() {
  if (!lastFrame) {
    if (captureStatusEl) captureStatusEl.textContent = "Start the sim first";
    return;
  }
  const tuning = lastFrame.tuning;

  const ts = new Date().toISOString().replace(/[:.]/g, "-");
  const frameUrl = canvasToDataUrl(capture);
  const processUrl = canvasToDataUrl(processCanvas);
  const maskCanvas = lastFrame.elementMask
    ? maskToCanvas(lastFrame.elementMask, tuning.processWidth, tuning.processHeight)
    : null;
  const maskUrl = maskCanvas ? canvasToDataUrl(maskCanvas) : null;

  const meta = {
    timestamp: ts,
    source: sourceSelect.value,
    detectOptions: lastFrame.detectOpts,
    grayscaleProcess: lastFrame.detectOpts?.grayscaleProcess ?? false,
    elementMetrics: readElementMetricsUi(),
    activeDetectors: lastFrame.activeDetectors,
    tuning,
    crop: tuning.crop,
    roi: lastFrame.roi,
    detections: lastFrame.detections,
    rawDetections: lastFrame.rawDetections,
    logic: describeLogic(
      bestByCategory(
        confirmedDetections(lastFrame.detections),
        tuning.processHeight,
        tuning.geometry,
      ),
      tuning,
    ),
  };

  if (captureStatusEl) captureStatusEl.textContent = "Saving capture…";

  try {
    const result = await saveCapture({
      timestamp: ts,
      frame: frameUrl,
      process: processUrl,
      mask: maskUrl,
      meta,
    });
    if (captureStatusEl) {
      captureStatusEl.textContent = `Saved to captures/${result.id}`;
    }
  } catch (err) {
    downloadDataUrl(frameUrl, `vidar-${ts}.png`);
    if (maskUrl) downloadDataUrl(maskUrl, `vidar-${ts}-mask.png`);
    if (captureStatusEl) {
      captureStatusEl.textContent = `Server save failed — downloaded locally. ${err.message}`;
    }
  }
}

/** @param {import('./temporal-filter.js').StableDetection[]} detections @param {import('./detection.js').Detection[]} rawDetections @param {ReturnType<typeof describeLogic>} logic @param {number} maskPixels @param {import('./config.js').VidarTuning} tuning @param {string} [activeDetectors] */
function updateSidebar(detections, rawDetections, logic, maskPixels, tuning, activeDetectors) {
  const temporalOn = temporalFilterCheck.checked;
  if (detections.length === 0) {
    const hint = activeDetectors?.includes("element")
      ? temporalOn
        ? `raw ${rawDetections.length} this frame · mask/edges <strong>${maskPixels}</strong> px`
        : `mask/edges <strong>${maskPixels}</strong> px — try the other element algorithm or adjust crop`
      : "element detection off — enable under Detection logic";
    detectionList.innerHTML = `<li class="empty">No blobs — ${hint}</li>`;
  } else {
    detectionList.innerHTML = detections
      .map((d) => {
        const status =
          d.temporal === "valid" ? "confirmed" :
          d.temporal === "coasting" ? "hold" :
          d.temporal === "potential" ? "potential" : "";
        const geom =
          d.range != null
            ? ` · ${d.range.toFixed(0)}in (size ${d.dSize?.toFixed(0) ?? "—"}/floor ${d.dFloor?.toFixed(0) ?? "—"}) conf ${((d.confidence ?? 0) * 100).toFixed(0)}%`
            : "";
        return `<li><span class="tag" style="color:${d.color}">${d.label}</span> (${d.cx.toFixed(0)}, ${d.cy.toFixed(0)}) · ${Math.round(d.area)} px${geom}${status ? ` · <em>${status}</em>` : ""}${d.circularity != null ? ` · circ ${d.circularity.toFixed(2)}` : ""}${d.interior != null ? ` · fill ${(d.interior * 100).toFixed(0)}%` : ""}</li>`;
      })
      .join("");
    if (temporalOn && rawDetections.length > detections.length) {
      detectionList.innerHTML +=
        `<li class="empty muted-line">+ ${rawDetections.length - detections.length} raw hits hidden (not in 3/5 window)</li>`;
    }
  }

  const detectOpts = readDetectOptions();
  const logicNote = !detectOpts.detectElement
    ? " (element off)"
    : temporalOn
      ? ` · logic uses 3/${TEMPORAL_WINDOW} confirmed`
      : "";
  logicState.textContent = logic.state + logicNote;
  elementErrorEl.textContent =
    logic.elementError == null ? "—" : `${logic.elementError.toFixed(1)} px`;
  robotDistEl.textContent =
    logic.robotDist == null ? "—" : `${logic.robotDist.toFixed(1)} px`;
  if (elementRangeEl) {
    elementRangeEl.textContent =
      logic.elementRange == null ? "—" : `${logic.elementRange.toFixed(1)} in`;
  }
  if (elementSizeRangeEl) {
    elementSizeRangeEl.textContent =
      logic.dSize == null ? "—" : `${logic.dSize.toFixed(1)} in`;
  }
  if (elementFloorRangeEl) {
    elementFloorRangeEl.textContent =
      logic.dFloor == null ? "—" : `${logic.dFloor.toFixed(1)} in`;
  }
  if (elementConfidenceEl) {
    elementConfidenceEl.textContent =
      logic.elementConfidence == null ? "—" : `${(logic.elementConfidence * 100).toFixed(0)}%`;
  }
  if (elementRobotXYEl) {
    elementRobotXYEl.textContent = logic.robotXY
      ? `(${logic.robotXY.x.toFixed(1)}, ${logic.robotXY.y.toFixed(1)}) in`
      : "—";
  }
  const tag = tagState.latest;
  const tagScout = tagState.lastScout;
  if (tagScoutEl) {
    tagScoutEl.textContent = tagScout
      ? `(${tagScout.cx.toFixed(0)}, ${tagScout.cy.toFixed(0)}) w=${tagScout.widthPx.toFixed(0)} ${tagScout.band}`
      : "—";
  }
  if (tagFixEl) {
    tagFixEl.textContent = tag
      ? `id=${tag.tagId} dec=${tag.decimation} ${tag.decodePixels ?? "—"}px age=${((performance.now() - tag.captureTimeMs) / 1000).toFixed(1)}s`
      : "—";
  }
  if (tagPoseNowEl) {
    const now = backdateFieldPose(tag, tagState.odom);
    tagPoseNowEl.textContent = now
      ? `(${now.x.toFixed(1)}, ${now.y.toFixed(1)}) ${now.h.toFixed(0)}°`
      : "—";
  }
}

function updateFps() {
  frames += 1;
  const now = performance.now();
  if (now - lastFpsTime >= 500) {
    const fps = (frames * 1000) / (now - lastFpsTime);
    fpsEl.textContent = fps.toFixed(0);
    frames = 0;
    lastFpsTime = now;
  }
}

tagSampleBtn?.addEventListener("click", () => {
  tagState.driverRequested = true;
});

init();
