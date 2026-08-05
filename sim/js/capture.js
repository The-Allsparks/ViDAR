/** Save current frame + debug mask to server captures/ folder for review. */
export async function saveCapture(payload) {
  const res = await fetch("/api/capture", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  if (!res.ok) {
    const text = await res.text();
    throw new Error(text || `Capture failed (${res.status})`);
  }
  return res.json();
}

export function canvasToDataUrl(canvas) {
  return canvas.toDataURL("image/png");
}

export function downloadDataUrl(dataUrl, filename) {
  const a = document.createElement("a");
  a.href = dataUrl;
  a.download = filename;
  a.click();
}
