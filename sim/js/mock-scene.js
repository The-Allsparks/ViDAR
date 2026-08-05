/**
 * Synthetic scene: white ball + robot with alliance plates (in lower crop band).
 * @param {CanvasRenderingContext2D} ctx
 * @param {number} width
 * @param {number} height
 * @param {number} tick
 */
export function drawMockScene(ctx, width, height, tick) {
  ctx.fillStyle = "#1a1a22";
  ctx.fillRect(0, 0, width, height);

  // Mock wall AprilTag in top half (above horizon midline)
  const tagX = width * 0.62;
  const tagY = height * 0.22;
  const tagOuter = 52;
  ctx.fillStyle = "#f4f4f4";
  ctx.fillRect(tagX - tagOuter / 2, tagY - tagOuter / 2, tagOuter, tagOuter);
  ctx.fillStyle = "#111";
  const cell = tagOuter / 8;
  for (let r = 0; r < 8; r++) {
    for (let c = 0; c < 8; c++) {
      if ((r + c + tick) % 3 === 0) {
        ctx.fillRect(
          tagX - tagOuter / 2 + c * cell,
          tagY - tagOuter / 2 + r * cell,
          cell,
          cell,
        );
      }
    }
  }

  ctx.strokeStyle = "#2a2a35";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 40) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y < height; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  // Robot body — lower half of frame (inside default y=240 crop)
  const botX = width * 0.18;
  const botY = height * 0.62;
  ctx.fillStyle = "#3a3a48";
  ctx.fillRect(botX - 30, botY - 25, 90, 55);

  // Red alliance plate
  ctx.fillStyle = "#d81818";
  ctx.fillRect(botX + 45, botY - 12, 28, 22);
  ctx.fillStyle = "#fff";
  ctx.font = "bold 14px sans-serif";
  ctx.fillText("42", botX + 50, botY + 4);

  // Blue plate (second robot marker)
  ctx.fillStyle = "#2266dd";
  ctx.fillRect(width * 0.55, botY + 10, 26, 20);

  // Moving white ball
  const ballX = ((tick * 3) % (width - 120)) + 70;
  const ballY = height * 0.72;
  const ballR = 28;
  ctx.fillStyle = "#ececf4";
  ctx.beginPath();
  ctx.arc(ballX, ballY, ballR, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = "#b0b0c0";
  ctx.lineWidth = 2;
  ctx.stroke();

  // Partially occluded ball
  const occX = width * 0.72;
  const occY = height * 0.68;
  ctx.fillStyle = "#2a2a35";
  ctx.fillRect(occX - 10, occY - 35, 50, 70);
  ctx.fillStyle = "#ececf4";
  ctx.beginPath();
  ctx.arc(occX + 15, occY, 24, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = "#2a2a35";
  ctx.fillRect(occX + 5, occY - 30, 35, 60);

  // Tiny speck (filtered by min area)
  ctx.fillStyle = "#f0f0f8";
  ctx.fillRect(10, height - 30, 5, 5);
}
