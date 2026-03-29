const getCanvas = (canvasId) => {
  const canvas = document.getElementById(canvasId);
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();

  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  return { canvas, ctx };
};

const getPointTemplate = (color, shadowColor) => {
  const canvas = document.createElement("canvas");
  const dpr = window.devicePixelRatio || 1;

  canvas.width = 100 * dpr;
  canvas.height = 100 * dpr;

  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);

  ctx.shadowColor = shadowColor;
  ctx.shadowBlur = 20;
  ctx.fillStyle = color;
  ctx.font = "40px serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText("➡", 50, 50);

  return canvas;
};

const { canvas, ctx } = getCanvas("app");
const pred_arrow = getPointTemplate("black", "blue");
const truth_arrow = getPointTemplate("white", "red");

const rect = canvas.getBoundingClientRect();
const numRows = 12;
const numCols = 12;
const scaleX = rect.width / numCols;
const scaleY = rect.height / numRows;

const gameLoop = async (runner) => {
  ctx.clearRect(0, 0, rect.width, rect.height);
  ctx.save();

  const pred = runner.predictAndUpdate();
  const truth = runner.getTruth();

  ctx.translate(pred.x * scaleX, rect.height - pred.y * scaleY);
  ctx.rotate(-pred.theta);
  ctx.drawImage(pred_arrow, -50, -50, 100, 100);

  ctx.restore();
  ctx.save();

  ctx.translate(truth.x * scaleX, rect.height - truth.y * scaleY);
  ctx.rotate(-truth.theta);
  ctx.drawImage(truth_arrow, -50, -50, 100, 100);

  ctx.restore();
};

var Module = {};

Module.onRuntimeInitialized = () => {
  console.log("WASM loaded!");

  const kalmanRunner = new Module.KalmanRunner("data/robot_log.csv");
  const mclRunner = new Module.MCLRunner("data/robot_log.csv");

  let currentRunner = null;
  let isRunning = false;

  const startLoop = (runner) => {
    currentRunner = runner;
    if (!isRunning) {
      isRunning = true;
      requestAnimationFrame(loop);
    }
  };

  const loop = () => {
    if (!isRunning) return;
    gameLoop(currentRunner);
    requestAnimationFrame(loop);
  };

  document.getElementById("mcl-btn").addEventListener("click", () => {
    startLoop(mclRunner);
  });

  document.getElementById("kalman-btn").addEventListener("click", () => {
    startLoop(kalmanRunner);
  });
};
