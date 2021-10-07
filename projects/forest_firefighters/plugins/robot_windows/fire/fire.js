
var canvas = document.getElementById("canvas");

// define all the constants
const centerOffset = canvas.height / 2;
const radius = 0.8 * centerOffset;
const pointerLength = 0.75 * centerOffset;
const axisWidth = 0.01 * centerOffset

var ctx = canvas.getContext("2d");
ctx.translate(centerOffset, centerOffset);

// initialize the robot window
window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.setTitle('Fire');
  drawDial(radius);
  window.robotWindow.receive = receive;
};

var mouseDown = false;

canvas.addEventListener('mousedown', e => {
  updateFormMouse(e);
  mouseDown = true;
});

canvas.addEventListener('mousemove', e => {
  if (mouseDown === true)
    updateFormMouse(e);
});

document.addEventListener('mouseup', e => {
  mouseDown = false;
});

function updateFormMouse(e) {
  let x = e.offsetX - centerOffset;
  let y = e.offsetY - centerOffset;
  let angle = Math.atan2(y, x);
  let intensity = Math.min(1, Math.sqrt(Math.pow(x / pointerLength, 2) + Math.pow(y / pointerLength, 2)));
  drawWindIndicator(angle, intensity);

  let message = JSON.stringify({angle: -angle, intensity})

  window.robotWindow.send(message)
}

function receive(message) {
  let wind = JSON.parse(message);
  drawWindIndicator(-wind.angle, wind.intensity);
}

function drawWindIndicator(angle, intensity) {
  drawDial();
  drawHand(angle, intensity);
}

function drawDial() {
  // white circle
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, 2*Math.PI);
  ctx.fillStyle = 'white';
  ctx.fill();

  // black disk
  ctx.beginPath();
  ctx.lineWidth = axisWidth;
  ctx.arc(0, 0, radius, 0, 2*Math.PI);
  ctx.strokeStyle = 'black';
  ctx.stroke();

  // small center black circle
  ctx.beginPath();
  ctx.arc(0, 0, 3 * axisWidth, 0, 2*Math.PI);
  ctx.fillStyle = 'black';
  ctx.fill();

  // x and y axis
  drawAxis('red', Math.PI / 2, 'x')
  drawAxis('green', 0, 'y')
}

function drawAxis(color, angle, letter) {

  // axis shape
  ctx.beginPath();
  ctx.moveTo(0,0);
  ctx.rotate(angle)

  ctx.lineCap = "round";
  ctx.lineWidth = axisWidth;

  ctx.lineTo(0, -pointerLength);
  ctx.lineTo(-4 * axisWidth, -pointerLength + 4 * axisWidth);
  ctx.lineTo(0, -pointerLength);
  ctx.lineTo(4 * axisWidth, -pointerLength + 4 * axisWidth);
  ctx.strokeStyle = color;
  ctx.stroke();

  // axis letter
  ctx.font = '25px Arial';
  ctx.fillStyle = color;
  ctx.fillText(letter, -4 * axisWidth, -radius - 5 * axisWidth);

  ctx.rotate(-angle)
}

function drawHand(angle, intensity) {
  // wind hand for the direction
  let alpha = angle + Math.PI/2 % (2 * Math.PI)
  ctx.beginPath();
  ctx.lineWidth = 2 * axisWidth;
  ctx.lineCap = "round";
  ctx.moveTo(0,0);
  ctx.rotate(alpha);
  ctx.lineTo(0, -pointerLength);
  ctx.strokeStyle = 'black';
  ctx.stroke();

  // wind circle for the intensity
  ctx.beginPath();
  ctx.arc(0, -intensity * pointerLength, 5 * axisWidth, 0, 2*Math.PI);
  ctx.fillStyle = 'black';
  ctx.fill();

  ctx.rotate(-alpha);
}

function toggleStopCheckbox(obj) {
  if (obj.checked) {
    obj.parentNode.classList.add('checked');
    obj.parentNode.lastChild.innerHTML = 'Start random evolution';
    window.robotWindow.send('stop');
  } else {
    obj.parentNode.classList.remove('checked');
    obj.parentNode.lastChild.innerHTML = 'Stop random evolution';
    window.robotWindow.send('start');
  }
}
