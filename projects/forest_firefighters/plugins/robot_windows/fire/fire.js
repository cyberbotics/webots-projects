
var canvas = document.getElementById("canvas");

// define all the constants
const centerOffset = canvas.height / 2;
const radius = 0.8 * centerOffset;
const pointerLength = 0.75 * centerOffset;
const axis_width = 0.01 * centerOffset

var ctx = canvas.getContext("2d");
ctx.translate(centerOffset, centerOffset);

window.onload = function() {
  window.robotWindow = webots.window();
  window.robotWindow.setTitle('Fire');
  drawFace(radius);
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
  let intensity = Math.min(1, Math.sqrt((x / pointerLength)**2 + (y / pointerLength)**2));
  drawWindIndicator(angle, intensity);

  let message = JSON.stringify({angle: -angle, intensity})

  window.robotWindow.send(message)
}

function receive(message) {
  let wind = JSON.parse(message);
  drawWindIndicator(-wind.angle, wind.intensity);
}

function drawWindIndicator(angle, intensity) {
  drawFace();
  displayWind(angle, intensity);
}

function drawFace() {
  ctx.beginPath();
  ctx.arc(0, 0, radius, 0, 2*Math.PI);
  ctx.fillStyle = 'white';
  ctx.fill();

  ctx.beginPath();
  ctx.lineWidth = axis_width;
  ctx.arc(0, 0, radius, 0, 2*Math.PI);
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, 0, 3 * axis_width, 0, 2*Math.PI);
  ctx.fillStyle = 'black';
  ctx.fill();

  drawAxis('red', Math.PI / 2, 'x')
  drawAxis('green', 0, 'y')
}

function drawAxis(color, angle, letter) {
  ctx.beginPath();
  ctx.moveTo(0,0);
  ctx.rotate(angle)

  ctx.lineCap = "round";
  ctx.lineWidth = axis_width;

  ctx.lineTo(0, -pointerLength);
  ctx.lineTo(-4 * axis_width, -pointerLength + 4 * axis_width);
  ctx.lineTo(0, -pointerLength);
  ctx.lineTo(4 * axis_width, -pointerLength + 4 * axis_width);
  ctx.strokeStyle = color;
  ctx.stroke();

  ctx.font = '30px verdana';
  ctx.fillStyle = color;
  ctx.fillText(letter, -4 * axis_width, -radius - 5 * axis_width);

  ctx.rotate(-angle)
  ctx.strokeStyle = 'black';
}

function displayWind(angle, intensity) {
  let alpha = angle + Math.PI/2 % (2 * Math.PI)
  ctx.beginPath();
  ctx.lineWidth = 2 * axis_width;
  ctx.lineCap = "round";
  ctx.moveTo(0,0);
  ctx.rotate(alpha);
  ctx.lineTo(0, -pointerLength);
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(0, -intensity * pointerLength, 5 * axis_width, 0, 2*Math.PI);
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
