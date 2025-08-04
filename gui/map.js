// layer1 is background with map, grid, and trail - can add but not remove
// layer2 is the current position of the robot, cleared between each move
let bounds = document.getElementById('map').getBoundingClientRect();
const layer1 = document.getElementById('layer1').getContext('2d');
const layer2 = document.getElementById('layer2').getContext('2d');

// transform to use site coordinates

// flip y-axis to point up
var width = bounds.width;
var height = bounds.height;
var scale, tx, ty;


function toMap(p) {
    return {x: p.x, y: p.y}
}

function getMousePos(evt) {
    return {
      x: evt.clientX - bounds.left,
      y: evt.clientY - bounds.top
    };
}

function canvasToWorld(x, y) {
    return {
        x: x / scale - tx,
        y: -y / scale - ty
    }
}

function onMouseMove(event) {
    pos = getMousePos(event);
    let x = pos.x / scale - tx;
    let y = -pos.y / scale - ty;
    // console.log("mouse", x, y);
    document.getElementById('mouse_x').innerText = x.toFixed(2);
    document.getElementById('mouse_y').innerText = y.toFixed(2);
}


function _drawPath(ctx, points) {
    let p0 = points.shift();
    ctx.moveTo(p0.x, p0.y);
    points.forEach(p => ctx.lineTo(p.x, p.y));
    ctx.closePath();
}

function drawMap(ctx, map) {
    console.log("MAP", map);

    // Draw obstacles/interiors with modern styling
    map.interiors.forEach(path => {
        _drawPath(ctx, path);
        ctx.fillStyle = '#e74c3c';
        ctx.fill();
        ctx.lineWidth = 0.05;
        ctx.strokeStyle = '#c0392b';
        ctx.stroke();
    });

    // Draw exterior boundary with modern styling
    _drawPath(ctx, map.exterior);
    ctx.fillStyle = '#2ecc71';
    ctx.fill();
    ctx.lineWidth = 0.05;
    ctx.strokeStyle = '#27ae60';
    ctx.stroke();
}

function trail_at(x, y) {
    let ctx = layer1;
    let radius = 0.08; // meter - slightly smaller for cleaner look
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI, false);
    ctx.fillStyle = '#3498db';
    ctx.fill();
    // Add subtle border
    ctx.lineWidth = 0.02;
    ctx.strokeStyle = '#2980b9';
    ctx.stroke();
}

function robot(x, y, heading) {
    let ctx = layer2;
    let radius = 0.25; // meter - slightly larger for better visibility
    
    // Robot body with gradient effect
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI, false);
    
    // Create gradient for robot body
    const gradient = ctx.createRadialGradient(x, y, 0, x, y, radius);
    gradient.addColorStop(0, '#ecf0f1');
    gradient.addColorStop(0.7, '#bdc3c7');
    gradient.addColorStop(1, '#95a5a6');
    
    ctx.fillStyle = gradient;
    ctx.fill();
    
    // Robot border
    ctx.lineWidth = 0.03;
    ctx.strokeStyle = '#2c3e50';
    ctx.stroke();

    // Heading indicator with arrow
    ctx.beginPath();
    ctx.moveTo(x, y);
    let V = 0.35; // meter
    let arrowX = x + V * Math.cos(heading);
    let arrowY = y + V * Math.sin(heading);
    ctx.lineTo(arrowX, arrowY);
    
    // Arrow head
    let arrowHeadLength = 0.08;
    let arrowHeadAngle = Math.PI / 6;
    let angle = Math.atan2(arrowY - y, arrowX - x);
    
    ctx.lineTo(arrowX - arrowHeadLength * Math.cos(angle - arrowHeadAngle), 
               arrowY - arrowHeadLength * Math.sin(angle - arrowHeadAngle));
    ctx.moveTo(arrowX, arrowY);
    ctx.lineTo(arrowX - arrowHeadLength * Math.cos(angle + arrowHeadAngle), 
               arrowY - arrowHeadLength * Math.sin(angle + arrowHeadAngle));
    
    ctx.lineWidth = 0.05;
    ctx.strokeStyle = '#e74c3c';
    ctx.stroke();

    // Update position display
    document.getElementById('robot_x').innerText = x.toFixed(2);
    document.getElementById('robot_y').innerText = y.toFixed(2);
    document.getElementById('robot_heading').innerText = (heading * 180 / Math.PI).toFixed(0) + '°';
}

function drawOrigin(ctx) {
    ctx.beginPath();
    ctx.arc(0, 0, 0.3, 0, 2 * Math.PI, false);
    ctx.arc(0, 0, 0.5, 0, 2 * Math.PI, false);
    ctx.fillStyle = '#34495e';
    ctx.lineWidth = 0.01;
    ctx.strokeStyle = '#2c3e50';
    ctx.stroke();
    ctx.lineWidth = 0.2;
    ctx.beginPath();
    ctx.arc(0, 0, 0.4, 0.5 * Math.PI, 1 * Math.PI, false);
    ctx.stroke()
    ctx.beginPath();
    ctx.arc(0, 0, 0.4, 1.5 * Math.PI, 2 * Math.PI, false);
    ctx.stroke()
}

function drawGrid(ctx, delta, width) {
    ctx.lineWidth = width;
    ctx.strokeStyle = '#ecf0f1';
    // TODO: use extent modulus delta instead of fixed limits
    // get extent from inverse mapping from canvas (0,0) and (width, height)
    /*
    let xmin = delta * Math.floor((extent.x.min - 1) / delta);
    let xmax = delta * Math.floor((extent.x.max + 1) / delta);
    let ymin = delta * Math.floor((extent.y.min - 1) / delta);
    let ymax = delta * Math.floor((extent.y.max + 1) / delta);
    console.log("grid", delta, xmin, xmax, ymin, ymax);
    */
    // console.log("extent", extent)
    function vline(i) {
      ctx.beginPath();
      ctx.moveTo(i, -100);
      ctx.lineTo(i, +100);
      ctx.stroke();
    }

    function hline(i) {
      ctx.beginPath();
      ctx.moveTo(-100, i);
      ctx.lineTo(+100, i);
      ctx.stroke();
    }

    for (i = -100; i <= 100; i += delta) {
      vline(i);
      hline(i);
    }
}

function robot_at(x, y, angle) {
  trail_at(x, y);

  layer2.save();
  layer2.resetTransform();
  layer2.clearRect(0, 0, width, height);
  layer2.restore();

  robot(x, y, angle);
}


function autoscale(exterior) {
    // console.log(exterior);
    let xs = exterior.map(p => p.x);
    let ys = exterior.map(p => p.y);
    let xmin = Math.min(...xs);
    let ymin = Math.min(...ys);
    let xmax = Math.max(...xs);
    let ymax = Math.max(...ys);
    let dx = xmax - xmin;
    let dy = ymax - ymin;

    console.log("Site extent", xmin, ymin, xmax, ymax);

    // 1m buffer on each side of exterior
    let scalex = width / (dx + 2);
    let scaley = height / (dy + 2);

    if (scalex > scaley) {
        scale = scaley;
        width = (dx + 2) * scale;
    }
    else {
        scale = scalex;
        height = (dy + 2) * scale;
    }

    // Auto-size canvas to fit exterior
    document.getElementById('layer1').width = width;
    document.getElementById('layer2').width = width;
    document.getElementById('layer1').height = height;
    document.getElementById('layer2').height = height;

    // console.log("scale", scale);
    tx = -xmin + 1;
    ty = -ymax - 1;
    // console.log("translate", tx, ty);

    layer1.resetTransform();
    layer2.resetTransform();
    layer1.scale(scale, -scale);
    layer2.scale(scale, -scale);
    layer1.translate(tx, ty);
    layer2.translate(tx, ty);

    return {
        x: {min: xmin, max: xmax},
        y: {min: ymin, max: ymax},
    }
}


function drawBackground(map) {
    // console.log("drawBackground", map.exterior.length)
    extent = autoscale(map.exterior);
    drawMap(layer1, map);
    drawGrid(layer1, 1, 0.01);
    drawGrid(layer1, 5, 0.03);
    drawOrigin(layer1);
}


function drawTrail(trail) {
    console.log("trail", trail.length);
    trail.forEach(state => trail_at(state.x, state.y));
}


function canvas_clear() {
  console.log("canvas_clear");
  layer1.resetTransform();
  layer1.clearRect(0, 0, width, height);
  layer2.resetTransform();
  layer2.clearRect(0, 0, width, height);
}


// Draw background before opening web socket - in case WS is down
// default_map = [{x:0, y:0}, {}]
// drawBackground([]);

/*
// simulator
var angle = 0;

function move() {
  const centerX = 0;
  const centerY = -3;
  const R = 2;

  angle += 0.1;
  let x = centerX + R * Math.cos(angle);
  let y = centerY + R * Math.sin(angle);
  robot_at(x, y, angle + Math.PI / 2);
}

setInterval(move, 200); // start simulator
*/
