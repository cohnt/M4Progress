//Constants
var robotChassisRadius = 0.3; //The radius of the circle that marks the robot's location, in meters.
var robotMarkerArrowAngle = Math.PI/6; //There's an arrow on the circle, showing which direction the robot is pointing. This is the angle between the centerline and one of the sides.
var yawIndex = 0; //This is the index in the returned Euler angle array (from quaternionToEuler) where the yaw is indexed.
var lidarForwardDistance = 0.2; //This is the distance between the robot's odometry center and the lidar module in the front, in meters. This is approximate.
var minPositionRecordDistance = Math.pow(0.02, 2); //This is how much you have to move before the position is recorded again.
var wallsFillMinDistanceSquaredFromCenter = Math.pow(0.05, 2); //This is how far away a point must be from the center of the lidar module to be considered legit.
var maxWallRenderConnectedDistance = Math.pow(0.1, 2); //This is how close points must be together to be considered a connected wall.
var epsilonFloat = 0.001; //The epsilon value used for floating-point values from the C++ backend.
var zoomScrollConstant = 120 * 4; //This depends on which mouse you use. For my mouse, one scrolled "tic" has |e.wheelDelta|=120.
var cylonModeCycleTime = 2.5 * 1000; //How long the visual scan takes to do a complete loop in cylon mode, in milliseconds.
var cylonModeNumLines = 6; //How many wall segments are highlighted by the cylon radar gradient.
var styles = {
	robotMarker: "#000000",
	robotPath: "#888888",
	wallLines: "#000000",
	wallFill: "#ffffff",
	background: "#eeeeee",
	robotFOV: "#42f4e2", //A sort of electic blue.
	cylon: ["#ff5555", "#ff7777", "#ff9999", "#ffaaaa", "#ffcccc", "#ffeeee"]
};
var userRotationRadPerSec = Math.PI;
var keycodes = {
	q: 81,
	e: 69
}
var maxUserTransformationTimeElapsed = 0.5 * 1000; //The maximum number of milliseconds the transformation update will accept for user-inputted rotation transformations.
var printMatrixMinColWidth = 20;

//Global variables.
var canvas; //A global variable 
var context;
var ws;
var page = {}; //An object which holds all grabbed html elements from the page in one nice, central location.
var path = []; //This record the list of 2D point where the robot has been, so the program can draw lines between them.
var zoom = 128; //As the path and information get bigger, it's useful to zoom out.
                //If zoom is 1, then 1px = 1m. If zoom is 100, then 1px = 1cm.
                //In other words, the units are pixels per meter.
var cylonMode = false;
var cylonModeStartTime;
var lastDataMessage;
var firstTransmission = true;
var keys = {};
var t0;
var currentUserDisplayTransform = [[zoom, 0, 0], [0, zoom, 0], [0, 0, 1]]; //By default, it's the zoom matrix for the initial zoom level.
var rotating = false;
var panning = false;
var mouseCurrentPosition = [0, 0];
var mouseLastPosition = [0, 0];

//Classes
function serverMessage(msg) {
	var splitMsg = msg.split("|"); //The message is pipe-delimited, as commas are used in the range list.

	this.position = splitMsg.slice(0, 2);
	this.position.push(1);
	this.quaternion = splitMsg.slice(3, 7);
	for(var i=0; i<this.position.length; ++i) {
		this.position[i] = Number(this.position[i]);
	}
	for(var i=0; i<this.quaternion.length; ++i) {
		this.quaternion[i] = Number(this.quaternion[i]);
	}
	this.euler = quaternionToEuler(this.quaternion);
	this.angle = euler[yawIndex];

	this.minAngle = Number(splitMsg[7]);
	this.maxAngle = Number(splitMsg[8]);
	this.incrementAngle = Number(splitMsg[9]);

	this.ranges = splitMsg[10].split(",");
	for(var i=0; i<this.ranges.length; ++i) {
		this.ranges[i] = Number(this.ranges[i]);
		if(isNaN(this.ranges[i])) {
			this.ranges[i] = Infinity;
		}
	}

	this.walls = splitMsg[11].split(";");
	for(var i=0; i<this.walls.length; ++i) {
		this.walls[i] = this.walls[i].slice(1, -1).split(",");
		this.walls[i][0] = Number(this.walls[i][0]); this.walls[i][1] = Number(this.walls[i][1]);
		var doNotNeed1 = Math.abs(this.walls[i][0]-this.position[0]) < epsilonFloat && Math.abs(this.walls[i][1]-this.position[1]) < epsilonFloat;
		var doNotNeed2 = isNaN(this.walls[i][0]) || isNaN(this.walls[i][1]);
		if(doNotNeed1 || doNotNeed2) {
			this.walls.splice(i, 1);
			--i;
		}
		else {
			this.walls[i].push(1);
			this.walls[i] = numeric.dot(makeTranslationMatrix(0, lidarForwardDistance), this.walls[i]);
		}
	}
}

//Functions
function setup() {
	page.canvas = document.getElementById("map"); //Grab the HTMl of the canvas.
	page.connectButton = document.getElementById("connect");
	page.cylonModeButton = document.getElementById("cylon");

	page.canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = page.canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.beginPath(); //This starts a path so lines can be drawn.

	page.connectButton.addEventListener("click", startServerConnection);
	page.cylonModeButton.addEventListener("click", toggleCylonMode);
	document.addEventListener("keydown", function(event) { keydown(event); });
	document.addEventListener("keyup", function(event) { keyup(event); });
	page.canvas.addEventListener("wheel", function(event) { zoomed(event); });
	document.body.addEventListener("mousemove", function(event) { mouseMoved(event); });
	page.canvas.addEventListener("mousedown", function(event) { canvasClicked(event); });
	document.body.addEventListener("mouseup", function(event) { clickReleased(event); });
}
function mainLoop() {
	var t = window.performance.now();
	var dt = t - t0; //In milliseconds
	t0 = t;
	var data = lastDataMessage;

	context.lineWidth = 1/zoom; //Make sure the lines are proper thickness given the zoom factor.
	context.fillStyle = styles.background; //Fill the screen (by default) with grey.
	context.setTransform(1, 0, 0, 1, 0, 0);
	context.clearRect(0, 0, page.canvas.width, page.canvas.height);
	context.fillRect(0, 0, page.canvas.width, page.canvas.height);
	context.transform(1, 0, 0, 1, page.canvas.width/2, page.canvas.height/2);
	context.transform(1, 0, 0, -1, 0, 0);
	var viewportTransform = computeViewportTransform(dt, data);
	context.transform(
		viewportTransform[0][0],
		viewportTransform[1][0],
		viewportTransform[0][1],
		viewportTransform[1][1],
		viewportTransform[0][2],
		viewportTransform[1][2]
	);

	if(path.length == 0 || distanceSquared(data.position, path[path.length-1]) > minPositionRecordDistance) {
		path.push(data.position.slice(0)); //Store the next point to the list.
	}

	drawWalls(data.walls);
	drawRobotPath();
	drawRobotMarker();
	drawRobotFrameOfView(data.walls);
	if(cylonMode) {
		context.lineWidth *= 2;
		drawCylonRadar(data.walls, t);
		context.lineWidth /= 2;
	}

	requestAnimationFrame(mainLoop);
}

function sendDataRequest() {
	ws.send("ready");
	//When this message is sent, the server knows that the webpage is ready to process more data.
	//The server will then proceed to send the most recent data avaiable.
}
function quaternionToEuler(quat) { //This takes the quaternion array [x, y, z, w] and returns the euler array [φ, θ, ψ]
	//The quaternion describes the orientation and rotation of the robot, but it's very complicated.
	//These formulas convert the XYZW quaternion into φθψ (phi-theta-psi) euler angles.
	//It's easiest to think of them as φ=pitch (index 0), θ=roll (index 1), and ψ=yaw (index 2).
	//These formulas were taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion

	euler = [];
	euler[0] = Math.atan2(2*((quat[0]*quat[1]) + (quat[2]*quat[3])), 1-(2*((quat[1]*quat[1]) + (quat[2]*quat[2]))));
	euler[1] = Math.asin(2*((quat[0]*quat[2]) - (quat[3]*quat[1])));
	euler[2] = Math.atan2(2*((quat[0]*quat[3]) + (quat[1]*quat[2])), 1-(2*((quat[2]*quat[2]) + (quat[3]*quat[3]))));
	return euler;
}
function startServerConnection() {
	ws = new WebSocket(document.getElementById("serverAddress").value); //This creates the websocket object.
	ws.onmessage = function(event) { //When a message is received...
		lastDataMessage = new serverMessage(event.data); //Update the last data message.
		requestAnimationFrame(sendDataRequest);
		if(firstTransmission) {
			firstTransmission = false;
			t0 = window.performance.now();
			requestAnimationFrame(mainLoop);
		}
	}
	ws.onopen = function() {
		console.log("Connection opened.");
		sendDataRequest();
	}
}
function drawRobotPath() {
	context.strokeStyle = styles.robotPath
	context.moveTo(path[0][0], path[0][1]); //Move to the first point in the path.
	context.beginPath();
	for(var i=1; i<path.length; ++i) { //This draws lines from point i to point i-1
		context.lineTo(path[i][0], path[i][1]); //Draw a line to the next point.
		context.stroke();
	}
}
function drawWalls(walls) {
	context.strokeStyle = styles.wallLines;
	context.beginPath();
	context.moveTo(walls[0][0], walls[0][1]);
	for(var i=1; i<walls.length; ++i) {
		if(distanceSquared(walls[i], walls[i-1]) < maxWallRenderConnectedDistance) {
			context.lineTo(walls[i][0], walls[i][1]);
		}
		else {
			context.stroke();
			context.beginPath();
			context.moveTo(walls[i][0], walls[i][1]);
		}
	}
	context.stroke();
	drawWallsFill(walls);
}
function addRangeLineSegment(p0, p1) {
	context.moveTo(p0[0], p0[1]);
	context.lineTo(p1[0], p1[1]);
}
function drawWallsFill(walls) {
	context.beginPath();
	context.moveTo(0, 0);

	for(var i=1; i<walls.length; ++i) {
		if(distanceSquared(walls[i], [0, 0, 0]) > wallsFillMinDistanceSquaredFromCenter) {
			context.lineTo(walls[i][0], walls[i][1]);
		}
	}
	context.lineTo(0, 0);
	
	context.fillStyle = styles.wallFill;
	context.closePath();
	context.fill();

	context.beginPath();
	context.moveTo(0, 0);
	context.lineTo(walls[0][0], walls[0][1]);
	context.lineTo(walls[1][0], walls[1][1]);
	context.lineTo(0, 0);
	context.closePath();
	context.fill();

	context.beginPath();
	context.moveTo(0, 0);
	context.lineTo(walls[walls.length-2][0], walls[walls.length-2][1]);
	context.lineTo(walls[walls.length-1][0], walls[walls.length-1][1]);
	context.lineTo(0, 0);
	context.closePath();
	context.fill();
}
function drawRobotMarker() {
	context.strokeStyle = styles.robotMarker;
	context.beginPath();
	context.arc(0, 0, robotChassisRadius, 0, 2*Math.PI); //This will draw a circle around the center for the robot marker.
	context.stroke();

	//These lines draw a triangle inside the circle, to show the direction of the robot.
	context.beginPath();
	context.moveTo(robotChassisRadius*Math.cos(0), robotChassisRadius*Math.sin(0));
	context.lineTo(robotChassisRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotChassisRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotChassisRadius*Math.cos(0), robotChassisRadius*Math.sin(0));
	context.lineTo(robotChassisRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotChassisRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotChassisRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotChassisRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.lineTo(robotChassisRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotChassisRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
}
function drawRobotFrameOfView(walls) {
	context.strokeStyle = cylonMode ? styles.cylon[0] : styles.robotFOV;
	context.beginPath();
	context.moveTo(0, 0);
	context.lineTo(walls[0][0], walls[0][1]);
	context.moveTo(0, 0);
	context.lineTo(walls[walls.length-1][0], walls[walls.length-1][1]);
	context.stroke();
}
function drawCylonRadar(walls, t) {
	var dtCylon = (t - cylonModeStartTime) % cylonModeCycleTime;
	var i = dtCylon - (cylonModeCycleTime/2);
	var directionModifier;

	if(i <= 0) {
		i += (cylonModeCycleTime/2);
		directionModifier = 1;
	}
	else {
		i = (cylonModeCycleTime/2)-i;
		directionModifier = -1;
	}

	i /= (cylonModeCycleTime/2);
	i *= walls.length;
	i = Math.floor(i);
	
	for(var n=0; n<cylonModeNumLines; ++n) {
		var k = i + (n*directionModifier);
		if(k > 0 && k < walls.length) {
			context.beginPath();
			context.strokeStyle = styles.cylon[(cylonModeNumLines-n)-1];
			console.log(cylonModeNumLines-n);
			context.beginPath();
			context.moveTo(0, 0);
			context.lineTo(walls[k][0], walls[k][1]);
			context.stroke();
		}
	}
}
function distanceSquared(p1, p2) {
	//Useful for quickly computing distance thresholds.
	var sum = 0;
	for(var i=0; i<p1.length; ++i) {
		sum += Math.pow(p2[i]-p1[i], 2);
	}
	return sum;
}
function zoomed(e) {
	var zoomMultiplier = Math.pow(2, e.wheelDelta/zoomScrollConstant); //Raising 2 to the power of wheelDelta changes it from a positive/negative number to a number that is greater than or less than 1, and so it's fit for a scale factor.
	zoom *= zoomMultiplier;
	console.log(Math.log2(zoom));
	var zoomMatrix = [
		[zoomMultiplier, 0, 0],
		[0, zoomMultiplier, 0],
		[0, 0, 1]
	];
	currentUserDisplayTransform = numeric.dot(zoomMatrix, currentUserDisplayTransform);
}
function toggleCylonMode() {
	cylonMode = !cylonMode;
	if(cylonMode) {
		cylonModeStartTime = window.performance.now();
	}
	else {
		cylonModeStartTime = NaN;
	}
}
function keydown(e) {
	var keycode = e.which;
	keys[keycode] = true;
	rotating = updateRotation()
}
function keyup(e) {
	var keycode = e.which;
	keys[keycode] = false;
	rotating = updateRotation()
}
function updateRotation() {
	var q = keys[keycodes.q];
	var e = keys[keycodes.e];
	return q ^ e;
}
function computeRotationTransform(dt) {
	if(rotating) {
		if(dt > maxUserTransformationTimeElapsed) {
			return;
		}
		var dT;
		dT = (dt/1000) * userRotationRadPerSec;
		dT *= (keys[keycodes.e] ? -1 : 1);
		var rotMatrix = makeRotationMatrix(dT);
		currentUserDisplayTransform = numeric.dot(rotMatrix, currentUserDisplayTransform);
	}
}
function makeRotationMatrix(theta) {
	var c = Math.cos(theta);
	var s = Math.sin(theta);
	return [
		[c, -s, 0],
		[s, c, 0],
		[0, 0, 1]
	];
}
function printMatrix(matrix) {
	for(var i=0; i<matrix.length; ++i) {
		var out = "[";
		for(var j=0; j<matrix[i].length; ++j) {
			var innerOut = String(matrix[i][j]);
			while(innerOut.length < printMatrixMinColWidth) {
				innerOut += " ";
			}
			out += innerOut;
			out += ",";
		}
		out += "]";
		console.log(out);
	}
}
function mouseMoved(e) {
	mouseLastPosition = mouseCurrentPosition.slice(0);
	mouseCurrentPosition = [e.clientX, window.innerHeight - e.clientY];
	var delta = numeric.add(mouseCurrentPosition, numeric.dot(-1, mouseLastPosition));
	if(panning) {
		var transformMatrix = [
			[1, 0, delta[1]],
			[0, 1, -delta[0]],
			[0, 0, 1]
		];
		currentUserDisplayTransform = numeric.dot(transformMatrix, currentUserDisplayTransform);
	}
}
function canvasClicked(e) {
	mouseCurrentPosition = [e.clientX, window.innerHeight - e.clientY];
	panning = true;
}
function clickReleased(e) {
	//
	panning = false;
}
function computeViewportTransform(dt, data) {
	var m = makeIdentityMatrix();
	computeRotationTransform(dt);
	m = numeric.dot(currentUserDisplayTransform, m);
	var posMatrix = makeTranslationMatrix(data.position[1], -data.position[0]);
	m = numeric.dot(posMatrix, m);
	return m;
}
function makeTranslationMatrix(dx, dy) {
	return [
		[1, 0, dx],
		[0, 1, dy],
		[0, 0, 1]
	];
}
function makeIdentityMatrix() {
	return [
		[1, 0, 0],
		[0, 1, 0],
		[0, 0, 1]
	];
}


//Executed Code
setup();