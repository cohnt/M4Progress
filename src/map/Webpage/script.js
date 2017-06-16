var robotMarkerRadius = 0.3; //The radius of the circle that marks the robot's location, in meters.
var robotMarkerArrowAngle = Math.PI/6; //There's an arrow on the circle, showing which direction the robot is pointing. This is the angle between the centerline and one of the sides.
var pointsRecord = []; //This record the list of 2D point where the robot has been, so the program can draw lines between them.
var zoom = 100; //As the path and information get bigger, it's useful to zoom out.
                //If zoom is 1, then 1px = 1m. If zoom is 100, then 1px = 1cm.
                //In other words, the units are pixels per meter.
var positionOffset = [0, 0]; //This is used to keep the robot's location on the screen centered.
var yawIndex = 0; //This is the index in the returned Euler angle array (from quaternionToEuler) where the yaw is indexed.

var canvas; //A global variable 
var context;
var ws;

function serverMessage(msg) {
	var splitMsg = msg.split("|"); //The message is pipe-delimited, as commas are used in the range list.

	this.position = splitMsg.slice(0, 3);
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
}

function setup() { //Call this to get the program going.
	canvas = document.getElementById("map"); //Grab the HTMl of the canvas.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path so lines can be drawn.
	document.getElementById("connect").addEventListener("click", startServerConnection);
}
function mainLoop(message) {
	var data = new serverMessage(message);

	pointsRecord.push(data.position.slice(0,2)); //Store the next point to the list.

	context.lineWidth = 1/zoom; //Make sure the lines are proper thickness given the zoom factor.
	context.fillStyle = "#eeeeee"; //Fill the screen (by default) with grey.
	context.setTransform(1, 0, 0, 1, 0, 0); //Reset all transforms on the context.
	context.clearRect(0, 0, canvas.width, canvas.height); //Clear the canvas.
	context.fillRect(0, 0, canvas.width, canvas.height); //Give the canvas its default background.
	context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2); //Put 0, 0 in the center of the canvas.
	context.transform(zoom, 0, 0, zoom, 0, 0); //Scale the canvas.
	context.transform(1, 0, 0, -1, 0, 0); //Flip the canvas so y+ is up.

	drawRanges(data.ranges.slice(0), data.minAngle, data.incrementAngle);
	context.transform(Math.cos(-data.angle), Math.sin(-data.angle), -Math.sin(-data.angle), Math.cos(-data.angle), 0, 0);
	context.transform(1, 0, 0, 1, -data.position[0], -data.position[1]);
	drawRobotPath();
	context.transform(1, 0, 0, 1, data.position[0], data.position[1]);
	context.transform(Math.cos(data.angle), Math.sin(data.angle), -Math.sin(data.angle), Math.cos(data.angle), 0, 0);
	drawRobotMarker();

	requestAnimationFrame(sendDataRequest);
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
		mainLoop(event.data); //Go into the main loop and use the data.
	}
	ws.onopen = function() {
		console.log("Connection opened.");
		sendDataRequest();
	}
}
function drawRobotMarker() {
	context.beginPath();
	context.arc(0, 0, robotMarkerRadius, 0, 2*Math.PI); //This will draw a circle around the center for the robot marker.
	context.stroke();

	//These lines draw a triangle inside the circle, to show the direction of the robot.
	context.beginPath();
	context.moveTo(robotMarkerRadius*Math.cos(0), robotMarkerRadius*Math.sin(0));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotMarkerRadius*Math.cos(0), robotMarkerRadius*Math.sin(0));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
	context.moveTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.lineTo(robotMarkerRadius*Math.cos(Math.PI-robotMarkerArrowAngle), -robotMarkerRadius*Math.sin(Math.PI-robotMarkerArrowAngle));
	context.stroke();
}
function drawRanges(r, tMin, tInc, tRobot) {
	var t = tMin;

	r[0] = [r[0]*Math.cos(t), -r[0]*Math.sin(t), false];
	for(var i=1; i<r.length; ++i) {
		t += tInc;
		r[i] = [r[i]*Math.cos(t), -r[i]*Math.sin(t)];
		if(r[i-1][0] == Infinity || r[i-1][0] == -Infinity || isNaN(r[i-1][0])) {
			r[i].push(false);
			r.splice(i-1, 1);
			--i;
		}
		else {
			r[i].push(true);
		}
	}
	context.beginPath();
	for(var i=1; i<r.length; ++i) {
		if(r[i][2]) {
			addRangeLineSegment(r[i-1], r[i]);
		}
	}
	context.stroke();
	drawRangesFill(r);
}
function addRangeLineSegment(p0, p1) {
	context.moveTo(p0[0], p0[1]);
	context.lineTo(p1[0], p1[1]);
}
function drawRangesFill(r) {
	context.beginPath();
	context.moveTo(0, 0);

	for(var i=1; i<r.length; ++i) {
		context.lineTo(r[i][0], r[i][1]);
	}
	context.lineTo(0, 0);
	
	context.fillStyle = "white";
	context.closePath();
	context.fill();
}
function drawRobotPath() {
	context.moveTo(pointsRecord[0][0], pointsRecord[0][1]); //Move to the first point in the path.
	context.beginPath();
	for(var i=1; i<pointsRecord.length; ++i) { //This draws lines from point i to point i-1
		context.lineTo(pointsRecord[i][0], pointsRecord[i][1]); //Draw a line to the next point.
		context.stroke();
	}
}

setup();