var robotMarkerRadius = 0.3; //The radius of the circle that marks the robot's location, in meters.
var robotMarkerArrowAngle = Math.PI/6; //There's an arrow on the circle, showing which direction the robot is pointing. This is the angle between the centerline and one of the sides.
var pointsRecord = []; //This record the list of 2D point where the robot has been, so the program can draw lines between them.
var scaleFactor = 100; //As the path and information get bigger, it's useful to zoom out.
var positionOffset = [0, 0]; //This is used to keep the robot's location on the screen centered.

var canvas;
var context;
var ws;

function setup() { //Call this to get the program going.
	canvas = document.getElementById("map"); //Grab the HTMl of the canvas.
	canvas.style.transform = "matrix(0, -1, 1, 0, 0, 0)"; //Rotate the canvas so up is forward, like in a map.
	context = canvas.getContext("2d"); //All canvas drawings are done through a context.
	context.fillStyle = "white"; //Set the fill style of closed shapes on the canvas to white.
	context.beginPath(); //This starts a path so lines can be drawn.
	document.getElementById("connect").addEventListener("click", startServerConnection);
}
function mainLoop(data) {	
	var temp = data.split("|");

	//Store the x, y, and z position in a separate variable.
	positionXYZ = [temp[0], temp[1], temp[2]];

	//Store the x, y, z, and w quaternion in a separate variable.
	quaternionXYZW = [temp[3], temp[4], temp[5], temp[6]];

	var eulerAngles = quaternionToEuler(quaternionXYZW); //Convert the quaternion to euler angles.
	var theta = eulerAngles[0]; //This is the XY-plane angle actually used, rotated 90 degrees so that forward is up instead of right.

	pointsRecord.push([positionXYZ[0], positionXYZ[1]]); //Store the next point to the list.

	context.lineWidth = 1/scaleFactor; //Make sure the lines don't freak out.

	context.setTransform(1, 0, 0, 1, 0, 0); //Reset all transforms on the context.
	context.clearRect(0, 0, canvas.width, canvas.height); //Clear the canvas.
	context.transform(1, 0, 0, 1, canvas.width/2, canvas.height/2); //Put 0, 0 in the center of the canvas.
	context.transform(scaleFactor, 0, 0, scaleFactor, 0, 0); //Scale the canvas.
	context.transform(1, 0, 0, -1, 0, 0); //Flip the canvas so y+ is up.

	context.moveTo(pointsRecord[0][0]-positionXYZ[0], pointsRecord[0][1]-positionXYZ[1]); //Move to the first point in the path.
	context.beginPath();
	for(var i=1; i<pointsRecord.length; ++i) { //This draws lines from point i to point i-1
		context.lineTo(pointsRecord[i][0]-positionXYZ[0], pointsRecord[i][1]-positionXYZ[1]); //Draw a line to the next point.
		context.stroke();
	}

	context.transform(Math.cos(theta), Math.sin(theta), -Math.sin(theta), Math.cos(theta), 0, 0); //Orient the path behind the robot properly.
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


	requestAnimationFrame(sendDataRequest); //When using data directly from the robot, use requestAnimationFrame().
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
		//console.log(event.data);
		mainLoop(event.data); //Go into the main loop and use the data.
	}
	ws.onopen = function() {
		console.log("Connection opened.");
		sendDataRequest();
	}
}

setup();