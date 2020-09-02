var robots = [];
var ROS = null;
var teleop = null;
var beingControlled = "";

function /*async*/ addRobot() {
	var index = getSelectedIndex("model");

	if (index == 0) {
		alert("You need to specify the TurtleBot3 model!");
	} else {
		var robot = robotFromDocument();
		
		post(JSON.stringify(robot), "robot");

		hideInputs();
	}
}
function /*object*/ robotFromDocument() {
	var name = getValueFrom("name");
	var IP = getValueFrom("IP");
	var username = getValueFrom("username");
	var password = getValueFrom("password");
	var model = getValueFrom("model");

	if (name.length == 0) {
		name = username + "@" + IP;
	}
	
	return robotFromParameters(name, IP, username, password, model);
}
function /*object*/ robotFromParameters(/*string*/ givenName, /*string*/ IP, /*string*/ username, /*string*/ password, /*string*/ robotModel) {
	var robot = {
		name : givenName,
		host : IP,
		user : username,
		pass : password,
		model : robotModel
	}
	
	return robot;
}

function /*void*/ removeRobot(/*int*/ namespace) {
	post(namespace, "dismiss");
}

function /*void*/ startControlling(/*string*/ namespace) {
	if (beingControlled == namespace) {
		closeROS();
		beingControlled = "";
		return;
	}

	closeROS();
	ensureROSconnection();

	teleop = new KEYBOARDTELEOP.Teleop({
		ros : ROS,
		topic : '/' + namespace + '/cmd_vel',
	});

	teleop.scale = 0.5;
	beingControlled = namespace;
}
function /*void*/ closeROS() {
	if (ROS != null) {
		ROS.close();
		ROS = null;
	}
}
function /*void*/ ensureROSconnection() {
	if (ROS == null) {
		ROS = new ROSLIB.Ros({
			url : WEBSOCKET_URL
		});
	}
}

function /*void*/ toggleVideoFor(/*int*/ namespace) {
	var isChecked = getElement(namespace + "videoCheckbox").checked;

	if (isChecked) {
		addVideoFeedFor(namespace);
	} else {
		removeVideoFeedOf(namespace);
	}
}