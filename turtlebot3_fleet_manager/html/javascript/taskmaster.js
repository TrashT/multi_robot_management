const ROSLIB = require("roslib");
const WEBSOCKET_PORT = 9090;
const SSH = require('simple-ssh');
const FS = require("fs");
const SOURCE_KINETIC = "source /opt/ros/kinetic/setup.bash; ";
const SOURCE_MELODIC = "source /opt/ros/melodic/setup.bash; ";
const SOURCE_WORKSPACE = "source ~/catkin_ws/devel/setup.bash; ";
const SECOND = 1000/*milliseconds*/;
const ACCEPTABLE_ROBOT_DOWNTIME = 5 * SECOND;

class Point {
	/*attributes
	int x;
	int y;
	*/
	constructor(/*int*/x, /*int*/y) {
		this.x = x;
		this.y = y;
	}
}

class Robot {
	/*attributes
	int ID;
	string model;
	string parametersText;
	string namespace;
	int lastMessageTime;
	Topic tf;
	Point position;
	float angle;
	Topic battery_state;
	float percentBattery;
	Topic routing;
	Object[] path;
	Topic scan;
	Object[] scannedPoints;
	Task busyWith;
	Date startedWorkingAt;
	*/
	constructor(/*string*/ parametersText) {
		this.initializeValuesFrom(parametersText);
		this.initializeUnprovidedValues();
		
		checkIn(this);
	}
	/*void*/ initializeValuesFrom(/*string*/ parametersText) {
		this.parametersText = parametersText;
		var json = JSON.parse(parametersText);
		this.name = json.name;
		this.model = json.model;
	}
	/*void*/ initializeUnprovidedValues() {
		this.lastMessageTime = new Date().getTime();
		this.position = new Point(0, 0);
		this.angle = 0;
		this.percentBattery = 0;
		this.busyWith = null;
		this.path = [];
		this.scannedPoints = [];
		this.position = { x : 0, y : 0 };
		this.registerIdentity();
	}
	/*void*/ registerIdentity() {
		this.ID = Robot.lowestFreeID();

		this.setNamespace();
	}
	static /*int*/ lowestFreeID() {
		var ID = 0;
		
		while (Robot.someoneAlreadyHasID(ID)) {
			++ID;
		}
		
		return ID;
	}
	static /*bool*/ someoneAlreadyHasID(/*int*/ ID) {
		for (var index = 0; index < robots.length; ++index) {
			var robot = robots[index];
			
			if (robot.ID == ID) {
				return true;
			}
		}
		
		return false;
	}
	/*void*/ setNamespace() {		
		this.namespace = "robot" + this.ID.toString();
	}
	//functions
	/*void*/ onOdom(/*Message*/ message) {
		var position = message.pose.pose.position;
		var rotation = message.pose.pose.orientation;
		
		this.position = positionFrom(position);
		this.angle = -angleFrom(rotation);
	}
	
	/*void*/ onBatteryState(/*Message*/ message) {
		this.percentBattery = message.percentage;
	}
	
	/*void*/ onPath(/*Message*/ message) {
		var poses = message.poses;
		this.path = [];

		for (var i = 0; i < poses.length; ++i) {
			var position = poses[i].pose.position;

			this.path.push( {
				x : position.x,
				y : position.y
			});
		}
	}
	
	/*void*/ onScan(/*Message*/ message) {
		var ranges = message.ranges;
		this.scannedPoints = [];
		var i = 0;

		for (var angle = message.angle_min; angle <= message.angle_max && i < ranges.length; angle += message.angle_increment) {
			var range = ranges[i];
			var absoluteAngle = angle - this.angle;
			
			var xCoord = Math.cos(absoluteAngle) * range + this.position.x;
			var yCoord = Math.sin(absoluteAngle) * range + this.position.y;

			this.scannedPoints.push( {
				x : xCoord,
				y : yCoord
			});

			++i
		}
	}

	/*void*/ retire() {
		this.unsubscribe();
		bringdown(this);
		removeFromSystem(this);
	}
	/*void*/ unsubscribe() {
		this.tf.unsubscribe();
		this.battery_state.unsubscribe();
		this.routing.unsubscribe();
		this.scan.unsubscribe();
	}

	/*Object*/ toJSONobject() {
		var JSONobject = {
			name : this.name,
			namespace : this.namespace,
			x : this.position.x,
			y : this.position.y,
			angle : this.angle,
			battery : this.percentBattery,
			path : this.path,
			scan : this.scannedPoints
		};

		return JSONobject;
	}

	/*bool*/ isWorkingOn(/*int*/ taskID) {
		if (this.busyWith == null) {
			return false;
		}
		
		return this.busyWith.ID == taskID;
	}

	/*bool*/ isFullerThan(/*Robot*/ robot) {
		return this.percentBattery > robot.percentBattery;
	}

	/*void*/ startDoing(/*Task*/ task) {
		this.startedWorkingAt = new Date();
		this.busyWith = task;
		this.goToOriginOf(task);
		this.amIdoneYet();
	}
	/*void*/ goToOriginOf(/*Task*/ task) {		
		var goalTopic = createTopicFor(this, "/move_base/goal");

		var x = task.subtasks[0].x;
		var y = task.subtasks[0].y;

		var navGoal = Robot.newNavGoal(x, y);

		console.log(JSON.stringify(navGoal));
		
		goalTopic.publish(navGoal);		
	}
	static /*MoveBaseActionGoal*/ newNavGoal(/*int*/ destinationX, /*int*/ destinationY) {
		var goal = new ROSLIB.Message({
			header : {
				seq : 0,
				stamp : {
					secs: 0,
					nsecs: 0
				},      
				frame_id : ''
			},
			goal_id : {
				stamp : {
					secs: 0,
					nsecs: 0
				},      
				id : ''
			},
			goal : {
				target_pose : {
					header : {
						seq : 0,
						stamp : {
							secs: 0,
							nsecs: 0
						},      
						frame_id : "map"
					},
					pose : {
						position : {
							x : destinationX,
							y : destinationY,
							z : 0.0
						},
						orientation : {
							w : 0.0,
							x : 0.0,
							y : 0.0,
							z : 1.0
						}
					}
				}
			}
		});
		
		return goal;
	}
	/*void*/ amIdoneYet() {
		var robot = this;
		
		if (this.currentSubtaskIsDone()) {
			this.checkIfTaskIsDone();
		} else {
			robot.askAgainLater();
		}
	}
	/*bool*/ currentSubtaskIsDone() {
		if (this.busyWith == null) {
			return true;
		}
		return new Date().getTime() - this.startedWorkingAt.getTime() >= this.busyWith.subtasks[0].duration;
	}
	/*void*/ checkIfTaskIsDone() {
		var task = this.busyWith;
		
		if (task == null || task.subtasks.length == 1) {
			this.stopWorking()
		} else {
			this.startWorkingOnNextSubtaskOf(task);
		}
	}
	/*void*/ startWorkingOnNextSubtaskOf(/*Task*/ task) {
		task.subtasks.splice(0, 1);
		
		this.startDoing(task);
	}		
	/*void*/ stopWorking() {
		cancel(this.busyWith);
		this.busyWith = null;
	}
	/*void*/ askAgainLater() {
		var robot = this;
		setTimeout(function() { robot.amIdoneYet(); }, SECOND);
	}
}

//exports
/*void*/ exports.recruit = function(/*string*/ text, /*WritableStream*/ response) {
	var robot = addRobotFromJSON(text, true);

	returnText(robot.name + " should be available momentarily.  You may then click its button to toggle WASD manual control.", response);	
};
function /*void*/ returnText(/*string*/ text, /*WritableStream*/ response) {
	writeDefaultHeader(response);
	
	response.end(text);
}
function /*JSON*/ writeDefaultHeader(/*WritableStream*/ response) {
	writeHeader(200, response);
}
function /*JSON*/ writeHeader(/*int*/ statusCode, /*WritableStream*/ response) {
	response.writeHead(
		statusCode, {
			'Content-Type': 'application/json',
			'Access-Control-Allow-Origin' : '*'
		}
	);	
}
function /*Robot*/ addRobotFromJSON(/*string*/ line, /*bool*/ isntBroughtUp) {
	var robot = new Robot(line);
	
	if (isntBroughtUp) {	
		console.log("Trying to bringup " + robot.name + "...");
		bringup(robot);
	}

	registerWithSystem(robot);

	return robot;
}
function /*void*/ bringup(/*Robot*/ robot) {
	var connection = connectToRobot(robot.parametersText);
	
	launchBringup(connection, robot);			
}
function /*SSH*/ connectToRobot(/*string*/ text) {
	var parameters = JSON.parse(text);
	
	var connection = newSSHconnection(parameters);
	
	return connection;
}
function /*SSH*/ newSSHconnection(/*object*/ parameters) {
	var connection = new SSH({
		host: parameters.host,
		user: parameters.user,
		pass: parameters.pass
	});

	console.log("Connected to " + parameters.user + "@" + parameters.host);

	return connection;
}
function /*void*/ launchBringup(/*SSH*/ connection, /*Robot*/ robot) {
	var commandLine = assembleBringupCommandLineFor(robot);
	
	executeViaSSH(connection, commandLine);
}
function /*string*/ assembleBringupCommandLineFor(/*Robot*/ robot) {
	var namespace = robot.namespace;
	var hostname = JSON.parse(robot.parametersText).host;

	var commandLine = SOURCE_KINETIC + SOURCE_MELODIC + SOURCE_WORKSPACE;

	commandLine += "export ROS_HOSTNAME=" + hostname + "; ";

	commandLine += "export ROS_IP=" + hostname + "; ";

	commandLine += "export ROS_MASTER_URI=http://" + SERVER_IP + ":11311; ";

	commandLine += "export TURTLEBOT3_MODEL=" + robot.model + "; ";

	commandLine += "ROS_NAMESPACE=" + namespace + " roslaunch --wait turtlebot3_bringup turtlebot3_robot.launch multi_robot_name:=" + namespace + " set_lidar_frame_id:=" + namespace + "/base_scan";

	commandLine += "& ROS_NAMESPACE=" + namespace + " rosrun astra_camera astra_camera_node";

	return commandLine;
}
function /*void*/ executeViaSSH(/*SSH*/ connection, /*string*/ command) {
	queueCommand(connection, command);
	
	connection.start();
}
function /*void*/ queueCommand(/*SSH*/ connection, /*string*/ command) {
	connection.exec(
		command, {
			out: function (stdout) { 
				console.log(stdout); 
			},
			err: function (stderr) { 
				console.log(stderr);
			},
			exit: function (code) {
				console.log(code);
			}
		}
	);
}
function /*void*/ registerWithSystem(/*Robot*/ robot) {
	workspaceProvider.assembleWorkspaceFor(robot.ID, robot.model);
	
	robots.push(robot);
}

function /*void*/ bringdown(/*Robot*/ robot) {
	var connection = connectToRobot(robot.parametersText);
	var commandLine = assembleBringdownCommandLineFor(robot);

	executeViaSSH(connection, commandLine);
}
function /*string*/ assembleBringdownCommandLineFor(/*Robot*/ robot) {
	var commandLine = SOURCE_KINETIC + SOURCE_MELODIC + SOURCE_WORKSPACE;

	commandLine += "rosnode kill /" + robot.namespace + "/turtlebot3_core &";
	commandLine += "rosnode kill /" + robot.namespace + "/turtlebot3_diagnostics &";
	commandLine += "rosnode kill /" + robot.namespace + "/turtlebot3_lds &";
	commandLine += "rosnode kill /" + robot.namespace + "/astra_camera";

	return commandLine;
}

/*void*/ exports.recruitAutonomous = function(/*string*/ text, /*WritableStream*/ response) {
	returnText("OK", response);

	addRobotFromJSON(text, false);	
}

/*void*/ exports.dismiss = function(/*string*/ text, /*WritableStream*/ response) {
	var robot = getRobot(text);

	if (robot != null) {
		robot.retire();
		returnText("Dismissed robot #" + text + ".", response);
	} else {
		returnText("Robot #" + text + " already retired.", response);
	}
}
function /*Robot*/ getRobot(/*string*/ namespace) {
	for (var i = 0; i < robots.length; ++i) {
		var robot = robots[i];

		if (robot.namespace == namespace) {
			return robot;
		}
	}

	return null;
}

/*JSONobject[]*/ exports.visualize = function() {
	var data = [];

	for (var i = 0; i < robots.length; ++i) {
		var robot = robots[i];
		
		data.push(robot.toJSONobject());
	}
	
	return data;
}

/*void*/ exports.startWorkingOn = function(/*Task*/ task) {
	var leniency = task.leniency + 5 * SECOND;
	var now = new Date().getTime();

	var delay = now - task.start;

	if (delay > leniency) {
		cancel(task);
	} else {
		task.isAssigned = true;
		assignToRobot(task);
	}		
}
function /*void*/ assignToRobot(/*Task*/ task) {
	robot = mostFullyChargedAvailableRobot();

	if (robot != null) {	
		robot.startDoing(task);
	} else {
		setTimeout(() => { exports.startWorkingOn(task); }, SECOND);
	}
}
function /*Robot*/ mostFullyChargedAvailableRobot() {
	var candidates = availableRobots();

	return mostFullyChargedOutOf(candidates);
}
function /*Robot[]*/ availableRobots() {
	var available = robots.filter(r => r.busyWith == null);
		
	return available;
}
function /*Robot*/ mostFullyChargedOutOf(/*Robot[]*/ candidates) {
	if (candidates.length == 0) {
		return null;
	}
	
	var bestCandidate = candidates[0];
	
	for (var i = 1; i < candidates.length; ++i) {
		var candidate = candidates[i];
		
		if (candidate.isFullerThan(bestCandidate)) {
			bestCandidate = candidate;
		}
	}
	
	return bestCandidate;
}

/*void*/ exports.stopWorkingOn = function(/*int*/ taskID) {
	for (var i = 0; i < robots.length; ++i) {
		var robot = robots[i];
		
		if (robot.isWorkingOn(taskID)) {
			robot.busyWith = null;
			
			break;
		}
	}
}

//private functions
function /*void*/ checkIn(/*Robot*/ robot) {
	subscribeToPosition(robot);
	subscribeToBattery(robot);
	subscribeToPath(robot);
	subscribeToScan(robot);
}
function /*void*/ subscribeToPosition(/*Robot*/ robot) {
	var topicName = "/amcl_pose";
	
	robot.tf = subscribeToTopic(robot, topicName, function(message) { robot.onOdom(message); });
}
function /*Topic*/ subscribeToTopic(/*Robot*/ robot, /*string*/ topicName, /*function(message)*/ onMessage) {
	var topic = createTopicFor(robot, topicName);
	
	topic.subscribe(
		function(message) { 
			robot.lastMessageTime = new Date().getTime(); 
			onMessage(message); 
		}
	);
	
	return topic;
}
function /*Topic*/ createTopicFor(/*Robot*/ robot, /*string*/ topicName) {
	var qualifiedName = "/" + robot.namespace + topicName;

	var topic = new ROSLIB.Topic({
		ros : ROS,
		name : qualifiedName,
		messageType : workspaceProvider.messageTypeOf(topicName)
	});
	
	return topic;
}
function /*void*/ subscribeToBattery(/*Robot*/ robot) {
	var topicName = "/battery_state";
		
	robot.battery_state = subscribeToTopic(robot, topicName, function(message) { robot.onBatteryState(message); });
}
function /*void*/ subscribeToPath(/*Robot*/ robot) {
	var topicName = "/move_base/NavfnROS/plan";
	
	robot.routing = subscribeToTopic(robot, topicName, function(message) { robot.onPath(message); });
}
function /*void*/ subscribeToScan(/*Robot*/ robot) {
	var topicName = "/scan";
	
	robot.scan = subscribeToTopic(robot, topicName, function(message) { robot.onScan(message); });
}

function /*Point*/ positionFrom(/*Translation*/ translation) {
	var x = translation.x;
	var y = translation.y;
	
	var coordinates = new Point(x, y);
	
	return coordinates;
}

function /*float*/ angleFrom(/*Rotation*/ rotation) {
	var w = rotation.w;
	var x = rotation.x;
	var y = rotation.y;
	var z = rotation.z;
	
	var siny_cosp = 2 * (w * z + x * y);
	var cosy_cosp = 1 - 2 * (y * y + z * z);
	
	var angle = Math.atan2(siny_cosp, cosy_cosp);
	
	return angle;
}

function /*void*/ removeFromSystem(/*Robot*/ robot) {
	workspaceProvider.dismantleWorkspace(robot.namespace);
	removeFromStack(robot);
}
function /*void*/ removeFromStack(/*Robot*/ robot) {
	robots = robots.filter(r => r.ID != robot.ID);
}

function /*void*/ cancel(/*Task*/ task) {
	if (task != null) {
		taskmaster.cancel(task.ID.toString() + ";0", null);
	}
}

/*void*/ exports.initialize = function(/*taskmaster*/ tm) {
	taskmaster = tm;
	console.log("Taskmaster for RR manager initialized!");
	SERVER_IP = FS.readFileSync("IP.txt");
	workspaceProvider = require("./workspace provider");
	
	workspaceProvider.createWorkEnvironment(SERVER_IP);
	
	connectToROS();
}
function /*void*/ connectToROS() {
	ROS = new ROSLIB.Ros({
		url : "ws://" + SERVER_IP + ":" + WEBSOCKET_PORT
	});
	
	ROS.on('connection', startWatchingEverybody);
	ROS.on('error', function(error) { console.log(error); });
	ROS.on('close', function() { console.log("Websocket closed.  Reconnecting..."); connectToROS(); });
}
function /*void*/ startWatchingEverybody() {
	console.log("RR manager reporting for duty!");
	keepWatchingEverybody();
}
function /*void*/ keepWatchingEverybody() {
	setTimeout(function() { watchEverybody(); }, SECOND);
}
function /*void*/ watchEverybody() {
	var now = new Date().getTime;
	
	for (var i = 0; i < robots.length; ++i) {
		var robot = robots[i];
		
		if (hasBeenSilentForTooLong(now, robot)) {
			robot.dismiss();
		}
	}		
	
	keepWatchingEverybody();
}
function /*bool*/ hasBeenSilentForTooLong(/*int*/ now, /*Robot*/ robot) {
	return now - robot.lastMessageTime > ACCEPTABLE_ROBOT_DOWNTIME;
}

var SERVER_IP;
var ROS;
var taskmaster;
var workspaceProvider;
var robots = [];

console.log(JSON.stringify(Robot.newNavGoal(3,14)));