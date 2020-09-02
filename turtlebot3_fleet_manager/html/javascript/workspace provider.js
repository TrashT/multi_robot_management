var SERVER_IP;

/*void*/ exports.createWorkEnvironment = function(/*string*/ serverIP) {
	SERVER_IP = serverIP;
	bash("startup", [SERVER_IP.toString()]);
}
function /*void*/ bash(/*string*/ scriptName, /*string[]*/ parameters) {
	parameters = ["bashfiles/" + scriptName + ".sh"].concat(parameters);
	
	const outputStream = executeCommand("bash", parameters);

	outputToConsole(outputStream);	
}
function /*ReadableStream*/ executeCommand(/*string*/ command, /*string[]*/ parameters) {
	const spawn = require('child_process').spawn/*, child*/;

	const outputStream = spawn(command, parameters);

	return outputStream;
}
function /*void*/ outputToConsole(/*ReadableStream*/ outputStream) {
	stdoutToConsole(outputStream);
	stderrToConsole(outputStream);
}
function /*void*/ stdoutToConsole(/*ReadableStream*/ outputStream) {
	outputStream.stdout.on('data', function(data){
		console.log(data.toString());
	});	
}
function /*void*/ stderrToConsole(/*ReadableStream*/ outputStream) {
	outputStream.stderr.on('data', function(data){
		console.error(data.toString());
	});	
}

/*string*/ exports.messageTypeOf = function(/*string*/ topicName) {
	switch (topicName) {
		case "/move_base/NavfnROS/plan":
			return "nav_msgs/Path";
		case "/amcl_pose":
			return "geometry_msgs/PoseWithCovarianceStamped";
		case "/odom":
			return "nav_msgs/Odometry";
		case "/battery_state":
			return "sensor_msgs/BatteryState";
		case "/move_base/goal":
			return "move_base_msgs/MoveBaseActionGoal";
		case "/global_planner":
			return "nav_msgs/Path";
		case "/particlecloud":
			return "geometry_msgs/PoseArray";
		case "/scan":
			return "sensor_msgs/LaserScan";
		default:
			throw new Error("Message type of " +  topicName + " is undefined.");
	}
}

/*void*/ exports.assembleWorkspaceFor = function(/*int*/ robotID, /*string*/ TBmodel) {
	var namespace = "robot" + robotID.toString();
	
	bash("newrobot", [SERVER_IP, TBmodel, namespace]);
}

/*void*/ exports.dismantleWorkspace = function(/*string*/ namespace) {
	var readableStream = executeCommand("rosnode", ["list"]);

	readableStream.stdout.on('data', function(data){
		var nodes = data.toString().split("\n");

		for (var i = 0; i < nodes.length; ++i) {
			if (nodes[i].includes("/" +  namespace + "/")) {
				executeCommand("rosnode", ["kill", nodes[i]]);
			}
		}
	});	
}

console.log("workspace provider reporting for duty!");
