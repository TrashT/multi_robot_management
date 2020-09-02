var NETWORK_INTERFACE;
const PORT = 8081;

function /*async*/ findOutServerIPthen(/*function()*/ followup) {
	tryReadingNetworkInterface();

	const { exec } = require('child_process');

	exec(linuxCommandToEchoIP(), (error, stdout, stderr) => {
	    if (error) {
	        console.log("error: ${error.message}");
	        return;
	    }
	    if (stderr) {
	        console.log("stderr: ${stderr}");
	        return;
	    }
		extractIPfrom(stdout);
		followup();
	});	
}
function /*void*/ tryReadingNetworkInterface() {
	if (process.argv.length < 3) {
		console.log('Please provide the network interface parameter, e.g. "node server.js eno1"');
		process.exit(1);
	} else {
		NETWORK_INTERFACE = process.argv[2];
	}
}
function /*string*/ linuxCommandToEchoIP() {
	return "/sbin/ip -o -4 addr list " + NETWORK_INTERFACE + " | awk '{print $4}' | cut -d/ -f1";
}
function /*void*/ extractIPfrom(/*string*/ stdout) {
	var IP = stdout.replace("\n", "");
	
	exportIP(IP);
}
function /*void*/ exportIP(/*string*/ IP) {
	SERVER_IP = IP;
	
	require("fs").writeFileSync("IP.txt", IP);
}
function /*void*/ openShop() {
	requireModules();
	startListening();
}
function /*void*/ requireModules() {
	RRmanager = require("./RRmanager");
	taskmaster = require("./taskmaster");

	taskmaster.initialize(RRmanager);
	RRmanager.initialize(taskmaster);
}
function /*void*/ startListening() {
	SERVER.on('request', respondToRequest);

	SERVER.listen(PORT);

	console.log('Server running at http://' + SERVER_IP + ':' + PORT.toString() + '/');	
}
function /*void*/ respondToRequest(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	setDefaultErrorHandling(request, response);
	
	respondAccordingToMethod(request, response);	
}
function /*void*/ setDefaultErrorHandling(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	setDefaultRequestErrorHandling(request, response);
	setDefaultResponseErrorHandling(response);
}
function /*void*/ setDefaultRequestErrorHandling(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	request.on('error',
		(error) => {
			console.error(error);

			returnStatusCode(400, response);
		}
	);
}
function /*void*/ returnStatusCode(/*int*/ statusCode, /*WritableStream*/ response) {
	response.statusCode = statusCode;

	writeHeader(statusCode, response);
	
	response.end();
}
function /*JSON*/ writeHeader(/*int*/ statusCode, /*WritableStream*/ response) {
	response.writeHead(
		statusCode, {
			'Content-Type': 'application/json',
			'Access-Control-Allow-Origin' : '*'
		}
	);	
}
function /*void*/ setDefaultResponseErrorHandling(/*WritableStream*/ response) {
	response.on('error',
		(error) => {
			console.error(error);

			returnStatusCode(500, response);
		}
	);
}
function /*void*/ respondAccordingToMethod(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	switch (request.method) {
		case "POST" :
			handlePostRequest(request, response);

			break;
		case "GET" :
			handleGetRequest(request, response);

			break;
		default: 
			returnStatusCode(404, response);
	}
}
function /*void*/ handlePostRequest(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	switch (request.url) {
		case "/robot" :
			readBody(request, response, RRmanager.recruit);

			break;
		case "/task" :
			readBody(request, response, taskmaster.schedule);

			break;
		case "/dismiss":
			readBody(request, response, RRmanager.dismiss);

			break;
		case "/cancel" :
			readBody(request, response, taskmaster.cancel);

			break;
		default: 
			returnStatusCode(404, response);
	}
}
function /*async*/ readBody(/*IncomingMessage*/ request, /*WritableStream*/ response, /*function(string, WritableStream)*/ afterReading) {
	var body = [];

	request.on('data',
		(chunk) => {
			body.push(chunk);
		}
	);
	request.on('end', 
		() => {
			body = Buffer.concat(body).toString();

			afterReading(body, response);
		}
	);	
}
function /*void*/ handleGetRequest(/*IncomingMessage*/ request, /*WritableStream*/ response) {
	switch (request.url) {
		case "/update" :
			returnRobotsAndTasks(response);

			break;
		default: 
			returnStatusCode(404, response);
	}
}
function /*void*/ returnRobotsAndTasks(/*WritableStream*/ response) {
	var robotsAndTasks = {
		activeRobots : RRmanager.visualize(),
		scheduledTasks: taskmaster.visualize()
	};

	returnObject(robotsAndTasks, response);
}
function /*void*/ returnObject(/*Object*/ object, /*WritableStream*/ response) {
	var dataString = JSON.stringify(object);

	returnText(dataString, response);
}
function /*void*/ returnText(/*string*/ text, /*WritableStream*/ response) {
	if (response != null) {	
		writeDefaultHeader(response);
		
		response.end(text);
	} else {
		console.log(text);
	}
}
function /*JSON*/ writeDefaultHeader(/*WritableStream*/ response) {
	writeHeader(200, response);
}


var SERVER_IP;

var RRmanager;
var taskmaster;

const SERVER = require("http").createServer();

findOutServerIPthen(openShop);

