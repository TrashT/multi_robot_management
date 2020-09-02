const UPDATE_INTERVAL = 200;
const ROW_HEIGHT = 42;
const MAP_TRANSLATE = new Point(-74, -245);
var lastUpdate;
var robotSprite = new Image();
robotSprite.src = "../img/robot.png";

function /*void*/ findOwnIP() {
	var xmlhttp = new XMLHttpRequest();

	xmlhttp.open("GET", "javascript/IP.txt", true);
	xmlhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			setIP(this.responseText);
			initialize();
		}
	};

	xmlhttp.send();
}
function /*void*/ setIP(/*string*/ IP) {
	SERVER_IP = IP;
	SERVER_URL = "http://" + SERVER_IP + ":8081";
	WEBSOCKET_URL = "ws://" + SERVER_IP + ":9090";
	VIDEO_URL = "http://" + SERVER_IP + ":8082";	
}
function /*void*/ initialize() {
	setDateTime(new Date());
	listRooms();
	update();
}
function /*void*/ setDateTime(/*Date*/ date) {
	setDate(date);
	setTime(date);
}
function /*void*/ setDate(/*Date*/ date) {
	var dateString = YYYYMMDD(date);

	setValueOf("date", dateString);
}
function /*string*/ YYYYMMDD(/*Date*/ date) {
	var year = YYYY(date);
	var month = MM(date);
	var day = DD(date);
	
	var dateString = year + "-" + month + "-" + day;

	return dateString;
}
function /*string*/ YYYY(/*Date*/ date) {
	var year = date.getFullYear().toString();

	return year;
}
function /*string*/ MM(/*Date*/ date) {
	var month = date.getMonth() + 1;

	if (month < 10) {
		month = "0" + month.toString();
	}
	else {
		month = month.toString();
	}

	return month;
}
function /*string*/ DD(/*Date*/ date) {
	var day = date.getDate();	

	if (day < 10) {
		day = "0" + day.toString();
	}
	else {
		day = day.toString();
	}

	return day;
}
function /*void*/ setValueOf(/*string*/ ID, /*string*/ value) {
	var element = getElement(ID);

	element.value = value;
}
function /*Element*/ getElement(/*string*/ ID) {
	var element = document.getElementById(ID);

	return element;	
}
function /*void*/ setTime(/*Date*/ date) {
	var timeString = hhmm(date);

	setValueOf("time", timeString);
}
function /*string*/ hhmm(/*Date*/ date) {
	var hour = hh(date);
	var minute = mm(date);
	
	var timeString = hour + ":" + minute + ":00";

	return timeString;
}
function /*string*/ hh(/*Date*/ date) {
	var hour = date.getHours();

	if (hour < 10) {
		hour = "0" + hour.toString();
	}
	else {
		hour = hour.toString();
	}

	return hour;
}
function /*string*/ mm(/*Date*/ date) {
	var minute = date.getMinutes();	

	if (minute < 10) {
		minute = "0" + minute.toString();
	}
	else {
		minute = minute.toString();
	}

	return minute;
}
function /*async*/ update() {
	lastUpdate = new Date();

	get("update", drawAndLoop);
}
function /*void*/ drawAndLoop(/*Response*/ response) {
	var state = JSON.parse(response.responseText);

	robots = state.activeRobots;
	reconstructTasks(state.scheduledTasks);

	drawSchedule();
	refreshMap();


	scheduleNextUpdate();
}
function /*void*/ drawSchedule() {
	var tasksATM = tasks.slice();
	var canvas = getElement("scheduleCanvas");

	if (toSchedule != null) {
		tasksATM.push(toSchedule);
	}
	var context = canvas.getContext("2d");

	var canvas = drawScheduleCanvas(tasksATM, canvas, context);

	for (var i = 0; i < tasksATM.length; ++i) {
		var task = tasksATM[i];

		drawSubtasks(task, canvas, context);		
	}

	if (toSchedule != null) {
		drawSubtasks(toSchedule, canvas, context);
		drawNumbersOnMap(context);
	}
	
	drawLineForCurrentTime(context, canvas.height);
}
function /*void*/ drawScheduleCanvas(/*Task[]*/ tasksATM, /*Canvas*/ canvas, /*2Dcontext*/ context) {
	var lines = maxRobotsNeededFor(tasksATM);
	
	canvas.height = (lines + 1) * 42;
	
	blackBackground(canvas, context);

	context.fillStyle = "#02ECFD";
	context.strokeStyle = "#02ECFD";
	context.font = "15px Roboto"; 
	context.textAlign = "center";
	
	drawColumns(context, canvas.height);	
	drawLine(context, 0, 42, 1439, 42);

	return canvas;
}
function /*int*/ maxRobotsNeededFor(/*Task[]*/ tasksATM) {
	assignRobotIndicesFor(tasksATM);
	var maxIndex = -1;

	for (var i = 0; i < tasksATM.length; ++i) {
		var index = tasksATM[i].robotIndex;

		if (index> maxIndex) {
			maxIndex = index;
		}
	}

	return maxIndex + 1;
}
function /*void*/ blackBackground(/*Canvas*/ canvas, /*2Dcontext*/ context) {
	context.fillStyle = "#000000";

	context.fillRect(0, 0, canvas.width, canvas.height);	
}
function /*void*/ drawColumns(/*2Dcontext*/ context, /*int*/ canvasHeight) {
	for (var i = 1; i < 24; ++i)
	{
		drawColumn(i, context, canvasHeight);
	}
	
	context.fillText("23:00", 1410, 27);	
}
function /*void*/ drawColumn(/*int*/ index, /*2Dcontext*/ context, /*int*/ canvasHeight) {
	var x = MINUTES_PER_HOUR * index;
	var y = canvasHeight - 1;
	
	drawLine(context, x, 0, x, y);
	
	var time = (index - 1).toString() + ":00";
	
	if (index <= 10)
	{
		time = "0" + time;
	}

	context.fillText(time, x - 30, 27); 		
}
function /*void*/ drawLine(/*2Dcontext*/ context, /*int*/ fromX, /*int*/ fromY, /*int*/ toX, /*int*/ toY) {
	context.moveTo(fromX, fromY);
	context.lineTo(toX, toY);
	context.stroke();
}
function /*void*/ drawSubtasks(/*Task*/ task, /*Canvas*/ canvas, /*2Dcontext*/ context) {
	var selectedDate = getSelectedDate();

	for (var i = 0; i < task.subtasks.length; ++i) {
		var subtask = task.subtasks[i];

		if (subtask.isHappeningDuring(selectedDate)) {
			drawSubtask(subtask, context, selectedDate, canvas.width, task == toSchedule);
		}		
	}
}
function /*int*/ getSelectedDate() {
	var dateString = getValueFrom("date");
	
	var date = new Date(Date.parse(dateString));

	return new Date(date.getFullYear(), date.getMonth(), date.getDate(), 0, 0, 0);
}
function /*string*/ getValueFrom(/*string*/ ID) {
	var element = getElement(ID);

	var value = element.value.toString();

	return value;
}
function /*void*/ drawSubtask(/*Subtask*/ subtask, /*2Dcontext*/ context, /*int*/ selectedDate, /*int*/canvasWidth, /*bool*/isQueued) {
		var rectangle = subtask.getRectangleOn(selectedDate, canvasWidth);
		
		if (subtask.task.robotIndex < robots.length) {
			context.fillStyle = "#39FF14";
		}
		else {
			context.fillStyle = "#FF073A";
		}
		
		if (isQueued) {
			context.globalAlpha = 0.25;
		}

		context.fillRect(rectangle.x, rectangle.y, rectangle.width, rectangle.height);	

		context.globalAlpha = 1;
}
function /*void*/ drawNumbersOnMap() {
	if (toSchedule == null) {
		return;
	}

	var canvas = getElement("map");
	var context = canvas.getContext("2d");
	context.font = "15px Roboto"; 
	context.textAlign = "center";

	for (var i = 0; i < toSchedule.subtasks.length; ++i) {
		var subtask = toSchedule.subtasks[i];

		var point = transformRealWorldToCanvas(canvas, subtask.x, subtask.y);

		context.fillStyle = "#02ECFD";
		context.fillRect(point.x - 7, point.y - 7, 15, 15);
		context.fillStyle = "white";
		context.fillText((i + 1).toString(), point.x, point.y + 6);
	}
}
function /*void*/ drawLineForCurrentTime(/*2Dcontext*/ context, /*int*/ canvasHeight) {
	var date = new Date(getSelectedDate());
	context.strokeStyle = "#02ECFD";

	if (isToday(date)) {
		var x = date.getHours() * MINUTES_PER_HOUR + date.getMinutes();

		drawLine(context, x, 0, x, canvasHeight - 1);
	} 
}
function /*bool*/ isToday(/*Date*/ date) {
	var today = new Date();

	return date.getFullYear() == today.getFullYear() && date.getMonth() == today.getMonth() && date.getDate() == today.getDate();
}
function /*void*/ refreshMap() {
	var canvas = document.getElementById("map");
	
	var context = canvas.getContext("2d");
	
	context.clearRect(0, 0, canvas.width, canvas.height);
	
	removeControlsOfAnyDismissedRobots();
	drawNumbersOnMap();
	drawRobots(canvas, context);
	drawTarget(context);
}
function /*void*/ removeControlsOfAnyDismissedRobots() {
	for (var i = 0; i < namespaces.length; ++i) {
		var namespace = namespaces[i];

		if (wasDisassembled(namespace)) {
			removeFromInterface(namespace);
		}
	}
}
function /*bool*/ wasDisassembled(/*string*/ namespace) {
	for (var i = 0; i < robots.length; ++i) {
		var robot = robots[i];

		if (robot.namespace == namespace) {
			return false;
		}
	}

	return true;
}
function /*void*/ removeFromInterface(/*string*/ namespace) {
	removeControlsOf(namespace);
	removeVideoFeedOf(namespace);
	namespaces = namespaces.filter(ns => ns != namespace);
}
function /*void*/ removeControlsOf(/*string*/ namespace) {
	var controls = getElement(namespace);

	controls.parentNode.removeChild(controls);

	stopControlling(namespace);
}
function /*void*/ stopControlling(/*string*/ namespace) {
	if (beingControlled == namespace) {
		beingControlled = "";
		closeROS();
	}
}
function /*void*/ removeVideoFeedOf(/*string*/ namespace) {
	var video = getElement(namespace + "video");

	if (video != null) {
		video.parentNode.removeChild(video);
	}
}
function /*void*/ drawRobots(/*Canvas*/ canvas, /*Context*/ context) {
	var robotsATM = robots.slice();

	for (var i = 0; i < robotsATM.length; ++i) {
		var robot = robotsATM[i];

		drawRobot(robot, canvas, context);
	}
}
function /*void*/ drawRobot(/*object*/ robot, /*Canvas*/ canvas, /*Context*/ context) {
	addAnyMissingControlsFor(robot);
	drawRobotOnMap(robot, canvas, context);

	var button = getElement(robot.namespace + "button");

	button.innerHTML =  robot.name + "(" + Math.round(robot.battery * 100) + "%)";

	if (robot.namespace == beingControlled) {
		button.innerHTML += "*";
	}
}
function /*void*/ addAnyMissingControlsFor(/*object*/ robot) {
	var namespace = robot.namespace;

	if (!namespaces.includes(namespace)) {
		namespaces.push(namespace);

		addDivFor(namespace)
	}
}
function /*div*/ addDivFor(/*string*/ namespace) {
	var div = document.createElement("div");

	div.setAttribute("id", namespace);
	div.setAttribute("class", "robot");
	div.appendChild(createControlsFor(namespace));
	div.appendChild(createCheckboxesFor(namespace));

	getElement("robot-list").appendChild(div);
}
function /*div*/ createControlsFor(/*string*/ namespace) {
	var div = document.createElement("div");

	div.setAttribute("class", "robotInputs");
	div.appendChild(createButtonToControl(namespace));
	div.appendChild(createButtonToDismiss(namespace));

	return div;
}
function /*button*/ createButtonToControl(/*string*/ namespace) {
	var button = document.createElement("button");

	button.setAttribute("class", "robotSelector");
	button.setAttribute("id", namespace + "button");
	button.setAttribute("onclick", "startControlling('" + namespace + "')");

	return button;
}
function /*button*/ createButtonToDismiss(/*string*/ namespace) {
	var button = document.createElement("button");

	button.setAttribute("class", "cancel");
	button.setAttribute("id", namespace + "button");
	button.setAttribute("onclick", "removeRobot('" + namespace + "')");
	button.setAttribute("style", "color : red");
	button.innerHTML = "X";

	return button;
}
function /*input*/ createCheckboxesFor(/*string*/ namespace) {
	var div = document.createElement("div");

	div.setAttribute("class", "robotInputs");
	div.appendChild(createScanCheckboxFor(namespace));
	div.appendChild(createScanLabelFor(namespace));
	div.appendChild(createVideoCheckboxFor(namespace));
	div.appendChild(createVideoLabelFor(namespace));

	return div;
}
function /*input*/ createScanCheckboxFor(/*string*/ namespace) {
	var input = document.createElement("input");

	input.setAttribute("type", "checkbox");
	input.setAttribute("id", namespace + "scanCheckbox");
	input.setAttribute("name", "scan");
	input.setAttribute("value", "scan");

	return input;
}
function /*label*/ createScanLabelFor(/*string*/ namespace) {
	var label = document.createElement("label");

	label.setAttribute("for", namespace + "scanCheckbox");
	label.setAttribute("class", "checkboxLabel");
	label.innerHTML = "scan";

	return label;
}
function /*input*/ createVideoCheckboxFor(/*string*/ namespace) {
	var input = document.createElement("input");

	input.setAttribute("type", "checkbox");
	input.setAttribute("id", namespace + "videoCheckbox");
	input.setAttribute("name", "video");
	input.setAttribute("value", "video");
	input.setAttribute("onclick", "toggleVideoFor('" + namespace + "')");

	return input;
}
function /*label*/ createVideoLabelFor(/*string*/ namespace) {
	var label = document.createElement("label");

	label.setAttribute("for", namespace + "videoCheckbox");
	label.setAttribute("class", "checkboxLabel");
	label.innerHTML = "video";

	return label;
}
function addVideoFeedFor(/*string*/ namespace) {
	var videos = getElement("videos");

	var videoURL = VIDEO_URL + "/stream?topic=/" + namespace + "/rgb/image&type=ros_compressed&width=640&height=480";

	var img = document.createElement("img");

	img.setAttribute("src", videoURL);
	img.setAttribute("width", 640);
	img.setAttribute("height", 480);
	img.setAttribute("style", "display : block");
	img.setAttribute("id", namespace + "video");

	videos.appendChild(img);
}
function /*void*/ drawRobotOnMap(/*object*/ robot, /*Canvas*/ canvas, /*Context*/ context) {
	var point = transformRealWorldToCanvas(canvas, robot.x, robot.y);

	point.x -= 7;
	point.y -= 7;


	drawPath(robot, canvas, context);

	if (getElement(robot.namespace +  "scanCheckbox").checked) {
		drawScan(robot, canvas, context);
	}

	/*in case the sprite doesnt work:
		context.fillStyle = "green";
		context.fillRect(point.x - 2, point.y - 2, 5, 5);
	*/

	rotateAndPaintImage(context, robotSprite, robot.angle, point, new Point(7, 7));
}
function rotateAndPaintImage (/*Context*/ context, /*Image*/ sprite, /*float*/ angleInRad , /*Point*/ position, /*Point*/ axis) {
	context.translate( position.x, position.y );
	context.rotate( angleInRad );
	context.drawImage( sprite, -axis.x, -axis.y );
	context.rotate( -angleInRad );
	context.translate( -position.x, -position.y );
}
function /*void*/ drawPath(/*object*/ robot, /*Canvas*/ canvas, /*Context*/ context) {	
	var path = robot.path;
	context.fillStyle = "white";

	for (var i = 0; i < path.length; ++i) {
		var point = transformRealWorldToCanvas(canvas, path[i].x, path[i].y);
		
		context.fillRect(point.x, point.y, 1, 1);
	}
}
function /*void*/ drawScan(/*object*/ robot, /*Canvas*/ canvas, /*Context*/ context) {	
	var scan = robot.scan;
	context.fillStyle = "red";

	for (var i = 0; i < scan.length; ++i) {
		var point = transformRealWorldToCanvas(canvas, scan[i].x, scan[i].y);
		
		context.fillRect(point.x, point.y, 1, 1);
	}
}
function /*void*/ drawTarget(/*Context*/ context) {
	var x = getIntValueFrom("xCoord");
	var y = getIntValueFrom("yCoord");
	
	context.fillStyle = "#02ECFD";
	context.fillRect(x - 2, y - 2, 5, 1);
	context.fillRect(x - 2, y + 2, 5, 1);
	context.fillRect(x - 2, y - 1, 1, 3);
	context.fillRect(x + 2, y - 1, 1, 3);
}
function /*int*/ getIntValueFrom(/*string*/ ID) {
	var value = parseInt(getValueFrom(ID));

	return value;
}
function /*void*/ scheduleNextUpdate() {
	var now = new Date();

	var timeElapsed = now.getTime() - lastUpdate.getTime();

	if (timeElapsed >= UPDATE_INTERVAL) {
		update();
	} else {
		var toWait = UPDATE_INTERVAL - timeElapsed;

		setTimeout(update, toWait);
	}
}

function /*void*/ clearElement(/*string*/ ID) {
	var element = getElement(ID);

	element.innerHTML = "";	
}

function /*int*/ getSelectedIndex(/*string*/ ID) {
	var index = getElement(ID).selectedIndex;

	return index;
}

function /*void*/ setSelectedIndex(/*string*/ ID, /*int*/ index) {
	var element = getElement(ID);

	element.selectedIndex = index;
}

function /*void*/ onScheduleClick(/*onmousemoveEvent*/ e) {
	if (!tryCancelling(e)) {
		setTimeWithClickOnSchedule(e);
	}
}
function /*void*/ setTimeWithClickOnSchedule(/*onmousemoveEvent*/ e) {
	var rect = e.target.getBoundingClientRect();
	var date = dateFromDocument();

	var x = e.clientX - rect.left;

	var hours = Math.floor(x / 60);
	var minutes = x % 60;	
	
	setTime(new Date(date.getFullYear(), date.getMonth(), date.getDate(), hours, minutes));
}

function /*void*/ disableDateTimeInputs() {
	getElement("date").disabled = true;
	getElement("time").disabled = true;	
	getElement("leniency").disabled = true;	
}

function /*void*/ enableDateTimeInputs() {
	getElement("date").disabled = false;
	getElement("time").disabled = false;	
	getElement("leniency").disabled = false;
}

function /*void*/ unhideInputs() {
	getElement("toUnhide").style.display = "flex";
	getElement("addTurtleBot3").style.display = "none"
}

function /*void*/ hideInputs() {
	getElement("toUnhide").style.display = "none";
	getElement("addTurtleBot3").style.display = "block"
}

function /*void*/ setCoordinatesWithClickOnMap(event) {
    var x = event.offsetX;
    var y = event.offsetY;
	
	var point = transformCanvasToRealWorld(getElement("map"), x, y);

	setCoord(x, y);
}
function setCoord (/*int*/ x, /*int*/ y) {
	setValueOf("xCoord", x.toString());
	setValueOf("yCoord", y.toString());
	
	setRoom(x, y);
}
function /*void*/ setRoom(/*int*/ x, /*int*/ y) {
	var point = new Point(x, y);
	var i;
	
	for (i = 0; i < rooms.length; ++i) {
		var room = rooms[i];
		
		if (room.contains(point)) {
			var menu = document.getElementById("rooms");
			
			menu.selectedIndex = i + 1;
			//don't break because some rooms overlap
		}
	}
}

function /*Date*/ dateFromDocument() {
	var dateString = getValueFrom("date");
	var timeString = getValueFrom("time");

	var dateArray = dateString.split("-");
	var timeArray = timeString.split(":");

	var year = parseInt(dateArray[0]);
	var month = parseInt(dateArray[1]) - 1;
	var day = parseInt(dateArray[2]);
	var hour = parseInt(timeArray[0]);
	var minute = parseInt(timeArray[1]);

	return new Date(year, month, day, hour, minute);
}

function /*Point*/ pointFromDocument() {
	var x = getValueFrom("xCoord");
	var y = getValueFrom("yCoord");

	x = parseInt(x);
	y = parseInt(y);

	return transformCanvasToRealWorld(getElement("map"), x, y);
}
function /*Point*/ transformRealWorldToCanvas(/*Canvas*/ canvas, /*int*/ x, /*int*/ y) {
	var X = (x - MAP_ORIGIN.x) / 0.05  + MAP_TRANSLATE.x;
	var Y = (y - MAP_ORIGIN.y) / 0.05  + MAP_TRANSLATE.y;
	
	Y = canvas.height - Y;
	
	var point = new Point(X, Y);
	
	return point;
}
function /*Point*/ transformCanvasToRealWorld(/*Canvas*/ canvas, /*int*/ x, /*int*/ y) {
	var X = (x - MAP_TRANSLATE.x) * 0.05 + MAP_ORIGIN.x
	var Y = canvas.height - y;

	Y = (Y - MAP_TRANSLATE.y) * 0.05 + MAP_ORIGIN.y
	
	
	var point = new Point(X, Y);
	
	return point;
}

function /*bool*/ isEmpty(/*string*/ elementID) {
	return getValueFrom(elementID).length < 1;
}

