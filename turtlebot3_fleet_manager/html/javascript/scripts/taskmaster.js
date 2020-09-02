var toSchedule = null;
var tasks = [];

function /*void*/ reconstructTasks(/*JSON*/ returnedTasks) {
	tasks = [];

	for (var i = 0; i < returnedTasks.length; ++i) {
		var task = new Task(returnedTasks[i]);

		tasks.push(task);
	}
}

function /*int*/ millisecondsOf(/*string*/ ID) {
	var value = getValueFrom(ID);

	if (value.length == 0) {
		value = "0";
	}

	var milliseconds = parseInt(value) * MINUTE;

	return milliseconds;
}

function /*void*/ tryQueueingSubtask() {
	if (isEmpty("duration")) {
		missingDurationAlert();

		return;
	}

	if (!areCoordinatesProvided) {
		missingCoordinatesAlert("queue");

		return;
	}

	queueSubtask();
}
function /*void*/ missingDurationAlert() {
	alert('You need to provide a duration to queue a subtask manually.  Click "schedule" to include this subtask as a final subtask before scheduling.');
}
function /*void*/ missingCoordinatesAlert(/*string*/ queueOrSchedule) {
	alert("You need to provide x- and y-coordinates to " + queueOrSchedule + " a task!");	
}
function /*void*/ queueSubtask() {
	if (toSchedule == null) {
		newTask();
	}

	var subtask = Subtask.fromDocument(toSchedule);

	toSchedule.subtasks.push(subtask);

	clearCoordinateInputs();
}
function /*void*/ newTask() {
	toSchedule = Task.newTask(dateFromDocument(), millisecondsOf("leniency"));

	disableDateTimeInputs();
}
function /*void*/ clearCoordinateInputs() {
	setValueOf("xCoord", "");
	setValueOf("yCoord", "");
}

function /*void */ trySchedulingTask() {
	if (areCoordinatesProvided()) {
		queueSubtask();
	} else if (toSchedule == null) {
		missingCoordinatesAlert("schedule");

		return;
	}

	var now = new Date();

	if (toSchedule.start < now) {
		var question = "The chosen start time lies in the past. Click OK to have a robot begin working on the task now.";
		
		if (confirm(question)) {
			toSchedule.start = now.getTime();
			toSchedule.leniency = ESTIMATED_TRAVEL_TIME + MINUTE;
		} else {
			return;
		}		
	}
	
	var tasksATM = tasks.slice();

	tasksATM.push(toSchedule);
	
	assignRobotIndicesFor(tasksATM);
		
	if (toSchedule.robotIndex < robots.length) {
		scheduleTask();
	}
	else {
		notEnoughRobotsAlert();
	}
}
function /*bool*/ areCoordinatesProvided() {
	return getValueFrom("xCoord").length > 0 && getValueFrom("xCoord").length > 0;
}
function /*void*/ assignRobotIndicesFor(/*Task[]*/ tasksATM) {
	var compareTasksAscending = function (task1, task2) {
	  if (task1.start > task2.start) return 1;
	  if (task1.start < task2.start) return -1;
	  return 0;
	};

	tasksATM.sort(compareTasksAscending);
	
	for (var i = 0; i < tasksATM.length; ++i) {
		var task = tasksATM[i];

		var robotIndex = 0; 

		while (isAlreadyBusyUntil(robotIndex, i, tasksATM)) {
			++robotIndex;
		}
		
		task.robotIndex = robotIndex;
	}
}
function /*bool*/ isAlreadyBusyUntil(/*int*/ robotIndex, /*int*/ taskIndex, /*Task[]*/ sortedTasks) {
	var currentTask = sortedTasks[taskIndex];
	
	for (var i = 0; i < taskIndex; ++i) {
		var task = sortedTasks[i];
		
		if (task.robotIndex == robotIndex && task.end() > currentTask.start) {
			return true;
		}
	}
	
	return false;
}
function /*void*/ scheduleTask() {
	var json = toSchedule.stringify();

	postThen(json, "task", function(response) {
		alert(response.responseText);
	});

	prepareScheduleFormForNewTask();
}
function /*void*/ notEnoughRobotsAlert(/*Task*/ task, /*bool*/ chainAnotherTask) {
	var question = "Your task probably won't be performed, due to a lack of free robots. Click OK to schedule it anyway."
	
	if (confirm(question)) {
		scheduleTask();
	} 
}
function /*void*/ prepareScheduleFormForNewTask() {
	enableDateTimeInputs();
	toSchedule = null;
	clearCoordinateInputs();	
}

function /*bool*/ tryCancelling(/*onmousemoveEvent*/ e) {
	var boundingRectangle = e.target.getBoundingClientRect();

	var x = e.clientX - boundingRectangle.left;
	var y = e.clientY - boundingRectangle.top;

	var point = new Point(x, y);

	if (!tryCancellingFromScheduledTask(point)) {
		return tryCancellingFromExistingTasks(point);
	}

	return true;
}

function /*bool*/ tryCancellingFromScheduledTask(/*Point*/ point) {
	if (toSchedule != null) {	
		var selectedDate = getSelectedDate();
		var canvasWidth = getElement("scheduleCanvas").width;

		for (var i = 0; i < toSchedule.subtasks.length; ++i) {
			var subtask = toSchedule.subtasks[i];

			if (subtask.isHappeningDuring(selectedDate)) {
				var rectangle = subtask.getRectangleOn(selectedDate, canvasWidth);

				if (rectangle.contains(point)) {
					cancelLocally(subtask);

					return true;
				}
			}
		}
	}

	return false;
}
function /*void*/ cancelLocally(/*Subtask*/ subtask) {
	if (toSchedule.subtasks.length == 1) {
		prepareScheduleFormForNewTask();
	} else {
		toSchedule.subtasks = toSchedule.subtasks.filter(st => st != subtask);
	}
}
function /*bool*/ tryCancellingFromExistingTasks(/*Point*/ point) {
	for (var i = 0; i < tasks.length; ++i) {
		if (tryCancellingFrom(tasks[i], point)) {
			return true;
		}
	}

	return false;
}
function /*bool*/ tryCancellingFrom(/*Task*/ task, /*Point*/ point) {
	var selectedDate = getSelectedDate();
	var canvasWidth = getElement("scheduleCanvas").width;

	for (var i = 0; i < task.subtasks.length; ++i) {
		var subtask = task.subtasks[i];

		if (subtask.isHappeningDuring(selectedDate)) {
			var rectangle = subtask.getRectangleOn(selectedDate, canvasWidth);

			if (rectangle.contains(point)) {
				cancel(task, i);

				return true;
			}
		}
	}

	return false;
}
function /*void*/ cancel(/*Task*/ task, /*int*/ subtaskIndex) {
	var question = "Unschedule subtask " + subtaskIndex.toString() + " of task #" + task.ID.toString() + "?";

	if (confirm(question)) {
		postThen(task.ID.toString() + ";" + subtaskIndex.toString(), "cancel", function(result) {
			alert(result.responseText);
		});
	}
}
