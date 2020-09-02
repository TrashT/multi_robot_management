const SECOND = 1000/*milliseconds*/;	
const MINUTE = 60 * SECOND;
const MINUTES_PER_HOUR = 60;
const HOUR = MINUTES_PER_HOUR * MINUTE;
const DAY = 24 * HOUR;
const ESTIMATED_TRAVEL_TIME = 5 * MINUTE;

class Task {
	/* attributes
	int ID;
	int start;	
	int leniency;
	Subtask[] subtasks;
	int robotIndex;
	*/
	constructor(/*JSONobject*/ jsonObject) {
		this.ID = jsonObject.ID;
		this.start = jsonObject.start;
		this.leniency = jsonObject.leniency;
		this.createSubtasks(jsonObject);
		this.robotIndex = -1;
	}
	/*void*/ createSubtasks(/*JSONobject*/ jsonObject) {
		this.subtasks = [];
		
		for (var i = 0; i < jsonObject.subtasks.length; ++i) {
			var subtaskJSON = jsonObject.subtasks[i];
			
			var subtask = new Subtask(subtaskJSON, this);
			
			this.subtasks.push(subtask);
		}
	}
	//functions
	static /*Task*/ newTask(/*Date*/ startDate, /*int*/ leniencyMilliseconds) {
		var jsonObject = {
			ID : -1,
			start : startDate.getTime(),
			leniency : leniencyMilliseconds,
			subtasks : []
		};

		return new Task(jsonObject);
	}
	/*int*/ end() {
		var endTime = this.start;
		var subtaskCount = this.subtasks.length;

		for (var i = 0; i < subtaskCount; ++i) {
			endTime += this.subtasks[i].duration;
		}

		endTime += (subtaskCount - 1) * ESTIMATED_TRAVEL_TIME;

		return endTime;
	}

	/*string*/ stringify() {
		var subtasks = this.subtasks;

		for (var i = 0; i < subtasks.length; ++i) {
			subtasks[i].task = null;
		}

		var string = JSON.stringify(this);

		for (var i = 0; i < subtasks.length; ++i) {
			subtasks.task = this;
		}

		return string;
	}
}
