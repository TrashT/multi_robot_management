const SQL_MAX_INT = 4294967295;

class Subtask {
	/* attributes
	Task task;
	int duration;
	int x;
	int y;
	*/
	constructor(/*Object*/ jsonObject, /*Task*/ task) {
		this.duration = jsonObject.duration;
		this.x = jsonObject.x;
		this.y = jsonObject.y;
		this.task = task;
	}
	//functions
	static /*Subtask*/ fromDocument(/*Task*/ task) {
		var destination = pointFromDocument();
		var msDuration = millisecondsOf("duration");

		if (msDuration == 0) {
			msDuration = SQL_MAX_INT;
		}


		var jsonObject = {
			duration : msDuration,
			x : destination.x,
			y : destination.y
		}

		var subtask = new Subtask(jsonObject, task);

		return subtask;
	}
	/*bool*/ isHappeningDuring(/*Date*/ day) {
		var dayTime = day.getTime();

		var nextDay = dayTime + DAY;

		var isntHappening = this.endDate() < dayTime || this.startDate() >= nextDay;

		return !isntHappening;
	}
	/*int*/ startDate() {
		var start = this.task.start;

		for (var i = 0; i < this.task.subtasks.length; ++i) {
			var subtask = this.task.subtasks[i];

			if (this == subtask) {
				break;
			} else {
				start += subtask.duration + ESTIMATED_TRAVEL_TIME;
			}
		}

		return start;
	}
	/*int*/ endDate() {
		var end = this.startDate() + this.duration;

		return end;
	}

	/*Rectangle*/ getRectangleOn(/*Date*/ date, /*int*/ canvasWidth) {
		var start = new Date(this.startDate());
		var end = new Date(this.endDate());

		var rectangleWidth = this.durationInMinutes();
		
		var x = start.getHours() * MINUTES_PER_HOUR + start.getMinutes() - 1;
		var y = (this.task.robotIndex + 1) * ROW_HEIGHT;
		
		if (start < date) {
			x = 0;
			
			var difference = (date.getTime() - start.getTime()) / MINUTE;
			
			rectangleWidth -= difference;
		}
		
		if (x + rectangleWidth > canvasWidth) {
			rectangleWidth = canvasWidth - x;
		}

		var rectangle = new Rectangle(x, y + 2, rectangleWidth, ROW_HEIGHT - 4);

		return rectangle;
	}	
	/*int*/ durationInMinutes() {
		return this.duration / MINUTE;
	}
}
