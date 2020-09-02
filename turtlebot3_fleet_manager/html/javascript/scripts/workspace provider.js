const MAP_ORIGIN = new Point(-29.7, -16.75);
var namespaces = [];

var rooms;

function /*void*/ listRooms() {
	rooms = [];

	rooms.push(new Room(new Point(9, 187), new Point(619, 123), new Point(44, 229)));
	rooms.push(new Room(new Point(91, 146), new Point(162, 43), new Point(180, 185)));
	rooms.push(new Room(new Point(47, 145), new Point(47, 23), new Point(93, 158)));
	rooms.push(new Room(new Point(51, 19), new Point(72, 113), new Point(104, 130)));
	rooms.push(new Room(new Point(126, 21), new Point(93, 129), new Point(146, 142)));
	rooms.push(new Room(new Point(211, 48), new Point(85, 104), new Point(233, 151)));
	rooms.push(new Room(new Point(253, 166), new Point(32, 24), new Point(252, 177)));
	rooms.push(new Room(new Point(300, 32), new Point(240, 170), new Point(340, 198)));
	rooms.push(new Room(new Point(297, 158), new Point(32, 24), new Point(326, 167)));
	rooms.push(new Room(new Point(566, 179), new Point(141, 42), new Point(581, 216)));
	rooms.push(new Room(new Point(550, 47), new Point(69, 134), new Point(589, 177)));
	rooms.push(new Room(new Point(618, 56), new Point(95, 130), new Point(629, 181)));
	rooms.push(new Room(new Point(711, 61), new Point(82, 168), new Point(705, 204)));
	rooms.push(new Room(new Point(386, 311), new Point(234, 175), new Point(572, 306)));
	rooms.push(new Room(new Point(392, 292), new Point(89, 50), new Point(440, 295)));
}

function /*void*/ center() {
	var index = getSelectedIndex("rooms");
	
	if (index > 0) {
		var room = rooms[index - 1];
				
		setCoord(room.center.x, room.center.y);
	}
}

function /*void*/ entrance() {
	var index = getSelectedIndex("rooms");
	
	if (index > 0) {
		var room = rooms[index - 1];
		
		setCoord(room.entrance.x, room.entrance.y);
	}
}
