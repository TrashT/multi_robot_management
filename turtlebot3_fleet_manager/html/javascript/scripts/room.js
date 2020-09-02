class Room {
	/*fields
	Rectangle area;
	Point entrance;
	Point center;
	*/
	constructor(/*Point*/ origin, /*Point*/ dimensions, /*Point*/ entrance) {
		this.area = new Rectangle(origin.x, origin.y, dimensions.x, dimensions.y);
		this.entrance = entrance;

		this.center = this.calculateCenter();
	}
	/*Point*/ calculateCenter() {
		var x = this.area.x + this.area.width / 2;
		var y = this.area.y + this.area.height / 2;
		
		x = Math.round(x);
		y = Math.round(y);

		var center = new Point(x, y);
		
		return center;
	}
	
	/*bool*/ contains(/*Point*/ point) {
		return this.area.contains(point);
	}
}
