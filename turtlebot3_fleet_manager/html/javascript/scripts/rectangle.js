class Rectangle {
	/*fields
	int x;
	int y;
	int width;
	int height;
	*/
	constructor(/*int*/ x, /*int*/ y, /*int*/ width, /*int*/ height) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
	}
	//methods
	/*bool*/ contains(/*Point*/ point) {
		return this.x <= point.x && this.x + this.width > point.x && this.y <= point.y && this.y + this.height > point.y;
	}
}
