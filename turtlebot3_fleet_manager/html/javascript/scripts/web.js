var SERVER_IP;
var SERVER_URL;
var WEBSOCKET_URL;
var VIDEO_URL;

function /*async*/ post(/*string*/ text, /*string*/ subfolder) {
	postThen(text, subfolder, function(result) {
		alert(result.responseText);
	});
}
function /*async*/ postThen(/*string*/ text, /*string*/ subfolder, /*function(Result)*/ withResult) {
	var xmlhttp = new XMLHttpRequest();

	preparePostRequest(xmlhttp, subfolder, withResult);
	
	xmlhttp.send(text);			
}
function /*void*/ preparePostRequest(/*XMLHttpRequest*/ xmlhttp, /*string*/subfolder, /*function(Result)*/ withResult) {
	xmlhttp.open("POST", SERVER_URL + "/" + subfolder, true);
	xmlhttp.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
	xmlhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			withResult(this);
		}
	};
}

function /*async*/ get(/*string*/ subfolder, /*function(Result)*/ withResult)
{
	var xmlhttp = new XMLHttpRequest();
	
	prepareGetRequest(xmlhttp, subfolder, withResult);

	xmlhttp.send();	
}
function /*void*/ prepareGetRequest(/*XMLHttpRequest*/ xmlhttp, /*string*/subfolder, /*function(Result)*/ withResult) {
	xmlhttp.open("GET", SERVER_URL + "/" + subfolder, true);
	xmlhttp.onreadystatechange = function() {
		if (this.readyState == 4 && this.status == 200) {
			withResult(this);
		}
	};		
}
