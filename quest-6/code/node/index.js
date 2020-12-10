// var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
// var http = require('http').Server(app);
const assert = require('assert');
const bodyParser = require('body-parser');
var MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://viv:1GBSt0rage%21@vivcluster.h5rba.mongodb.net/quest6?retryWrites=true&w=majority";
// const client = new MongoClient(uri, { useUnifiedTopology: true,useNewUrlParser: true});
const { exec } = require("child_process");
var fs = require('fs');

var images_dir = './images/';
if (!fs.existsSync(images_dir)){
	fs.mkdirSync(images_dir);
}

var log='retrieving...';
var filterForUserID = -1;
var maxEvents = 5;


function getDateTime(){
	var today = new Date();
	var date = today.getFullYear()+'-'+(today.getMonth()+1)+'-'+today.getDate();
	var time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
	return date+' '+time;
}

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());

app.get('/', function(req, res) {
  	res.sendFile(path.join(__dirname + '/index.html'));
});


app.get('/log', async function(req, res) {
	try {
		var client = await MongoClient.connect(uri);
		// console.log("CONNECTED");
		var collection = client.db("quest6").collection("log");
		if (filterForUserID == -1){ // send all
			log = await collection.find({}).skip(collection.count() - maxEvents).toArray();
			console.log(log)
		}
		else{
			log = await collection.find({userID: filterForUserID}).toArray();
			if (log.length > maxEvents) // only show last 50
			{
				log = log.slice(log.length - maxEvents, log.length);
			}
		}
		// console.log(log);
		client.close();
	} catch (err) {
		console.log(err.message);
	}
	res.send(log);
});


// app.get('/read', function(req,res) {
//   res.send(read_flag);
// })

app.post('/query', (req,res) => {
	console.log('querying ' + req.body.userID);
	filterForUserID = parseInt(req.body.userID);
	res.end('yes');
});



app.listen(4000);



// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1234;
var HOST = '192.168.1.111';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

function isPwd(xyzObj, pwd) {	
	var allowance = 2
	if (xyzObj.x >= pwd.x-allowance && xyzObj.x <= pwd.x+allowance)
		if (xyzObj.y >= pwd.y-allowance && xyzObj.y <= pwd.y+allowance)
			if (xyzObj.z >= pwd.z-allowance && xyzObj.z <= pwd.z+allowance)
				return true;
	return false;
}

// On connection, print out received message
server.on('message', async function (message, remote) {
	console.log(remote.address + ':' + remote.port +' - ' + message);
	var buffer = message;
	// console.log('MESSAGE RECEIVED:');
	// console.log(buffer);
	// console.log(buffer.toString());
	// begin password stuff
	buffer = buffer.toString();
	buffer = buffer.split(',');
	var setbit = parseInt(buffer[0]);
	var userID = parseInt(buffer[1]);
	var xyz = buffer.slice(2).map(Number);
	// var isCorrect = isPwd(xyz) ? '1' : '0';
	var usersObj = {userID: userID , x: xyz[0] , y: xyz[1], z: xyz[2]};
	var event; // int 0 for incorrect password, 1 for correct password, 2 for new password set

	try {
		client = await MongoClient.connect(uri);
		// console.log("CONNECTED");
		var collection = client.db("quest6").collection("users");
		var usersQuery = {userID: userID} //  query existing password for user userID
		if (setbit==1) { // update password
			await collection.findOneAndReplace(usersQuery, usersObj, {upsert: true});

			console.log("User " + usersObj.userID.toString() + " Password updated");
			event = 2;
		}
		else {	// check password
			var pwd = await collection.findOne(usersQuery); 	// query existing password into pwd variable.
			console.log('checking against existing password');
			console.log(pwd);
			if (pwd == null)	// no password has been set for this user
			{
				console.log('No password set for user ' + userID.toString());
				event = 0;
			}
			else if (isPwd(usersObj,pwd)) {
				console.log("Password is correct.");
				event = 1;
			}
			else {
				console.log("Password is incorrect. Try again.");
				event = 0;
			}
		}
		client.close();
	} catch (err) {
		console.log(err.message);
	}
	
	var dateTime = getDateTime();
	console.log('event is ');
	console.log(event);

	if (event==0){	// take a picture if the password is wrong into the images directory
		console.log('taking pic');
		var filename = images_dir + dateTime + ".jpg";
		filename = filename.split(' ').join('_');
		exec("raspistill -n -v -o " + filename, (error, stdout, stderr) => {
			if (error) {
				console.log(`error: ${error.message}`);
				return;
			}
			if (stderr) {
				console.log(`stderr: ${stderr}`);
				return;
			}
			console.log(`stdout: ${stdout}`);
		});
	}

	// end password stuff

	//begin log stuff
	
	
	var logEntry = {userID : userID , event: event, dateTime: dateTime};

	try {
		client = await MongoClient.connect(uri);
		var collection = client.db("quest6").collection("log");
		await collection.insertOne(logEntry);
		console.log("1 event logged");
		client.close();

	} catch (err) {
		console.log(err.message);
	} 

	// end log stuff

	//send UDP response
	server.send(event.toString(),remote.port,remote.address,function(error){
	if(error){
		console.log('Error: could not reply');
	}
	else {
		console.log('Sent: ' + event.toString());
	}
	});


});

// Bind server to port and IP
server.bind(PORT, HOST);
