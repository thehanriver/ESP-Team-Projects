// var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
// var http = require('http').Server(app);
const assert = require('assert');
const bodyParser = require('body-parser');
var MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://viv:1GBSt0rage%21@vivcluster.h5rba.mongodb.net/quest6?retryWrites=true&w=majority";
const client = new MongoClient(uri, { useUnifiedTopology: true,useNewUrlParser: true});

var pwd = [0,0,10];
var log='retrieving...';
var filterForUserID = -1;


function getDateTime(){
	var today = new Date();
	var date = today.getFullYear()+'-'+(today.getMonth()+1)+'-'+today.getDate();
	var time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
	dateTime = date+' '+time;
}

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());

app.get('/', function(req, res) {
  	res.sendFile(path.join(__dirname + '/index.html'));
});


app.get('/log', async function(req, res) {
	try {
		var client = await MongoClient.connect(uri);
		console.log("CONNECTED");
		var collection = client.db("quest6").collection("log");
		if (filterForUserID == -1){ // send all
			log = await collection.find({}).toArray();
		}
		else{
			log = await collection.find({userID: filterForUserID}).toArray();
		}
		console.log(log);
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
var HOST = '192.168.1.139';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
	var address = server.address();
	console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

function isPwd(xyzObj, pwd) {	
	var allowance = 2
	if (input.x >= pwd.x-allowance || input.x <= pwd.x+allowance)
		if (input.y >= pwd.y-allowance || input.y <= pwd.y+allowance)
			if (input.z >= pwd.z-allowance || input.z <= pwd.z+allowance)
				return true;
	return false;
}

// On connection, print out received message
server.on('message', async function (message, remote) {
	console.log(remote.address + ':' + remote.port +' - ' + message);

	// begin password stuff
	message = message.split(',');
	var setbit = parseInt(message[0]);
	var userID = parseInt(message[1]);
	var xyz = message.slice(2).map(Number);
	// var isCorrect = isPwd(xyz) ? '1' : '0';
	var usersObj = {userID: userID , x: xyz[0] , y: xyz[1], z: xyz[2]};
	var event; // int 0 for incorrect password, 1 for correct password, 2 for new password set

	try {
		client = await MongoClient.connect(uri);
		console.log("CONNECTED");
		var collection = client.db("quest6").collection("users");
		var usersQuery = {userID: userID} //  query existing password for user userID
		if (setbit==1) { // update password
			await collection.findOneAndReplace(usersQuery, usersObj, {upsert: true});

			console.log("User " + usersObj.userID.toString() + " Password updated");
			event = 2;
		}
		else {	// check password
			var pwd = await collection.findOne(usersQuery) 	// query existing password into pwd variable.
			if(isPwd(usersObj,pwd)) {
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
	


	// end password stuff

	//begin log stuff
	
	var logEntry = {userID : userID , event: event, dateTime: getDateTime()};

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
		console.log('Sent: ' + result);
	}
	});


});

// Bind server to port and IP
server.bind(PORT, HOST);
