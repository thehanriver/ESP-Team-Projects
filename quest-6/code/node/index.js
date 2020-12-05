// var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
// var http = require('http').Server(app);
const assert = require('assert');
const bodyParser = require('body-parser');
var MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://viv:1GBSt0rage%21@vivcluster.h5rba.mongodb.net/quest5?retryWrites=true&w=majority";
const client = new MongoClient(uri, { useUnifiedTopology: true,useNewUrlParser: true});

var pwd = [0,0,10];
var log='retrieving...';
var filterForKeyID = -1;


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


app.get('/log', function(req, res) {
	if (filterForKeyID == -1) // send all
	{
		MongoClient.connect(uri, async function(err, client) {
			console.log("CONNECTED");
			assert.equal(null, err);
			var collection = client.db("quest5").collection("log");
			try {
				log = await collection.find({}).toArray();
			} catch (err) {
				console.log(err);
			}
			// console.log(log)
			client.close();
		});
		// getLog();
	}
	else{
		MongoClient.connect(uri, async function(err, client) {
			console.log("CONNECTED");
			assert.equal(null, err);
			var query = {keyID: filterForKeyID};
			var collection = client.db("quest5").collection("log");
			try {
				log = await collection.find(query).toArray();
			} catch (err) {
				console.log(err);
			}
			// console.log(log)
			client.close();
		});
	}
	res.send(log)
});



// query the below address from all documents in 'customers'



function getLog(){
	MongoClient.connect(uri, function(err, client) {
		assert.equal(null, err);
		var collection = client.db("quest5").collection("log");
		collection.find({}).toArray(function(err, result) {
			if (err) throw err;
			log = result;
		});
		client.close();
	});
}


// app.get('/read', function(req,res) {
//   res.send(read_flag);
// })

app.post('/query', (req,res) => {
	console.log('querying ' + req.body.keyID);
	filterForKeyID = parseInt(req.body.keyID);
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

function isPwd(input) {
	var allowance = 2
	if (input[0] >= pwd[0]-allowance || input[0] <= pwd[0]+allowance)
		if (input[1] >= pwd[1]-allowance || input[1] <= pwd[1]+allowance)
			if (input[2] >= pwd[2]-allowance || input[2] <= pwd[2]+allowance)
				return true;
	return false;
}

// On connection, print out received message
server.on('message', function (message, remote) {
	console.log(remote.address + ':' + remote.port +' - ' + message);

	// begin password stuff
	message = message.split(',');
	var setbit = message[0];
	var keyID = message[1];
	var input = message.slice(2).map(Number);
	var isCorrect = isPwd(input) ? '1' : '0';
	var event;	// MARIO, set this variable to int 0 for failure, 1 for success, 2 for set password

	// end password stuff

	//begin log stuff
	
	var logEntry = {keyID : keyID , event: event, dateTime: getDateTime()};

	MongoClient.connect(uri, function(err, client) {
		assert.equal(null, err);
		var collection = client.db("quest5").collection("log");
		collection.insertOne(logEntry, function(err, res) {
		  if (err) throw err;
		  console.log("1 event logged");
		  // db.close();
		});
		client.close();
	});

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
