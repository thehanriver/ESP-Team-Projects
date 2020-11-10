var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
var http = require('http').Server(app);
var io = require('socket.io')(http);
var mongo = require('mongodb');

const readline = require('readline');
const Stream = require('stream');
const bodyParser = require('body-parser');
const router = express.Router();

const MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://mario:1GBSt0rage%21@cluster0.zottf.mongodb.net/VivCluster?retryWrites=true&w=majority";
const client = new MongoClient(uri, {useUnifiedTopology: true, useNewUrlParser: true});
const fs = require('fs');

var myobj = [];

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());

app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/index.html'));
});

function readMongo(){
  client.connect((err,db) => {
    if(err) throw err;
    console.log("connected");
    db.close();
  });
}

app.post('/incoming', (req,res) => {
  res.end(myobj);
});

app.listen(8080);

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

// On connection, print out received message
server.on('message', function (message, remote) {
    var id;
    var vote;
    var buffer;
    var myObj;

    buffer = message.toString();
    buffer = buffer.split(',');
    id = parseInt(buffer[0]);
    vote = buffer[1];
    myObj = {id: id, vote: vote};

    client.connect((err,db) => {
      if(err) throw err;
      var dbo = db.db("Election");
      dbo.collection("Voters").insertOne(myObj, function(err, res) {
        if (err) throw err;
        console.log("%u document inserted ", id);
        db.close();
      });
    });

    console.log(remote.address + ':' + remote.port +' - ' + message);

    server.send(led_status.toString(),remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else {
        console.log('Sent: ' + );
      }
    });

});

// Bind server to port and IP
server.bind(PORT, HOST);
