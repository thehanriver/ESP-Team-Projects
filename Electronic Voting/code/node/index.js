var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
var http = require('http').Server(app);
const assert = require('assert');

const bodyParser = require('body-parser');

var MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://viv:1GBSt0rage%21@cluster0.zottf.mongodb.net/Election?retryWrites=true&w=majority";

//const uri = "mongodb+srv://viv:1GBSt0rage%21@vivcluster.h5rba.mongodb.net/Election?retryWrites=true&w=majority";

const client = new MongoClient(uri, { useUnifiedTopology: true,useNewUrlParser: true});

var clear_flag = '0';
var read_flag = '0';
var all ;
var red;
var blue;
var green;
var dateTime;

function getDate(){
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


function clearData(){
  if(clear_flag == '1'){
    client.connect(function(err, db) {
      if (err) throw err;
      var dbo = db.db("Election");
      dbo.collection("Voters").deleteMany({});

      all = [];
      red = [];
      blue = [];
      green = [];
      console.log("Collection deleted");
    });
  }
}

function readData(){
  if(read_flag == '1'){
    client.connect(function(err, db) {
      if (err) throw err;
      var dbo = db.db("Election");
      var query;
      dbo.collection("Voters").find({}).toArray(function(err, result) {
        if (err) throw err;
        all = result;
        console.log(result);
        // db.close();
      });
      query = { vote: /^R/ };
      dbo.collection("Voters").find(query).toArray(function(err, result) {
        if (err) throw err;
        red = result;
        console.log(result);
        // db.close();
      });
      query = { vote: /^G/ };
      dbo.collection("Voters").find(query).toArray(function(err, result) {
        if (err) throw err;
        green = result;
        console.log(result);
        // db.close();
      });
      query = { vote: /^B/ };
      dbo.collection("Voters").find(query).toArray(function(err, result) {
        if (err) throw err;
        blue = result;
        console.log(result);
        // db.close();
      });
      console.log("Collection read");
    });
  }
}

app.get('/all', function(req, res) {
  res.send(all);  // Send array of data back to requestor
});

app.get('/all/red', function(req, res) {
  res.send(red);
});

app.get('/all/green', function(req, res) {
  res.send(green);
});

app.get('/all/blue', function(req, res) {
  res.send(blue);
});

app.get('/read', function(req,res) {
  res.send(read_flag);
})

app.post('/read', (req,res) => {
  read_flag = req.body.read;
  res.end('yes');
});

app.post('/clear', (req,res) => {
  clear_flag = req.body.clear;
  res.end('yes');
});

app.get('/clear', function(req,res) {
  res.send(clear_flag);
})

function checkClear() {
  if(clear_flag == '1'){
    clearData();
    clear_flag = '0';
  }
}

function checkRead(){
  if(read_flag == '1'){
    readData();
    read_flag = '0';
  }
}
console.log("clear_flag: ", clear_flag);
console.log("read_flag: ", read_flag);
setInterval(function(){checkClear()}, 50);
setInterval(function(){checkRead()}, 50);
app.listen(4440);



// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1111;
var HOST = '192.168.1.131';

// Create socket
var server = dgram.createSocket('udp4');

// Create server
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// On connection, print out received message
server.on('message', function (message, remote) {
    var myObj = [];
    var id;
    var vote;
    var buffer;
    var myObj;
    console.log("Message: ",message);
    getDate();

    buffer = message.toString();
    buffer = buffer.split(':');
    id = parseInt(buffer[0]);
    vote = parseInt(buffer[1]);
    switch (vote) {
      case 3:
        vote = 'R';
        break;
      case 4:
        vote = 'G';
        break;
      case 5:
        vote = 'B';
        break;
    }
    myObj.push({id : id , vote: vote , date_time: dateTime});

    MongoClient.connect(uri, function(err, client) {
      assert.equal(null, err);
      console.log("CONNECTED");
      var collection = client.db("Election").collection("Voters");
      collection.insertMany(myObj, function(err, res) {
        if (err) throw err;
        console.log("1 vote inserted");
        // db.close();
      });
      client.close();
    });



    console.log(remote.address + ':' + remote.port +' - ' + message);
    server.send("vote " + vote.toString() + " from fob " + id.toString() + " recorded",remote.port,remote.address,function(error){
      if(error){
        console.log('Error: could not reply');
      }
      else {
        console.log('Sent: ' + "vote " + vote.toString() + " from fob " + id.toString() + " recorded");
      }
    });


});

// Bind server to port and IP
server.bind(PORT, HOST);
