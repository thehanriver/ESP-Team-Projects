var $ = require( "jquery" );
var express = require('express');
var app = express();
var path = require('path');
var http = require('http').Server(app);


const bodyParser = require('body-parser');

const MongoClient = require('mongodb').MongoClient;
const uri = "mongodb+srv://<username>:<password>@vivcluster.h5rba.mongodb.net/<dbname>?retryWrites=true&w=majority";
const client = new MongoClient(uri, { useNewUrlParser: true, useUnifiedTopology: true,  });
var clear_flag = 0;
var all;
var red;
var blue;
var green;

// function candVotes(){
//     client.connect(function(err, db) {
//       if (err) throw err;
//       var dbo = db.db("Election");
//       var query = { vote: /^R/ };
//       dbo.collection("Voters").find(query).toArray(function(err, result) {
//         if (err) throw err;
//         red = result;
//         console.log(result);
//         db.close();
//       });
//       var query = { vote: /^G/ };
//       dbo.collection("Voters").find(query).toArray(function(err, result) {
//         if (err) throw err;
//         green = result;
//         console.log(result);
//         db.close();
//       });
//       var query = { vote: /^B/ };
//       dbo.collection("Voters").find(query).toArray(function(err, result) {
//         if (err) throw err;
//         blue = result;
//         console.log(result);
//         db.close();
//       });
//     });
// }

// function readAll(){
//   client.connect(function(err, db) {
//       if (err) throw err;
//       var dbo = db.db("Election");
//       dbo.collection("Voters").find({}).toArray(function(err, result) {
//         if (err) throw err;
//         all = result;
//         console.log(result);
//         db.close();
//       });
//     });
// }

function clearData(){
    client.connect(function(err, db) {
      if (err) throw err;
      var dbo = db.db("Election");
      dbo.collection("Voters").drop(function(err, delOK) {
        if (err) throw err;
        if (delOK)
        {
          all = [];
          red = [];
          blue = [];
          green = [];
          console.log("Collection deleted");
        }
        db.close();
      });
    });
}

app.use(bodyParser.urlencoded({ extended: false }));
app.use(bodyParser.json());

app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/index.html'));
});

app.get('/all', function(req, res) {
  client.connect(function(err, db) {
    if (err) throw err;
    var dbo = db.db("Election");
    dbo.collection("Voters").find({}).toArray(function(err, result) {
      if (err) throw err;
      all = result;
      console.log(result);
      db.close();
    });
  });
  res.send(all);  // Send array of data back to requestor
});

app.get('/all/red', function(req, res) {
  client.connect(function(err, db) {
    if (err) throw err;
    var dbo = db.db("Election");
    var query = { vote: /^R/ };
    dbo.collection("Voters").find(query).toArray(function(err, result) {
      if (err) throw err;
      red = result;
      console.log(result);
      db.close();
    });
  });
  res.send(red);
});

app.get('/all/green', function(req, res) {
  client.connect(function(err, db) {
    if (err) throw err;
    var dbo = db.db("Election");
    var query = { vote: /^G/ };
    dbo.collection("Voters").find(query).toArray(function(err, result) {
      if (err) throw err;
      green = result;
      console.log(result);
      db.close();
    });
  });
  res.send(green);
});

app.get('/all/blue', function(req, res) {
  client.connect(function(err, db) {
    if (err) throw err;
    var dbo = db.db("Election");
    var query = { vote: /^B/ };
    dbo.collection("Voters").find(query).toArray(function(err, result) {
      if (err) throw err;
      blue = result;
      console.log(result);
      db.close();
    });
  });
  res.send(blue);
});


app.post('/clear', (req,res) => {
  clear_flag = req.body.clear;
  res.end('yes');
});

app.get('/clear', function(req,res) {
  res.send(clear_flag);
})


app.listen(4000);


function checkClear() {
  if(clear_flag == 1 ){
    clearData();
    console.log("cleared");
    clear_flag == 0;
  }
}



setInterval(function(){checkClear()}, 50);

// Required module
var dgram = require('dgram');

// Port and IP
var PORT = 1111;
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
    var today = new Date();
    var date = today.getFullYear()+'-'+(today.getMonth()+1)+'-'+today.getDate();
    var time = today.getHours() + ":" + today.getMinutes() + ":" + today.getSeconds();
    var dateTime = date+' '+time;
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
    myObj = {fob_ID: id, vote: vote, date_time: dateTime};
    
    // insert a document into 'customers'
    client.connect(err=> {
      const collection = client.db("Election").collection("Voters");
      collection.insertOne(myObj, function(err, res) {
        if (err) throw err;
        console.log("1 document inserted");
        db.close();
      });
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


// // insert a document into 'customers'
// MongoClient.connect(url, function(err, db) {
//   if (err) throw err;
//   var dbo = db.db("mydb");
//   var myobj = { name: "Company Inc", address: "Highway 37" };
//   dbo.collection("customers").insertOne(myobj, function(err, res) {
//     if (err) throw err;
//     console.log("1 document inserted");
//     db.close();
//   });
// });

// Bind server to port and IP
server.bind(PORT, HOST);
