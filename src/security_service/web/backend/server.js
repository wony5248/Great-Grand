//----------------------------------
// lib
const express = require("express");
const session = require('express-session');
const bodyParser = require("body-parser");
const MySQLStore = require('express-mysql-session')(session);
const app = express();
const path = require("path");
const cors = require("cors");
const { response } = require("express");
const axios = require('axios');
const fs = require("fs");
const { Server } = require("http");
require('dotenv').config();

//----------------------------------
// websocket
const server = require('http').createServer(app);
const io = require('socket.io')(server);
const port2 = 3002;

// --------------------------------------------
// env
const envJson = require(`${__dirname}/env/env.json`);
const uploadFilePath = envJson.uploadFilePath;
const port = envJson.port ? envJson.port : 3001;

//----------------------------------
// middleware
app.use(cors({
  origin: true,
  credentials: true,
}));


// bodyParser
app.use(bodyParser.json());
app.use(bodyParser.urlencoded({ extended: true }));

// db
const { pool } = require(`${__dirname}/mysql`);

let sessionStore = new MySQLStore({}, pool);

app.use(session({
  secret: "EZEZ",
  store: sessionStore,
  resave: false,
  saveUninitialized: true,
  cookie: {
    secure: false,
    maxAge: 1000*60*60*24,
    httpOnly: false,
  }
}));

//----------------------------------
// routes
app.use(uploadFilePath, express.static(path.join(__dirname + uploadFilePath)));
// app.use("/callback", require(`${__dirname}/routes/callback/callback`));
// app.use("/user", require(`${__dirname}/routes/user/user`));
// app.use("/fridge", require(`${__dirname}/routes/fridge/fridge`));
// app.use("/calendar", require(`${__dirname}/routes/calendar/calendar`));
// app.use("/foodlist", require(`${__dirname}/routes/calendar/foodlist`));
// app.use("/recipe", require(`${__dirname}/routes/recipe/recipe`));


app.get("/", function (req, res) {
  res.send("Hello CI/CD");
});

// app.get("/img", function(req, res){
//   const rID = req.query.id;
//   fs.readFile(`../../images/${rID}`, function(err, data){
//     res.writeHead(200, {'Content-Type': 'text/html'});
//     res.end(data);
//   })
// })

const roomName = 'team';

io.on('connection', function(socket) {
  socket.join(roomName);

  // ?????? 3. ???????????? ????????? ????????? WebClient??? ????????? ??????
  socket.on('safety_status', (message) => {
    socket.to(roomName).emit('sendSafetyStatus', message);
  });

  socket.on('PatrolStatus', (message) => {
    socket.to(roomName).emit('sendPatrolStatus', message);
  });

  socket.on('PatrolOnToServer', (data) => {
    socket.to(roomName).emit('patrolOn', data);
    console.log('Patrol On!');
  });

  socket.on('PatrolOffToServer', (data) => {
    socket.to(roomName).emit('patrolOff', data);
  });

  socket.on('turnleftToServer', (data) => {
    socket.to(roomName).emit('turnleft', data);
  });

  socket.on('gostraightToServer', (data) => {
    socket.to(roomName).emit('gostraight', data);
  });

  socket.on('turnrightToServer', (data) => {
    socket.to(roomName).emit('turnright', data);
  });

  socket.on('disconnect', () => {
    console.log('disconnected from server');
  });

  // ???????????? ???????????? jpg ????????? ??????
  socket.on('streaming', (message) => {
    socket.to(roomName).emit('sendStreaming', message);
    // console.log(message);
    buffer = Buffer.from(message, "base64");
    fs.writeFileSync(path.join(picPath, "/../build/cam.jpg"), buffer);
  });
})

server.listen(port2, ()=>{
  console.log(`{init : ${port2}}`);
})

app.listen(port, () => {
  console.log(`{init : ${port}}`);
});
