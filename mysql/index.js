const mysql = require('mysql2/promise');
require('dotenv').config();

const pool = mysql.createPool({
    host: '15.164.48.55',
    user: 'jihun',
    password: process.env.dbPassword,
    database: 'refreci',
    waitForConnections: true,
    connectionLimit: 10,
    queueLimit: 0,
    port: 3306,
});

module.exports = { pool };
