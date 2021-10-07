const express = require("express");
const app = express.Router();
const axios = require("axios");
const crypto = require('crypto');
const nodemailer = require('nodemailer');

require('dotenv').config();

const { pool } = require(`./../../mysql`);

const smtpTransport = nodemailer.createTransport({
    service: "Gmail",
    auth: {
        user: process.env.emailID,
        pass: process.env.emailPW
    },
    tls: {
        rejectUnauthorized: false
    }
});

app.post("/register", async (req, res) => {
    const userName = req.body.userName;
    const userID = req.body.userID;
    const inputPassword = req.body.userPW;
    const hashPassword = crypto.createHash("sha512").update(inputPassword).digest("hex");

    console.log(`userName ${userName}`);
    console.log(`userID ${userID}`);
    console.log(`inputPassword ${inputPassword}`);
    console.log(`hashPassword ${hashPassword}`);
    
    //닉네임이 2자 미만이면 안된다.
    console.log(`닉네임 글자수 ${userName.length}`);
    if (userName.length < 2){
        res.send({ value: 'Short userName' });
        return;
    }

    //비밀번호가 8자 미만이면 안된다.
    console.log(`비밀번호 글자수 ${inputPassword.length}`);
    if (inputPassword.length < 8) {
        res.send({ value: 'Short password' });
        return;
    }

    try {
        const data = await pool.query("INSERT INTO User VALUES (null, ?, ?, ?, NOW(), 0, 0)", [
            userName,
            userID,
            hashPassword,
        ]);

        res.send({ value: 'Success' });
    }
    catch (err) {
        console.log('===========회원가입 중 에러 발생===========');
        console.log(err);
    }

})

app.post("/login", async (req, res) => {
    const userID = req.body.userID;
    const password = req.body.userPW;

    // console.log(userID);
    // console.log(password);

    try {
        const [rows, fields] = await pool.query("SELECT uID, userPW FROM User WHERE userID = ?", [
            userID
        ]);
        
        let dbUserPW = JSON.stringify(rows[0].userPW);
        let t = dbUserPW.replace("\"","");
        let s = t.replace("\"", "");
        dbUserPW = s;
        
        //salt값으로 해쉬 처리 해주는 부분
        const hashPassword = crypto.createHash("sha512").update(password).digest("hex");

        // console.log(`password: ${password}`);
        // console.log(`hashPassword: ${hashPassword}`);
        // console.log(`dbUserPW: ${dbUserPW}`);

        if(dbUserPW === hashPassword){
            console.log("비밀번호 일치");
            req.session.uid = rows[0].uID;
            console.log(req.session);
            req.session.save(() => {
                res.send(true);
                console.log(req.session);
            });
            
        }
        else{
            console.log("비밀번호 불일치");
            res.send(false);
        }
        
    }
    catch (err) {
        console.log('===========로그인 중 에러 발생===========');
        console.log(err);
    }
});

app.get("/logout", async(req, res) => {
    try{
        req.session.destroy(()=>{
            res.clearCookie('connect.sid');
            console.log('로그아웃 됨');
            // res.redirect('http://refreci.s3-website.ap-northeast-2.amazonaws.com/signin');
            // res.redirect('/signin');
            res.redirect('http://localhost:3000/');
        })
    }
    catch(err){
        console.log(err);
    }
})

app.post("/searchID", async (req, res) => {
    const userID = req.body.userID;

    // console.log(userID);

    try {
        const reg_email = /^([0-9a-zA-Z_\.-]+)@([0-9a-zA-Z_-]+)(\.[0-9a-zA-Z_-]+){1,2}$/;

        if (!reg_email.test(userID)) {
            res.send({ value: 'Wrong Email' });
        }
        else {
            const [rows, fields] = await pool.query("SELECT * FROM User WHERE userID = ?", [
                userID
            ]);

            if (rows.length === 0) {
                res.send({ value: 'Success' });
            }
            else {
                res.send({ value: 'Duplicate Email' });
            }
        }
    }
    catch (err) {
        console.log('===========아이디 검색 중 에러 발생===========');
        console.log(err);
    }
});

app.post("/emailAuth", async (req, res) => {
    const userID = req.body.userID;
    const randomNumber = Math.floor((Math.random() * (999999 - 100000) + 100000));

    const mailOptions = {
        from: "refreci21@gmail.com",
        to: userID,
        subject: "Ref:Reci 이메일 인증",
        html: `화면에서 다음 숫자를 입력해주세요. <strong>${randomNumber}</strong>`
    };
    console.log(userID);
    console.log(randomNumber);
    //res.send(JSON.stringify(randomNumber));

    try{
        const email = await smtpTransport.sendMail(mailOptions);
        smtpTransport.close();
        res.send({value: 'Email Sent', number: randomNumber});
    }
    catch(err){
        console.log('===========이메일 전송 중 에러 발생===========');
        console.log(err);
        res.send({ value: 'Email Error' });
    }
});

app.post("/changePassword", async (req, res) => {
    const userID = req.body.userID;
    const password = req.body.userPW;

    console.log(`바꿀 비밀번호 ${password}`);

    if (password.length < 8) {
        res.send({ value: 'Short password' });
        return;
    }

    try {
        const hashPassword = crypto.createHash("sha512").update(password).digest("hex");

        await pool.query("UPDATE User SET userPW = ? WHERE userID = ?", [
            hashPassword,
            userID,
        ]);

        res.send({ value: 'Success' });
    }
    catch (err) {
        console.log('===========비밀번호 변경 중 에러 발생===========');
        console.log(err);
    }
});

app.get("/isLogin", async (req, res) => {
    if(req.session.uid){
        console.log(`환영합니다 유저 넘버 ${req.session.uid}`);
    }
    else{
        console.log('로그인이 되어있지 않습니다.')
    }
    
    console.log("req.session.uid", req.session.uid);
    console.log(req.sessionID)
    console.log(req.session);
    // res.send({value:req.session.uid});
    res.send({value:1});
})

app.get("/userInfo", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    try {
        const [rows1, fields1] = await pool.query("SELECT userID, userName FROM User WHERE uID = ?", [
            uID
        ]);

        const userID = rows1[0].userID;
        const userName = rows1[0].userName;

        const [rows2, fields2] = await pool.query("SELECT COUNT(productName) AS cnt FROM UserProduct WHERE uID = ?", [
            uID
        ]);

        const foodCount = rows2[0].cnt;
        
        const [rows3, fields3] = await pool.query("SELECT COUNT(productName) AS cnt FROM UserProduct WHERE uID = ? AND 0 <= DATE(productShelfLife) - DATE(NOW()) AND DATE(productShelfLife) - DATE(NOW()) <= 3", [
            uID
        ]);

        const expire3FoodCount = rows3[0].cnt;

        const [rows4, fields4] = await pool.query("SELECT COUNT(productName) AS cnt FROM UserProduct WHERE uID = ? AND 0 > DATE(productShelfLife) - DATE(NOW())", [
            uID
        ]);

        const expiredFoodCount = rows4[0].cnt;

        res.send({
            userID: userID,
            userName: userName,
            foodCount: foodCount,
            expire3FoodCount: expire3FoodCount,
            expiredFoodCount: expiredFoodCount,
        })
    }
    catch (err) {
        console.log('===========유저 정보 조회 중 에러 발생===========');
        console.log(err);
    }
})

app.post("/checkPassword", async(req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    const inputPassword = req.body.password;

    try{
        const [rows, field] = await pool.query("SELECT userPW FROM User WHERE uID = ?", [
            uID
        ])

        const dbPassword = rows[0].userPW;
        const hashPassword = crypto.createHash("sha512").update(inputPassword).digest("hex");

        if (dbPassword === hashPassword){
            console.log('비밀번호 일치')
            res.send(true);
        }
        else{
            console.log('비밀번호 불일치')
            res.send(false);
        }

    }
    catch(err){

    }
})

app.post("/changeUserName", async(req, res) => {
    const uID = 1;
    // const uID = req.session.uID;
    
    const userName = req.body.userName;

    if (userName.length < 2) {
        res.send({ value: 'Short userName' });
        return;
    }

    await pool.query("UPDATE User SET userName = ? WHERE uID = ?", [
        userName,
        uID,
    ]);

})

app.post("/changeUserID", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uID;
    
    const userID = req.body.userID;

    await pool.query("UPDATE User SET userID = ? WHERE uID = ?", [
        userID,
        uID,
    ]);
})

app.post("/changeUserPW", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uID;
    
    const userPW = req.body.userPW;

    if (userPW.length < 8) {
        res.send({ value: 'Short password' });
        return;
    }

    const hashPassword = crypto.createHash("sha512").update(userPW).digest("hex");

    await pool.query("UPDATE User SET userPW = ? WHERE uID = ?", [
        hashPassword,
        uID,
    ]);
})

app.get("/deleteUser", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uID;

    try{
        await pool.query("DELETE FROM User WHERE uID = ?", [
            uID,
        ]);
        res.send({value: 'Success'});
    }
    catch(e){
        console.log(e);
        res.send({ value: 'Fail' });
    }
    
})

module.exports = app;
