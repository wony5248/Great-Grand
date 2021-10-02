const express = require("express");
const app = express.Router();
const axios = require("axios");
const { pool } = require(`../../mysql`)



app.post('/changeCount', async (req, res) =>{
    console.log(req.body)
    const uID = 1
    // const uID = req.session.uid;
    let  sql1, sql2;
    if (req.body.Type === 1){
        sql1 = `UPDATE refreci.UserProduct
        SET productCount = productCount-1
        Where uID = ? AND productName = ?`

        sql2 = `SELECT productCount as Count
        FROM refreci.UserProduct
        WHERE uID=? AND productName=?`
    }
    else if (req.body.Type === 2){
        sql1 = `UPDATE refreci.UserProduct 
        SET productCount = productCount+1
        Where uID = ? AND productName = ?`

        sql2 = `SELECT productCount as Count
        FROM refreci.UserProduct
        WHERE uID=? AND productName=?`
    }
    try {
        await pool.query(sql1, [uID, req.body.Name])
        const data = await pool.query(sql2, [uID, req.body.Name])
        console.log(data[0])
        res.send(data[0])
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})


//dueday 음식 수량 photo
//classification
app.get('/get7days', async (req, res) => {
    const uID = 1
    // const uID = req.session.uid;
    const sql = `SELECT a.Classification2Image as Img, DATEDIFF(productShelfLife, now()) as Dday, b.productName as Name, b.productCount as Count
    FROM refreci.UserProduct as b
    right join refreci.Classification2 as a
    on b.productClassification2 = a.c2ID
    Where b.uID=? AND DATEDIFF(productShelfLife, now()) <= 8
    Order by DATEDIFF(productShelfLife, now()) ASC`

    try {
        const data = await pool.query(sql, uID)
        console.log(data[0])
        let jsonArray 	= new Array();
        for (let i=0; i<data[0].length; i++) {
            let jsonObj		= new Object();
            jsonObj.Img = data[0][i].Img
            jsonObj.Dday = data[0][i].Dday;
            jsonObj.Name = data[0][i].Name;
            jsonObj.Count = data[0][i].Count;
            jsonObj = JSON.stringify(jsonObj);
            //String 형태로 파싱한 객체를 다시 json으로 변환
            jsonArray.push(JSON.parse(jsonObj));
        }
        console.log(jsonArray);
        res.send(jsonArray)

    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.get('/getAllItem', async (req, res) => {
    const uID = 1
    // const uID = req.session.uid;
    const sql = `SELECT a.Classification2Image as Img, DATEDIFF(productShelfLife, now()) as Dday, b.productName as Name, b.productCount as Count
    FROM refreci.UserProduct as b
    right join refreci.Classification2 as a
    on b.productClassification2 = a.c2ID
    Where b.uID=?
    Order by DATEDIFF(productShelfLife, now()) ASC`
    try {
        const data = await pool.query(sql, uID)
        let jsonArray 	= new Array();
        for (let i=0; i<data[0].length; i++) {
            let jsonObj		= new Object();
            jsonObj.Img = data[0][i].Img
            jsonObj.Dday = data[0][i].Dday;
            jsonObj.Name = data[0][i].Name;
            jsonObj.Count = data[0][i].Count;
            
            jsonObj = JSON.stringify(jsonObj);
            //String 형태로 파싱한 객체를 다시 json으로 변환
            jsonArray.push(JSON.parse(jsonObj));
        }
        res.send(jsonArray)

    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.post("/getItems", async (req, res) =>{
    const uID = 1
    // const uID = req.session.uid;
    let diffDay;
    console.log(req.body)
    let type
    let  sql
    let getDate, date, getyear, getmonth, getday, year, month, day
    //처음 페이지가 실행됐을 때 NaN-NaN-NaN 처리
    if (req.body.date == ''){
        type = 1
        sql = `SELECT a.Classification2Image as Img, DATEDIFF(productShelfLife, now()) as Dday, b.productName as Name, b.productCount as Count
        FROM refreci.UserProduct as b
        right join refreci.Classification2 as a
        on b.productClassification2 = a.c2ID
        Where b.uID=?
        Order by DATEDIFF(productShelfLife, now()) ASC`
    }
    else{
        type = 2
        getDate = new Date(req.body.date)
        date = new Date()
        sql = `SELECT a.Classification2Image as Img, DATEDIFF(productShelfLife, ?) as Dday, b.productName as Name, b.productCount as Count
        FROM refreci.UserProduct as b
        right join refreci.Classification2 as a
        on b.productClassification2 = a.c2ID
        Where b.uID=? AND productShelfLife = ?;`

        diffDay = Math.floor((getDate - date) / 1000 / 60 / 60 / 24)

        getyear = getDate.getFullYear().toString();
        getmonth = getDate.getMonth() + 1;
        getmonth = getmonth < 10 ? '0' + getmonth.toString() : getmonth.toString();
        getday = getDate.getDate();
        getday = getday < 10 ? '0' + getday.toString() : getday.toString();


        year = date.getFullYear().toString();
        month = date.getMonth() + 1;
        month = month < 10 ? '0' + month.toString() : month.toString();
        day = date.getDate();
        day = day < 10 ? '0' + day.toString() : day.toString();
    }
    // console.log(year+'-'+month+'-'+day)
    try {
        let data
        if (type == 1){
            data = await pool.query(sql, uID)
            // console.log(data[0])
        }
        else if (type == 2){
            data = await pool.query(sql, [year+'-'+month+'-' + day, uID, getyear + '-' + getmonth + '-' + getday])
            // console.log(data[0])
        }
        let jsonArray 	= new Array();
        for (let i=0; i<data[0].length; i++) {
            let jsonObj		= new Object();
            jsonObj.Img = data[0][i].Img
            // jsonObj.Dday = diffDay;
            jsonObj.Dday = data[0][i].Dday;
            jsonObj.Name = data[0][i].Name;
            jsonObj.Count = data[0][i].Count;
            jsonObj = JSON.stringify(jsonObj);
            //String 형태로 파싱한 객체를 다시 json으로 변환
            jsonArray.push(JSON.parse(jsonObj));
        }
        // console.log(jsonArray)

        res.send(jsonArray)
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }

})

app.post("/updateDate", async(req, res) => {
    const upID = req.body.upID;
    const date = req.body.date;
    const newDate = new Date(date)

    try{
        await pool.query("UPDATE UserProduct SET productShelfLife = ? WHERE upID = ?", [
            newDate,
            upID,
        ])
    }
    catch(e){
        console.log(e);
    }
})

module.exports = app;