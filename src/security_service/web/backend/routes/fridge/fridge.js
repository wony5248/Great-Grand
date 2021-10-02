const express = require("express");
const app = express.Router();
const axios = require("axios");
const { pool } = require(`../../mysql`)


//현재 날짜를 yyyy-mm-dd 포맷으로 리턴
function getCurrentDate()
{
    const date = new Date();
    const year = date.getFullYear().toString();

    let month = date.getMonth() + 1;
    month = month < 10 ? '0' + month.toString() : month.toString();

    let day = date.getDate();
    day = day < 10 ? '0' + day.toString() : day.toString();

    return year + '-' + month + '-' + day ;
}



app.get("/read", async (req, res) =>{
    try {
        const uID = 1;
        // const uID = req.session.uid;
        
        const [rows, fields] = await pool.query('SELECT productName, productClassification2 FROM UserProduct WHERE uID = ?', [
            uID
        ])
        
        let datas = [];
        for (let i = 0; i < rows.length; i++){
            let data = { name: rows[i].productName, category: rows[i].productClassification2};
            datas.push(data);
        }
        
        res.json(datas);
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.get("/classification1", async (req, res) => {
    try {
        const list = [{c1ID: 0, classification1Name: "전체"}];
        const [rows, fields] = await pool.query('SELECT c1ID, classification1Name FROM Classification1', [])
        const len = rows.length;

        for(let i=0; i<len; i++){
            list.push(rows[i]);
        }
        // console.log(list);
        res.json(list);
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.get("/classification2", async (req, res) => {
    const cl1 = req.query.cl1ID;

    try {
        const [rows, fields] = await pool.query('SELECT c2ID, classification2Name FROM Classification2 WHERE classification2to1 = ?', [
            cl1
        ])
        
        // console.log(rows);
        res.json(rows);
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.get("/searchUserProduct", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    
    const cl1 = req.query.cl1ID;

    try {
        let list = [];
        const [rows, fields] = await pool.query('SELECT upID, productClassification2, productName, productCount, productShelfLife, productImage FROM UserProduct WHERE uID = ? AND productClassification1 = ?', [
            uID,
            cl1
        ])
        let len = rows.length;
        for(let i=0; i<len; i++){
            let tmp_obj = new Object();
            tmp_obj.upID = rows[i].upID;
            tmp_obj.productClassification2 = rows[i].productClassification2;
            tmp_obj.productName = rows[i].productName;
            tmp_obj.productCount = rows[i].productCount;
            tmp_obj.productImage = rows[i].productImage;
            if (rows[i].productShelfLife === null){
                tmp_obj.productShelfLife = "0000-00-00"
            }
            else{
                tmp_obj.productShelfLife = rows[i].productShelfLife;
            }
            list.push(tmp_obj)
        }

        res.json(list);
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

app.get("/allUserProduct", async (req, res) => {
    const uID = 1;
    // const uID = req.session.uid;
    const sql ='SELECT up.productName, up.productCount, up.productShelfLife, up.productImage, c1.classification1Name, c2.classification2Name, up.productClassification2 \
                FROM UserProduct AS up \
                JOIN Classification1 AS c1 \
                ON up.productClassification1 = c1.c1ID \
                JOIN Classification2 AS c2 \
                ON up.productClassification2 = c2.c2ID \
                WHERE up.uID = ?'

    try {
        let list = [];
        const [rows, fields] = await pool.query(sql, [
            uID,
        ])

        let len = rows.length;
        for (let i = 0; i < len; i++) {
            let tmp_obj = new Object();
            tmp_obj.productName = rows[i].productName;
            tmp_obj.productCount = rows[i].productCount;
            tmp_obj.productImage = rows[i].productImage;
            tmp_obj.classification1Name = rows[i].classification1Name;
            tmp_obj.classification2Name = rows[i].classification2Name;
            tmp_obj.productClassification2 = rows[i].productClassification2;
            if (rows[i].productShelfLife === null) {
                tmp_obj.productShelfLife = "0000-00-00"
            }
            else {
                tmp_obj.productShelfLife = rows[i].productShelfLife;
            }
            list.push(tmp_obj)
        }

        console.log("list", list)
        res.json(list);
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }
})

//재료 삽입
app.post("/", async (req, res) =>{
    const nowDay = getCurrentDate()
    try {
        const uID = req.body.uID
        const productName = req.body.productName
        const productCount = req.body.productCount
        const productClassification1 = req.body.productClassification1
        const productClassification2 = req.body.productClassification2
        const productShelfLife = req.body.productShelfLife

        if (!productName || !productCount || !productClassification1 || !productClassification2 || !productShelfLife) {
            return res.status(404).json({ message: "post does not exist" });
        }
        const sql = `INSERT INTO refreci.UserProduct
        (uID, productName, productCount, createdDate, productClassification1, productClassification2, productShelfLife, isDeleted)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?);`
        
    
        await pool.query(sql,[uID, productName, productCount, nowDay,productClassification1, productClassification2, productShelfLife, 0])

        console.log('Call Updated DB')
        const [rows, fields] = await pool.query('SELECT * FROM refreci.UserProduct WHERE uID = 1')
        res.redirect('/')
        for (let i = 0; i < rows.length; i++){
            console.log(rows[i])
        }
    }
    catch (err) {
        console.log(err);
        return new Error(err)
    }

})
//재료명 수정시 소분류 매핑 방법을 생각해 보아야할 듯
app.put("/", async (req, res) =>{
    try {
        console.log(req.body);
        const upID = req.body.upID
        const productName = req.body.productName
        const productCount = req.body.productCount
        const productClassification1 = req.body.productClassification1
        const productClassification2 = req.body.productClassification2
        const productShelfLife = req.body.productShelfLife
    
        const sql = `UPDATE refreci.UserProduct
        SET productName = ?, productCount = ?, productClassification1 = ?, productClassification2 = ?, productShelfLife =  ?
        WHERE upID = ?;`
        console.log("Update DB by PUT Method")
        await pool.query(sql,[productName, productCount, productClassification1, productClassification2, productShelfLife, upID])
        res.redirect('/')
        // 테스트를 위한 냉장고 DB 호출 코드

        console.log('Call Updated DB')
        const [rows, fields] = await pool.query('SELECT * FROM refreci.UserProduct WHERE uID = 1')
    
        for (let i = 0; i < rows.length; i++){
            console.log(rows[i])
        }
    }
    catch (err) {
        console.log(err);
        return new Error(err)
    }
})

app.delete('/', async (req, res) => {
    try{
        const uID = req.body.uID
        const upID = req.body.upID
        await pool.query('DELETE FROM refreci.UserProduct WHERE uID = ? and upID = ?;',[uID,upID])
        // 테스트를 위한 냉장고 DB 호출 코드

        console.log('Call Updated DB')
        const [rows, fields] = await pool.query('SELECT * FROM refreci.UserProduct WHERE uID = 1')
            
        for (let i = 0; i < rows.length; i++){
            console.log(rows[i])
        }
    }
    catch (err) {
        console.log(err);
        return new Error(err)
    }
})

module.exports = app;
