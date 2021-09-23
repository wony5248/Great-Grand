const express = require("express");
const app = express.Router();
const axios = require("axios");
const { pool } = require(`../../mysql`)


app.get("/getEvents", async (req, res) =>{
    const uID = 1
    // const uID = req.session.uid;
    const sql = `SELECT DISTINCT(productShelfLife)
    FROM refreci.UserProduct 
    WHERE uID = ?`
    try {

        const data = await pool.query(sql, [uID])
        // console.log(data)
        let jsonArray 	= new Array();
        for (let i=0; i<data[0].length; i++) {
            
            let jsonObj		= new Object();
                
            jsonObj.title = '';
            jsonObj.start	= new Date(data[0][i].productShelfLife);
            jsonObj.end	= new Date(data[0][i].productShelfLife);
            if (data[0][i].productShelfLife == null){
                jsonObj.title = '';
                jsonObj.start = "0000-00-00";
                jsonObj.end	= "0000-00-00";
                console.log(data[0][i])
            }
            jsonObj = JSON.stringify(jsonObj);
            //String 형태로 파싱한 객체를 다시 json으로 변환
            jsonArray.push(JSON.parse(jsonObj));
        }

        console.log(jsonArray)
        res.send(jsonArray)
    }
    catch (err) {
        console.log(err)
        return new Error(err)
    }

})

module.exports = app;
